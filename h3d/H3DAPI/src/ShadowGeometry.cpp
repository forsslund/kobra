//////////////////////////////////////////////////////////////////////////////
//    Copyright 2004-2019, SenseGraphics AB
//
//    This file is part of H3D API.
//
//    H3D API is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    H3D API is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with H3D API; if not, write to the Free Software
//    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
//    A commercial license is also available. Please contact us at
//    www.sensegraphics.com for more information.
//
//
/// \file ShadowGeometry.cpp
/// \brief CPP file for ShadowGeometry
///
//
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/ShadowGeometry.h>
#include <H3D/GraphicsOptions.h>
#include <H3D/GlobalSettings.h>
#include <H3D/ShadowCasterShaders.h>
#include <H3D/Sphere.h>

using namespace H3D;

// Add this node to the H3DNodeDatabase system.
H3DNodeDatabase ShadowGeometry::database( "ShadowGeometry",
                                     &(newInstance<ShadowGeometry>),
                                     typeid( ShadowGeometry ),
                                     &H3DShadowObjectNode::database );

namespace ShadowGeometryInternals {
  FIELDDB_ELEMENT( ShadowGeometry, geometry, INPUT_OUTPUT )
}

ShadowGeometry::ShadowGeometry( Inst< SFNode>  _metadata,
                                Inst< SFTransformNode > _transform,
                                Inst< SFGeometryNode > _geometry,
                                Inst< SFBool > _enabled ) :
H3DShadowObjectNode( _metadata, _transform, _enabled ),
  triangles_changed( new Field ),
  geometry( _geometry ),
  use_geometry_shader_last_loop( false ),
  rebuild_triangles_ts( false ) {

  is_enabled_ts = enabled->getValue();

  type_name = "ShadowGeometry";
  database.initFields( this );
}


void ShadowGeometry::addDirectionalLightQuadPoints( vector< Vec4d > &triangle_points,
                                                    const Vec3d &v1,
                                                    const Vec3d &v2,
                                                    const Vec3d& dir ) {
  Vec4d ev1(v1, 1);
  Vec4d ev2(v2, 1);
  Vec4d inf( dir, 0 );

  triangle_points.push_back( ev1 );
  triangle_points.push_back( inf );
  triangle_points.push_back( inf );

  triangle_points.push_back( ev1 );
  triangle_points.push_back( inf );
  triangle_points.push_back( ev2 );
}

void ShadowGeometry::addPointLightQuadPoints( vector< Vec4d > &triangle_points,
                                             const Vec3d &v1, const Vec3d &v2,
                                             const Vec3d& light_pos ) {
  Vec3d dir1 = v1 - light_pos;
  dir1.normalizeSafe();

  Vec3d dir2 = v2 - light_pos;
  dir2.normalizeSafe();

  Vec4d ev1(v1, 1 );
  Vec4d ev2(v2, 1 );
  Vec4d inf1( dir1, 0 );
  Vec4d inf2( dir2, 0 );

  triangle_points.push_back( ev1 );
  triangle_points.push_back( inf1 );
  triangle_points.push_back( inf2 );

  triangle_points.push_back( ev1 );
  triangle_points.push_back( inf2 );
  triangle_points.push_back( ev2 );
}

void ShadowGeometry::updateSilhouetteEdgesDirectionalLight( const vector< HAPI::Collision::Triangle > &_triangles,
                                                            const vector<int> &_neighbours,
                                                            const Vec3d &direction ) {

  triangle_facing_light.resize( _triangles.size(), false );
  is_silhouette_edge.resize( _triangles.size()*3, false );

  for( size_t i = 0; i < _triangles.size(); ++i ) {
    triangle_facing_light[i] = direction.dotProduct( _triangles[i].normal ) <= 0;
  }

  for( size_t i = 0; i < _neighbours.size(); ++i ) {
    if(!triangle_facing_light[i/3]) {
      // silhouette edges are only on triangles facing the light
      is_silhouette_edge[i] = false;
    } else {
      // silhouette if no neighbour or the neighbour is not facing the light
      is_silhouette_edge[i] = _neighbours[i] == -1 || !triangle_facing_light[_neighbours[i]];
    }
  }
}


void ShadowGeometry::updateSilhouetteEdgesPointLight( const vector< HAPI::Collision::Triangle > &_triangles,
                                                      const vector<int> &_neighbours,
                                                      const Vec3d &pos ) {
  triangle_facing_light.resize( _triangles.size(), false );
  is_silhouette_edge.resize( _triangles.size()*3, false );

  for( size_t i = 0; i < _triangles.size(); ++i ) {
    Vec3d direction = _triangles[i].a - pos;
    triangle_facing_light[i] = direction.dotProduct( _triangles[i].normal ) <= 0;
  }

  for( size_t i = 0; i < _neighbours.size(); ++i ) {
    if(!triangle_facing_light[i/3]) {
      // silhouette edges are only on triangles facing the light
      is_silhouette_edge[i] = false;
    } else {
      // silhouette if no neighbour or the neighbour is not facing the light
      is_silhouette_edge[i] = _neighbours[i] == -1 || !triangle_facing_light[_neighbours[i]];
    }
  }

 }


bool operator< (const Vec3d & s1, const Vec3d &s2) {
  return (s1.x < s2.x ||
          (s2.x == s1.x && s1.y < s2.y) ||
          ( s2.x == s1.x && s2.y == s1.y && s1.z < s2.z) );
}

struct lt {
  bool operator()(const pair<Vec3d, Vec3d>& _Left,
                  const pair<Vec3d, Vec3d >& _Right ) const {
    return (_Left.first < _Right.first ||
            (!(_Right.first < _Left.first) && _Left.second < _Right.second));
  }
};

void ShadowGeometry::updateNeighbours( const vector< HAPI::Collision::Triangle > &_triangles ) {
  neighbours.clear();
  neighbours.resize( _triangles.size()*3, -1 );

  // map from triangle edge(as pair of vertex) to pair of
  // (triangle index, edge index within triangle)
  typedef map< pair< Vec3d, Vec3d >, pair<int, int>, lt >  EdgeTriangleMap;
  EdgeTriangleMap edges;

  for( unsigned int i = 0; i < _triangles.size(); ++i ) {
    const HAPI::Collision::Triangle &tri = _triangles[i];

    // ignore invalid triangles that are lines or points
    if( tri.a == tri.b || tri.b == tri.c || tri.a == tri.c ) {
      continue;
    }

    // edges of the triangles. We keep a strict ordering when defining an edge so that
    // the edge (a,b) will be the same as (b,a).
    pair< Vec3d, Vec3d > edge0 = tri.a < tri.b ? make_pair(tri.a, tri.b) : make_pair(tri.b, tri.a );
    pair< Vec3d, Vec3d > edge1 = tri.b < tri.c ? make_pair(tri.b, tri.c) : make_pair(tri.c, tri.b );
    pair< Vec3d, Vec3d > edge2 = tri.c < tri.a ? make_pair(tri.c, tri.a) : make_pair(tri.a, tri.c );

    // Check if the edge exists in previously processed triangle.
    EdgeTriangleMap::iterator edge0_i = edges.find( edge0 );
    EdgeTriangleMap::iterator edge1_i = edges.find( edge1 );
    EdgeTriangleMap::iterator edge2_i = edges.find( edge2 );

    if( edge0_i != edges.end() ) {
      // shared edge found, update the neighbour array.
      int triangle = (*edge0_i).second.first;
      int edge =  (*edge0_i).second.second;
      neighbours[i*3] = triangle;
      if( neighbours[triangle*3+edge] == -1 ) {
         neighbours[triangle*3+edge] = i;
      }
    } else {
      edges[edge0] = make_pair(i, 0);
    }

   if( edge1_i != edges.end() ) {
      // shared edge found, update the neighbour array.
      int triangle = (*edge1_i).second.first;
      int edge =  (*edge1_i).second.second;
      neighbours[i*3+1] = triangle;
      if( neighbours[triangle*3+edge] == -1 ) {
         neighbours[triangle*3+edge] = i;
      }
    } else {
      edges[edge1] = make_pair(i, 1);
    }

   if( edge2_i != edges.end() ) {
     // shared edge found, update the neighbour array.
      int triangle = (*edge2_i).second.first;
      int edge =  (*edge2_i).second.second;
      neighbours[i*3+2] = triangle;
      if( neighbours[triangle*3+edge] == -1 ) {
        neighbours[triangle*3+edge] = i;
      }
    } else {
      edges[edge2] = make_pair(i, 2);
    }
  }
}


void ShadowGeometry::SFGeometryNode::onAdd( Node *n ) {
  TypedSFNode< X3DGeometryNode >::onAdd( n );
  X3DGeometryNode*geom = dynamic_cast< X3DGeometryNode * >( n );
  ShadowGeometry*shadow_geom = dynamic_cast< ShadowGeometry * >( getOwner() );
  if( geom ) {
    geom->boundTree->route( shadow_geom->triangles_changed );
  }
}

void ShadowGeometry::SFGeometryNode::onRemove( Node *n ) {
  X3DGeometryNode*geom = dynamic_cast< X3DGeometryNode * >( n );
  ShadowGeometry*shadow_geom = dynamic_cast< ShadowGeometry * >( getOwner() );
  if( geom ) {
    geom->boundTree->unroute( shadow_geom->triangles_changed );
  }
  TypedSFNode< X3DGeometryNode >::onRemove( n );
}

void ShadowGeometry::updateAdjacentVertexArray( const vector< HAPI::Collision::Triangle > &_triangles,
                                                 vector< Vec3d > &_triangle_points,
                                                 vector< unsigned int > &_adjacency_index ) {
  updateNeighbours( _triangles );

  _triangle_points.clear();
  _triangle_points.reserve( _triangles.size() * 3 );
  _adjacency_index.clear();
  _adjacency_index.reserve( _triangles.size() * 6 );

  for( unsigned int i = 0; i < _triangles.size(); ++i ) {

    _triangle_points.push_back( _triangles[i].a );
    _triangle_points.push_back( _triangles[i].b );
    _triangle_points.push_back( _triangles[i].c );

        // if( triangles[i].a == triangles[i].b ||
        //      triangles[i].b == triangles[i].c ||
        //      triangles[i].c == triangles[i].a ) { Console(LogLevel::Error) << "Degenerate" << endl; continue; }
    //    cerr << triangles[i].a << endl;

    //    _adjacency_index.push_back( i*3 );
    //    _adjacency_index.push_back( i*3 +1);
    //    _adjacency_index.push_back( i*3 +2);
    //    continue;
    // Triangle adjacency specification order
    // 1---2---3
    // |  /|  /
    // | / | /
    // |/  |/
    // 0---4
    // |  /
    // | /
    // |/
    // 5

    _adjacency_index.push_back( i*3 );

    // find neighbour triangle vertex index
    int n0_index = neighbours[ i*3 ];
    if( n0_index == -1 ) {
      // no neighbour so just repeat the vertex
      //Console(LogLevel::Error) << "No neighbour" << endl;
      _adjacency_index.push_back( i*3 );
    } else {
      const HAPI::Collision::Triangle &n0 = _triangles[n0_index];
      _adjacency_index.push_back( n0_index*3 +
                                 getMissingPointIndex( n0,
                                                       _triangles[i].a,
                                                       _triangles[i].b ) );
    }

    _adjacency_index.push_back( i*3+1 );

    // find neighbour triangle vertex index
    int n1_index = neighbours[ i*3 + 1];
    if( n1_index == -1 ) {
      // no neighbour so just repeat the vertex
      // Console(LogLevel::Error) << "No neighbour" << endl;
      _adjacency_index.push_back( i*3 + 1 );
    } else {
      const HAPI::Collision::Triangle &n1 = _triangles[n1_index];
      _adjacency_index.push_back( n1_index*3 +
                                 getMissingPointIndex( n1,
                                                       _triangles[i].b,
                                                       _triangles[i].c ) );
    }

    _adjacency_index.push_back( i*3+2 );

    // find neighbour triangle vertex index
    int n2_index = neighbours[ i*3 + 2];
    if( n2_index == -1 ) {
      // no neighbour so just repeat the vertex
      // Console(LogLevel::Error) << "No neighbour" << endl;
      _adjacency_index.push_back( i*3 + 2 );
    } else {
      const HAPI::Collision::Triangle &n2 = _triangles[n2_index];
      _adjacency_index.push_back( n2_index*3 +
                                 getMissingPointIndex( n2,
                                                       _triangles[i].c,
                                                       _triangles[i].a ) );
    }
  }
}


int ShadowGeometry::getMissingPointIndex( const HAPI::Collision::Triangle  &t,
                                          const Vec3d &p0,
                                          const Vec3d &p1 ) {
  if( t.a != p0 && t.a != p1 ) return 0;
  if( t.b != p0 && t.b != p1 ) return 1;
  if( t.c != p0 && t.c != p1 ) return 2;
  return -1;
}


void ShadowGeometry::update() {
  H3DShadowObjectNode::update();

  is_enabled_ts = enabled->getValue();

  if( transform->getValue() ) {
    transform_matrix_ts = transform->getValue()->matrix->getValue();
  } else {
    transform_matrix_ts = Matrix4f();
  }

  // Determine if geometry needs rebuild.
  X3DGeometryNode* g = geometry->getValue();
  if( !g ) { return; }

  GraphicsOptions *graphics_options = NULL;
  GlobalSettings *default_settings = GlobalSettings::getActive();
  if( default_settings ) {
    default_settings->getOptionNode( graphics_options );
  }

  bool use_geometry_shader =
    GLEW_EXT_geometry_shader4 &&
    (!graphics_options ||
    (graphics_options &&
      graphics_options->defaultShadowGeometryAlgorithm->getValue() == "GEOMETRY_SHADER"));

  rebuild_triangles_ts = !triangles_changed->isUpToDate() ||
    use_geometry_shader_last_loop != use_geometry_shader;

  triangles_changed->upToDate();
  // update triangle information
  if( rebuild_triangles_ts ) {
    triangles.clear();
    g->boundTree->getValue()->getAllTriangles( triangles );
  }

  use_geometry_shader_last_loop = use_geometry_shader;
}

void ShadowGeometry::renderShadowGPU( const LightDataStruct& light_data, Matrix4f acc_fwd, bool render_caps ) {

  bool is_enabled = is_enabled_ts;
  const Matrix4f& transform_matrix = transform_matrix_ts;

  if( !is_enabled ) {
    return;
  }

  if( rebuild_triangles_ts ) {
    triangles_ts.clear();
    // swap data as coping is slow
    triangles_ts.swap( triangles );
    updateAdjacentVertexArray( triangles_ts, triangle_points_geom_shader, index_geom_shader );
    rebuild_triangles_ts = false;
  }

  Matrix4f m = acc_fwd * transform_matrix;

  ShadowCasterShaders::setTransformMatrix( m );
  // draw shadow geometry using vertex arrays
  glEnableClientState( GL_VERTEX_ARRAY );
  glVertexPointer( 3, GL_DOUBLE, 0, &(*triangle_points_geom_shader.begin()) );
  glDrawElements( GL_TRIANGLES_ADJACENCY_EXT, (unsigned int)index_geom_shader.size(), GL_UNSIGNED_INT, &(*(index_geom_shader.begin())) );
  glDisableClientState( GL_VERTEX_ARRAY );
}


void ShadowGeometry::computeShadowVolumeInformationCPU( const LightDataStruct& light_data, Matrix4f acc_fwd, bool render_caps, std::vector< Vec4d >& coord ) {

  bool is_enabled = is_enabled_ts;
  const Matrix4f&  transform_matrix = transform_matrix_ts;
  bool rebuild_tris = rebuild_triangles_ts;

  if( !is_enabled ) {
    return;
  }

  if( rebuild_triangles_ts ) {
    triangles_ts.clear();
    // swap data as coping is slow
    triangles_ts.swap( triangles );
    rebuild_triangles_ts = false;
  }

  Matrix4f m = acc_fwd * transform_matrix;
  buildShadowVolumeDataCPU( light_data, render_caps, m, m.inverse(), rebuild_tris, coord );
}

void ShadowGeometry::buildShadowVolumeDataCPU( LightDataStruct light_data,
  bool draw_caps,
  const Matrix4f &m,
  const Matrix4f &m_inv,
  bool rebuild_triangle_info,
  std::vector< Vec4d >&  coord ) {

  if( rebuild_triangle_info ) {
    updateNeighbours( triangles_ts );
  }

  // to check if the light has changed this frame
  std::vector<Vec4d> triangle_points;
  triangle_points.reserve( triangles_ts.size() * 3 * 2 );

  // draw quads for each silhouette edge and its projection at infinity.
  if( light_data.isDirectionalLight() ) {
    Vec3f dir = light_data.getLightDirection();
    dir = m_inv.getRotationPart() * dir;
    updateSilhouetteEdgesDirectionalLight( triangles_ts, neighbours, dir );
    for( size_t i = 0; i < triangles_ts.size(); ++i ) {
      // no silhouette edges are on triangles not facing the light
      if( !triangle_facing_light[i] ) {
        continue;
      }

      if( is_silhouette_edge[i * 3] ) {
        addDirectionalLightQuadPoints( triangle_points, triangles_ts[i].a, triangles_ts[i].b, dir );
      }

      if( is_silhouette_edge[i * 3 + 1] ) {
        addDirectionalLightQuadPoints( triangle_points, triangles_ts[i].b, triangles_ts[i].c, dir );
      }

      if( is_silhouette_edge[i * 3 + 2] ) {
        addDirectionalLightQuadPoints( triangle_points, triangles_ts[i].c, triangles_ts[i].a, dir );
      }
    }

    if( draw_caps ) {
      for( size_t i = 0; i < triangles_ts.size(); ++i ) {
        if( triangle_facing_light[i] ) {
          triangle_points.push_back( Vec4d( triangles_ts[i].a, 1 ) );
          triangle_points.push_back( Vec4d( triangles_ts[i].b, 1 ) );
          triangle_points.push_back( Vec4d( triangles_ts[i].c, 1 ) );
        }
        // directional lights do not need a far cap since the shadow volume
        // converge to the same point at infinity.
      }
    }
  } else if( light_data.isPointLight() ) {
    Vec3d light_pos = m_inv * light_data.getLightPosition();
    updateSilhouetteEdgesPointLight( triangles_ts, neighbours, light_pos );
    for( size_t i = 0; i < triangles_ts.size(); ++i ) {
      // no silhouette edges are on triangles not facing the light
      if( !triangle_facing_light[i] ) {
        continue;
      }

      if( is_silhouette_edge[i * 3] ) {
        addPointLightQuadPoints( triangle_points, triangles_ts[i].a, triangles_ts[i].b, light_pos );
      }

      if( is_silhouette_edge[i * 3 + 1] ) {
        addPointLightQuadPoints( triangle_points, triangles_ts[i].b, triangles_ts[i].c, light_pos );
      }

      if( is_silhouette_edge[i * 3 + 2] ) {
        addPointLightQuadPoints( triangle_points, triangles_ts[i].c, triangles_ts[i].a, light_pos );
      }
    }

    if( draw_caps ) {
      // draw all triangles facing the light as a near cap and all others
      // at infinity.
      for( size_t i = 0; i < triangles_ts.size(); ++i ) {
        if( triangle_facing_light[i] ) {
          triangle_points.push_back( Vec4d( triangles_ts[i].a, 1 ) );
          triangle_points.push_back( Vec4d( triangles_ts[i].b, 1 ) );
          triangle_points.push_back( Vec4d( triangles_ts[i].c, 1 ) );
        } else {
          Vec3d v1 = triangles_ts[i].a - light_pos;
          Vec3d v2 = triangles_ts[i].b - light_pos;
          Vec3d v3 = triangles_ts[i].c - light_pos;

          triangle_points.push_back( Vec4d( v1, 0 ) );
          triangle_points.push_back( Vec4d( v2, 0 ) );
          triangle_points.push_back( Vec4d( v3, 0 ) );
        }
      }
    }
  }

  // Safety check
  if( triangle_points.empty() ) {
    return;
  }

  size_t coord_idx = coord.size();
  coord.resize( coord.size() + triangle_points.size() );
  // convert the coords to global
  for( size_t i = 0; i < triangle_points.size(); ++i ) {
    const Vec4d& p = m * triangle_points[i];
    coord[coord_idx] = p;
    ++coord_idx;
  }
}