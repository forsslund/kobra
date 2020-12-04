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
/// \file ShadowSphere.cpp
/// \brief CPP file for ShadowSphere
///
//
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/ShadowSphere.h>
#include <H3D/ShadowCasterShaders.h>

// Purely included for the utility functions
// to convert from quad/poly to triangles.
#include <H3D/X3DComposedGeometryNode.h>
#include <H3D/GraphicsOptions.h>
#include <H3D/GlobalSettings.h>


using namespace H3D;

// Add this node to the H3DNodeDatabase system.
H3DNodeDatabase ShadowSphere::database( "ShadowSphere",
                                     &(newInstance<ShadowSphere>),
                                     typeid( ShadowSphere ),
                                     &H3DShadowObjectNode::database );

namespace ShadowSphereInternals {
  FIELDDB_ELEMENT( ShadowSphere, radius, INPUT_OUTPUT )
  FIELDDB_ELEMENT( ShadowSphere, position, INPUT_OUTPUT )
  FIELDDB_ELEMENT( ShadowSphere, detailLevel, INPUT_OUTPUT )
}

ShadowSphere::ShadowSphere( Inst< SFNode>  _metadata,
                            Inst< SFFloat > _radius,
                            Inst< SFVec3f > _position,
                            Inst< SFInt32 > _detailLevel ) :
  H3DShadowObjectNode( _metadata ),
  radius( _radius ),
  position( _position ),
  detailLevel( _detailLevel ),
  radius_ts( 1 ),
  detail_level_ts( 120 ) {

  type_name = "ShadowSphere";
  database.initFields( this );

  radius->setValue( radius_ts );
  position->setValue( Vec3f( 0, 0, 0 ) );
  detailLevel->setValue( detail_level_ts );

  is_enabled_ts = enabled->getValue();
}

void H3D::ShadowSphere::update() {
  H3DShadowObjectNode::update();

  is_enabled_ts = enabled->getValue();
  if( transform->getValue() ) {
    transform_matrix_ts = transform->getValue()->matrix->getValue();
  } else {
    transform_matrix_ts = Matrix4f();
  }

  radius_ts = radius->getValue();
  detail_level_ts = detailLevel->getValue();
  position_ts = position->getValue();
}

void ShadowSphere::buildGeometryData( bool is_dir_light, int detail_level, H3DFloat _radius, bool render_caps,
  Vec3f /*light_dir*/, Vec3f light_pos, std::vector<Vec4d>& coord, Matrix4f local_to_global, bool coords_in_global ) {

  int nr_faces = detail_level;
  H3DFloat r = _radius;

  // render side
  std::vector<Vec4d> side_tris;
  side_tris.reserve( (nr_faces * 2) + 2 );

  Vec4d v0, v1, v2, v3;

  for( int i = 0; i <= nr_faces; ++i ) {
    float ratio = (float)i / nr_faces;
    if( i == nr_faces ) {
      ratio = 0;
    }
    float angle = (float)(ratio * (Constants::pi * 2));
    float sina = sin( angle );
    float cosa = cos( angle );

    //glNormal3f(-sina, 0, -cosa);

    Vec3d v( -r * sina, 0, -r * cosa );
    Vec3d d;
    if( !is_dir_light ) {
      d = v - light_pos;
    } else {
      d = Vec3d( 0, 1, 0 );
    }

    if( coords_in_global ) {
      side_tris.push_back( local_to_global * Vec4d( v.x, v.y, v.z, 1.0f ) );
      side_tris.push_back( local_to_global * Vec4d( d.x, d.y, d.z, 0.0f ) );
    } else {
      side_tris.push_back( Vec4d( v.x, v.y, v.z, 1.0f ) );
      side_tris.push_back( Vec4d( d.x, d.y, d.z, 0.0f ) );
    }
  }

  // Now we have correct triangle indices into the coordinates. Fetch them in
  // right order and insert them into coordinate attribute.
  size_t coord_idx = coord.size();
  // resize the buffer to that need to fill the data from side tris
  coord.resize( coord.size() + ((side_tris.size() - 2) / 2 * 6) );
  for( size_t i = 0; i < side_tris.size() - 2; i += 2 ) {
    coord[coord_idx] = side_tris[i];  ++coord_idx;
    coord[coord_idx] = side_tris[i + 3]; ++coord_idx;
    coord[coord_idx] = side_tris[i + 1]; ++coord_idx;

    coord[coord_idx] = side_tris[i];  ++coord_idx;
    coord[coord_idx] = side_tris[i + 2];  ++coord_idx;
    coord[coord_idx] = side_tris[i + 3];  ++coord_idx;
  }


  if( render_caps ) {
    std::vector<Vec4d> cap_tris;
    std::vector<int> cap_indices;
    std::vector<int> temp_polygon_indices;

    if( !is_dir_light ) {
      // render top at infinity. Do not have to do this for DirectionalLight
      // since then it is already capped since all point converge to the same point
      // at infinity
      for( int i = 0; i <= nr_faces; ++i ) {
        float ratio = (float)i / nr_faces;
        float angle = (float)(ratio * (Constants::pi * 2));
        float sina = sin( angle );
        float cosa = cos( angle );
        Vec3d v = Vec3d( -r * sina, 0, -r * cosa ) - light_pos;
        if( coords_in_global ) {
          cap_tris.push_back( local_to_global * Vec4d( v.x, v.y, v.z, 0.0f ) );
        } else {
          cap_tris.push_back( Vec4d( v.x, v.y, v.z, 0.0f ) );
        }
        temp_polygon_indices.push_back( i );
      }

      temp_polygon_indices.push_back( 0 );

      addConvexPolygon( temp_polygon_indices, 0, temp_polygon_indices.size() - 1, cap_indices );

      coord_idx = coord.size();
      // Add all of the bottom vertices.
      coord.resize( coord.size() + cap_indices.size() );
      for( size_t i = 0; i < cap_indices.size(); ++i ) {
        coord[coord_idx] = cap_tris[cap_indices[i]];
        ++coord_idx;
      }

      cap_indices.clear();
      cap_tris.clear();
      temp_polygon_indices.clear();
    }

    //render bottom
    for( int i = 0; i <= nr_faces; ++i ) {
      float ratio = (float)i / nr_faces;
      float angle = (float)(ratio * (Constants::pi * 2));
      float sina = sin( angle );
      float cosa = cos( angle );
      if( coords_in_global ) {
        cap_tris.push_back( local_to_global * Vec4d( -r * sina, 0, -r * cosa, 1.0f ) );
      } else {
        cap_tris.push_back( Vec4d( -r * sina, 0, -r * cosa, 1.0f ) );
      }

      temp_polygon_indices.push_back( i );
    }

    temp_polygon_indices.push_back( 0 );

    // We need to do one of these but CW instead of
    // CCW because different facing direction ...
    addConvexPolygon( temp_polygon_indices, 0,temp_polygon_indices.size() - 1, cap_indices );

    // Add all of the bottom vertices.
    // Different winding order because different facing.
    coord_idx = coord.size();
    coord.resize( coord.size() + cap_indices.size() );
    for( size_t i = 0; i < cap_indices.size(); i += 3 ) {
      coord[coord_idx] = cap_tris[cap_indices[i + 2]]; ++coord_idx;
      coord[coord_idx] = cap_tris[cap_indices[i + 1]]; ++coord_idx;
      coord[coord_idx] = cap_tris[cap_indices[i + 0]]; ++coord_idx;
    }
  }
}

void ShadowSphere::renderShadowGPU( const LightDataStruct& light_data, Matrix4f acc_fwd, bool render_caps ) {
  bool is_enabled = is_enabled_ts;
  if( !is_enabled ) {
    return;
  }

  Matrix4f  transform_matrix = transform_matrix_ts;
  H3DFloat r = radius_ts;
  Vec3f pos = position_ts;
  int nr_faces = detail_level_ts;
  Matrix4f local_to_global = acc_fwd;

  Matrix4f m = local_to_global * transform_matrix;
  Matrix4f m_inv = m.inverse();

  Vec3f light_dir, light_pos;
  Rotation rot;

  ShadowCasterShaders::shaderToggle( false );

  if( light_data.isDirectionalLight() ) {
    light_pos = pos;
    light_dir = light_data.getLightDirection();
    Vec3f dir = m_inv.getRotationPart() * light_dir;
    dir.normalizeSafe();
    rot = Rotation( Vec3f( 0, 1, 0 ), dir );
  } else if( light_data.isPointLight() ) {
    light_pos = (m_inv *  light_data.getLightPosition());
    Vec3f dir = pos - light_pos;

    // if lightsource is inside sphere, we skip the shadow volume.
    if( (dir * dir) <= (r * r) ) {
      ShadowCasterShaders::shaderToggle( true );
      return;
    }

    H3DFloat d = dir.length();
    if( d > Constants::f_epsilon ) {
      dir.normalizeSafe();
      rot = Rotation( Vec3f( 0, 1, 0 ), dir );
    }

    light_pos = (-rot * light_pos);
  } else {
    ShadowCasterShaders::shaderToggle( true );
    return;
  }

  glMatrixMode( GL_MODELVIEW );
  glPushMatrix();
  GLfloat mv[] = {
    m[0][0], m[1][0], m[2][0], 0,
    m[0][1], m[1][1], m[2][1], 0,
    m[0][2], m[1][2], m[2][2], 0,
    m[0][3], m[1][3], m[2][3], 1 };

  glMultMatrixf( mv );

  glTranslatef( pos.x, pos.y, pos.z );
  glRotatef( (GLfloat)(rot.angle * 180 / Constants::pi),
    rot.axis.x, rot.axis.y, rot.axis.z );

  // render side
  glBegin( GL_QUAD_STRIP );
  for( int i = 0; i <= nr_faces; ++i ) {
    float ratio = (float)i / nr_faces;
    if( i == nr_faces ) {
      ratio = 0;
    }
    float angle = (float)(ratio * (Constants::pi * 2));


    float sina = sin( angle );
    float cosa = cos( angle );
    //glNormal3f( -sina, 0, -cosa );
    Vec3f v( -r * sina, 0, -r * cosa );

    Vec3f d;
    if( light_data.isPointLight() ) d = v - light_pos;
    else d = Vec3f( 0, 1, 0 );
    // point at infinity
    glVertex4f( d.x, d.y, d.z, 0 );
    glVertex4f( v.x, v.y, v.z, 1 );
  }
  glEnd();

  if( render_caps ) {
    if( !light_data.isDirectionalLight() ) {
      // render top at infinity. Do not have to do this for DirectionalLight
      // since then it is already capped since all point converge to the same point
      // at infinity
      glBegin( GL_POLYGON );
      for( int i = 0; i < nr_faces; ++i ) {
        float ratio = (float)i / nr_faces;
        float angle = (float)(ratio * (Constants::pi * 2));
        float sina = sin( angle );
        float cosa = cos( angle );
        Vec3f v = Vec3f( -r * sina, 0, -r * cosa ) - light_pos;
        glVertex4f( v.x, v.y, v.z, 0 );
      }
      glEnd();
    }
    // render bottom
    glBegin( GL_POLYGON );
    glNormal3f( 0, -1, 0 );
    for( int i = nr_faces; i >= 0; --i ) {
      float ratio = (float)i / nr_faces;
      float angle = (float)(ratio * (Constants::pi * 2));
      float sina = sin( angle );
      float cosa = cos( angle );
      glVertex3f( -r * sina, 0, -r * cosa );
    }
    glEnd();
  }
  glPopMatrix();

  ShadowCasterShaders::shaderToggle( true );
}

void ShadowSphere::computeShadowVolumeInformationCPU( const LightDataStruct& light_data, Matrix4f acc_fwd, bool render_caps, std::vector<Vec4d>& coord ) {
  Matrix4f  transform_matrix = transform_matrix_ts;
  H3DFloat r = radius_ts;
  Vec3f pos = position_ts;

  Matrix4f local_to_global = acc_fwd;

  Matrix4f m = local_to_global * transform_matrix;
  Matrix4f m_inv = m.inverse();

  Vec3f light_dir, light_pos;
  Rotation rot;

  if( light_data.isDirectionalLight() ) {
    light_pos = pos;
    light_dir = light_data.getLightDirection();
    Vec3f dir = m_inv.getRotationPart() * light_dir;
    dir.normalizeSafe();
    rot = Rotation( Vec3f( 0, 1, 0 ), dir );
  } else if( light_data.isPointLight() ) {
    light_pos = (m_inv *  light_data.getLightPosition());
    Vec3f dir = pos - light_pos;

    // if lightsource is inside sphere, we skip the shadow volume.
    if( (dir * dir) <= (r * r) ) {
      return;
    }

    H3DFloat d = dir.length();
    if( d > Constants::f_epsilon ) {
      dir.normalizeSafe();
      rot = Rotation( Vec3f( 0, 1, 0 ), dir );
    }

    light_pos = (-rot * light_pos);
  } else {
    return;
  }

  // combine transform
  m = m * Matrix4f( pos, rot );

  buildGeometryData( light_data.isDirectionalLight(), detail_level_ts, r, render_caps, light_dir, light_pos, coord, m, true );
}

void ShadowSphere::addTriangle( unsigned int a, unsigned int b, unsigned int c, vector<int> &indices ) {
  indices.push_back( a );
  indices.push_back( b );
  indices.push_back( c );
}

/// Adds the indices for the triangles of the convex polygon to indices.
void ShadowSphere::addConvexPolygon( const vector< H3DInt32 > polygon_indices, size_t start_i, size_t end_i, vector<int> &indices ) {
  size_t nr_vertices = end_i - start_i;
  if( nr_vertices < 3 ) return;

  for( size_t i = start_i + 1; i < end_i; ++i ) {
    addTriangle( polygon_indices[start_i],
      polygon_indices[i],
      polygon_indices[i + 1],
      indices );
  }
}