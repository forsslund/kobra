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
/// \file IndexedTetraSet.cpp
/// \brief Source file for IndexedTetraSet, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/H3DPhysics/IndexedTetraSet.h>
#include <H3D/Normal.h>
#include <H3D/GlobalSettings.h>
#include <H3D/GraphicsOptions.h>

using namespace H3D;

H3DNodeDatabase IndexedTetraSet::database( "IndexedTetraSet", 
                                            &(newInstance<IndexedTetraSet>), 
                                            typeid( IndexedTetraSet ),
                                            &X3DComposedGeometryNode::database);

namespace IndexedTetraSetInternals {
  FIELDDB_ELEMENT( IndexedTetraSet, index, INPUT_OUTPUT )
  FIELDDB_ELEMENT( IndexedTetraSet, renderMode, INPUT_OUTPUT )
}

IndexedTetraSet::IndexedTetraSet (
                                    Inst< SFNode           > _metadata,
                                    Inst< SFBound          > _bound,
                                    Inst< DisplayList      > _displayList,
                                    Inst< SFColorNode      > _color,
                                    Inst< SFCoordinateNode > _coord,
                                    Inst< SFNormalNode     > _normal,
                                    Inst< SFTextureCoordinateNode > _texCoord,
                                    Inst< SFBool           > _ccw,
                                    Inst< SFBool           > _colorPerVertex,
                                    Inst< SFBool           > _normalPerVertex,
                                    Inst< SFBool           > _solid,
                                    Inst< MFVertexAttributeNode > _attrib,
                                    Inst< SFFogCoordinate     > _fogCoord,
                                    Inst< MFInt32          > _index,
                                    Inst< AutoNormal       > _autoNormal,
                                    Inst< SFString         > _renderMode
                                   )
  : X3DComposedGeometryNode(  _metadata, _bound, _displayList, 
                              _color, _coord, _normal, _texCoord, 
                              _ccw, _colorPerVertex, _normalPerVertex,
                              _solid, _attrib, _fogCoord ),
    index ( _index ),
    autoNormal ( _autoNormal ),
    renderMode ( _renderMode ),
    coordinate_node ( NULL ),
    fog_coord_node ( NULL ),
    color_node ( NULL ),
    normal_node ( NULL ),
    triIndex ( 0 ),
    vertexNormals ( false ),
    tex_coords_per_vertex( false ),
    vboFieldsUpToDate( new Field ),
    vbo_id( NULL ),
    autoTangent( new AutoTangent ),
    render_tangents( false ),
    tex_coord_node( NULL )
{
  // init fields
  type_name = "IndexedTetraSet";
  database.initFields( this );

  autoNormal->setName( "autoNormal" );
  autoNormal->setOwner( this );
  autoTangent->setName( "autoTangent" );
  autoTangent->setOwner( this );

  renderMode->addValidValue( "TRIANGLES" );
  renderMode->addValidValue( "LINES_ADJACENCY" );
  renderMode->setValue( "TRIANGLES" );

  index->route( displayList );
  renderMode->route( displayList );

  normalPerVertex->route( autoNormal );
  coord->route( autoNormal );
  index->route( autoNormal );
  ccw->route( autoNormal );

  normalPerVertex->route( autoTangent );
  coord->route( autoTangent );
  index->route( autoTangent );
  texCoord->route( autoTangent );

  coord->route( bound );

  normalPerVertex->setValue ( false );

  vboFieldsUpToDate->setName( "vboFieldsUpToDate" );
  index->route( vboFieldsUpToDate );
}

IndexedTetraSet::~IndexedTetraSet() {
  if( vbo_id )
    delete vbo_id;
}

void IndexedTetraSet::render ()
{
  coordinate_node = coord->getValue();
  tex_coord_node = texCoord->getValue();
  fog_coord_node = fogCoord->getValue();
  color_node = color->getValue();
  normal_node = normal->getValue();

  if( !normal_node ) {
    normal_node= autoNormal->getValue();
  }

  bool tex_coord_gen = 
    !tex_coord_node || (tex_coord_node && tex_coord_node->supportsTexGen());
  tex_coords_per_vertex = 
    tex_coord_node && tex_coord_node->supportsExplicitTexCoords();

  vertexNormals= normalPerVertex->getValue();
  const vector< int >& indices= index->getValue();

  if( coordinate_node && !indices.empty() ) {
    
    // Check that the number of available coords are not 0 since we use
    // "coordinate_node->nrAvailableCoords() - 1" as argument to
    // glDrawRangeElements and we do not want some strange error. Besides
    // if the number of coordinates is 0 then nothing will be rendered so
    // this is a slight optimization.
    if( coordinate_node->nrAvailableCoords() <= 0 )
      return;

    // no X3DTextureCoordinateNode, so we generate texture coordinates
    // based on the bounding box.
    if( tex_coord_gen ) {
      startTexGen( tex_coord_node );
    }

    if( tex_coords_per_vertex &&
      coordinate_node->nrAvailableCoords() > 
      tex_coord_node->nrAvailableTexCoords() ) {
        stringstream s;
        s << "Must contain at least as many elements as coord (" 
          << coordinate_node->nrAvailableCoords() << ") in \"" 
          << getName() << "\" node. ";
        throw IndexedTriangleSet::NotEnoughTextureCoordinates( tex_coord_node->nrAvailableTexCoords(),
          s.str(), H3D_FULL_LOCATION );
    }

    // if we have a color node we use the color from that instead
    // of the previously installed Material node.
    if( color_node ) {
      // Make sure we have enough colors      
      if( coordinate_node->nrAvailableCoords() > 
        color_node->nrAvailableColors() ) {
          stringstream s;
          s << "Must contain at least as many elements as coord (" 
            << coordinate_node->nrAvailableCoords() << ") in \"" 
            << getName() << "\" node. ";
          throw IndexedTriangleSet::NotEnoughColors( color_node->nrAvailableColors(),
            s.str(), H3D_FULL_LOCATION );
      }
      color_node->preRender();
    }

    // set fog to get fog depth from fog coordinates if available
    if( GLEW_EXT_fog_coord && fog_coord_node ) {
      glPushAttrib( GL_FOG_BIT );
      glFogi(GL_FOG_COORDINATE_SOURCE_EXT, GL_FOG_COORDINATE_EXT);  
    }

    GLhandleARB shader_program = 0;
    // Set the attribute index to use for all vertex attributes
    if( GLEW_ARB_shader_objects && GLEW_ARB_vertex_shader ) {
      shader_program = glGetHandleARB( GL_PROGRAM_OBJECT_ARB );
      if( shader_program ) {
        for( unsigned int i = 0; i < attrib->size(); ++i ) {
          X3DVertexAttributeNode *attr = attrib->getValueByIndex( i );
          if( attr ) {
            GLint loc = 
              glGetAttribLocationARB( shader_program, 
              attr->name->getValue().c_str()); 
            attr->setAttribIndex( loc );
          }
        }

        // render tangents as an attribute if needed.
        if( render_tangents ) {
          for( unsigned int i = 0; i < autoTangent->size(); ++i ) {
            X3DVertexAttributeNode *attr = autoTangent->getValueByIndex( i );
            if( attr ) {
              GLint loc = 
                glGetAttribLocationARB( shader_program, 
                                        attr->name->getValue().c_str()); 
              attr->setAttribIndex( loc );
            }
          }
        }
      }
    }

    // How many tetrahedrons?
    size_t nr_tetra= indices.size() / 4;

    string render_mode = renderMode->getValue();
    if( render_mode != "TRIANGLES" && render_mode != "LINES_ADJACENCY" ) {
      Console(4) << "Warning: Invalid renderMode \"" + render_mode + "\" in IndexedTetraSet node. Must be one of \"TRIANGLES\" or \"LINES_ADJACENCY\". Using \"TRIANGLES\" instead" << endl;
      render_mode = "TRIANGLES";
    }

    if( vertexNormals ) {

      bool prefer_vertex_buffer_object = false;
      if( GLEW_ARB_vertex_buffer_object ) {
        GraphicsOptions * go = NULL;
        getOptionNode( go );
        if( !go ) {
          GlobalSettings * gs = GlobalSettings::getActive();
          if( gs ) {
            gs->getOptionNode( go );
          }
        }
        if( go ) {
          prefer_vertex_buffer_object =
            go->preferVertexBufferObject->getValue();
        }
      }
      // if normal per vertex we can use arrays or vertex buffer objects
      // to render the geometry, they all use the same indices.
      if( prefer_vertex_buffer_object ) {
        // Create and bind vertex buffer objects for all the different
        // features.
        coordinate_node->renderVertexBufferObject();
        normal_node->renderVertexBufferObject();
        if( color_node ) color_node->renderVertexBufferObject();
        if( tex_coords_per_vertex )
          renderTexCoordVertexBufferObject( tex_coord_node );
        if( fog_coord_node ) fog_coord_node->renderVertexBufferObject();
        if( render_tangents ) {
          for( unsigned int attrib_index = 0;
               attrib_index < autoTangent->size(); ++attrib_index ) {
            X3DVertexAttributeNode *attr = 
              autoTangent->getValueByIndex( attrib_index );
            if( attr ) attr->renderVertexBufferObject();
          }
        }

        for( unsigned int attrib_index = 0;
             attrib_index < attrib->size(); ++attrib_index ) {
          X3DVertexAttributeNode *attr = 
              attrib->getValueByIndex( attrib_index );
            if( attr ) attr->renderVertexBufferObject();
        }

        if( !vboFieldsUpToDate->isUpToDate() ) {
         
          // Only transfer data when it has been modified.
          vboFieldsUpToDate->upToDate();
          if( !vbo_id ) {
            vbo_id = new GLuint;
            glGenBuffersARB( 1, vbo_id );
          }
          glBindBufferARB( GL_ELEMENT_ARRAY_BUFFER_ARB, *vbo_id );
         
          if( render_mode == "LINES_ADJACENCY" ) {
            glBufferDataARB( GL_ELEMENT_ARRAY_BUFFER_ARB,
                             indices.size() * sizeof(GLuint),
                             &(*(indices.begin()) ), GL_STATIC_DRAW_ARB );
          } else {
            vector< int > triangle_indices;
            getTriangleIndices( triangle_indices, indices );
            glBufferDataARB( GL_ELEMENT_ARRAY_BUFFER_ARB,
                             triangle_indices.size() * sizeof(GLuint),
                             &(*(triangle_indices.begin()) ), GL_STATIC_DRAW_ARB );
          }
        
        } else {
          glBindBufferARB( GL_ELEMENT_ARRAY_BUFFER_ARB, *vbo_id );
        }

        if( render_mode == "LINES_ADJACENCY" ) {
          glDrawRangeElements( GL_LINES_ADJACENCY_EXT,
                               0,
                               coordinate_node->nrAvailableCoords() - 1,
                               (GLsizei)(4 * nr_tetra),
                               GL_UNSIGNED_INT,
                               NULL );
        } else {
          // Draw the triangles, the last parameter is NULL since vertex buffer
          // objects are used.
          glDrawRangeElements( GL_TRIANGLES,
                               0,
                               coordinate_node->nrAvailableCoords() - 1,
                               (GLsizei)(4 * 3 * nr_tetra),
                               GL_UNSIGNED_INT,
                               NULL );
         
        }

        // Disable all vertex buffer objects.
        glBindBufferARB( GL_ELEMENT_ARRAY_BUFFER_ARB, 0 );
        coordinate_node->disableVertexBufferObject();
        normal_node->disableVertexBufferObject();
        if( color_node ) color_node->disableVertexBufferObject();
        if( tex_coords_per_vertex )
          disableTexCoordVertexBufferObject( tex_coord_node );
        if( fog_coord_node) fog_coord_node->disableVertexBufferObject();
          for( unsigned int attrib_index = 0;
          attrib_index < attrib->size(); ++attrib_index ) {
            X3DVertexAttributeNode *attr = 
              attrib->getValueByIndex( attrib_index );
            if( attr ) attr->disableVertexBufferObject();
        } 

        if( render_tangents ) {
          for( unsigned int attrib_index = 0;
               attrib_index < autoTangent->size(); ++attrib_index ) {
            X3DVertexAttributeNode *attr = 
              autoTangent->getValueByIndex( attrib_index );
            if( attr ) attr->disableVertexBufferObject();
          }
        }
      } else {
        // Use vertex arrays.
        coordinate_node->renderArray();
        normal_node->renderArray();
        if( color_node ) color_node->renderArray();
        if( tex_coords_per_vertex ) renderTexCoordArray( tex_coord_node );
        if( fog_coord_node ) fog_coord_node->renderArray();
        if( render_tangents ) {
          for( unsigned int attrib_index = 0;
               attrib_index < autoTangent->size(); ++attrib_index ) {
            X3DVertexAttributeNode *attr = 
              autoTangent->getValueByIndex( attrib_index );
            if( attr ) attr->renderArray();
          }
        }

        for( unsigned int attrib_index = 0;
             attrib_index < attrib->size(); ++attrib_index ) {
          X3DVertexAttributeNode *attr = 
              attrib->getValueByIndex( attrib_index );
            if( attr ) attr->renderArray();
        }

        if( render_mode == "LINES_ADJACENCY" ) {
          glDrawRangeElements( GL_LINES_ADJACENCY_EXT,
                               0,
                               coordinate_node->nrAvailableCoords() - 1,
                               (GLsizei)(4 * nr_tetra),
                               GL_UNSIGNED_INT,
                               &(*(indices.begin() ) ) );
        } else {
          // Draw the triangles, the last parameter is NULL since vertex buffer
          // objects are used.
          vector< int > triangle_indices;
          getTriangleIndices( triangle_indices, indices );
          glDrawRangeElements( GL_TRIANGLES,
                               0,
                               coordinate_node->nrAvailableCoords() - 1,
                               (GLsizei)(4 * 3 * nr_tetra),
                               GL_UNSIGNED_INT,
                               &(*(triangle_indices.begin() ) ) );
         
        }

        coordinate_node->disableArray();
        normal_node->disableArray();
        if( color_node ) color_node->disableArray();
        if( tex_coords_per_vertex ) disableTexCoordArray( tex_coord_node );
        if( fog_coord_node) fog_coord_node->disableArray();
        for( unsigned int attrib_index = 0;
          attrib_index < attrib->size(); ++attrib_index ) {
            X3DVertexAttributeNode *attr = 
              attrib->getValueByIndex( attrib_index );
            if( attr ) attr->disableArray();
        } 

        if( render_tangents ) {
          for( unsigned int attrib_index = 0;
               attrib_index < autoTangent->size(); ++attrib_index ) {
            X3DVertexAttributeNode *attr = 
              autoTangent->getValueByIndex( attrib_index );
            if( attr ) attr->disableArray();
          }
        }
      }
    } else {

      
      if( render_mode == "LINES_ADJACENCY" ) {
        glBegin( GL_LINES_ADJACENCY_EXT );
        for ( unsigned int i= 0; i < nr_tetra*4; ++i ) {

          renderVertex( indices[i] );
        }
        glEnd();

      } else {
        glBegin( GL_TRIANGLES );
        
        // For each tetra
        triIndex= 0;
        for ( unsigned int i= 0; i < nr_tetra*4; i+= 4 ) {
          // Render each face
          renderTriangle ( indices[i], indices[i+1], indices[i+2] );
          renderTriangle ( indices[i], indices[i+2], indices[i+3] );
          renderTriangle ( indices[i], indices[i+3], indices[i+1] );
          renderTriangle ( indices[i+1], indices[i+3], indices[i+2] );
        }
        
        glEnd ();
      }
    }

    if( GLEW_EXT_fog_coord && fog_coord_node ) {
      glPopAttrib();
    }

    // disable texture coordinate generation.
    if( tex_coord_gen ) {
      stopTexGen( tex_coord_node);
    }

    if ( color_node ) {
      color_node->postRender();
    }
  }
}

void IndexedTetraSet::traverseSG( TraverseInfo &ti ) {
  X3DComposedGeometryNode::traverseSG( ti );
  // use backface culling if solid is true
  if( solid->getValue() ) useBackFaceCulling( true );
  else useBackFaceCulling( false );

  // In order to avoid problems with caching when the IndexedTriangleSet
  // is reused in several places in the scene graph where some places
  // require normals and some not, we always render tangents after 
  // one usage requires it. Otherwise it can  be cached as 
  // off if unlucky.
  // 
  bool * shader_requires_tangents = NULL;
  if( !render_tangents && 
      !ti.getUserData( "shaderRequiresTangents", 
           (void **)&shader_requires_tangents) ) {
    render_tangents = true;
    displayList->breakCache();
  }
}

void IndexedTetraSet::renderTriangle ( unsigned int a, unsigned int b, unsigned int c )
{
  if ( !vertexNormals ) {
    normal_node->render ( triIndex++ );
  } else {
    normal_node->render ( a );
  }
  renderVertex ( a );
  if ( vertexNormals ) {
    normal_node->render ( b );
  }
  renderVertex ( b );
  if ( vertexNormals ) {
    normal_node->render ( c );
  }
  renderVertex ( c );
}

void IndexedTetraSet::renderVertex ( unsigned int _index )
{
  if( render_tangents ) {
    for( unsigned int attrib_index = 0;
      attrib_index < autoTangent->size(); ++attrib_index ) {
        X3DVertexAttributeNode *attr = 
          autoTangent->getValueByIndex( attrib_index );
        if( attr ) attr->render( _index );
    }
  }
  if( color_node ) color_node->render( _index );
  if( tex_coords_per_vertex )
    renderTexCoord( _index, tex_coord_node );
  if( fog_coord_node) fog_coord_node->render(_index);
  for( unsigned int attrib_index = 0;
    attrib_index < attrib->size(); ++attrib_index ) {
      X3DVertexAttributeNode *attr = 
        attrib->getValueByIndex( attrib_index );
      if( attr ) attr->render( _index );
  }
  coordinate_node->render( _index );
}

void IndexedTetraSet::AutoNormal::update() {
  bool normals_per_vertex = 
    static_cast< SFBool * >( routes_in[0] )->getValue();
  X3DCoordinateNode *_coord = 
    static_cast< X3DCoordinateNode * >( static_cast< SFCoordinateNode * >
    ( routes_in[1] )->getValue() );
  const vector<int> &_index = 
    static_cast< MFInt32 * >( routes_in[2] )->getValue();
  bool _ccw = static_cast< SFBool * >( routes_in[3] )->getValue();

  if( normals_per_vertex ) 
    value = generateNormalsPerVertex( _coord, _index, _ccw );
  else
    value = generateNormalsPerFace( _coord, _index, _ccw );
}

X3DNormalNode *IndexedTetraSet::AutoNormal::generateNormalsPerVertex( 
  X3DCoordinateNode *_coord,
  const vector< int > &_index,
  bool _ccw ) {
    Normal *_normal = new Normal;
    if( _coord ) {
      vector< Vec3f > normals( _coord->nrAvailableCoords(), 
        Vec3f( 0, 0, 0 ) );
      for( unsigned int i = 0; i < _index.size(); i+=4 ) {
        
        // Check for valid tetra (i.e. 4 indices)
        if ( i+3 < _index.size() ) {
          Vec3f norm;

          norm= triangleNormal ( _index[i], _index[i+1], _index[i+2], _coord, _ccw );
          normals[_index[ i ]] += norm;
          normals[_index[ i+1 ]] += norm;
          normals[_index[ i+2 ]] += norm;

          norm= triangleNormal ( _index[i], _index[i+2], _index[i+3], _coord, _ccw );
          normals[_index[ i ]] += norm;
          normals[_index[ i+2 ]] += norm;
          normals[_index[ i+3 ]] += norm;

          norm= triangleNormal ( _index[i], _index[i+3], _index[i+1], _coord, _ccw );
          normals[_index[ i ]] += norm;
          normals[_index[ i+3 ]] += norm;
          normals[_index[ i+1 ]] += norm;

          norm= triangleNormal ( _index[i+1], _index[i+3], _index[i+2], _coord, _ccw );
          normals[_index[ i+1 ]] += norm;
          normals[_index[ i+3 ]] += norm;
          normals[_index[ i+2 ]] += norm;
        }
      }

      for( vector<Vec3f>::iterator i = normals.begin(); 
        i != normals.end(); 
        ++i ) {
          (*i).normalizeSafe();
      }
      _normal->vector->setValue( normals );
    }
    return _normal;
}

X3DNormalNode *IndexedTetraSet::AutoNormal::generateNormalsPerFace( 
  X3DCoordinateNode *_coord,
  const vector< int > &_index,
  bool _ccw ) {
    Normal *_normal = new Normal;
    if( _coord ) {
      vector< Vec3f > normals;
      for( size_t i = 0; i < _index.size(); i+=4 ) {
        // make sure we have a valid face. If not use dummy normals. 
        if( i+3 >= _index.size() ) {
          Vec3f norm =  Vec3f( 1, 0, 0 );
          normals.push_back( norm );
          normals.push_back( norm );
          normals.push_back( norm );
          normals.push_back( norm );
        } else {
          // Add a normal for each of the triangle faces in the tetra
          normals.push_back( triangleNormal ( _index[i], _index[i+1], _index[i+2], _coord, _ccw ) );
          normals.push_back( triangleNormal ( _index[i], _index[i+2], _index[i+3], _coord, _ccw ) );
          normals.push_back( triangleNormal ( _index[i], _index[i+3], _index[i+1], _coord, _ccw ) );
          normals.push_back( triangleNormal ( _index[i+1], _index[i+3], _index[i+2], _coord, _ccw ) );
        }
      }
      _normal->vector->setValue( normals );
    }
    return _normal;
}

Vec3f IndexedTetraSet::AutoNormal::triangleNormal ( int a, int b, int c, X3DCoordinateNode* _coord, bool _ccw ) {
  Vec3f norm, A, B, C, AB, BC;

  // calculate a normal for the triangle
  A = _coord->getCoord( a );
  B = _coord->getCoord( b );
  C = _coord->getCoord( c );

  AB = B - A;
  BC = C - B;

  norm = AB % BC;

  try {
    norm.normalize();
  } catch ( const ArithmeticTypes::Vec3f::Vec3fNormalizeError & ) {
    norm = Vec3f( 1, 0, 0 );
  }

  if( !_ccw ) 
    norm = -norm;

  return norm;
}

void IndexedTetraSet::getTriangleIndices(
  vector< int > &triangle_indices,
  const vector< int > &tetra_indices ) {
  triangle_indices.reserve( tetra_indices.size() * 3 );
  // Should we consider calculating minimum and maximum index in
  // this loop, since we are copying tetra_indices anyways.
  for( unsigned int i = 0; i < tetra_indices.size(); i+=4 ) {
    if ( i+3 < tetra_indices.size() ) {
      triangle_indices.push_back( tetra_indices[i] );
      triangle_indices.push_back( tetra_indices[i+1] );
      triangle_indices.push_back( tetra_indices[i+2] );

      triangle_indices.push_back( tetra_indices[i] );
      triangle_indices.push_back( tetra_indices[i+2] );
      triangle_indices.push_back( tetra_indices[i+3] );

      triangle_indices.push_back( tetra_indices[i] );
      triangle_indices.push_back( tetra_indices[i+3] );
      triangle_indices.push_back( tetra_indices[i+1] );

      triangle_indices.push_back( tetra_indices[i+1] );
      triangle_indices.push_back( tetra_indices[i+3] );
      triangle_indices.push_back( tetra_indices[i+2] );
    }
  }
}

void IndexedTetraSet::AutoTangent::update() {
  bool normals_per_vertex = 
    static_cast< SFBool * >( routes_in[0] )->getValue();
  X3DCoordinateNode *_coord = 
    static_cast< X3DCoordinateNode * >( static_cast< SFCoordinateNode * >
    ( routes_in[1] )->getValue() );
  const vector<int> &_index = 
    static_cast< MFInt32 * >( routes_in[2] )->getValue();
  X3DTextureCoordinateNode *tex_coord = 
    static_cast< X3DTextureCoordinateNode * >( static_cast< SFTextureCoordinateNode * >
                                               ( routes_in[3] )->getValue() );

  if( value.empty() ) {
    value.push_back( new FloatVertexAttribute );
    value.push_back( new FloatVertexAttribute );
  }

  FloatVertexAttribute *tangent = static_cast< FloatVertexAttribute * >(value[0]);
  FloatVertexAttribute *binormal = static_cast< FloatVertexAttribute * >(value[1]);
  
  if( normals_per_vertex ) 
    generateTangentsPerVertex( _coord, tex_coord, _index, tangent, binormal );
  else
    generateTangentsPerFace( _coord, tex_coord, _index, tangent, binormal );
}

void IndexedTetraSet::AutoTangent::generateTangentsPerVertex( 
                                                           X3DCoordinateNode *_coord,
                                                           X3DTextureCoordinateNode *tex_coord,
                                                           const vector< int > &_index,
                                                           FloatVertexAttribute *tangent_node,
                                                           FloatVertexAttribute *binormal_node) {
  tangent_node->name->setValue( "tangent" );
  tangent_node->numComponents->setValue( 3 );
  tangent_node->value->clear();
  binormal_node->name->setValue( "binormal" );
  binormal_node->numComponents->setValue( 3 );
  binormal_node->value->clear();

  if( _coord ) {
    // the tangent and binormal for a vertex is the average of the tangent
    // of all triangles sharing that vertex.
    vector< H3DFloat > tangents( _coord->nrAvailableCoords() * 3, 0 );
    vector< H3DFloat > binormals( _coord->nrAvailableCoords() * 3, 0 );
    for( unsigned int j = 0; j < _index.size(); j+=4 ) {
      // make sure we have a valid face. If not use a dummy tangent.
      if( j+3 >= _index.size() ) {
        tangents[ _index[j]*3 ] += 0;
        tangents[ _index[j]*3+1 ] += 1;
        tangents[ _index[j]*3+2 ] += 0;

        binormals[ _index[j]*3   ] += 0;
        binormals[ _index[j]*3+1 ] += 0;
        binormals[ _index[j]*3+2 ] += 1;
      } else {
        calculateAndAddTangentPerVertex( _coord, tex_coord, tangents, binormals,
                                         _index[j], _index[j+1], _index[j+2] );
        calculateAndAddTangentPerVertex( _coord, tex_coord, tangents, binormals,
                                         _index[j], _index[j+2], _index[j+3] );
        calculateAndAddTangentPerVertex( _coord, tex_coord, tangents, binormals,
                                         _index[j], _index[j+3], _index[j+1] );
        calculateAndAddTangentPerVertex( _coord, tex_coord, tangents, binormals,
                                         _index[j+1], _index[j+3], _index[j+2] );
      }
    }
    
    for( unsigned int i = 0; i < tangents.size(); i+=3 ) {
      Vec3f t( tangents[i], tangents[i+1], tangents[i+2] );
      Vec3f b( binormals[i], binormals[i+1], binormals[i+2] );
      t.normalizeSafe();
      b.normalizeSafe();
      tangents[i] = t.x;
      tangents[i+1] = t.y;
      tangents[i+2] = t.z;
      binormals[i] = b.x;
      binormals[i+1] = b.y;
      binormals[i+2] = b.z;
    }

    tangent_node->value->setValue( tangents );
    binormal_node->value->setValue( binormals );
  }
}

void IndexedTetraSet::AutoTangent::generateTangentsPerFace( 
                                                              X3DCoordinateNode *_coord,
                                                              X3DTextureCoordinateNode *tex_coord,
                                                              const vector< int > &_index,
                                                              FloatVertexAttribute *tangent_node,
                                                              FloatVertexAttribute *binormal_node ) {

  tangent_node->name->setValue( "tangent" );
  tangent_node->numComponents->setValue( 3 );
  tangent_node->value->clear();
  binormal_node->name->setValue( "binormal" );
  binormal_node->numComponents->setValue( 3 );
  binormal_node->value->clear();

  if( _coord ) {
    vector< float > tangents;
    vector< float > binormals;

    tangents.reserve( 4 * _index.size() / 4 );
    binormals.reserve( 4 * _index.size() / 4 );

    for( size_t j = 0; j < _index.size(); j+=4 ) {
      // make sure we have a valid face. If not use a dummy normal.
      if( j+3 >= _index.size() ) {
        tangents.push_back( 0 );
        tangents.push_back( 1 );
        tangents.push_back( 0 );

        binormals.push_back( 0 );
        binormals.push_back( 0 );
        binormals.push_back( 1 );
      } else {  
        // calculate a tangent
        calculateAndAddTangentPerFace( _coord, tex_coord, tangents, binormals,
                                       _index[j], _index[j+1], _index[j+2] );
        calculateAndAddTangentPerFace( _coord, tex_coord, tangents, binormals,
                                       _index[j], _index[j+2], _index[j+3] );
        calculateAndAddTangentPerFace( _coord, tex_coord, tangents, binormals,
                                       _index[j], _index[j+3], _index[j+1] );
        calculateAndAddTangentPerFace( _coord, tex_coord, tangents, binormals,
                                       _index[j+1], _index[j+3], _index[j+2] );
      }
    }

    tangent_node->value->setValue( tangents );
    binormal_node->value->setValue( binormals );
  }
}



Vec3f IndexedTetraSet::AutoTangent::getTexCoord( X3DCoordinateNode *_coord,
                                                    X3DTextureCoordinateNode *tex_coord,
                                                    int _index ) {
  if( tex_coord ) {
    if( tex_coord->supportsGetTexCoord( 0 ) ) {
      Vec4f tc = tex_coord->getTexCoord( _index, 0 );
      return Vec3f( tc.x, tc.y, tc.z ) / tc.w;
    } else {
      Console(4) << "Warning: X3DTextureCoordinateNode does not support getTexCoord() function. Tangents and binormals cannot be calculated for IndexedTetraSet." << endl;
    }
  } else {
    IndexedTetraSet *its = static_cast< IndexedTetraSet * >( getOwner() );
    Matrix4f to_str = its->getDefaultTexGenMatrix();
    return to_str * _coord->getCoord( _index );
  }

  return Vec3f(0,0,0);
}

void IndexedTetraSet::AutoTangent::calculateTangent( const Vec3f &a, const Vec3f &b, const Vec3f &c,
                                                     const Vec3f &ta, const Vec3f &tb, const Vec3f &tc,
                                                     Vec3f &tangent, Vec3f &binormal ) {
  const Vec3f& v1 = a;
  const Vec3f& v2 = b;
  const Vec3f& v3 = c;
  
  Vec2f w1(ta.x, ta.y );
  Vec2f w2(tb.x, tb.y );
  Vec2f w3(tc.x, tc.y) ;
    
  H3DFloat x1 = v2.x - v1.x;
  H3DFloat x2 = v3.x - v1.x;
  H3DFloat y1 = v2.y - v1.y;
  H3DFloat y2 = v3.y - v1.y;
  H3DFloat z1 = v2.z - v1.z;
  H3DFloat z2 = v3.z - v1.z;
  
  H3DFloat s1 = w2.x - w1.x;
  H3DFloat s2 = w3.x - w1.x;
  H3DFloat t1 = w2.y - w1.y;
  H3DFloat t2 = w3.y - w1.y;
  
  H3DFloat denom = (s1 * t2 - s2 * t1);

  if( denom == 0 ) {
    tangent = Vec3f( 0, 0, 0 );
    binormal = Vec3f( 0, 0, 0 );
    return;
  }

  H3DFloat r = 1.0F / denom;
  Vec3f sdir((t2 * x1 - t1 * x2) * r, 
             (t2 * y1 - t1 * y2) * r,
             (t2 * z1 - t1 * z2) * r);
  Vec3f tdir((s1 * x2 - s2 * x1) * r, 
             (s1 * y2 - s2 * y1) * r,
             (s1 * z2 - s2 * z1) * r);

  tangent = sdir;
  binormal = tdir;
}


void IndexedTetraSet::AutoTangent::calculateAndAddTangentPerVertex(
  X3DCoordinateNode *_coord,
  X3DTextureCoordinateNode *tex_coord,
  vector< H3DFloat > &tangents,
  vector< H3DFloat > &binormals,
  int i, int j, int k ) {
  Vec3f tangent, binormal;
  // calculate a tangent
  Vec3f a = _coord->getCoord( i );
  Vec3f b = _coord->getCoord( j );
  Vec3f c = _coord->getCoord( k );

  Vec3f ta = getTexCoord( _coord, tex_coord, i );
  Vec3f tb = getTexCoord( _coord, tex_coord, j );
  Vec3f tc = getTexCoord( _coord, tex_coord, k );

  calculateTangent( a, b, c,
                    ta, tb, tc,
                    tangent, binormal );

  tangents[ i*3 ] += tangent.x;
  tangents[ i*3+1 ] += tangent.y;
  tangents[ i*3+2 ] += tangent.z;
  tangents[ j*3   ] += tangent.x;
  tangents[j*3+1 ] += tangent.y;
  tangents[j*3+2 ] += tangent.z;
  tangents[k*3   ] += tangent.x;
  tangents[k*3+1 ] += tangent.y;
  tangents[k*3+2 ] += tangent.z;

  binormals[i*3   ] += binormal.x;
  binormals[i*3+1 ] += binormal.y;
  binormals[i*3+2 ] += binormal.z;
  binormals[j*3   ] += binormal.x;
  binormals[j*3+1 ] += binormal.y;
  binormals[j*3+2 ] += binormal.z;
  binormals[k*3   ] += binormal.x;
  binormals[k*3+1 ] += binormal.y;
  binormals[k*3+2 ] += binormal.z;
}

void IndexedTetraSet::AutoTangent::calculateAndAddTangentPerFace(
  X3DCoordinateNode *_coord,
  X3DTextureCoordinateNode *tex_coord,
  vector< H3DFloat > &tangents,
  vector< H3DFloat > &binormals,
  int i, int j, int k ) {
  Vec3f tangent, binormal;
  // calculate a tangent
  Vec3f a = _coord->getCoord( i );
  Vec3f b = _coord->getCoord( j );
  Vec3f c = _coord->getCoord( k );

  Vec3f ta = getTexCoord( _coord, tex_coord, i );
  Vec3f tb = getTexCoord( _coord, tex_coord, j );
  Vec3f tc = getTexCoord( _coord, tex_coord, k );

  calculateTangent( a, b, c,
                    ta, tb, tc,
                    tangent, binormal );

  try {
    binormal.normalize();
  } catch ( const ArithmeticTypes::Vec3f::Vec3fNormalizeError & ) {
    binormal = Vec3f( 0, 1, 0 );
  }

  binormals.push_back( binormal.x );
  binormals.push_back( binormal.y );
  binormals.push_back( binormal.z );

  try {
    tangent.normalize();
  } catch ( const ArithmeticTypes::Vec3f::Vec3fNormalizeError & ) {
    tangent = Vec3f( 0, 0, 1 );
  }
  tangents.push_back( tangent.x );
  tangents.push_back( tangent.y );
  tangents.push_back( tangent.z );

  
}

