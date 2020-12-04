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
/// \file Matrix4VertexAttribute.cpp
/// \brief CPP file for Matrix4VertexAttribute, X3D scene-graph node
///
//
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/Matrix4VertexAttribute.h>
#include "GL/glew.h"

using namespace H3D;

// Add this node to the H3DNodeDatabase system.
H3DNodeDatabase Matrix4VertexAttribute::database( 
                                   "Matrix4VertexAttribute", 
                                   &(newInstance<Matrix4VertexAttribute>), 
                                   typeid( Matrix4VertexAttribute ),
                                   &X3DVertexAttributeNode::database );

namespace Matrix4VertexAttributeInternals {
  FIELDDB_ELEMENT( Matrix4VertexAttribute, value, INPUT_OUTPUT )
  FIELDDB_ELEMENT ( Matrix4VertexAttribute, isDynamic, INPUT_OUTPUT )
  unsigned int num_components_per_element = 4;
}

Matrix4VertexAttribute::Matrix4VertexAttribute( Inst< SFNode   > _metadata,
                                                Inst< SFString > _name,
                                                Inst< MFMatrix4f  > _value ):
  X3DVertexAttributeNode( _metadata, _name ),
  value( _value ),
  attribDataUpToDate( new Field ) {

  value->route(propertyChanged);
  value->route(vboFieldsUpToDate);
  value->route( attribDataUpToDate );

  type_name = "Matrix4VertexAttribute";
  database.initFields( this );
}

Matrix4VertexAttribute::~Matrix4VertexAttribute() {
  if( attrib_data ) {
    GLfloat * attrib_data_float = static_cast<GLfloat *>(attrib_data);
    delete[] attrib_data_float;
    attrib_data = nullptr;
    attrib_size = 0;
  }
}

// Perform the OpenGL commands to set the vertex attribute
// with the given index.
void Matrix4VertexAttribute::render( int value_index ) {
  if( GLEW_ARB_vertex_program && attrib_index >= 0 ) {
    GLfloat v0, v1, v2, v3;
    const Matrix4f &m = value->getValueByIndex( value_index );
    v0 = m[0][0]; v1 = m[1][0]; v2 = m[2][0]; v3 = m[3][0];
    glVertexAttrib4fARB( attrib_index, v0, v1, v2, v3 );
    v0 = m[0][1]; v1 = m[1][1]; v2 = m[2][1]; v3 = m[3][1];
    glVertexAttrib4fARB( attrib_index + 1, v0, v1, v2, v3 );
    v0 = m[0][2]; v1 = m[1][2]; v2 = m[2][2]; v3 = m[3][2];
    glVertexAttrib4fARB( attrib_index + 2, v0, v1, v2, v3 );
    v0 = m[0][3]; v1 = m[1][3]; v2 = m[2][3]; v3 = m[3][3];
    glVertexAttrib4fARB( attrib_index + 3, v0, v1, v2, v3 );
  }
}

// Perform the OpenGL commands to set the vertex attributes
// as a an vertex attribute array.
void Matrix4VertexAttribute::renderArray() {
  using namespace Matrix4VertexAttributeInternals;
  if( GLEW_ARB_vertex_program && attrib_index >= 0 ) {
    setAttributeData();
    unsigned int stride = num_components_per_element * num_components_per_element * sizeof( GLfloat );
    GLfloat * offset = &(static_cast<GLfloat *>(attrib_data)[0]);
    for( unsigned int i = 0; i < num_components_per_element; ++i ) {
      glEnableVertexAttribArrayARB( attrib_index + i );
      glVertexAttribPointerARB( attrib_index + i,
                                num_components_per_element,
                                GL_FLOAT,
                                GL_FALSE,
                                stride,
                                offset );
      offset += num_components_per_element;
    }
  }
}

// Disable the array state enabled in renderArray().
void Matrix4VertexAttribute::disableArray() {
  if( GLEW_ARB_vertex_program && attrib_index >= 0 ) {
    for( unsigned int i = 0; i < Matrix4VertexAttributeInternals::num_components_per_element; ++i ) {
      glDisableVertexAttribArrayARB( attrib_index + i );
    }
  }
}

bool Matrix4VertexAttribute::preRenderCheckFail ( ){
  return GLVertexAttributeObject::preRenderCheckFail ( ) ||
    value->empty ( ) || attrib_index < 0;
}

void Matrix4VertexAttribute::setAttributeData ( ){
  using namespace Matrix4VertexAttributeInternals;
  if( !attribDataUpToDate->isUpToDate() ) {
    attribDataUpToDate->upToDate();
    if( attrib_data ) {
      GLfloat * attrib_data_float = static_cast<GLfloat *>(attrib_data);
      delete[] attrib_data_float;
      attrib_data = nullptr;
      attrib_size = 0;
    }
    unsigned int matrix_size = num_components_per_element * num_components_per_element;
    GLfloat *data = new GLfloat[matrix_size * value->size()];
    std::size_t j = 0;
    for( unsigned int i = 0; i < value->size(); ++i ) {
      const Matrix4f &m = value->getValueByIndex( i );
      for( unsigned int row = 0; row < num_components_per_element; ++row ) {
        for( unsigned int col = 0; col < num_components_per_element; ++col ) {
          data[j++] = m[col][row];
        }
      }
    }
    attrib_data = (GLvoid*)data;
    attrib_size = value->size() * matrix_size * sizeof( GLfloat );
  }
}

void Matrix4VertexAttribute::renderVBO ( ){
  using namespace Matrix4VertexAttributeInternals;
  unsigned int stride = num_components_per_element * num_components_per_element * sizeof( GLfloat );
  if ( use_bindless )
  {
    glEnableClientState( GL_VERTEX_ATTRIB_ARRAY_UNIFIED_NV );
    for( unsigned int i = 0; i < num_components_per_element; ++i ) {
      glEnableVertexAttribArrayARB( attrib_index + i );
      unsigned int offset = i * num_components_per_element * sizeof( GLfloat );
      glVertexAttribFormatNV( attrib_index + i, num_components_per_element, GL_FLOAT, GL_FALSE, stride );
      glBufferAddressRangeNV( GL_VERTEX_ATTRIB_ARRAY_ADDRESS_NV, attrib_index + i, vbo_GPUaddr + offset, attrib_size );
    }
  } else{
    for( unsigned int i = 0; i < num_components_per_element; ++i ) {
      glEnableVertexAttribArrayARB( attrib_index + i );
      size_t offset = i * num_components_per_element * sizeof( GLfloat );
      glVertexAttribPointerARB( attrib_index + i,
                                num_components_per_element,
                                GL_FLOAT,
                                GL_FALSE,
                                stride,
                                (void*)(offset) );
    }
  }
}

void Matrix4VertexAttribute::disableVBO ( ){
  if ( use_bindless )
  {
    glDisableClientState ( GL_VERTEX_ATTRIB_ARRAY_UNIFIED_NV );
  }
  for( unsigned int i = 0; i < Matrix4VertexAttributeInternals::num_components_per_element; ++i ) {
    glDisableVertexAttribArrayARB( attrib_index + i );
  }
}
