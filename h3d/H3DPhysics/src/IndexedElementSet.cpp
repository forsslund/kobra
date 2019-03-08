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
/// \file IndexedElementSet.cpp
/// \brief Source file for IndexedElementSet, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/H3DPhysics/IndexedElementSet.h>
#include <H3D/Normal.h>

using namespace H3D;  

H3DNodeDatabase IndexedElementSet::database( "IndexedElementSet", 
                                            &(newInstance<IndexedElementSet>), 
                                            typeid( IndexedElementSet ),
                                            &X3DComposedGeometryNode::database);

namespace IndexedElementSetInternals {
  FIELDDB_ELEMENT( IndexedElementSet, index, INPUT_OUTPUT )
}

IndexedElementSet::IndexedElementSet (Inst< SFNode           > _metadata,
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
                                      Inst< AutoNormal       > _autoNormal ) :
X3DComposedGeometryNode(  _metadata, _bound, _displayList, 
                        _color, _coord, _normal, _texCoord, 
                        _ccw, _colorPerVertex, _normalPerVertex,
                        _solid, _attrib, _fogCoord ),
                        index ( _index ),
                        autoNormal ( _autoNormal ),
                        coordinate_node ( NULL ),
                        fog_coord_node ( NULL ),
                        color_node ( NULL ),
                        normal_node ( NULL ),
                        faceIndex ( 0 ),
                        vertexNormals ( false ),
                        tex_coords_per_vertex( false ),
                        tex_coord_node( NULL )
{
  // init fields
  type_name = "IndexedElementSet";
  database.initFields( this );

  autoNormal->setName( "autoNormal" );
  autoNormal->setOwner( this );

  index->route( displayList );

  normalPerVertex->route( autoNormal );
  coord->route( autoNormal );
  index->route( autoNormal );
  ccw->route( autoNormal );

  coord->route( bound );

  normalPerVertex->setValue ( false );
}

void IndexedElementSet::render ()
{
  coordinate_node = coord->getValue();
  const vector< int >& indices= index->getValue();

  if( coordinate_node && !indices.empty() ) {
    fog_coord_node = fogCoord->getValue();
    color_node = color->getValue();
    normal_node = normal->getValue();
    tex_coord_node = texCoord->getValue();

    vertexNormals= normalPerVertex->getValue();
    if( !normal_node ) {
      normal_node= autoNormal->getValue();
    }

    bool tex_coord_gen = 
      !tex_coord_node || (tex_coord_node && tex_coord_node->supportsTexGen());
    tex_coords_per_vertex = 
      tex_coord_node && tex_coord_node->supportsExplicitTexCoords();

    // set fog to get fog depth from fog coordinates if available
    if( GLEW_EXT_fog_coord && fog_coord_node ) {
      glPushAttrib( GL_FOG_BIT );
      glFogi(GL_FOG_COORDINATE_SOURCE_EXT, GL_FOG_COORDINATE_EXT);
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

    // no X3DTextureCoordinateNode, so we generate texture coordinates
    // based on the bounding box according to the X3D specification.
    if( tex_coord_gen ) {
      startTexGen( tex_coord_node );
    }

    if( tex_coords_per_vertex &&
      coordinate_node->nrAvailableCoords() > 
      tex_coord_node->nrAvailableTexCoords() ) {
      // check if texture coordinate generator
      if( tex_coord_node->nrAvailableTexCoords() != -1 ) {
        stringstream s;
        s << "Must contain at least as many elements as coord (" 
          << coordinate_node->nrAvailableCoords() << ") in \"" 
          << getName() << "\" node. ";
        throw IndexedTriangleSet::NotEnoughTextureCoordinates(
                tex_coord_node->nrAvailableTexCoords(),
                s.str(), H3D_FULL_LOCATION );
      }
    }

    faceIndex= 0;
    for( unsigned int i = 0; i < indices.size(); i++ ) {

      std::vector<int> elementIndices;
      for(;  i < indices.size() && indices[i] != -1; i++ )
        elementIndices.push_back( indices[i] );

      if( elementIndices.size() == 1 )
      {
        // Render a point
        glBegin( GL_POINTS );
        renderPoint ( elementIndices[0] );
        glEnd();

      } else if ( elementIndices.size() == 4 ) {

        // Render a tetrahedra
        glBegin( GL_TRIANGLES );
        renderTriangle ( elementIndices[0], elementIndices[1], elementIndices[2] );
        renderTriangle ( elementIndices[0], elementIndices[2], elementIndices[3] );
        renderTriangle ( elementIndices[0], elementIndices[3], elementIndices[1] );
        renderTriangle ( elementIndices[1], elementIndices[3], elementIndices[2] );
        glEnd ();

      } else if ( elementIndices.size() == 8 ) {

        // Render a hexahedra.
        glBegin( GL_QUADS );
        renderHexa ( elementIndices[0], elementIndices[1], elementIndices[2], elementIndices[3] );
        renderHexa ( elementIndices[0], elementIndices[4], elementIndices[5], elementIndices[1] );
        renderHexa ( elementIndices[1], elementIndices[5], elementIndices[6], elementIndices[2] );
        renderHexa ( elementIndices[2], elementIndices[6], elementIndices[7], elementIndices[3] );
        renderHexa ( elementIndices[3], elementIndices[7], elementIndices[4], elementIndices[0] );
        renderHexa ( elementIndices[5], elementIndices[4], elementIndices[7], elementIndices[6] );      
        glEnd ();

      } else {

        Console( 4 ) << "Invalid entry in the indices field of IndexedElementSet. " <<
          "The number of indices betwenn two \"-1\" should be either 1, 4, or 8." << endl;
      }

    }

    // restore previous fog attributes
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

void IndexedElementSet::renderPoint ( unsigned int a )
{
  // The point is assumed to have one face in case normals per face is used.
  if ( !vertexNormals ) {
    normal_node->render ( faceIndex++ );
  } else {
    normal_node->render ( a );
  }
  renderVertex ( a );
}

void IndexedElementSet::renderTriangle ( unsigned int a, unsigned int b, unsigned int c )
{
  if ( !vertexNormals ) {
    normal_node->render ( faceIndex++ );
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

void IndexedElementSet::renderHexa ( unsigned int a, unsigned int b, unsigned int c, unsigned int d )
{
  if ( !vertexNormals ) {
    normal_node->render ( faceIndex++ );
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
  if ( vertexNormals ) {
    normal_node->render ( d );
  }
  renderVertex ( d );
}

void IndexedElementSet::renderVertex ( unsigned int _index )
{
  if( color_node ) color_node->render( _index );
  if( tex_coords_per_vertex ) renderTexCoord( _index,
    tex_coord_node );
  if( fog_coord_node) fog_coord_node->render(_index);
  for( unsigned int attrib_index = 0;
    attrib_index < attrib->size(); ++attrib_index ) {
      X3DVertexAttributeNode *attr = 
        attrib->getValueByIndex( attrib_index );
      if( attr ) attr->render( _index );
  }
  coordinate_node->render( _index );
}

void IndexedElementSet::AutoNormal::update() {
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

X3DNormalNode *IndexedElementSet::AutoNormal::generateNormalsPerVertex( 
  X3DCoordinateNode *_coord,
  const vector< int > &_index,
  bool _ccw ) {
    Normal *_normal = new Normal;
    if( _coord ) {
      vector< Vec3f > normals( _coord->nrAvailableCoords(), 
        Vec3f( 0, 0, 0 ) );

      for( unsigned int i = 0; i < _index.size(); ++i ) {

        std::vector<int> elementIndices;

        for(;  i < _index.size() && _index[i] != -1; ++i )
          elementIndices.push_back( _index[i] );

        if( elementIndices.size() == 1 )
        {
          // Use dummy normal for the point.
          Vec3f norm =  Vec3f( 0, 0, 1 );
          normals[elementIndices[0]] += norm;

        } else if ( elementIndices.size() == 4 ) {

          Vec3f norm;

          norm= triangleNormal ( elementIndices[0], elementIndices[1], elementIndices[2], _coord, _ccw );
          normals[elementIndices[0]] += norm;
          normals[elementIndices[1]] += norm;
          normals[elementIndices[2]] += norm;

          norm= triangleNormal ( elementIndices[0], elementIndices[2], elementIndices[3], _coord, _ccw );
          normals[elementIndices[0]] += norm;
          normals[elementIndices[2]] += norm;
          normals[elementIndices[3]] += norm;

          norm= triangleNormal ( elementIndices[0], elementIndices[3], elementIndices[1], _coord, _ccw );
          normals[elementIndices[0]] += norm;
          normals[elementIndices[3]] += norm;
          normals[elementIndices[1]] += norm;

          norm= triangleNormal ( elementIndices[1], elementIndices[3], elementIndices[2], _coord, _ccw );
          normals[elementIndices[1]] += norm;
          normals[elementIndices[3]] += norm;
          normals[elementIndices[2]] += norm;

        } else if ( elementIndices.size() == 8 ) {

          Vec3f norm;

          norm= hexaNormal ( elementIndices[0], elementIndices[1], elementIndices[2], elementIndices[3], _coord, _ccw );
          normals[elementIndices[0]] += norm;
          normals[elementIndices[1]] += norm;
          normals[elementIndices[2]] += norm;
          normals[elementIndices[3]] += norm;

          norm= hexaNormal ( elementIndices[0], elementIndices[4], elementIndices[5], elementIndices[1], _coord, _ccw );
          normals[elementIndices[0]] += norm;
          normals[elementIndices[4]] += norm;
          normals[elementIndices[5]] += norm;
          normals[elementIndices[1]] += norm;

          norm= hexaNormal ( elementIndices[1], elementIndices[5], elementIndices[6], elementIndices[2], _coord, _ccw );
          normals[elementIndices[1]] += norm;
          normals[elementIndices[5]] += norm;
          normals[elementIndices[6]] += norm;
          normals[elementIndices[2]] += norm;

          norm= hexaNormal ( elementIndices[2], elementIndices[6], elementIndices[7], elementIndices[3], _coord, _ccw );
          normals[elementIndices[2]] += norm;
          normals[elementIndices[6]] += norm;
          normals[elementIndices[7]] += norm;
          normals[elementIndices[3]] += norm;

          norm= hexaNormal ( elementIndices[3], elementIndices[7], elementIndices[4], elementIndices[0], _coord, _ccw );
          normals[elementIndices[ 3 ]] += norm;
          normals[elementIndices[ 7 ]] += norm;
          normals[elementIndices[ 4 ]] += norm;
          normals[elementIndices[ 0 ]] += norm;

          norm= hexaNormal ( elementIndices[5], elementIndices[4], elementIndices[7], elementIndices[6], _coord, _ccw );
          normals[elementIndices[ 5 ]] += norm;
          normals[elementIndices[ 4 ]] += norm;
          normals[elementIndices[ 7 ]] += norm;
          normals[elementIndices[ 6 ]] += norm;

        } else {

          Console( 4 ) << "Invalid entry in the indices field of IndexedElementSet. " <<
            "The number of indices between two \"-1\" should be either 1, 4, or 8." << endl;
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

X3DNormalNode *IndexedElementSet::AutoNormal::generateNormalsPerFace( 
  X3DCoordinateNode *_coord,
  const vector< int > &_index,
  bool _ccw ) {
    Normal *_normal = new Normal;
    if( _coord ) {
      vector< Vec3f > normals;

      for( unsigned int i = 0; i < _index.size(); ++i ) {

        std::vector<int> elementIndices;

        for(;  i < _index.size() && _index[i] != -1; ++i )
          elementIndices.push_back( _index[i] );

        if( elementIndices.size() == 1 )
        {
          // Use dummy normal for the point, however assume that
          // it has one face.
          Vec3f norm =  Vec3f( 0, 0, 1 );
          normals.push_back( norm );

        } else if ( elementIndices.size() == 4 ) {

          Vec3f norm;
          // Add a normal for each of the triangle faces in the tetra
          normals.push_back( triangleNormal ( elementIndices[0], elementIndices[1], elementIndices[2], _coord, _ccw ) );
          normals.push_back( triangleNormal ( elementIndices[0], elementIndices[2], elementIndices[3], _coord, _ccw ) );
          normals.push_back( triangleNormal ( elementIndices[0], elementIndices[3], elementIndices[1], _coord, _ccw ) );
          normals.push_back( triangleNormal ( elementIndices[1], elementIndices[3], elementIndices[2], _coord, _ccw ) );

        } else if ( elementIndices.size() == 8 ) {

          Vec3f norm;
          // Add a normal for each of the hexa faces in the hexahedra
          normals.push_back( hexaNormal ( elementIndices[0], elementIndices[1], elementIndices[2], elementIndices[3], _coord, _ccw ) );
          normals.push_back( hexaNormal ( elementIndices[0], elementIndices[4], elementIndices[5], elementIndices[1], _coord, _ccw ) );
          normals.push_back( hexaNormal ( elementIndices[1], elementIndices[5], elementIndices[6], elementIndices[2], _coord, _ccw ) );
          normals.push_back( hexaNormal ( elementIndices[2], elementIndices[6], elementIndices[7], elementIndices[3], _coord, _ccw ) );
          normals.push_back( hexaNormal ( elementIndices[3], elementIndices[7], elementIndices[4], elementIndices[0], _coord, _ccw ) );
          normals.push_back( hexaNormal ( elementIndices[5], elementIndices[4], elementIndices[7], elementIndices[6], _coord, _ccw ) );


        } else {

          Console( 4 ) << "Invalid entry in the indices field of IndexedElementSet. " <<
            "The number of indices between two \"-1\" should be either 1, 4, or 8." << endl;
        }

      } 

      _normal->vector->setValue( normals );
    }
    return _normal;
}

Vec3f IndexedElementSet::AutoNormal::triangleNormal ( int a, int b, int c, X3DCoordinateNode* _coord, bool _ccw ) {
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
// d does not have to be given assuming the hexa is verified.
Vec3f IndexedElementSet::AutoNormal::hexaNormal ( int a, int b, int c, int d, X3DCoordinateNode* _coord, bool _ccw ) {
  Vec3f norm, A, B, C, AB, BC;

  // calculate a normal for the hexa
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

void IndexedElementSet::traverseSG( TraverseInfo &ti ) {
  X3DComposedGeometryNode::traverseSG( ti );
  // use backface culling if solid is true
  if( solid->getValue() ) useBackFaceCulling( true );
  else useBackFaceCulling( false );
}
