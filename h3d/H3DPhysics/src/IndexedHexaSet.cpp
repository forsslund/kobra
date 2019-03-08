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
/// \file IndexedHexaSet.cpp
/// \brief Source file for IndexedHexaSet, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/H3DPhysics/IndexedHexaSet.h>
#include <H3D/Normal.h>

using namespace H3D;  

H3DNodeDatabase IndexedHexaSet::database( "IndexedHexaSet", 
                                         &(newInstance<IndexedHexaSet>), 
                                         typeid( IndexedHexaSet ),
                                         &X3DComposedGeometryNode::database);

namespace IndexedHexaSetInternals {
  FIELDDB_ELEMENT( IndexedHexaSet, index, INPUT_OUTPUT )
}

IndexedHexaSet::IndexedHexaSet (Inst< SFNode           > _metadata,
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
                        verifyHexaSet( new VerifyHexaSet() ),
                        coordinate_node ( NULL ),
                        fog_coord_node ( NULL ),
                        color_node ( NULL ),
                        normal_node ( NULL ),
                        hexaIndex ( 0 ),
                        vertexNormals ( false ),
                        tex_coords_per_vertex( false ),
                        tex_coord_node( NULL )
{
  // init fields
  type_name = "IndexedHexaSet";
  database.initFields( this );

  autoNormal->setName( "autoNormal" );
  autoNormal->setOwner( this );

  index->route( displayList );

  normalPerVertex->route( autoNormal );
  coord->route( autoNormal );
  index->route( autoNormal );
  ccw->route( autoNormal );

  verifyHexaSet->setName( "verifyHexaSet" );
  verifyHexaSet->setOwner( this );
  verifyHexaSet->setValue( false );

  coord->route( bound );

  normalPerVertex->setValue ( false );
}

void IndexedHexaSet::render ()
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

    // How many hexahedrons?
    size_t nrHexa= indices.size() / 8;

    glBegin( GL_QUADS );

    // For each hexa
    hexaIndex= 0;
    for ( unsigned int i= 0; i < nrHexa*8; i+= 8 ) {
      // Render each face
      renderHexa ( indices[i]  , indices[i+1], indices[i+2], indices[i+3] );
      renderHexa ( indices[i]  , indices[i+4], indices[i+5], indices[i+1] );
      renderHexa ( indices[i+1], indices[i+5], indices[i+6], indices[i+2] );
      renderHexa ( indices[i+2], indices[i+6], indices[i+7], indices[i+3] );
      renderHexa ( indices[i+3], indices[i+7], indices[i+4], indices[i]   );
      renderHexa ( indices[i+5], indices[i+4], indices[i+7], indices[i+6] );      
    }

    glEnd ();

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

void IndexedHexaSet::renderHexa ( unsigned int a, unsigned int b, unsigned int c, unsigned int d )
{
  if ( !vertexNormals ) {
    normal_node->render ( hexaIndex++ );
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

void IndexedHexaSet::renderVertex ( unsigned int _index )
{
  if( color_node ) color_node->render( _index );
  if( tex_coords_per_vertex ) renderTexCoord( _index, tex_coord_node );
  if( fog_coord_node) fog_coord_node->render(_index);
  for( unsigned int attrib_index = 0;
    attrib_index < attrib->size(); ++attrib_index ) {
      X3DVertexAttributeNode *attr = 
        attrib->getValueByIndex( attrib_index );
      if( attr ) attr->render( _index );
  }
  coordinate_node->render( _index );
}

void IndexedHexaSet::VerifyHexaSet::update() {
  // If set to true check the IndexedHexaSet.
  if( value == true )
  {
    IndexedHexaSet *hexaSet = 
      static_cast< IndexedHexaSet * >( getOwner() );
    X3DCoordinateNode *_coord = hexaSet->coord->getValue();
    const vector<int> &_index = hexaSet->index->getValue();

    if( _coord ) {
      for( unsigned int i = 0; i < _index.size(); i+=8 ) {
        // Check for valid hexa (i.e. 8 indices)
        if ( i+7 < _index.size() ) {
          if(!verifyHexaFace( _index[i]  , _index[i+1], _index[i+2], _index[i+3], _coord )) break;
          if(!verifyHexaFace( _index[i]  , _index[i+4], _index[i+5], _index[i+1], _coord )) break;
          if(!verifyHexaFace( _index[i+1], _index[i+5], _index[i+6], _index[i+2], _coord )) break;
          if(!verifyHexaFace( _index[i+2], _index[i+6], _index[i+7], _index[i+3], _coord )) break;
          if(!verifyHexaFace( _index[i+3], _index[i+7], _index[i+4], _index[i]  , _coord )) break;
          if(!verifyHexaFace( _index[i+5], _index[i+4], _index[i+7], _index[i+6], _coord )) break;
        }
      }
    }
    value = false;
  }
  H3D::SFBool::update();
}

bool IndexedHexaSet::VerifyHexaSet::verifyHexaFace(int a, int b, int c, int d, X3DCoordinateNode* _coord ) {
  Vec3f norm1, norm2, A, B, C, D, AB, AC, AD;

  // calculate a normal for the hexa
  A = _coord->getCoord( a );
  B = _coord->getCoord( b );
  C = _coord->getCoord( c );
  D = _coord->getCoord( d );

  AB = B - A;
  AC = C - A;
  AD = D - A;

  norm1 = AB % AC;
  norm2 = AC % AD;
  norm1.normalizeSafe();
  norm2.normalizeSafe();
  if( !(norm1 == norm2) )
  {
    Console(3) << "Warning: The IndexedHexaSet includes invalid quads with indices : " <<
      a << " , " << b << " , " << c << " , " << d << endl;

    return false;
  }
  return true;  
}

void IndexedHexaSet::AutoNormal::update() {
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

X3DNormalNode *IndexedHexaSet::AutoNormal::generateNormalsPerVertex( 
  X3DCoordinateNode *_coord,
  const vector< int > &_index,
  bool _ccw ) {
    Normal *_normal = new Normal;
    if( _coord ) {
      vector< Vec3f > normals( _coord->nrAvailableCoords(), 
        Vec3f( 0, 0, 0 ) );
      for( unsigned int i = 0; i < _index.size(); i+=8 ) {

        // Check for valid hexa (i.e. 8 indices)
        if ( i+7 < _index.size() ) {
          Vec3f norm;

          norm= hexaNormal ( _index[i], _index[i+1], _index[i+2], _index[i+3], _coord, _ccw );
          normals[_index[ i ]] += norm;
          normals[_index[ i+1 ]] += norm;
          normals[_index[ i+2 ]] += norm;
          normals[_index[ i+3 ]] += norm;

          norm= hexaNormal ( _index[i], _index[i+4], _index[i+5], _index[i+1], _coord, _ccw );
          normals[_index[ i ]] += norm;
          normals[_index[ i+4 ]] += norm;
          normals[_index[ i+5 ]] += norm;
          normals[_index[ i+1 ]] += norm;

          norm= hexaNormal ( _index[i+1], _index[i+5], _index[i+6], _index[i+2], _coord, _ccw );
          normals[_index[ i+1 ]] += norm;
          normals[_index[ i+5 ]] += norm;
          normals[_index[ i+6 ]] += norm;
          normals[_index[ i+2 ]] += norm;

          norm= hexaNormal ( _index[i+2], _index[i+6], _index[i+7], _index[i+3], _coord, _ccw );
          normals[_index[ i+2 ]] += norm;
          normals[_index[ i+6 ]] += norm;
          normals[_index[ i+7 ]] += norm;
          normals[_index[ i+3 ]] += norm;

          norm= hexaNormal ( _index[i+3], _index[i+7], _index[i+4], _index[i], _coord, _ccw );
          normals[_index[ i+3 ]] += norm;
          normals[_index[ i+7 ]] += norm;
          normals[_index[ i+4 ]] += norm;
          normals[_index[ i ]] += norm;

          norm= hexaNormal ( _index[i+5], _index[i+4], _index[i+7], _index[i+6], _coord, _ccw );
          normals[_index[ i+5 ]] += norm;
          normals[_index[ i+4 ]] += norm;
          normals[_index[ i+7 ]] += norm;
          normals[_index[ i+6 ]] += norm;

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

X3DNormalNode *IndexedHexaSet::AutoNormal::generateNormalsPerFace( 
  X3DCoordinateNode *_coord,
  const vector< int > &_index,
  bool _ccw ) {
    Normal *_normal = new Normal;
    if( _coord ) {
      vector< Vec3f > normals;
      for( size_t i = 0; i < _index.size(); i+=8 ) {
        // make sure we have a valid face. If not use dummy normals. 
        if( i+7 >= _index.size() ) {
          Vec3f norm =  Vec3f( 1, 0, 0 );
          normals.push_back( norm );
          normals.push_back( norm );
          normals.push_back( norm );
          normals.push_back( norm );
          normals.push_back( norm );
          normals.push_back( norm );
        } else {  
          // Add a normal for each of the hexa faces in the hexahedra
          normals.push_back( hexaNormal ( _index[i]  , _index[i+1], _index[i+2], _index[i+3], _coord, _ccw ) );
          normals.push_back( hexaNormal ( _index[i]  , _index[i+4], _index[i+5], _index[i+1], _coord, _ccw ) );
          normals.push_back( hexaNormal ( _index[i+1], _index[i+5], _index[i+6], _index[i+2], _coord, _ccw ) );
          normals.push_back( hexaNormal ( _index[i+2], _index[i+6], _index[i+7], _index[i+3], _coord, _ccw ) );
          normals.push_back( hexaNormal ( _index[i+3], _index[i+7], _index[i+4], _index[i]  , _coord, _ccw ) );
          normals.push_back( hexaNormal ( _index[i+5], _index[i+4], _index[i+7], _index[i+6], _coord, _ccw ) );

        }
      }
      _normal->vector->setValue( normals );
    }
    return _normal;
}
// d does not have to be given assuming the hexa is verified.
Vec3f IndexedHexaSet::AutoNormal::hexaNormal ( int a, int b, int c, int d, X3DCoordinateNode* _coord, bool _ccw ) {
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

void IndexedHexaSet::traverseSG( TraverseInfo &ti ) {
  X3DComposedGeometryNode::traverseSG( ti );
  // use backface culling if solid is true
  if( solid->getValue() ) useBackFaceCulling( true );
  else useBackFaceCulling( false );
}
