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
/// \file IndexedPointSet.cpp
/// \brief Source file for IndexedPointSet, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/H3DPhysics/IndexedPointSet.h>
#include <H3D/X3DTextureNode.h>
#include <H3D/Normal.h>

using namespace H3D;  

H3DNodeDatabase IndexedPointSet::database( "IndexedPointSet", 
                                          &(newInstance<IndexedPointSet>), 
                                          typeid( IndexedPointSet ),
                                          &X3DComposedGeometryNode::database);

namespace IndexedPointSetInternals {
  FIELDDB_ELEMENT( IndexedPointSet, index, INPUT_OUTPUT )
}

IndexedPointSet::IndexedPointSet (Inst< SFNode           > _metadata,
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
                                  Inst< MFInt32          > _index ) :
X3DComposedGeometryNode(  _metadata, _bound, _displayList, 
                        _color, _coord, _normal, _texCoord, 
                        _ccw, _colorPerVertex, _normalPerVertex,
                        _solid, _attrib, _fogCoord ),
                        index ( _index ),
                        coordinate_node ( NULL ),
                        fog_coord_node ( NULL ),
                        color_node ( NULL ),
                        normal_node ( NULL ),
                        tex_coords_per_vertex( false ),
                        tex_coord_node( NULL )
{
  // init fields
  type_name = "IndexedPointSet";
  database.initFields( this );

  index->route( displayList );

  coord->route( bound );

  normalPerVertex->setValue ( true );
}

void IndexedPointSet::render ()
{
  coordinate_node = coord->getValue();

  const vector< int >& indices= index->getValue();

  if( coordinate_node && !indices.empty() ) {
    fog_coord_node = fogCoord->getValue();
    color_node = color->getValue();
    normal_node = normal->getValue();
    tex_coord_node = texCoord->getValue();
    // Save the old state of GL_LIGHTING 
    GLboolean lighting_enabled = GL_FALSE;
    if( !normal_node ) {
      glGetBooleanv( GL_LIGHTING, &lighting_enabled );
      glDisable( GL_LIGHTING );
    }

    // disable texturing
    X3DTextureNode *texture = NULL;
    if( tex_coord_node ) {
      if( coordinate_node->nrAvailableCoords() > 
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
    } else {
      X3DTextureNode::getActiveTexture();
      if( texture ) texture->disableTexturing();
    }

    if( color_node ) {
      // make sure we have enough colors
      if( coordinate_node->nrAvailableCoords() > color_node->nrAvailableColors() ) {
        stringstream s;
        s << "Must be at least as many as coordinates in the coord field ("
          << coordinate_node->nrAvailableCoords() << ") in "
          << getName() << "\" node) ";
        throw IndexedTriangleSet::NotEnoughColors( color_node->nrAvailableColors(), s.str(),
                               H3D_FULL_LOCATION );
      }
    }
    
    // set fog to get fog depth from fog coordinates if available
    if( GLEW_EXT_fog_coord && fog_coord_node ) {
      glPushAttrib( GL_FOG_BIT );
      glFogi(GL_FOG_COORDINATE_SOURCE_EXT, GL_FOG_COORDINATE_EXT);
    }

    glBegin( GL_POINTS );

    // For each point
    for ( unsigned int i= 0; i < indices.size(); ++i ) 
      renderPoint ( indices[i] );

    glEnd ();

    // restore previous fog attributes
    if( GLEW_EXT_fog_coord && fog_coord_node ) {
      glPopAttrib();
    }

    // reenable lighting if it was enabled before
    if( lighting_enabled )
      glEnable( GL_LIGHTING );

    if( texture ) texture->enableTexturing();
  }
}

void IndexedPointSet::renderPoint ( unsigned int a )
{
  if( normal_node ) normal_node->render ( a );
  renderVertex ( a );
}

void IndexedPointSet::renderVertex ( unsigned int _index )
{
  if( color_node ) color_node->render( _index );
  if( tex_coord_node ) renderTexCoord( _index, tex_coord_node );
  if( fog_coord_node) fog_coord_node->render(_index);
  for( unsigned int attrib_index = 0;
    attrib_index < attrib->size(); ++attrib_index ) {
      X3DVertexAttributeNode *attr = 
        attrib->getValueByIndex( attrib_index );
      if( attr ) attr->render( _index );
  }
  coordinate_node->render( _index );
}

