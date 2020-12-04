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
/// \file H3DWidgetNode.cpp
/// \brief CPP file for H3DWidgetNode, X3D scene-graph node
///
//
//
//////////////////////////////////////////////////////////////////////////////

// UI includes
#include <H3D/UI/H3DWidgetNode.h>

using namespace H3D;

H3DNodeDatabase H3DWidgetNode::database( 
        "H3DWidgetNode", 
        NULL,
        typeid( H3DWidgetNode ),
        &X3DChildNode::database 
        );

namespace H3DWidgetNodeInternals {
  FIELDDB_ELEMENT( H3DWidgetNode, tag, INPUT_OUTPUT )
  FIELDDB_ELEMENT( H3DWidgetNode, enabled, INPUT_OUTPUT )
  FIELDDB_ELEMENT( H3DWidgetNode, desiredSize, INPUT_OUTPUT )
  FIELDDB_ELEMENT( H3DWidgetNode, actualSize, OUTPUT_ONLY )
  FIELDDB_ELEMENT( H3DWidgetNode, layoutInfo, INPUT_OUTPUT )
}

H3DWidgetNode::H3DWidgetNode( Inst< SFNode      > _metadata   ,
                              Inst< SFBound     > _bound      ,
                              Inst< DisplayList > _displayList,
                              Inst< SFString    > _tag        ,
                              Inst< SFBool      > _enabled    ,
                              Inst< SFVec3f     > _desiredSize,
                              Inst< SFVec3f     > _actualSize,
                              Inst< SFLayoutInfoNode > _layoutInfo ) :
  X3DChildNode( _metadata ),
  X3DBoundedObject( _bound ),
  H3DDisplayListObject( _displayList ),
  tag( _tag ),
  enabled  ( _enabled   ),
  desiredSize( _desiredSize ),
  actualSize( _actualSize ),
  layoutInfo( _layoutInfo ),
  transform( new Transform ),
  parent( NULL ),
  repack( new Field ) {

  type_name = "H3DWidgetNode";
  database.initFields( this );

  displayList->setOwner( this );
  bound->setOwner( this );

  enabled->setValue( true );
  desiredSize->setValue( Vec3f( 1, 1, 0 ) );
  actualSize->setValue( Vec3f( 0, 0, 0 ), id );

  enabled->route( displayList );
  actualSize->route( displayList );
  transform->setName( "transform" );
  transform->transformedBound->route( bound );
  transform->displayList->route( displayList );

  repack->setOwner( this );
  repack->setName( "repack" );

  desiredSize->route( repack );
  actualSize->route( repack );
}


void H3DWidgetNode::traverseSG( TraverseInfo & ) {
  if( !parent ) 
    resize( desiredSize->getValue() );
}

bool H3DWidgetNode::lineIntersect( const Vec3f &from,
                                   const Vec3f &to,
                                   LineIntersectResult &result ) {
  return transform->lineIntersect( from, to, result );
}

void H3DWidgetNode::closestPoint( const Vec3f &p,
                                 NodeIntersectResult &result ) {
  transform->closestPoint( p, result );
}

bool H3DWidgetNode::movingSphereIntersect( H3DFloat radius,
                                           const Vec3f &from, 
                                           const Vec3f &to,
                                           NodeIntersectResult &result ) {
  return transform->movingSphereIntersect( radius, from, to, result );
}

