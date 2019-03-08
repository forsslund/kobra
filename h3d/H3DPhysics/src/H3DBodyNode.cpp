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
/// \file H3DBodyNode.cpp
/// \brief Source file for H3DBodyNode, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/H3DPhysics/H3DBodyNode.h>

using namespace H3D;
using namespace PhysicsEngineParameters;

H3DBodyNode::BodyMap H3DBodyNode::body_id_map;

H3DNodeDatabase H3DBodyNode::database( "H3DBodyNode", 
                                      NULL, 
                                      typeid( H3DBodyNode ),
                                      &X3DNode::database);

H3DBodyNode::H3DBodyNode( Inst<SFNode> _metadata ):
X3DNode( _metadata ),
engine_thread( NULL ),
body_id( 0 ) {

  type_name = "H3DBodyNode";
  database.initFields( this );
}

bool H3DBodyNode::initializeBody( PhysicsEngineThread& pt ) {
  if( body_id == 0 ) {
    engine_thread = NULL;
    return false;
  }
  engine_thread= &pt;
  body_id_map[body_id]= this;
  return true;
}

bool H3DBodyNode::deleteBody() {
  if ( isInitialized() ) {
    BodyMap::iterator j = body_id_map.find( body_id );
    if( j != body_id_map.end() ) body_id_map.erase( j );
    body_id= 0;
    engine_thread= NULL;
    return true;
  }
  return false;
}

bool H3DBodyNode::isInitialized() {
  return engine_thread != NULL;
}

H3DBodyId H3DBodyNode::getBodyId() {
  return body_id;
}

H3DBodyNode* H3DBodyNode::getBodyFromId( H3DBodyId id ) {
  BodyMap::iterator j = body_id_map.find( id );
  if( j != body_id_map.end() ) return (*j).second;
  else return NULL;
}
