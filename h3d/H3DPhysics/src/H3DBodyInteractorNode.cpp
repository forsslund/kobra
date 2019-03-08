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
/// \file H3DBodyInteractorNode.cpp
/// \brief Source file for H3DBodyInteractorNode, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/H3DPhysics/H3DBodyInteractorNode.h>

using namespace H3D;  

H3DNodeDatabase H3DBodyInteractorNode::database( "H3DBodyInteractorNode", 
                                                NULL, 
                                                typeid( H3DBodyInteractorNode ),
                                                &X3DNode::database);

namespace H3DBodyInteractorNodeInternals {
  FIELDDB_ELEMENT( H3DBodyInteractorNode, body1, INPUT_OUTPUT )
}

H3DBodyInteractorNode::H3DBodyInteractorNode(  Inst< SFNode > _metadata,
                                             Inst< SFH3DBodyNode > _body1 ) :
X3DNode ( _metadata ),
body1 ( _body1 ),
engine_thread ( NULL ),
output_bit_mask ( 0 )
{
  // init fields
  type_name = "H3DBodyInteractorNode";
  database.initFields( this );  

}

bool H3DBodyInteractorNode::isInitialized() {
  return engine_thread != NULL;
}

// Print warning that specified body is not initialized
void H3DBodyInteractorNode::uninitializedBodyWarning ( H3DBodyNode& body ) {
  Console (4) << "Warning: Body " << body.getName() << " in interactor " <<
    getName() << " is not initialized. All interactors must be added to" <<
    " the interactors(contraints/modifiers) field of a PhysicsBodyCollection."
    << endl; 
}

void H3DBodyInteractorNode::SFH3DBodyNode::onAdd( Node *n ) {
  TypedSFNode< H3DBodyNode >::onAdd( n );
}
