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
/// \file H3DPIDNode.cpp
/// \brief Source file for H3DPIDNode
///
//
//////////////////////////////////////////////////////////////////////////////
#include <H3D/H3DPhysics/H3DPIDNode.h>

using namespace H3D;
using namespace PhysicsEngineParameters;

H3DNodeDatabase H3DPIDNode::database( "H3DPIDNode",
                                      NULL,
                                      typeid(H3DPIDNode),
                                      &X3DNode::database );

H3DPIDNode::H3DPIDNode( Inst< SFNode      > _metadata ) :
  X3DNode( _metadata ),
  engine_thread( NULL ) {

  type_name = "H3DPIDNode";
  database.initFields( this );
}

void H3DPIDNode::traverseSG( TraverseInfo &ti ) {
  X3DNode::traverseSG( ti );

  PhysicsEngineThread *pt;
  // obtain the physics thread
  ti.getUserData( "PhysicsEngine", (void * *)&pt );

  if( pt ) {
    if( !engine_thread ) {
      initialize( *pt );
    }
  }
}

void H3DPIDNode::initialize( PhysicsEngineThread& pt ) {
  engine_thread = &pt;
}
