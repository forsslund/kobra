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
/// \file H3DPhysicsFrictionNode.cpp
/// \brief Source file for H3DPhysicsFrictionNode, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////
#include <H3D/H3DPhysics/H3DPhysicsFrictionNode.h>

using namespace H3D;

H3DNodeDatabase H3DPhysicsFrictionNode::database( "H3DPhysicsFrictionNode", 
                                                 NULL, 
                                                 typeid( H3DPhysicsFrictionNode ),
                                                 &H3DPhysicsMaterialPropertyNode::database);

namespace H3DPhysicsFrictionNodeInternals {
  FIELDDB_ELEMENT( H3DPhysicsFrictionNode, friction, INPUT_OUTPUT )
}

H3DPhysicsFrictionNode::H3DPhysicsFrictionNode(
  Inst< SFNode > _metadata,
  Inst< ValueUpdater  > _valueUpdater,
  Inst< SFString > _unitType,
  Inst< SFFriction > _friction ):
H3DPhysicsMaterialPropertyNode( _metadata, _valueUpdater, _unitType ),
friction( _friction ){

  type_name = "H3DPhysicsFrictionNode";
  database.initFields( this );

  friction->setOwner( this );
  friction->setValue( 0.0 );
  friction->route( materialPropertyChanged );

  friction->route( valueUpdater );
}
PhysicsEngineParameters::MaterialPropertyParameters* H3DPhysicsFrictionNode::getMaterialPropertyParameters(bool all_params ) {
  return H3DPhysicsMaterialPropertyNode::getMaterialPropertyParameters( all_params );  
}
