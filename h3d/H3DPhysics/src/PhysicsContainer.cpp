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
/// \file PhysicsContainer.cpp
/// \brief Source file for PhysicsContainer, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/H3DPhysics/PhysicsContainer.h>

using namespace H3D;

H3DNodeDatabase PhysicsContainer::database( "PhysicsContainer", 
                                            &(newInstance< PhysicsContainer >), 
                                            typeid( PhysicsContainer ),
                                            &X3DChildNode::database);

namespace PhysicsContainerInternals {
  FIELDDB_ELEMENT( PhysicsContainer, bodies, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PhysicsContainer, joints, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PhysicsContainer, collidables, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PhysicsContainer, softBodies, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PhysicsContainer, modifiers, INPUT_OUTPUT )
}

PhysicsContainer::PhysicsContainer (
  Inst< SFNode > _metadata,
  Inst< MFRigidBody > _bodies,
  Inst< MFBodyConstraint > _joints,
  Inst< MFCollidable > _collidables,
  Inst< MFSoftBody > _softBodies,
  Inst< MFBodyModifier > _modifiers ) :
X3DChildNode( _metadata ),
bodies ( _bodies ),
joints ( _joints ),
collidables ( _collidables ),
softBodies ( _softBodies ),
modifiers ( _modifiers )
{
  // init fields
  type_name = "PhysicsContainer";
  database.initFields( this );
}
