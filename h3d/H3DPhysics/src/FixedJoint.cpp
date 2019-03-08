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
/// \file FixedJoint.cpp
/// \brief Source file for FixedJoint, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////
#include <H3D/H3DPhysics/FixedJoint.h>

using namespace H3D;

H3DNodeDatabase FixedJoint::database( "FixedJoint", 
                                    &(newInstance< FixedJoint >), 
                                    typeid( FixedJoint ),
                                    &H3DRigidBodyJointNode::database);

FixedJoint::FixedJoint(Inst< SFNode      > _metadata,
                     Inst< ValueUpdater > _value_updater,
                     Inst< SFRigidBody > _body1,
                     Inst< SFRigidBody > _body2,
                     Inst< MFString    > _forceOutput,
                     Inst< MFEngineOptions > _engineOptions ):
H3DRigidBodyJointNode( _metadata, _value_updater, _body1, _body2,
                      _forceOutput, _engineOptions ) {

  type_name = "FixedJoint";
  database.initFields( this );
}

PhysicsEngineParameters::ConstraintParameters* FixedJoint::createConstraintParameters() {
  return new PhysicsEngineParameters::FixedJointParameters();
}

PhysicsEngineParameters::ConstraintParameters* FixedJoint::getConstraintParameters( bool all_params ) {
  FixedJointParameters* params= 
    static_cast<FixedJointParameters*>(H3DRigidBodyJointNode::getConstraintParameters(all_params));

  return params;
}
