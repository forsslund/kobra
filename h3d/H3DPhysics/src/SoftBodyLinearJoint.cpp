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
/// \file SoftBodyLinearJoint.cpp
/// \brief Source file for SoftBodyLinearJoint, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////


#include <H3D/H3DPhysics/SoftBodyLinearJoint.h>

using namespace H3D;

H3DNodeDatabase SoftBodyLinearJoint::database( "SoftBodyLinearJoint", 
                                              &(newInstance<SoftBodyLinearJoint>), 
                                              typeid( SoftBodyLinearJoint ),
                                              &H3DSoftBodyJointNode::database);

namespace SoftBodyLinearJointInternals {
  FIELDDB_ELEMENT( SoftBodyLinearJoint, anchorPoint, INPUT_OUTPUT )
}

SoftBodyLinearJoint::SoftBodyLinearJoint(
  Inst< SFNode            > _metadata,
  Inst< ValueUpdater      > _value_updater,
  Inst< SFH3DSoftBodyNode > _body1,
  Inst< SFH3DBodyNode     > _body2,
  Inst< MFString          > _forceOutput,
  Inst< MFEngineOptions   > _engineOptions,
  Inst< SFVec3f           > _anchorPoint,
  Inst< SFTransformNode > _transform ):
H3DSoftBodyJointNode ( _metadata, _value_updater, _body1, _body2, _forceOutput, _engineOptions, _transform ),
anchorPoint ( _anchorPoint )
{
  type_name = "SoftBodyLinearJoint";
  database.initFields( this );

  anchorPoint->route ( valueUpdater );  
}

PhysicsEngineParameters::ConstraintParameters* SoftBodyLinearJoint::createConstraintParameters() {
  return new SoftBodyLinearJointParameters();
}

PhysicsEngineParameters::ConstraintParameters* SoftBodyLinearJoint::getConstraintParameters( bool all_params ) {

  SoftBodyLinearJointParameters* params= 
    static_cast<SoftBodyLinearJointParameters*>(H3DSoftBodyJointNode::getConstraintParameters(all_params));

  if ( params && ( all_params || valueUpdater->hasCausedEvent ( anchorPoint ) ) ) {
    params->setAnchorPoint ( anchorPoint->getValue() );
  }

  return params;
}

void SoftBodyLinearJoint::applyTransform ( const Matrix4f& _transform ) { 
  anchorPoint->setValue( _transform*anchorPoint->getValue() );
}
