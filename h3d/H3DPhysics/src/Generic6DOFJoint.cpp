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
/// \file Generic6DOFJoint.cpp
/// \brief Source file for Generic6DOFJoint, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////
#include <H3D/H3DPhysics/Generic6DOFJoint.h>

using namespace H3D;

H3DNodeDatabase Generic6DOFJoint::database( "Generic6DOFJoint", 
                                           &(newInstance< Generic6DOFJoint >), 
                                           typeid( Generic6DOFJoint ),
                                           &H3DRigidBodyJointNode::database);

namespace Generic6DOFJointInternals {
  FIELDDB_ELEMENT( Generic6DOFJoint, anchorPoint, INPUT_OUTPUT )
  FIELDDB_ELEMENT( Generic6DOFJoint, axis1, INPUT_OUTPUT )
  FIELDDB_ELEMENT( Generic6DOFJoint, axis2, INPUT_OUTPUT )
  FIELDDB_ELEMENT( Generic6DOFJoint, axis3, INPUT_OUTPUT )
  FIELDDB_ELEMENT( Generic6DOFJoint, desiredAngularVelocity1, INPUT_OUTPUT )
  FIELDDB_ELEMENT( Generic6DOFJoint, desiredAngularVelocity2, INPUT_OUTPUT )
  FIELDDB_ELEMENT( Generic6DOFJoint, desiredAngularVelocity3, INPUT_OUTPUT )
  FIELDDB_ELEMENT( Generic6DOFJoint, maxTorque1, INPUT_OUTPUT )
  FIELDDB_ELEMENT( Generic6DOFJoint, maxTorque2, INPUT_OUTPUT )
  FIELDDB_ELEMENT( Generic6DOFJoint, maxTorque3, INPUT_OUTPUT )
  FIELDDB_ELEMENT( Generic6DOFJoint, maxAngle1, INPUT_OUTPUT )
  FIELDDB_ELEMENT( Generic6DOFJoint, maxAngle2, INPUT_OUTPUT )
  FIELDDB_ELEMENT( Generic6DOFJoint, maxAngle3, INPUT_OUTPUT )
  FIELDDB_ELEMENT( Generic6DOFJoint, minAngle1, INPUT_OUTPUT )
  FIELDDB_ELEMENT( Generic6DOFJoint, minAngle2, INPUT_OUTPUT )
  FIELDDB_ELEMENT( Generic6DOFJoint, minAngle3, INPUT_OUTPUT )
  FIELDDB_ELEMENT( Generic6DOFJoint, desiredLinearVelocity1, INPUT_OUTPUT )
  FIELDDB_ELEMENT( Generic6DOFJoint, desiredLinearVelocity2, INPUT_OUTPUT )
  FIELDDB_ELEMENT( Generic6DOFJoint, desiredLinearVelocity3, INPUT_OUTPUT )
  FIELDDB_ELEMENT( Generic6DOFJoint, minLimit1, INPUT_OUTPUT )
  FIELDDB_ELEMENT( Generic6DOFJoint, minLimit2, INPUT_OUTPUT )
  FIELDDB_ELEMENT( Generic6DOFJoint, minLimit3, INPUT_OUTPUT )
  FIELDDB_ELEMENT( Generic6DOFJoint, maxLimit1, INPUT_OUTPUT )
  FIELDDB_ELEMENT( Generic6DOFJoint, maxLimit2, INPUT_OUTPUT )
  FIELDDB_ELEMENT( Generic6DOFJoint, maxLimit3, INPUT_OUTPUT )
  FIELDDB_ELEMENT( Generic6DOFJoint, maxForce1, INPUT_OUTPUT )
  FIELDDB_ELEMENT( Generic6DOFJoint, maxForce2, INPUT_OUTPUT )
  FIELDDB_ELEMENT( Generic6DOFJoint, maxForce3, INPUT_OUTPUT )
}

Generic6DOFJoint::Generic6DOFJoint(
                                   Inst< SFNode      > _metadata,
                                   Inst< ValueUpdater > _value_updater,
                                   Inst< SFRigidBody > _body1,
                                   Inst< SFRigidBody > _body2,
                                   Inst< MFString    > _forceOutput,
                                   Inst< MFEngineOptions > _engineOptions,
                                   Inst< SFTransformNode > _transform,
                                   Inst< SFVec3f     > _anchorPoint,
                                   Inst< SFVec3f     > _axis1,
                                   Inst< SFVec3f     > _axis2,
                                   Inst< SFVec3f     > _axis3,
                                   Inst< SFFloat     > _desiredAngularVelocity1,
                                   Inst< SFFloat     > _desiredAngularVelocity2,
                                   Inst< SFFloat     > _desiredAngularVelocity3,
                                   Inst< SFFloat     > _minAngle1,
                                   Inst< SFFloat     > _minAngle2,
                                   Inst< SFFloat     > _minAngle3,
                                   Inst< SFFloat     > _maxAngle1,
                                   Inst< SFFloat     > _maxAngle2,
                                   Inst< SFFloat     > _maxAngle3,
                                   Inst< SFFloat     > _maxTorque1,
                                   Inst< SFFloat     > _maxTorque2,
                                   Inst< SFFloat     > _maxTorque3,
                                   Inst< SFFloat     > _desiredLinearVelocity1,
                                   Inst< SFFloat     > _desiredLinearVelocity2,
                                   Inst< SFFloat     > _desiredLinearVelocity3,
                                   Inst< SFFloat     > _minLimit1,
                                   Inst< SFFloat     > _minLimit2,
                                   Inst< SFFloat     > _minLimit3,
                                   Inst< SFFloat     > _maxLimit1,
                                   Inst< SFFloat     > _maxLimit2,
                                   Inst< SFFloat     > _maxLimit3,
                                   Inst< SFFloat     > _maxForce1,
                                   Inst< SFFloat     > _maxForce2,
                                   Inst< SFFloat     > _maxForce3 ) : 
H3DRigidBodyJointNode ( _metadata, _value_updater, _body1, _body2, _forceOutput, _engineOptions, _transform ),
anchorPoint ( _anchorPoint ),
axis1 ( _axis1 ),
axis2 ( _axis2 ),
axis3 ( _axis3 ),
desiredAngularVelocity1 ( _desiredAngularVelocity1 ),
desiredAngularVelocity2 ( _desiredAngularVelocity2 ),
desiredAngularVelocity3 ( _desiredAngularVelocity3 ),
minAngle1 ( _minAngle1 ),
minAngle2 ( _minAngle2 ),
minAngle3 ( _minAngle3 ),
maxAngle1 ( _maxAngle1 ),
maxAngle2 ( _maxAngle2 ),
maxAngle3 ( _maxAngle3 ),
maxTorque1 ( _maxTorque1 ),
maxTorque2 ( _maxTorque2 ),
maxTorque3 ( _maxTorque3 ),
desiredLinearVelocity1 ( _desiredLinearVelocity1 ),
desiredLinearVelocity2 ( _desiredLinearVelocity2 ),
desiredLinearVelocity3 ( _desiredLinearVelocity3 ),
minLimit1 ( _minLimit1 ),
minLimit2 ( _minLimit2 ),
minLimit3 ( _minLimit3 ),
maxLimit1 ( _maxLimit1 ),
maxLimit2 ( _maxLimit2 ),
maxLimit3 ( _maxLimit3 ),
maxForce1 ( _maxForce1 ),
maxForce2 ( _maxForce2 ),
maxForce3 ( _maxForce3 ) {

  type_name = "Generic6DOFJoint";
  database.initFields( this );

  anchorPoint->setValue ( Vec3f(0, 0, 0) );
  axis1->setValue ( Vec3f (1, 0, 0) );
  axis2->setValue ( Vec3f (0, 1, 0) );
  axis3->setValue ( Vec3f (0, 0, 1) );
  desiredAngularVelocity1->setValue ( 0.0f );
  desiredAngularVelocity2->setValue ( 0.0f );
  desiredAngularVelocity3->setValue ( 0.0f );
  maxAngle1->setValue ( 1.0f );
  maxAngle2->setValue ( 1.0f );
  maxAngle3->setValue ( 1.0f );
  minAngle1->setValue ( -1.0f );
  minAngle2->setValue ( -1.0f );
  minAngle3->setValue ( -1.0f );
  maxTorque1->setValue ( 0.0f );
  maxTorque2->setValue ( 0.0f );
  maxTorque3->setValue ( 0.0f );
  desiredLinearVelocity1->setValue ( 0.0f );
  desiredLinearVelocity2->setValue ( 0.0f );
  desiredLinearVelocity3->setValue ( 0.0f );
  minLimit1->setValue ( 0.0f );
  minLimit2->setValue ( 0.0f );
  minLimit3->setValue ( 0.0f );
  maxLimit1->setValue ( 0.0f );
  maxLimit2->setValue ( 0.0f );
  maxLimit3->setValue ( 0.0f );
  maxForce1->setValue ( 0.0f );
  maxForce2->setValue ( 0.0f );
  maxForce3->setValue ( 0.0f );

  anchorPoint->route ( valueUpdater );
  axis1->route ( valueUpdater );
  axis2->route ( valueUpdater );
  axis3->route ( valueUpdater );
  desiredAngularVelocity1->route ( valueUpdater );
  desiredAngularVelocity2->route ( valueUpdater );
  desiredAngularVelocity3->route ( valueUpdater );
  maxAngle1->route ( valueUpdater );
  maxAngle2->route ( valueUpdater );
  maxAngle3->route ( valueUpdater );
  minAngle1->route ( valueUpdater );
  minAngle2->route ( valueUpdater );
  minAngle3->route ( valueUpdater );
  maxTorque1->route ( valueUpdater );
  maxTorque2->route ( valueUpdater );
  maxTorque3->route ( valueUpdater );
  desiredLinearVelocity1->route ( valueUpdater );
  desiredLinearVelocity2->route ( valueUpdater );
  desiredLinearVelocity3->route ( valueUpdater );
  minLimit1->route ( valueUpdater );
  minLimit2->route ( valueUpdater );
  minLimit3->route ( valueUpdater );
  maxLimit1->route ( valueUpdater );
  maxLimit2->route ( valueUpdater );
  maxLimit3->route ( valueUpdater );
  maxForce1->route ( valueUpdater );
  maxForce2->route ( valueUpdater );
  maxForce3->route ( valueUpdater );
}

PhysicsEngineParameters::ConstraintParameters* Generic6DOFJoint::createConstraintParameters () {
  return new PhysicsEngineParameters::Generic6DOFJointParameters();
}

PhysicsEngineParameters::ConstraintParameters * Generic6DOFJoint::getConstraintParameters( bool all_params ) {

  PhysicsEngineParameters::Generic6DOFJointParameters* params= 
    static_cast<PhysicsEngineParameters::Generic6DOFJointParameters*>(H3DRigidBodyJointNode::getConstraintParameters(all_params));

  if( all_params || valueUpdater->hasCausedEvent( anchorPoint ) ) {
    params->setAnchorPoint( anchorPoint->getValue() );
  }

  if( all_params || valueUpdater->hasCausedEvent( axis1 ) ) {
    params->setAxis1( axis1->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( axis2 ) ) {
    params->setAxis2( axis2->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( axis3 ) ) {
    params->setAxis3( axis3->getValue() );
  }

  if( all_params || valueUpdater->hasCausedEvent( desiredAngularVelocity1 ) ) {
    params->setDesiredAngularVelocity1( desiredAngularVelocity1->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( desiredAngularVelocity2 ) ) {
    params->setDesiredAngularVelocity2( desiredAngularVelocity2->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( desiredAngularVelocity3 ) ) {
    params->setDesiredAngularVelocity3( desiredAngularVelocity3->getValue() );
  }

  if( all_params || valueUpdater->hasCausedEvent( minAngle1 ) ) {
    params->setMinAngle1( minAngle1->getValue() );
  }  
  if( all_params || valueUpdater->hasCausedEvent( minAngle2 ) ) {
    params->setMinAngle2( minAngle2->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( minAngle3 ) ) {
    params->setMinAngle3( minAngle3->getValue() );
  }

  if( all_params || valueUpdater->hasCausedEvent( maxAngle1 ) ) {
    params->setMaxAngle1( maxAngle1->getValue() );
  }  
  if( all_params || valueUpdater->hasCausedEvent( maxAngle2 ) ) {
    params->setMaxAngle2( maxAngle2->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( maxAngle3 ) ) {
    params->setMaxAngle3( maxAngle3->getValue() );
  }

  if( all_params || valueUpdater->hasCausedEvent( maxTorque1 ) ) {
    params->setMaxTorque1( maxTorque1->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( maxTorque2 ) ) {
    params->setMaxTorque2( maxTorque2->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( maxTorque3 ) ) {
    params->setMaxTorque3( maxTorque3->getValue() );
  }

  if( all_params || valueUpdater->hasCausedEvent( desiredLinearVelocity1 ) ) {
    params->setDesiredLinearVelocity1( desiredLinearVelocity1->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( desiredLinearVelocity2 ) ) {
    params->setDesiredLinearVelocity2( desiredLinearVelocity2->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( desiredLinearVelocity3 ) ) {
    params->setDesiredLinearVelocity3( desiredLinearVelocity3->getValue() );
  }

  if( all_params || valueUpdater->hasCausedEvent( minLimit1 ) ) {
    params->setMinLimit1( minLimit1->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( minLimit2 ) ) {
    params->setMinLimit2( minLimit2->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( minLimit3 ) ) {
    params->setMinLimit3( minLimit3->getValue() );
  }

  if( all_params || valueUpdater->hasCausedEvent( maxLimit1 ) ) {
    params->setMaxLimit1( maxLimit1->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( maxLimit2 ) ) {
    params->setMaxLimit2( maxLimit2->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( maxLimit3 ) ) {
    params->setMaxLimit3( maxLimit3->getValue() );
  }

  if( all_params || valueUpdater->hasCausedEvent( maxForce1 ) ) {
    params->setMaxForce1( maxForce1->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( maxForce2 ) ) {
    params->setMaxForce2( maxForce2->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( maxForce3 ) ) {
    params->setMaxForce3( maxForce3->getValue() );
  }

  return params;
}

/*void Generic6DOFJoint::updateOutputFields() {
H3DRigidBodyJointNode::updateOutputFields();

PhysicsEngineParameters::DoubleAxisHingeJointParameters params;

// look at the forceOutput field to determine which fields to update
unsigned int bitmask = 0;
const vector<string> &output = forceOutput->getValue();
bool has_none = std::find( output.begin(), output.end(), "NONE" ) != output.end();

if( has_none ) return;

engine_thread->getConstraintParameters( getConstraintId(), &params );

// set the fields with the updated data
if( params.haveHinge1Angle() && params.getHinge1Angle() != hinge1Angle->getValue() ) {
hinge1Angle->setValue( params.getHinge1Angle(), id );
}
if( params.haveHinge2Angle() && params.getHinge2Angle() != hinge2Angle->getValue() ) {
hinge2Angle->setValue( params.getHinge2Angle(), id );
}
if( params.haveHinge1AngleRate() && params.getHinge1AngleRate() != hinge1AngleRate->getValue() ) {
hinge1AngleRate->setValue( params.getHinge1AngleRate(), id );
}
if( params.haveHinge2AngleRate() && params.getHinge2AngleRate() != hinge2AngleRate->getValue() ) {
hinge2AngleRate->setValue( params.getHinge2AngleRate(), id );
}
if( params.haveBody1AnchorPoint() && 
params.getBody1AnchorPoint() != body1AnchorPoint->getValue() ) {
body1AnchorPoint->setValue( params.getBody1AnchorPoint(), id );
}
if( params.haveBody2AnchorPoint() && 
params.getBody2AnchorPoint() != body2AnchorPoint->getValue() ) {
body2AnchorPoint->setValue( params.getBody2AnchorPoint(), id );
}
if( params.haveBody1Axis() && 
params.getBody1Axis() != body1Axis->getValue() ) {
body1Axis->setValue( params.getBody1Axis(), id );
}
if( params.haveBody2Axis() && 
params.getBody2Axis() != body2Axis->getValue() ) {
body2Axis->setValue( params.getBody2Axis(), id );
}
}*/

void Generic6DOFJoint::applyTransform ( const Matrix4f& _transform ) { 
  anchorPoint->setValue( _transform*anchorPoint->getValue() );
  axis1->setValue( _transform.getRotationPart()*axis1->getValue() );
  axis2->setValue( _transform.getRotationPart()*axis2->getValue() );
  axis3->setValue( _transform.getRotationPart()*axis3->getValue() );
}
