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
/// \file DoubleAxisHingeJoint.cpp
/// \brief Source file for DoubleAxisHingeJoint, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////
#include <H3D/H3DPhysics/DoubleAxisHingeJoint.h>

using namespace H3D;

H3DNodeDatabase DoubleAxisHingeJoint::database( "DoubleAxisHingeJoint", 
                                               &(newInstance< DoubleAxisHingeJoint >), 
                                               typeid( DoubleAxisHingeJoint ),
                                               &H3DRigidBodyJointNode::database);

namespace DoubleAxisHingeJointInternals {
  FIELDDB_ELEMENT( DoubleAxisHingeJoint, anchorPoint, INPUT_OUTPUT )
  FIELDDB_ELEMENT( DoubleAxisHingeJoint, axis1, INPUT_OUTPUT )
  FIELDDB_ELEMENT( DoubleAxisHingeJoint, axis2, INPUT_OUTPUT )
  FIELDDB_ELEMENT( DoubleAxisHingeJoint, desiredAngularVelocity1, INPUT_OUTPUT )
  FIELDDB_ELEMENT( DoubleAxisHingeJoint, desiredAngularVelocity2, INPUT_OUTPUT )
  FIELDDB_ELEMENT( DoubleAxisHingeJoint, maxAngle1, INPUT_OUTPUT )
  FIELDDB_ELEMENT( DoubleAxisHingeJoint, maxTorque1, INPUT_OUTPUT )
  FIELDDB_ELEMENT( DoubleAxisHingeJoint, maxTorque2, INPUT_OUTPUT )
  FIELDDB_ELEMENT( DoubleAxisHingeJoint, minAngle1, INPUT_OUTPUT )
  FIELDDB_ELEMENT( DoubleAxisHingeJoint, stopBounce1, INPUT_OUTPUT )
  FIELDDB_ELEMENT( DoubleAxisHingeJoint, stopConstantForceMix1, INPUT_OUTPUT )
  FIELDDB_ELEMENT( DoubleAxisHingeJoint, stopErrorCorrection1, INPUT_OUTPUT )
  FIELDDB_ELEMENT( DoubleAxisHingeJoint, suspensionErrorCorrection, INPUT_OUTPUT )
  FIELDDB_ELEMENT( DoubleAxisHingeJoint, suspensionForce, INPUT_OUTPUT )
  FIELDDB_ELEMENT( DoubleAxisHingeJoint, body1AnchorPoint, OUTPUT_ONLY )
  FIELDDB_ELEMENT( DoubleAxisHingeJoint, body1Axis, OUTPUT_ONLY )
  FIELDDB_ELEMENT( DoubleAxisHingeJoint, body2AnchorPoint, OUTPUT_ONLY )
  FIELDDB_ELEMENT( DoubleAxisHingeJoint, body2Axis, OUTPUT_ONLY )
  FIELDDB_ELEMENT( DoubleAxisHingeJoint, hinge1Angle, OUTPUT_ONLY )
  FIELDDB_ELEMENT( DoubleAxisHingeJoint, hinge1AngleRate, OUTPUT_ONLY )
  FIELDDB_ELEMENT( DoubleAxisHingeJoint, hinge2Angle, OUTPUT_ONLY )
  FIELDDB_ELEMENT( DoubleAxisHingeJoint, hinge2AngleRate, OUTPUT_ONLY )
}

DoubleAxisHingeJoint::DoubleAxisHingeJoint(
  Inst< SFNode  >  _metadata,
  Inst< ValueUpdater > _value_updater,
  Inst< SFRigidBody > _body1,
  Inst< SFRigidBody > _body2, 
  Inst< MFString > _forceOutput,
  Inst< MFEngineOptions > _engineOptions,
  Inst< SFTransformNode   > _transform,
  Inst< SFVec3f > _anchorPoint,
  Inst< SFVec3f > _axis1,
  Inst< SFVec3f > _axis2,
  Inst< SFFloat  > _desiredAngularVelocity1,
  Inst< SFFloat  > _desiredAngularVelocity2,
  Inst< SFFloat  > _maxAngle1,
  Inst< SFFloat  > _maxTorque1,
  Inst< SFFloat  > _maxTorque2,
  Inst< SFFloat  > _minAngle1,
  Inst< SFFloat  > _stopBounce1,
  Inst< SFFloat  > _stopConstantForceMix1,
  Inst< SFFloat  > _stopErrorCorrection1,
  Inst< SFFloat  > _suspensionErrorCorrection,
  Inst< SFFloat  > _suspensionForce,
  Inst< SFVec3f > _body1AnchorPoint,
  Inst< SFVec3f > _body1Axis,
  Inst< SFVec3f > _body2AnchorPoint,
  Inst< SFVec3f > _body2Axis,
  Inst< SFFloat  > _hinge1Angle,
  Inst< SFFloat  > _hinge1AngleRate,
  Inst< SFFloat  > _hinge2Angle,
  Inst< SFFloat  > _hinge2AngleRate ) : 
H3DRigidBodyJointNode ( _metadata, _value_updater, _body1, _body2, _forceOutput, _engineOptions, _transform ),
anchorPoint       ( _anchorPoint ),
axis1             ( _axis1 ),
axis2             ( _axis2 ),
desiredAngularVelocity1( _desiredAngularVelocity1 ),
desiredAngularVelocity2( _desiredAngularVelocity2 ),
maxAngle1         ( _maxAngle1 ),
maxTorque1        ( _maxTorque1 ),
maxTorque2        ( _maxTorque2 ),
minAngle1         ( _minAngle1 ),
stopBounce1       ( _stopBounce1 ),
stopConstantForceMix1 ( _stopConstantForceMix1 ),
stopErrorCorrection1  ( _stopErrorCorrection1 ),
suspensionErrorCorrection( _suspensionErrorCorrection ),
suspensionForce   ( _suspensionForce ),
body1AnchorPoint  ( _body1AnchorPoint ),
body1Axis         ( _body1Axis ),
body2AnchorPoint  ( _body2AnchorPoint ),
body2Axis         ( _body2Axis ),
hinge1Angle       ( _hinge1Angle ),
hinge1AngleRate   ( _hinge1AngleRate ),
hinge2Angle       ( _hinge2Angle ),
hinge2AngleRate   ( _hinge2AngleRate ) {

  type_name = "DoubleAxisHingeJoint";
  database.initFields( this );

  anchorPoint->setValue( Vec3f( 0, 0, 0 ) );
  axis1->setValue( Vec3f( 0, 0, 0 ) );
  axis2->setValue( Vec3f( 0, 0, 0 ) );
  desiredAngularVelocity1->setValue( (H3DFloat)0.0 );
  desiredAngularVelocity2->setValue( (H3DFloat)0.0 );
  maxAngle1->setValue( (H3DFloat) Constants::pi );
  maxTorque1->setValue( (H3DFloat)0.0 );
  maxTorque2->setValue( (H3DFloat)0.0 );
  minAngle1->setValue( (H3DFloat) -Constants::pi );
  stopBounce1->setValue( (H3DFloat)0.0 );
  stopConstantForceMix1->setValue( (H3DFloat)0.001 );
  stopErrorCorrection1->setValue( (H3DFloat)0.8 );
  suspensionErrorCorrection->setValue( (H3DFloat)0.8 );
  suspensionForce->setValue( (H3DFloat)0.0 );

  anchorPoint->route( valueUpdater );
  axis1->route( valueUpdater );
  axis2->route( valueUpdater );
  desiredAngularVelocity1->route( valueUpdater );
  desiredAngularVelocity2->route( valueUpdater );
  maxAngle1->route( valueUpdater );
  minAngle1->route( valueUpdater );
  maxTorque1->route( valueUpdater );
  maxTorque2->route( valueUpdater );
  stopBounce1->route( valueUpdater );
  stopConstantForceMix1->route( valueUpdater );
  stopErrorCorrection1->route( valueUpdater );
  suspensionErrorCorrection->route( valueUpdater );
  suspensionForce->route( valueUpdater );
}

PhysicsEngineParameters::ConstraintParameters* DoubleAxisHingeJoint::createConstraintParameters() {
  return new PhysicsEngineParameters::DoubleAxisHingeJointParameters();
}

PhysicsEngineParameters::ConstraintParameters * DoubleAxisHingeJoint::getConstraintParameters( bool all_params ) {
  DoubleAxisHingeJointParameters* params= 
    static_cast<DoubleAxisHingeJointParameters*>(H3DRigidBodyJointNode::getConstraintParameters(all_params));

  if( all_params || valueUpdater->hasCausedEvent( anchorPoint ) ) {
    params->setAnchorPoint( anchorPoint->getValue() );
  }   
  if( all_params || valueUpdater->hasCausedEvent( axis1 ) ) {
    params->setAxis1( axis1->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( axis2 ) ) {
    params->setAxis2( axis2->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( desiredAngularVelocity1 ) ) {
    params->setDesiredAngularVelocity1( desiredAngularVelocity1->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( desiredAngularVelocity2 ) ) {
    params->setDesiredAngularVelocity2( desiredAngularVelocity2->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( minAngle1 ) ) {
    params->setMinAngle1( minAngle1->getValue() );
  }  
  if( all_params || valueUpdater->hasCausedEvent( maxAngle1 ) ) {
    params->setMaxAngle1( maxAngle1->getValue() );
  }  
  if( all_params || valueUpdater->hasCausedEvent( maxTorque1 ) ) {
    params->setMaxTorque1( maxTorque1->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( maxTorque2 ) ) {
    params->setMaxTorque2( maxTorque2->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( stopBounce1 ) ) {
    params->setStopBounce1( stopBounce1->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( stopConstantForceMix1 ) ) {
    params->setStopConstantForceMix1( stopConstantForceMix1->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( stopErrorCorrection1 ) ) {
    params->setStopErrorCorrection1( stopErrorCorrection1->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( suspensionErrorCorrection ) ) {
    params->setSuspensionErrorCorrection( suspensionErrorCorrection->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( suspensionForce ) ) {
    params->setSuspensionForce( suspensionForce->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( forceOutput ) ) {
    const vector<string> &output = forceOutput->getValue();
    bool has_none = std::find( output.begin(), output.end(), "NONE" ) != output.end();
    if( !has_none ) {
      bool has_all = std::find( output.begin(), output.end(), "ALL" ) != output.end();
      if( has_all ) {
        params->enableHinge1Angle();
        params->enableHinge2Angle();
        params->enableHinge1AngleRate();
        params->enableHinge2AngleRate();
        params->enableBody1AnchorPoint();
        params->enableBody2AnchorPoint();
        params->enableBody1Axis();
        params->enableBody2Axis();
      } else {
        for( vector<string>::const_iterator i = output.begin(); i != output.end(); ++i ) {
          if( (*i) == "NONE" || (*i) == "ALL" ) continue;
          if( (*i) == "hinge1Angle"  ) {
            params->enableHinge1Angle();
          } else if( (*i) == "hinge2angle" ) {
            params->enableHinge2Angle();
          } else if( (*i) == "hinge1AngleRate" ) {
            params->enableHinge1AngleRate();
          } else if( (*i) == "hinge2AngleRate" ) {
            params->enableHinge2AngleRate();
          } else if( (*i) == "body1AnchorPoint" ) {
            params->enableBody1AnchorPoint();
          } else if( (*i) == "body2AnchorPoint" ) {
            params->enableBody2AnchorPoint();
          } else if( (*i) == "body1Axis" ) {
            params->enableBody1Axis();
          } else if( (*i) == "body2Axis" ) {
            params->enableBody2Axis();
          } else {
            Console(4) << "Invalid forceOutput value in DoubleAxisHingeJoint: " << (*i) << endl;
          }
        }
      }
    }

    // Save requested output fields in bit mask for next time
    output_bit_mask= params->getUpdateBitMask();
  }

  // To avoid recalculating the bit mask from the forceOutput field each time,
  // copy the output_bit_mask saved last time forceOutput was updated
  params->copyOutputFlags ( output_bit_mask );

  return params;
}

void DoubleAxisHingeJoint::updateOutputFields() {
  H3DRigidBodyJointNode::updateOutputFields();
  
  PhysicsEngineParameters::DoubleAxisHingeJointParameters params;

  // look at the forceOutput field to determine which fields to update
  unsigned int bitmask = 0;
  const vector<string> &output = forceOutput->getValue();
  bool has_none = std::find( output.begin(), output.end(), "NONE" ) != output.end();

  if( has_none ) return;

  engine_thread->getConstraintParameters( getConstraintId(), params );

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
}

void DoubleAxisHingeJoint::applyTransform ( const Matrix4f& _transform ) { 
  anchorPoint->setValue( _transform*anchorPoint->getValue() );
  axis1->setValue( _transform.getRotationPart()*axis1->getValue() );
  axis2->setValue( _transform.getRotationPart()*axis2->getValue() );
}