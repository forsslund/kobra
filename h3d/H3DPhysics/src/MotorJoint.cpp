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
/// \file MotorJoint.cpp
/// \brief Source file for MotorJoint, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////
#include <H3D/H3DPhysics/MotorJoint.h>

using namespace H3D;

H3DNodeDatabase MotorJoint::database( "MotorJoint", 
                                     &(newInstance< MotorJoint >), 
                                     typeid( MotorJoint ),
                                     &H3DRigidBodyJointNode::database);

namespace MotorJointInternals {
  FIELDDB_ELEMENT( MotorJoint, axis1Angle          , INPUT_OUTPUT )
  FIELDDB_ELEMENT( MotorJoint, axis1Torque         , INPUT_OUTPUT )
  FIELDDB_ELEMENT( MotorJoint, axis2Angle          , INPUT_OUTPUT )
  FIELDDB_ELEMENT( MotorJoint, axis2Torque         , INPUT_OUTPUT )
  FIELDDB_ELEMENT( MotorJoint, axis3Angle          , INPUT_OUTPUT )
  FIELDDB_ELEMENT( MotorJoint, axis3Torque         , INPUT_OUTPUT )
  FIELDDB_ELEMENT( MotorJoint, enabledAxes         , INPUT_OUTPUT )
  FIELDDB_ELEMENT( MotorJoint, motor1Axis          , INPUT_OUTPUT )
  FIELDDB_ELEMENT( MotorJoint, motor2Axis          , INPUT_OUTPUT )
  FIELDDB_ELEMENT( MotorJoint, motor3Axis          , INPUT_OUTPUT )
  FIELDDB_ELEMENT( MotorJoint, stop1Bounce         , INPUT_OUTPUT )
  FIELDDB_ELEMENT( MotorJoint, stop1ErrorCorrection, INPUT_OUTPUT )
  FIELDDB_ELEMENT( MotorJoint, stop2Bounce         , INPUT_OUTPUT )
  FIELDDB_ELEMENT( MotorJoint, stop2ErrorCorrection, INPUT_OUTPUT )
  FIELDDB_ELEMENT( MotorJoint, stop3Bounce         , INPUT_OUTPUT )
  FIELDDB_ELEMENT( MotorJoint, stop3ErrorCorrection, INPUT_OUTPUT )
  FIELDDB_ELEMENT( MotorJoint, motor1Angle         , OUTPUT_ONLY )
  FIELDDB_ELEMENT( MotorJoint, motor1AngleRate     , OUTPUT_ONLY )
  FIELDDB_ELEMENT( MotorJoint, motor2Angle          , OUTPUT_ONLY )
  FIELDDB_ELEMENT( MotorJoint, motor2AngleRate      , OUTPUT_ONLY )
  FIELDDB_ELEMENT( MotorJoint, motor3Angle          , OUTPUT_ONLY )
  FIELDDB_ELEMENT( MotorJoint, motor3AngleRate      , OUTPUT_ONLY )
  FIELDDB_ELEMENT( MotorJoint, autoCalc             , INITIALIZE_ONLY )
}

MotorJoint::MotorJoint(Inst< SFNode  >  _metadata,
                       Inst< ValueUpdater > _value_updater,
                       Inst< SFRigidBody  >  _body1,
                       Inst< SFRigidBody  >  _body2,
                       Inst< MFString>  _forceOutput,
                       Inst< MFEngineOptions > _engineOptions,
                       Inst< SFTransformNode > _transform,
                       Inst< SFFloat >  _axis1Angle,
                       Inst< SFFloat >  _axis1Torque,
                       Inst< SFFloat >  _axis2Angle,
                       Inst< SFFloat >  _axis2Torque,
                       Inst< SFFloat >  _axis3Angle,
                       Inst< SFFloat >  _axis3Torque,
                       Inst< SFInt32 >  _enabledAxes,
                       Inst< SFVec3f >  _motor1Axis,
                       Inst< SFVec3f >  _motor2Axis,
                       Inst< SFVec3f >  _motor3Axis,
                       Inst< SFFloat >  _stop1Bounce,
                       Inst< SFFloat >  _stop1ErrorCorrection,
                       Inst< SFFloat >  _stop2Bounce,
                       Inst< SFFloat >  _stop2ErrorCorrection,
                       Inst< SFFloat >  _stop3Bounce,
                       Inst< SFFloat >  _stop3ErrorCorrection,
                       Inst< SFFloat >  _motor1Angle,
                       Inst< SFFloat >  _motor1AngleRate,
                       Inst< SFFloat >  _motor2Angle,
                       Inst< SFFloat >  _motor2AngleRate,
                       Inst< SFFloat >  _motor3Angle,
                       Inst< SFFloat >  _motor3AngleRate,
                       Inst< SFBool  >  _autoCalc ) :
H3DRigidBodyJointNode( _metadata, _value_updater, _body1, _body2, _forceOutput, _engineOptions, _transform ),
axis1Angle          ( _axis1Angle  ),
axis1Torque         ( _axis1Torque ),
axis2Angle          ( _axis2Angle  ),
axis2Torque         ( _axis2Torque ),
axis3Angle          ( _axis3Angle  ),
axis3Torque         ( _axis3Torque ),
enabledAxes         ( _enabledAxes ),
motor1Axis          ( _motor1Axis  ),
motor2Axis          ( _motor2Axis  ),
motor3Axis          ( _motor3Axis  ),
stop1Bounce         ( _stop1Bounce ),
stop1ErrorCorrection( _stop1ErrorCorrection ),
stop2Bounce         ( _stop2Bounce ),
stop2ErrorCorrection( _stop2ErrorCorrection ),
stop3Bounce         ( _stop3Bounce ),
stop3ErrorCorrection( _stop3ErrorCorrection ),
motor1Angle         ( _motor1Angle ),
motor1AngleRate     ( _motor1AngleRate ),
motor2Angle         ( _motor2Angle ),
motor2AngleRate     ( _motor2AngleRate ),
motor3Angle         ( _motor3Angle ),
motor3AngleRate     ( _motor3AngleRate ),
autoCalc            ( _autoCalc )
{
  type_name = "MotorJoint";
  database.initFields( this );

  axis1Angle->setValue( (H3DFloat) 0 );
  axis1Torque->setValue( (H3DFloat) 0 );
  axis2Angle->setValue( (H3DFloat) 0 );
  axis2Torque->setValue( (H3DFloat) 0 );
  axis3Angle->setValue( (H3DFloat) 0 );
  axis3Torque->setValue( (H3DFloat) 0 );
  enabledAxes->setValue( (H3DInt32) 1 );
  motor1Axis->setValue( Vec3f(0, 0, 0) );
  motor2Axis->setValue( Vec3f(0, 0, 0) );
  motor3Axis->setValue( Vec3f(0, 0, 0) );
  stop1Bounce->setValue( (H3DFloat) 0 );
  stop1ErrorCorrection->setValue( (H3DFloat) 0 );
  stop2Bounce->setValue( (H3DFloat) 0 );
  stop2ErrorCorrection->setValue( (H3DFloat) 0 );
  stop3Bounce->setValue( (H3DFloat) 0 );
  stop3ErrorCorrection->setValue( (H3DFloat) 0 );
  autoCalc->setValue( false );

  axis1Angle->route( valueUpdater );
  axis1Torque->route( valueUpdater );
  axis2Angle->route( valueUpdater );
  axis2Torque->route( valueUpdater );
  axis3Angle->route( valueUpdater );
  axis3Torque->route( valueUpdater );
  enabledAxes->route( valueUpdater );
  motor1Axis->route( valueUpdater );
  motor2Axis->route( valueUpdater );
  motor3Axis->route( valueUpdater );
  stop1Bounce->route( valueUpdater );
  stop1ErrorCorrection->route( valueUpdater );
  stop2Bounce->route( valueUpdater );
  stop2ErrorCorrection->route( valueUpdater );
  stop3Bounce->route( valueUpdater );
  stop3ErrorCorrection->route( valueUpdater );
  autoCalc->route( valueUpdater );
}

PhysicsEngineParameters::ConstraintParameters* MotorJoint::createConstraintParameters () {
  return new PhysicsEngineParameters::MotorJointParameters();
}

PhysicsEngineParameters::ConstraintParameters * MotorJoint::getConstraintParameters( bool all_params ) {

  PhysicsEngineParameters::MotorJointParameters* params= 
    static_cast<PhysicsEngineParameters::MotorJointParameters*>(H3DRigidBodyJointNode::getConstraintParameters(all_params));

  if( all_params || valueUpdater->hasCausedEvent( axis1Angle ) ) {
    params->setAxis1Angle( axis1Angle->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( axis1Torque ) ) {
    params->setAxis1Torque( axis1Torque->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( axis2Angle ) ) {
    params->setAxis2Angle( axis2Angle->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( axis2Torque ) ) {
    params->setAxis2Torque( axis2Torque->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( axis3Angle ) ) {
    params->setAxis3Angle( axis3Angle->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( axis3Torque ) ) {
    params->setAxis3Torque( axis3Torque->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( enabledAxes ) ) {
    params->setEnabledAxes( enabledAxes->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( motor1Axis ) ) {
    params->setMotor1Axis( motor1Axis->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( motor2Axis ) ) {
    params->setMotor2Axis( motor2Axis->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( motor3Axis ) ) {
    params->setMotor3Axis( motor3Axis->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( stop1Bounce ) ) {
    params->setStop1Bounce( stop1Bounce->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( stop1ErrorCorrection ) ) {
    params->setStop1ErrorCorrection( stop1ErrorCorrection->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( stop2Bounce ) ) {
    params->setStop2Bounce( stop2Bounce->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( stop2ErrorCorrection ) ) {
    params->setStop2ErrorCorrection( stop2ErrorCorrection->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( stop3Bounce ) ) {
    params->setStop3Bounce( stop3Bounce->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( stop3ErrorCorrection ) ) {
    params->setStop3ErrorCorrection( stop3ErrorCorrection->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( autoCalc ) ) {
    params->setAutoCalc( autoCalc->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( forceOutput ) ) {
    const vector<string> &output = forceOutput->getValue();
    bool has_none = std::find( output.begin(), output.end(), "NONE" ) != output.end();
    if( !has_none ) {
      bool has_all = std::find( output.begin(), output.end(), "ALL" ) != output.end();
      if( has_all ) {
        params->enableMotor1Angle();
        params->enableMotor1AngleRate();
        params->enableMotor2Angle();
        params->enableMotor2AngleRate();
        params->enableMotor3Angle();
        params->enableMotor3AngleRate();
      } else {
        for( vector<string>::const_iterator i = output.begin(); i != output.end(); ++i ) {
          if( (*i) == "NONE" || (*i) == "ALL" ) continue;
          if( (*i) == "motor1Angle"  ) {
            params->enableMotor1Angle();
          } else if( (*i) == "motor1AngleRate" ) {
            params->enableMotor1AngleRate();
          } else if( (*i) == "motor2Angle" ) {
            params->enableMotor2Angle();
          } else if( (*i) == "motor2AngleRate" ) {
            params->enableMotor2AngleRate();
          } else if( (*i) == "motor3Angle" ) {
            params->enableMotor3Angle();
          } else if( (*i) == "motor3AngleRate" ) {
            params->enableMotor3AngleRate();
          } else {
            Console(4) << "Invalid forceOutput value in MotorJoint: " << (*i) << endl;
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


void MotorJoint::updateOutputFields() {
  H3DRigidBodyJointNode::updateOutputFields();
  PhysicsEngineParameters::MotorJointParameters params;

  // look at the forceOutput field to determine which fields to update
  const vector<string> &output = forceOutput->getValue();
  bool has_none = std::find( output.begin(), output.end(), "NONE" ) != output.end();
  if( has_none ) return;

  engine_thread->getConstraintParameters( getConstraintId(), params );

  // set the fields with the updated data
  if( params.haveMotor1Angle() && params.getMotor1Angle() != motor1Angle->getValue() ) {
    motor1Angle->setValue( params.getMotor1Angle(), id );
  }
  if( params.haveMotor1AngleRate() && params.getMotor1AngleRate() != motor1AngleRate->getValue() ) {
    motor1AngleRate->setValue( params.getMotor1AngleRate(), id );
  }
  if( params.haveMotor2Angle() && params.getMotor2Angle() != motor2Angle->getValue() ) {
    motor2Angle->setValue( params.getMotor2Angle(), id );
  }
  if( params.haveMotor2AngleRate() && params.getMotor2AngleRate() != motor2AngleRate->getValue() ) {
    motor2AngleRate->setValue( params.getMotor2AngleRate(), id );
  }
  if( params.haveMotor3Angle() && params.getMotor3Angle() != motor3Angle->getValue() ) {
    motor3Angle->setValue( params.getMotor3Angle(), id );
  }
  if( params.haveMotor3AngleRate() && params.getMotor3AngleRate() != motor3AngleRate->getValue() ) {
    motor3AngleRate->setValue( params.getMotor3AngleRate(), id );
  }
}

void MotorJoint::applyTransform ( const Matrix4f& _transform ) { 
  motor1Axis->setValue( _transform.getRotationPart()*motor1Axis->getValue() );
  motor2Axis->setValue( _transform.getRotationPart()*motor2Axis->getValue() );
  motor3Axis->setValue( _transform.getRotationPart()*motor3Axis->getValue() );
}

