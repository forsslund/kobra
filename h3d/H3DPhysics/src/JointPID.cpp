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
/// \file JointPID.cpp
/// \brief Source file for JointPID, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////
#include <H3D/H3DPhysics/JointPID.h>
#include <H3D/H3DPhysics/SingleAxisHingeJoint.h>
#include <H3D/H3DPhysics/DoubleAxisHingeJoint.h>
#include <H3D/H3DPhysics/Generic6DOFJoint.h>
#include <H3D/H3DPhysics/SliderJoint.h>

using namespace H3D;
using namespace PhysicsEngineParameters;

H3DNodeDatabase JointPID::database(
  "JointPID",
  &newInstance<JointPID>,
  typeid(JointPID),
  &H3DPIDNode::database );

namespace JointPIDInternals {
  FIELDDB_ELEMENT( JointPID, linearControl, INPUT_OUTPUT )
  FIELDDB_ELEMENT( JointPID, angularControl1, INPUT_OUTPUT )
  FIELDDB_ELEMENT( JointPID, angularControl2, INPUT_OUTPUT )
  FIELDDB_ELEMENT( JointPID, angularControl3, INPUT_OUTPUT )
  FIELDDB_ELEMENT( JointPID, joint, INPUT_OUTPUT )
  FIELDDB_ELEMENT( JointPID, errorSleepThreshold, INPUT_OUTPUT )
  FIELDDB_ELEMENT( JointPID, useJointMotor, INPUT_OUTPUT )
  FIELDDB_ELEMENT( JointPID, switchForcesToBody2, INPUT_OUTPUT )
  FIELDDB_ELEMENT( JointPID, applyTorqueAsForce, INPUT_OUTPUT )
}

/// Constructor
JointPID::JointPID(
  Inst< SFNode      > _metadata,
  Inst< SFPIDController >  _linearControl,
  Inst< SFPIDController >  _angularControl1,
  Inst< SFPIDController >  _angularControl2,
  Inst< SFJointNode > _joint,
  Inst< SFFloat > _errorSleepThreshold,
  Inst< SFBool > _useJointMotor,
  Inst< SFBool > _switchForcesToBody2,
  Inst< SFBool > _applyTorqueAsForce,
  Inst< SFPIDController >  _angularControl3 ) :
  H3DPIDNode( _metadata ),
  linearControl( _linearControl ),
  angularControl1( _angularControl1 ),
  angularControl2( _angularControl2 ),
  angularControl3( _angularControl3 ),
  joint( _joint ),
  errorSleepThreshold( _errorSleepThreshold ),
  useJointMotor( _useJointMotor ),
  rt_use_joint_motor( false ),
  body_id1( 0 ),
  body_id2( 0 ),
  joint_id( 0 ),
  linear_PID( NULL ),
  angular_PID1( NULL ),
  angular_PID2( NULL ),
  angular_PID3( NULL ),
  joint_type( JointType::Unsupported ),
  fixed( false ),
  switchForcesToBody2( _switchForcesToBody2 ),
  applyTorqueAsForce( _applyTorqueAsForce ),
  rt_switch_force_to_body2( false ),
  rt_apply_torque_as_force( false ),
  rt_error_sleep_threshold( 0 ) {

  type_name = "JointPID";
  database.initFields( this );

  errorSleepThreshold->setValue( rt_error_sleep_threshold );
  useJointMotor->setValue( false );
  applyTorqueAsForce->setValue( rt_apply_torque_as_force );
  switchForcesToBody2->setValue( rt_switch_force_to_body2 );
}

void JointPID::traverseSG( TraverseInfo &ti ) {
  H3DPIDNode::traverseSG( ti );

  PhysicsEngineThread *pt = NULL;
  // obtain the physics thread
  ti.getUserData( "PhysicsEngine", (void * *)&pt );

  if( pt && pt == engine_thread ) {
    if( linear_PID ) {
      linear_PID->traverseSG( ti );
    }
    if( angular_PID1 ) {
      angular_PID1->traverseSG( ti );
    }
    if( angular_PID2 ) {
      angular_PID2->traverseSG( ti );
    }
    if( angular_PID3 ) {
      angular_PID3->traverseSG( ti );
    }
  }

  rt_error_sleep_threshold = errorSleepThreshold->getValue();
  rt_use_joint_motor = useJointMotor->getValue();
  rt_switch_force_to_body2 = switchForcesToBody2->getValue();
  rt_apply_torque_as_force = applyTorqueAsForce->getValue();
}

void JointPID::updateActuation() {
  if( engine_thread && joint_type != JointType::Unsupported ) {
    // Get current body and joint state
    RigidBodyParameters* bodyParams1 = new RigidBodyParameters;



    engine_thread->getRigidBodyParameters( body_id1, *bodyParams1 );

    // update error... HERE

    if( !bodyParams1->getFixed() &&
        (bodyParams1->getEnabled() ||
        (linear_PID && H3DAbs( linear_PID->getCurrentError() ) > rt_error_sleep_threshold) ||
        (angular_PID1 && H3DAbs( angular_PID1->getCurrentError() ) > rt_error_sleep_threshold) ||
        (angular_PID2 && H3DAbs( angular_PID2->getCurrentError() ) > rt_error_sleep_threshold) ||
        (angular_PID3 && H3DAbs( angular_PID3->getCurrentError() ) > rt_error_sleep_threshold)) ) {

      // Get the params for body and joint from physics engine
      RigidBodyParameters* bodyParams2 = new RigidBodyParameters;
      JointParameters* jointParams = createJointParameters();
      engine_thread->getConstraintParameters( joint_id, *jointParams );
      if( body_id2 ) {
        engine_thread->getRigidBodyParameters( body_id2, *bodyParams2 );
      }

      // Accumulate control force and torque
      Vec3f force, torque;
      bool use_joint_motor = rt_use_joint_motor;
      bool switch_forces_to_body2 = rt_switch_force_to_body2;
      bool apply_torque_as_force = rt_apply_torque_as_force;

      // Linear PIDs
      if( linear_PID ) {
        force += doPIDControl( *linear_PID, *bodyParams1, *bodyParams2, *jointParams, ControlType::Linear, AxisType::Axis1, use_joint_motor, !switch_forces_to_body2 );
      }

      // Angular PIDs
      if( angular_PID1 ) {
        if( apply_torque_as_force ) {
          force += doPIDControl( *angular_PID1, *bodyParams1, *bodyParams2, *jointParams, ControlType::Angular, AxisType::Axis1, use_joint_motor, !switch_forces_to_body2, true );
        } else {
          torque += doPIDControl( *angular_PID1, *bodyParams1, *bodyParams2, *jointParams, ControlType::Angular, AxisType::Axis1, use_joint_motor, !switch_forces_to_body2 );
        }

      }
      if( angular_PID2 ) {
        if( apply_torque_as_force ) {
          force += doPIDControl( *angular_PID2, *bodyParams1, *bodyParams2, *jointParams, ControlType::Angular, AxisType::Axis2, use_joint_motor, !switch_forces_to_body2, true );
        } else {
          torque += doPIDControl( *angular_PID2, *bodyParams1, *bodyParams2, *jointParams, ControlType::Angular, AxisType::Axis2, use_joint_motor, !switch_forces_to_body2 );
        }
      }
      if( angular_PID3 ) {
        if( apply_torque_as_force ) {
          force += doPIDControl( *angular_PID3, *bodyParams1, *bodyParams2, *jointParams, ControlType::Angular, AxisType::Axis3, use_joint_motor, !switch_forces_to_body2, true );
        } else {
          torque += doPIDControl( *angular_PID3, *bodyParams1, *bodyParams2, *jointParams, ControlType::Angular, AxisType::Axis3, use_joint_motor, !switch_forces_to_body2 );
        }
      }

      if( !use_joint_motor ) {
        if( !fixed && switch_forces_to_body2 ) {
          // body2
          bodyParams2->setForce( bodyParams2->getForce() - force );
          bodyParams2->setTorque( bodyParams2->getTorque() - torque );
          engine_thread->setRigidBodyParameters( body_id2, *bodyParams2 );
          delete bodyParams1;
        } else {
          // body1
          bodyParams1->setForce( bodyParams1->getForce() + force );
          bodyParams1->setTorque( bodyParams1->getTorque() + torque );
          engine_thread->setRigidBodyParameters( body_id1, *bodyParams1 );
          delete bodyParams2;
        }
        delete jointParams;
      } else {
        jointParams->setMotorTarget( force.x + torque.x );
        engine_thread->setConstraintParameters( joint_id, *jointParams );
        delete bodyParams1;
        delete bodyParams2;
      }

    } else {
      // reset PIDControllers
      if( linear_PID ) {
        linear_PID->resetPID();
      }
      if( angular_PID1 ) {
        angular_PID1->resetPID();
      }
      if( angular_PID2 ) {
        angular_PID2->resetPID();
      }
      if( angular_PID3 ) {
        angular_PID3->resetPID();
      }
      delete bodyParams1;
    }
  }
}

void JointPID::initialize( PhysicsEngineThread& pt ) {
  if( !engine_thread ) {
    H3DRigidBodyJointNode* j = joint->getValue();
    if( j ) {
      RigidBody* b = dynamic_cast<RigidBody*>(j->body1->getValue());
      if( b ) {
        if( j->isInitialized() && b->isInitialized() ) {
          joint_id = j->getConstraintId();
          body_id1 = b->getBodyId();
          initial_orientation1 = b->orientation->getValue();
          RigidBody* b2 = dynamic_cast<RigidBody*>(j->body2->getValue());
          if( b2 && b2->isInitialized() ) {
            body_id2 = b2->getBodyId();
            initial_orientation2 = b2->orientation->getValue();
          }

          linear_PID = linearControl->getValue();
          angular_PID1 = angularControl1->getValue();
          angular_PID2 = angularControl2->getValue();
          angular_PID3 = angularControl3->getValue();

          fixed = !j->body2->getValue();

          // Get joint type
          if( dynamic_cast<SingleAxisHingeJoint*>(j) ) {
            joint_type = JointType::SingleAxisHinge;
          } else if( dynamic_cast<DoubleAxisHingeJoint*>(j) ) {
            joint_type = JointType::DoubleAxisHinge;
          } else if( dynamic_cast<SliderJoint*>(j) ) {
            joint_type = JointType::Slider;
          } else if( dynamic_cast<Generic6DOFJoint*>(j) ) {
            joint_type = JointType::Generic6DOF;
          } else {
            joint_type = JointType::Unsupported;
            Console(4) << "Warning: Joint type " << j->getTypeName() << " is not supported by JointPID!" << endl;
          }

          engine_thread = &pt;
        }
      }
    }
  }
}

JointParameters* JointPID::createJointParameters() {
  switch( joint_type ) {
    case JointType::SingleAxisHinge:
      return new SingleAxisHingeJointParameters;
    case JointType::DoubleAxisHinge:
      return new DoubleAxisHingeJointParameters;
    case JointType::Slider:
      return new SliderJointParameters;
    case JointType::Generic6DOF:
      return new Generic6DOFJointParameters;
    default:;
  }

  return NULL;
}

H3DFloat JointPID::getValue( JointParameters& _joint, ControlType::e control_type, AxisType::e axis_type ) {
  H3DFloat value = 0.0f;

  switch( joint_type ) {
    case JointType::SingleAxisHinge: {
      SingleAxisHingeJointParameters* params = static_cast<SingleAxisHingeJointParameters*>(&_joint);
      value = (control_type == ControlType::Angular) ? params->getAngle() : 0;
      break;
    }
    case JointType::DoubleAxisHinge: {
      DoubleAxisHingeJointParameters* params = static_cast<DoubleAxisHingeJointParameters*>(&_joint);
      switch( axis_type ) {
        case AxisType::Axis1:
          value = (control_type == ControlType::Angular) ? params->getHinge1Angle() : 0;
          break;
        case AxisType::Axis2:
          value = (control_type == ControlType::Angular) ? params->getHinge2Angle() : 0;
          break;
      }
      break;
    }
    case JointType::Slider: {
      SliderJointParameters* params = static_cast<SliderJointParameters*>(&_joint);
      value = (control_type == ControlType::Linear) ? params->getSeparation() : 0;
      break;
    }
    case JointType::Generic6DOF: {
      Generic6DOFJointParameters* params = static_cast<Generic6DOFJointParameters*>(&_joint);
      switch( axis_type ) {
      case AxisType::Axis1:
        value = (control_type == ControlType::Angular) ? params->getHinge1Angle() : 0;
        break;
      case AxisType::Axis2:
        value = (control_type == ControlType::Angular) ? params->getHinge2Angle() : 0;
        break;
      case AxisType::Axis3:
        value = (control_type == ControlType::Angular) ? params->getHinge3Angle() : 0;
        break;
      }
      break;
    }
    default: {
      Console( LogLevel::Warning ) << "This joint type is currently unsupported" << endl;
    }
  }

  return value;
}

H3DFloat JointPID::getVelocity( JointParameters& _joint, ControlType::e control_type, AxisType::e axis_type ) {
  H3DFloat velocity = 0.0f;

  switch( joint_type ) {
    case JointType::SingleAxisHinge: {
      SingleAxisHingeJointParameters* params = static_cast<SingleAxisHingeJointParameters*>(&_joint);
      velocity = (control_type == ControlType::Angular) ? params->getAngleRate() : 0;
      break;
    }
    case JointType::DoubleAxisHinge: {
      DoubleAxisHingeJointParameters* params = static_cast<DoubleAxisHingeJointParameters*>(&_joint);
      switch( axis_type ) {
        case AxisType::Axis1:
          velocity = (control_type == ControlType::Angular) ? params->getHinge1AngleRate() : 0;
          break;
        case AxisType::Axis2:
          velocity = (control_type == ControlType::Angular) ? params->getHinge2AngleRate() : 0;
          break;
      }
      break;
    }
    case JointType::Slider: {
      SliderJointParameters* params = static_cast<SliderJointParameters*>(&_joint);
      velocity = (control_type == ControlType::Linear) ? params->getSeparationRate() : 0;
      break;
    }
    case JointType::Generic6DOF: {
      Generic6DOFJointParameters* params = static_cast<Generic6DOFJointParameters*>(&_joint);
      switch( axis_type ) {
      case AxisType::Axis1:
        velocity = (control_type == ControlType::Angular) ? params->getHinge1AngleRate() : 0;
        break;
      case AxisType::Axis2:
        velocity = (control_type == ControlType::Angular) ? params->getHinge2AngleRate() : 0;
        break;
      case AxisType::Axis3:
        velocity = (control_type == ControlType::Angular) ? params->getHinge3AngleRate() : 0;
        break;
      }
      break;
    }
    default: {
      Console( LogLevel::Warning ) << "This joint type is currently unsupported" << endl;
    }
  }

  return velocity;
}

H3DFloat JointPID::getMinTarget( JointParameters& _joint, ControlType::e control_type, AxisType::e axis_type ) {
  H3DFloat target = 0.0f;

  switch( joint_type ) {
    case JointType::SingleAxisHinge: {
      SingleAxisHingeJointParameters* params = static_cast<SingleAxisHingeJointParameters*>(&_joint);
      target = (control_type == ControlType::Angular) ? params->getMinAngle() : 1;
      break;
    }
    case JointType::DoubleAxisHinge: {
      DoubleAxisHingeJointParameters* params = static_cast<DoubleAxisHingeJointParameters*>(&_joint);
      switch( axis_type ) {
        case AxisType::Axis1:
          target = (control_type == ControlType::Angular) ? params->getMinAngle1() : 1;
          break;
        case AxisType::Axis2:
          target = 1;
          break;
      }
      break;
    }
    case JointType::Slider: {
      SliderJointParameters* params = static_cast<SliderJointParameters*>(&_joint);
      target = (control_type == ControlType::Linear) ? params->getMinSeparation() : 1;
      break;
    }
    case JointType::Generic6DOF: {
      Generic6DOFJointParameters* params = static_cast<Generic6DOFJointParameters*>(&_joint);
      switch( axis_type ) {
      case AxisType::Axis1:
        target = (control_type == ControlType::Angular) ? params->getMinAngle1() : 1;
        break;
      case AxisType::Axis2:
        target = (control_type == ControlType::Angular) ? params->getMinAngle2() : 1;
        break;
      case AxisType::Axis3:
        target = (control_type == ControlType::Angular) ? params->getMinAngle3() : 1;
        break;
      }
      break;
    }
    default: {
      Console( LogLevel::Warning ) << "This joint type is currently unsupported" << endl;
    }
  }

  return target;
}

H3DFloat JointPID::getMaxTarget( JointParameters& _joint, ControlType::e control_type, AxisType::e axis_type ) {
  H3DFloat target = 0.0f;

  switch( joint_type ) {
    case JointType::SingleAxisHinge: {
      SingleAxisHingeJointParameters* params = static_cast<SingleAxisHingeJointParameters*>(&_joint);
      target = (control_type == ControlType::Angular) ? params->getMaxAngle() : -1;
      break;
    }
    case JointType::DoubleAxisHinge: {
      DoubleAxisHingeJointParameters* params = static_cast<DoubleAxisHingeJointParameters*>(&_joint);
      switch( axis_type ) {
        case AxisType::Axis1:
          target = (control_type == ControlType::Angular) ? params->getMaxAngle1() : -1;
          break;
        case AxisType::Axis2:
          target = -1;
          break;
      }
      break;
    }
    case JointType::Slider: {
      SliderJointParameters* params = static_cast<SliderJointParameters*>(&_joint);
      target = (control_type == ControlType::Linear) ? params->getMaxSeparation() : -1;
      break;
    }
    case JointType::Generic6DOF: {
      Generic6DOFJointParameters* params = static_cast<Generic6DOFJointParameters*>(&_joint);
      switch( axis_type ) {
      case AxisType::Axis1:
        target = (control_type == ControlType::Angular) ? params->getMaxAngle1() : -1;
        break;
      case AxisType::Axis2:
        target = (control_type == ControlType::Angular) ? params->getMaxAngle2() : -1;
        break;
      case AxisType::Axis3:
        target = (control_type == ControlType::Angular) ? params->getMaxAngle3() : -1;
        break;
      }
      break;
    }
    default: {
      Console( LogLevel::Warning ) << "This joint type is currently unsupported" << endl;
    }
  }

  return target;
}

Vec3f JointPID::getAnchorPoint( JointParameters& _joint, ControlType::e control_type, AxisType::e axis_type, bool apply_to_body1 ) {
  Vec3f anchor_point;

  // getBody1AnchorPoint and getBody2AnchorPoint both return the anchor in world coordinates,
  // if the joint is not "violated" they should return the same value
  switch( joint_type ) {
    case JointType::SingleAxisHinge: {
      SingleAxisHingeJointParameters* params = static_cast<SingleAxisHingeJointParameters*>(&_joint);
      anchor_point = (control_type == ControlType::Angular) ? (apply_to_body1 ? params->getBody1AnchorPoint() : params->getBody2AnchorPoint()) : Vec3f();
      break;
    }
    case JointType::DoubleAxisHinge: {
      DoubleAxisHingeJointParameters* params = static_cast<DoubleAxisHingeJointParameters*>(&_joint);
      if( (axis_type == AxisType::Axis1 || axis_type == AxisType::Axis2) && control_type == ControlType::Angular ) {
        if( apply_to_body1 ) {
          anchor_point = params->getBody1AnchorPoint();
        } else {
          anchor_point = params->getBody2AnchorPoint();
        }
      }
      break;
    }
    case JointType::Slider: {
      anchor_point = Vec3f();
      break;
    }
    case JointType::Generic6DOF: {
      Generic6DOFJointParameters* params = static_cast<Generic6DOFJointParameters*>(&_joint);
      if( (axis_type == AxisType::Axis1 || axis_type == AxisType::Axis2 || axis_type == AxisType::Axis3) && control_type == ControlType::Angular ) {
        if( apply_to_body1 ) {
          anchor_point = params->getBody1AnchorPoint();
        } else {
          anchor_point = params->getBody2AnchorPoint();
        }
      }
      break;
    }
    default: {
      Console( LogLevel::Warning ) << "This joint type is currently unsupported" << endl;
    }
  }

  return anchor_point;
}

Vec3f JointPID::getAxis( JointParameters& _joint, ControlType::e control_type, AxisType::e axis_type ) {
  Vec3f axis;

  switch( joint_type ) {
    case JointType::SingleAxisHinge: {
      SingleAxisHingeJointParameters* params = static_cast<SingleAxisHingeJointParameters*>(&_joint);
      axis = (control_type == ControlType::Angular) ? params->getAxis() : Vec3f();
      break;
    }
    case JointType::DoubleAxisHinge: {
      DoubleAxisHingeJointParameters* params = static_cast<DoubleAxisHingeJointParameters*>(&_joint);
      switch( axis_type ) {
        case AxisType::Axis1:
          axis = (control_type == ControlType::Angular) ? params->getAxis1() : Vec3f();
          break;
        case AxisType::Axis2:
          axis = (control_type == ControlType::Angular) ? params->getAxis2() : Vec3f();
          break;
      }
      break;
    }
    case JointType::Slider: {
      SliderJointParameters* params = static_cast<SliderJointParameters*>(&_joint);
      axis = (control_type == ControlType::Linear) ? params->getAxis() : Vec3f();
      break;
    }
    case JointType::Generic6DOF: {
      Generic6DOFJointParameters* params = static_cast<Generic6DOFJointParameters*>(&_joint);
      switch( axis_type ) {
      case AxisType::Axis1:
        axis = (control_type == ControlType::Angular) ? params->getAxis1() : Vec3f();
        break;
      case AxisType::Axis2:
        axis = (control_type == ControlType::Angular) ? params->getAxis2() : Vec3f();
        break;
      case AxisType::Axis3:
        axis = (control_type == ControlType::Angular) ? params->getAxis3() : Vec3f();
        break;
      }
      break;
    }
    default: {
      Console( LogLevel::Warning ) << "This joint type is currently unsupported" << endl;
    }
  }

  return axis;
}

Vec3f JointPID::doPIDControl( PIDController& pid,
                              RigidBodyParameters& rigid_body1,
                              RigidBodyParameters& rigid_body2,
                              JointParameters& _joint,
                              ControlType::e control_type,
                              AxisType::e axis_type,
                              bool use_joint_motor,
                              bool apply_to_body1,
                              bool convert_torque_to_force ) {
  // Get current value and velocity
  H3DFloat value = getValue( _joint, control_type, axis_type );
  H3DFloat velocity = getVelocity( _joint, control_type, axis_type );
  H3DFloat minTarget = getMinTarget( _joint, control_type, axis_type );
  H3DFloat maxTarget = getMaxTarget( _joint, control_type, axis_type );

  // Do pid control
  H3DFloat output = pid.doControl(
    value, velocity,
    control_type == ControlType::Angular, // Check for angle wrap?
    minTarget, maxTarget );

  if( use_joint_motor ) {
    return Vec3f( output, 0, 0 );
  }

  // Transform actuation force
  Vec3f correctedAxis = pid.getAxis();
  if( correctedAxis.length() < Constants::f_epsilon ) {
    // Use joint axis instead
    correctedAxis = getAxis( _joint, control_type, axis_type );
  }

  if( !fixed ) {
    if( apply_to_body1 ) {
      correctedAxis = rigid_body1.getOrientation()*(-initial_orientation1)*correctedAxis;
    } else {
      correctedAxis = rigid_body2.getOrientation()*(-initial_orientation2)*correctedAxis;
    }
  }

  if( convert_torque_to_force ) {

    Vec3f ap = getAnchorPoint( _joint, control_type, axis_type, apply_to_body1 );
    Vec3f d;

    if( apply_to_body1 ) {
      d = ap - rigid_body1.getPosition();
    } else {
      d = ap - rigid_body2.getPosition();
    }

    if( d.length() > Constants::f_epsilon ) {
      correctedAxis = d.crossProduct( correctedAxis );
      output /= d.length();
    }
  }

  return output * correctedAxis;
}
