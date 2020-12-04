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
/// \file PhysicsEngineParameters.cpp
/// \brief cpp file for PhysicsEngineParameters.
///
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/H3DPhysics/PhysicsEngineParameters.h>

using namespace H3D;
using namespace PhysicsEngineParameters;

/// Copy the input parameters from the src parameters to this.
void WorldParameters::copyInputParameters( WorldParameters& src ) {
  if( src.haveAutoDisable() ) {
    auto_disable = src.auto_disable;
  }
  if( src.haveConstantForceMix() ) {
    constant_force_mix = src.constant_force_mix;
  }
  if( src.haveContactSurfaceThickness() ) {
    contact_surface_thickness = src.contact_surface_thickness;
  }
  if( src.haveDisableAngularSpeed() ) {
    disable_angular_speed = src.disable_angular_speed;
  }
  if( src.haveDisableLinearSpeed() ) {
    disable_linear_speed = src.disable_linear_speed;
  }
  if( src.haveDisableTime() ) {
    disable_time = src.disable_time;
  }
  if( src.haveErrorCorrection() ) {
    error_correction = src.error_correction;
  }
  if( src.haveGravity() ) {
    gravity = src.gravity;
  }
  if( src.haveIterations() ) {
    iterations = src.iterations;
  }
  if( src.haveMaxCorrectionSpeed() ) {
    max_correction_speed = src.max_correction_speed;
  }
  if( src.havePreferAccuracy() ) {
    prefer_accuracy = src.prefer_accuracy;
  }
  if( src.haveUseStaticTimeStep() ) {
    useStaticTimeStep = src.useStaticTimeStep;
  }
  if( src.haveEngineOptions() ) {
    engineOptions = src.engineOptions;
  }
}

void PhysicsEngineParameters::RigidBodyParameters::copyInputParameters( PhysicsEngineParameters::RigidBodyParameters& src ) {

#ifdef H3DPHYSICS_ENGINE_PARAMETERS_INCLUDE_NAME
  name = src.name;
#endif

  // NB. PhysicsEngineThread uses these like output fields to apply damping
  // If we dont use these conditionals the values will be wrong after subsequent setParameter() calls
  if( src.haveLinearDampingFactor() )
    linear_damping_factor = src.linear_damping_factor;
  if( src.haveAngularDampingFactor() )
    angular_damping_factor = src.angular_damping_factor;
  if( src.haveAutoDamp() )
    auto_damp = src.auto_damp;

  start_angular_velocity = src.start_angular_velocity;
  auto_disable = src.auto_disable;
  center_of_mass = src.center_of_mass;
  disable_angular_speed = src.disable_angular_speed;
  disable_linear_speed = src.disable_linear_speed;
  disable_time = src.disable_time;
  enabled = src.enabled;
  finite_rotation_axis = src.finite_rotation_axis;
  fixed = src.fixed;
  inertia = src.inertia;
  start_linear_velocity = src.start_linear_velocity;
  mass = src.mass;
  mass_density_model = src.mass_density_model;
  start_orientation = src.orientation;
  start_position = src.position;
  use_finite_rotation = src.use_finite_rotation;
  use_global_gravity = src.use_global_gravity;

  if( src.haveForce() )
    force = src.force;
  if( src.haveTorque() )
    torque = src.torque;
  if( src.haveGraphicsFrameForce() )
    graphicsFrameForce = src.graphicsFrameForce;
  if( src.haveGraphicsFrameTorque() )
    graphicsFrameTorque = src.graphicsFrameTorque;

  kinematicControl = src.kinematicControl;

  if( src.haveDebug() )
    debug = src.debug;

  if( src.havePosition() )
    position = src.position;
  if( src.haveOrientation() )
    orientation = src.orientation;
  if( src.haveGeometry() )
    geometry = src.geometry;
  if( src.havePositions() )
    positions = src.positions;
  if( src.haveOrientations() )
    orientations = src.orientations;
}

void PhysicsEngineParameters::RigidBodyParameters::copyOutputParameters(
  PhysicsEngineParameters::RigidBodyParameters& src ) {
  position = src.position;
  orientation = src.orientation;
  linear_velocity = src.linear_velocity;
  angular_velocity = src.angular_velocity;

  force = src.force;
  torque = src.torque;
  graphicsFrameForce = src.graphicsFrameForce;
  graphicsFrameTorque = src.graphicsFrameTorque;

  positions = src.positions;
  orientations = src.orientations;
}


PhysicsEngineParameters::RigidBodyParameters::RigidBodyParameters() :
  angular_damping_factor( (H3DFloat)0.001 ),
  angular_velocity( Vec3f( 0, 0, 0 ) ),
  auto_damp( false ),
  auto_disable( false ),
  center_of_mass( Vec3f( 0, 0, 0 ) ),
  disable_angular_speed( 0 ),
  disable_linear_speed( 0 ),
  disable_time( 0 ),
  enabled( true ),
  finite_rotation_axis( Vec3f( 0, 0, 0 ) ),
  fixed( false ),
  inertia( Matrix3f() ),
  linear_damping_factor( (H3DFloat)0.001 ),
  linear_velocity( Vec3f( 0, 0, 0 ) ),
  start_linear_velocity( Vec3f( 0, 0, 0 ) ),
  mass( 1 ),
  mass_density_model( NULL ),
  start_orientation( Rotation( 0, 0, 1, 0 ) ),
  start_position( Vec3f( 0, 0, 0 ) ),
  use_finite_rotation( false ),
  use_global_gravity( false ),
  orientation( Rotation( 0, 0, 1, 0 ) ),
  position( Vec3f( 0, 0, 0 ) ),
  kinematicControl( false ),
  update_bit_mask( 0 ),
  debug( false ),
  engine_thread( NULL ),
  body_id( 0 ),
  force( Vec3f( 0, 0, 0 ) ),
  torque( Vec3f( 0, 0, 0 ) ),
  graphicsFrameForce( Vec3f( 0, 0, 0 ) ),
  graphicsFrameTorque( Vec3f( 0, 0, 0 ) ),
  engineOptions( NULL )
#ifdef H3DPHYSICS_ENGINE_PARAMETERS_INCLUDE_NAME
  , name( "" )
#endif
{
}

void PhysicsEngineParameters::ArticulatedRigidBodyParameters::copyInputParameters( PhysicsEngineParameters::RigidBodyParameters& src ) {
  RigidBodyParameters::copyInputParameters( src );
  ArticulatedRigidBodyParameters* a_src = static_cast<ArticulatedRigidBodyParameters*>(&src);
  start_positions = a_src->start_positions;
  start_orientations = a_src->start_orientations;
  max_projection_iterations = a_src->max_projection_iterations;
  separation_tolerance = a_src->separation_tolerance;
  joint_internal_compliance = a_src->joint_internal_compliance;
  joint_external_compliance = a_src->joint_external_compliance;
  swing_limit = a_src->swing_limit;
  twist_limit = a_src->twist_limit;
}

void PhysicsEngineParameters::ArticulatedRigidBodyParameters::copyOutputParameters(
  PhysicsEngineParameters::RigidBodyParameters& src ) {

  ArticulatedRigidBodyParameters* a_src = static_cast<ArticulatedRigidBodyParameters*>(&src);
  bodies = a_src->bodies;
}

PhysicsEngineParameters::SpaceParameters::SpaceParameters() :
  engine_thread( NULL ),
  space_id( 0 ),
  parent_space_id( 0 ),
  enabled( true ),
  use_geometry( false ) {
}

PhysicsEngineParameters::CollidableParameters::CollidableParameters() :
  rotation( Rotation( 0, 0, 1, 0 ) ),
  translation( Vec3f( 0, 0, 0 ) ),
  scale( Vec3f( 1, 1, 1 ) ),
  enabled( true ),
  update_bit_mask( 0 ),
  engineOptions( NULL ),
  engine_thread( NULL ),
  collidable_id( 0 ),
  parent_space_id( 0 )
#ifdef H3DPHYSICS_ENGINE_PARAMETERS_INCLUDE_NAME
  , name( "" )
#endif
{
}

void PhysicsEngineParameters::CollidableParameters::copyInputParameters( PhysicsEngineParameters::CollidableParameters& src ) {
#ifdef H3DPHYSICS_ENGINE_PARAMETERS_INCLUDE_NAME
  name = src.name;
#endif
  if( src.haveEnabled() )
    enabled = src.enabled;
  if( src.haveTranslation() )
    translation = src.translation;
  if( src.haveRotation() )
    rotation = src.rotation;
  if( src.haveScale() )
    scale = src.scale;
  if( src.haveEngineOptions() )
    engineOptions = src.engineOptions;
}

void PhysicsEngineParameters::CollidableParameters::copyOutputParameters( PhysicsEngineParameters::CollidableParameters& src ) {

}

PhysicsEngineParameters::ShapeParameters::ShapeParameters() :
  CollidableParameters(),
  originalShape( NULL ),
  selfCollide( false ),
  updateShapeBounds( true ) {
}

void PhysicsEngineParameters::ShapeParameters::copyInputParameters( PhysicsEngineParameters::ShapeParameters& src ) {
  CollidableParameters::copyInputParameters( src );
  if( src.haveShape() )
    setShape( src.shape.get() );
}

void PhysicsEngineParameters::ShapeParameters::copyOutputParameters( PhysicsEngineParameters::ShapeParameters &src ) {
}

PhysicsEngineParameters::OffsetParameters::OffsetParameters() :
  collidable( 0 ) {
}

void PhysicsEngineParameters::OffsetParameters::copyInputParameters( PhysicsEngineParameters::OffsetParameters& src ) {
  CollidableParameters::copyInputParameters( src );
  collidable = src.collidable;
}

void PhysicsEngineParameters::OffsetParameters::copyOutputParameters( PhysicsEngineParameters::OffsetParameters& src ) {
}
void PhysicsEngineParameters::ConstraintParameters::copyInputParameters( ConstraintParameters& s ) {

  copyOutputFlags( s.update_bit_mask );
  if( s.haveBody1() )
    body1_id = s.body1_id;
}
void PhysicsEngineParameters::ConstraintParameters::copyOutputParameters( ConstraintParameters& s ) {

  copyOutputFlags( s.update_bit_mask );
}

PhysicsEngineParameters::JointParameters::JointParameters() :
  body2_id( 0 ),
  motor_target( 0 ) {
}

void PhysicsEngineParameters::JointParameters::copyInputParameters( ConstraintParameters& s ) {
  JointParameters *src = dynamic_cast<JointParameters *>(&s);
  if( src ) {

    ConstraintParameters::copyInputParameters( *src );
    if( src->haveBody2() )
      body2_id = src->body2_id;
    if( src->haveMotorTarget() )
      motor_target = src->motor_target;
  }
}
void PhysicsEngineParameters::JointParameters::copyOutputParameters( ConstraintParameters& s ) {

  JointParameters *src = dynamic_cast<JointParameters *>(&s);
  if( src ) {
    ConstraintParameters::copyOutputParameters( *src );
  }
}
PhysicsEngineParameters::BallJointParameters::BallJointParameters() :
  anchor_point( Vec3f( 0, 0, 0 ) ),
  body1_anchor_point( Vec3f( 0, 0, 0 ) ),
  body2_anchor_point( Vec3f( 0, 0, 0 ) ) {
  type = "BallJoint";
  all_output = BODY1_ANCHOR_POINT | BODY2_ANCHOR_POINT;
}

void PhysicsEngineParameters::BallJointParameters::copyOutputParameters( ConstraintParameters& s ) {
  BallJointParameters *src = dynamic_cast<BallJointParameters *>(&s);
  if( src ) {
    JointParameters::copyOutputParameters( *src );

    body1_anchor_point = src->body1_anchor_point;
    body2_anchor_point = src->body2_anchor_point;
  }
}

void PhysicsEngineParameters::BallJointParameters::copyInputParameters( ConstraintParameters& s ) {
  BallJointParameters *src = dynamic_cast<BallJointParameters *>(&s);
  if( src ) {
    JointParameters::copyInputParameters( *src );

    anchor_point = src->anchor_point;
  }
}

PhysicsEngineParameters::DistanceJointParameters::DistanceJointParameters() :
  max_distance( 0.1f ),
  min_distance( 0.0f ),
  stiffness( 100.f ),
  damping( 0.1f ),
  tolerance( 0.f ),
  distance( 0 ) {
  type = "DistanceJoint";
  all_output = DISTANCE;
}

void PhysicsEngineParameters::DistanceJointParameters::copyOutputParameters( ConstraintParameters& s ) {
  DistanceJointParameters *src = dynamic_cast<DistanceJointParameters *>(&s);
  if( src ) {
    JointParameters::copyOutputParameters( *src );

    distance = src->distance;
  }
}

void PhysicsEngineParameters::DistanceJointParameters::copyInputParameters( ConstraintParameters& s ) {
  DistanceJointParameters *src = dynamic_cast<DistanceJointParameters *>(&s);
  if( src ) {
    JointParameters::copyInputParameters( *src );

    max_distance = src->max_distance;
    min_distance = src->min_distance;
    stiffness = src->stiffness;
    damping = src->damping;
    tolerance = src->tolerance;
  }
}

// SingleAxisHingeJoint
PhysicsEngineParameters::SingleAxisHingeJointParameters::SingleAxisHingeJointParameters() :
  anchor_point( Vec3f( 0, 0, 0 ) ),
  axis( Vec3f( 0, 0, 0 ) ),
  max_angle( (H3DFloat)Constants::pi ),
  min_angle( (H3DFloat)-Constants::pi ),
  stop_bounce( (H3DFloat) 0.0 ),
  stop_error_correction( (H3DFloat) 0.8 ),
  angle( 0 ),
  angle_rate( 0 ),
  body1_anchor_point( Vec3f( 0, 0, 0 ) ),
  body2_anchor_point( Vec3f( 0, 0, 0 ) ),
  bias( (H3DFloat) 0.3 ),
  softness( (H3DFloat) 0.9 ) {
  type = "SingleAxisHingeJoint";
  all_output = ANGLE | ANGLE_RATE | BODY1_ANCHOR_POINT | BODY2_ANCHOR_POINT;
}

void PhysicsEngineParameters::SingleAxisHingeJointParameters::copyOutputParameters( ConstraintParameters& s ) {
  SingleAxisHingeJointParameters *src = dynamic_cast<SingleAxisHingeJointParameters *>(&s);
  if( src ) {
    JointParameters::copyOutputParameters( *src );

    body1_anchor_point = src->body1_anchor_point;
    body2_anchor_point = src->body2_anchor_point;
    angle = src->angle;
    angle_rate = src->angle_rate;
    max_angle = src->max_angle;
    min_angle = src->min_angle;
    axis = src->axis;
  }
}

void PhysicsEngineParameters::SingleAxisHingeJointParameters::copyInputParameters( ConstraintParameters& s ) {
  SingleAxisHingeJointParameters *src = dynamic_cast<SingleAxisHingeJointParameters *>(&s);
  if( src ) {
    JointParameters::copyInputParameters( *src );

    if( src->haveAnchorPoint() ) {
      anchor_point = src->anchor_point;
    }
    if( src->haveAxis() ) {
      axis = src->axis;
    }
    if( src->haveMaxAngle() ) {
      max_angle = src->max_angle;
    }
    if( src->haveMinAngle() ) {
      min_angle = src->min_angle;
    }
    if( src->haveStopBounce() ) {
      stop_bounce = src->stop_bounce;
    }
    if( src->haveStopErrorCorrection() ) {
      stop_error_correction = src->stop_error_correction;
    }
    if( src->haveBias() ) {
      bias = src->bias;
    }
    if( src->haveSoftness() ) {
      softness = src->softness;
    }
  }
}

// DoubleAxisHingeJoint
PhysicsEngineParameters::DoubleAxisHingeJointParameters::DoubleAxisHingeJointParameters() :
  anchor_point( Vec3f( 0, 0, 0 ) ),
  axis1( Vec3f( 0, 0, 0 ) ),
  axis2( Vec3f( 0, 0, 0 ) ),
  desired_angular_velocity1( (H3DFloat) 0.0 ),
  desired_angular_velocity2( (H3DFloat) 0.0 ),
  max_angle1( (H3DFloat)Constants::pi ),
  min_angle1( (H3DFloat)-Constants::pi ),
  max_torque1( (H3DFloat) 0.0 ),
  max_torque2( (H3DFloat) 0.0 ),
  stop_bounce1( (H3DFloat) 0.0 ),
  stop_constant_force_mix1( (H3DFloat) 0.001 ),
  stop_error_correction1( (H3DFloat) 0.8 ),
  suspension_error_correction( (H3DFloat) 0.8 ),
  suspension_force( (H3DFloat) 0.0 ),
  hinge1_angle( 0 ),
  hinge1_angle_rate( 0 ),
  hinge2_angle( 0 ),
  hinge2_angle_rate( 0 ),
  body1_anchor_point( Vec3f( 0, 0, 0 ) ),
  body1_axis( Vec3f( 0, 0, 0 ) ),
  body2_anchor_point( Vec3f( 0, 0, 0 ) ),
  body2_axis( Vec3f( 0, 0, 0 ) ) {
  type = "DoubleAxisHingeJoint";
  all_output = HINGE1_ANGLE | HINGE2_ANGLE | HINGE1_ANGLE_RATE | HINGE2_ANGLE_RATE |
    BODY1_ANCHOR_POINT | BODY2_ANCHOR_POINT | BODY1_AXIS | BODY2_AXIS;
}

void PhysicsEngineParameters::DoubleAxisHingeJointParameters::copyInputParameters( ConstraintParameters& s ) {
  DoubleAxisHingeJointParameters *src = dynamic_cast<DoubleAxisHingeJointParameters *>(&s);
  if( src ) {
    JointParameters::copyInputParameters( *src );

    if( src->haveAnchorPoint() ) {
      anchor_point = src->anchor_point;
    }
    if( src->haveAxis1() ) {
      axis1 = src->axis1;
    }
    if( src->haveAxis2() ) {
      axis2 = src->axis2;
    }
    if( src->haveDesiredAngularVelocity1() ) {
      desired_angular_velocity1 = src->desired_angular_velocity1;
    }
    if( src->haveDesiredAngularVelocity2() ) {
      desired_angular_velocity2 = src->desired_angular_velocity2;
    }
    if( src->haveMaxAngle1() ) {
      max_angle1 = src->max_angle1;
    }
    if( src->haveMinAngle1() ) {
      min_angle1 = src->min_angle1;
    }
    if( src->haveMaxTorque1() ) {
      max_torque1 = src->max_torque1;
    }
    if( src->haveMaxTorque2() ) {
      max_torque2 = src->max_torque2;
    }
    if( src->haveStopBounce1() ) {
      stop_bounce1 = src->stop_bounce1;
    }
    if( src->haveStopConstantForceMix1() ) {
      stop_constant_force_mix1 = src->stop_constant_force_mix1;
    }
    if( src->haveStopErrorCorrection1() ) {
      stop_error_correction1 = src->stop_error_correction1;
    }
    if( src->haveSuspensionErrorCorrection() ) {
      suspension_error_correction = src->suspension_error_correction;
    }
    if( src->haveSuspensionForce() ) {
      suspension_force = src->suspension_force;
    }
  }
}

void PhysicsEngineParameters::DoubleAxisHingeJointParameters::copyOutputParameters( ConstraintParameters& s ) {
  DoubleAxisHingeJointParameters *src = dynamic_cast<DoubleAxisHingeJointParameters *>(&s);
  if( src ) {
    JointParameters::copyOutputParameters( *src );

    body1_anchor_point = src->body1_anchor_point;
    body2_anchor_point = src->body2_anchor_point;
    body1_axis = src->body1_axis;
    body2_axis = src->body2_axis;
    hinge1_angle = src->hinge1_angle;
    hinge2_angle = src->hinge2_angle;
    hinge1_angle_rate = src->hinge1_angle_rate;
    hinge2_angle_rate = src->hinge2_angle_rate;
    max_angle1 = src->max_angle1;
    min_angle1 = src->min_angle1;
    axis1 = src->axis1;
    axis2 = src->axis2;
  }
}


// MotorJoint
PhysicsEngineParameters::MotorJointParameters::MotorJointParameters() :
  axis1_angle( 0 ),
  axis1_torque( 0 ),
  axis2_angle( 0 ),
  axis2_torque( 0 ),
  axis3_angle( 0 ),
  axis3_torque( 0 ),
  enabled_axes( 1 ),
  motor1_axis( Vec3f( 0, 0, 0 ) ),
  motor2_axis( Vec3f( 0, 0, 0 ) ),
  motor3_axis( Vec3f( 0, 0, 0 ) ),
  stop1_bounce( 0 ),
  stop1_error_correction( (H3DFloat) 0.8 ),
  stop2_bounce( 0 ),
  stop2_error_correction( (H3DFloat) 0.8 ),
  stop3_bounce( 0 ),
  stop3_error_correction( (H3DFloat) 0.8 ),
  auto_calc( false ),
  motor1_angle( 0 ),
  motor1_angle_rate( 0 ),
  motor2_angle( 0 ),
  motor2_angle_rate( 0 ),
  motor3_angle( 0 ),
  motor3_angle_rate( 0 ) {
  type = "MotorJoint";
  all_output = MOTOR1_ANGLE | MOTOR1_ANGLE_RATE | MOTOR2_ANGLE | MOTOR2_ANGLE_RATE |
    MOTOR3_ANGLE | MOTOR3_ANGLE_RATE;
}

void PhysicsEngineParameters::MotorJointParameters::copyInputParameters( ConstraintParameters& s ) {
  MotorJointParameters *src = dynamic_cast<MotorJointParameters *>(&s);
  if( src ) {
    JointParameters::copyInputParameters( *src );

    axis1_angle = src->axis1_angle;
    axis1_torque = src->axis1_torque;
    axis2_angle = src->axis2_angle;
    axis2_torque = src->axis2_torque;
    axis3_angle = src->axis3_angle;
    axis3_torque = src->axis3_torque;
    enabled_axes = src->enabled_axes;
    motor1_axis = src->motor1_axis;
    motor2_axis = src->motor2_axis;
    motor3_axis = src->motor3_axis;
    stop1_bounce = src->stop1_bounce;
    stop1_error_correction = src->stop1_error_correction;
    stop2_bounce = src->stop2_bounce;
    stop2_error_correction = src->stop2_error_correction;
    stop3_bounce = src->stop3_bounce;
    stop3_error_correction = src->stop3_error_correction;
  }
}

void PhysicsEngineParameters::MotorJointParameters::copyOutputParameters( ConstraintParameters& s ) {
  MotorJointParameters *src = dynamic_cast<MotorJointParameters *>(&s);
  if( src ) {
    JointParameters::copyOutputParameters( *src );

    motor1_angle = src->motor1_angle;
    motor1_angle_rate = src->motor1_angle_rate;
    motor2_angle = src->motor2_angle;
    motor2_angle_rate = src->motor2_angle_rate;
    motor3_angle = src->motor3_angle;
    motor3_angle_rate = src->motor3_angle_rate;
  }
}

// SliderJoint
PhysicsEngineParameters::SliderJointParameters::SliderJointParameters() :
  axis( Vec3f( 0, 1, 0 ) ),
  max_separation( 1 ),
  min_separation( 0 ),
  stop_bounce( 0 ),
  stop_error_correction( 1 ),
  separation( 0 ),
  separation_rate( 0 ),
  slider_force( 0 ) {
  type = "SliderJoint";
  all_output = SEPARATION | SEPARATION_RATE;
}

void PhysicsEngineParameters::SliderJointParameters::copyInputParameters( ConstraintParameters& s ) {
  SliderJointParameters *src = dynamic_cast<SliderJointParameters *>(&s);
  if( src ) {
    JointParameters::copyInputParameters( *src );

    if( src->haveAxis() ) {
      axis = src->axis;
    }
    if( src->haveMaxSeparation() ) {
      max_separation = src->max_separation;
    }
    if( src->haveMinSeparation() ) {
      min_separation = src->min_separation;
    }
    if( src->haveStopBounce() ) {
      stop_bounce = src->stop_bounce;
    }
    if( src->haveStopErrorCorrection() ) {
      stop_error_correction = src->stop_error_correction;
    }
    if( src->haveSliderForce() ) {
      slider_force = src->slider_force;
    }
  }
}

void PhysicsEngineParameters::SliderJointParameters::copyOutputParameters( ConstraintParameters& s ) {
  SliderJointParameters *src = dynamic_cast<SliderJointParameters *>(&s);
  if( src ) {
    JointParameters::copyOutputParameters( *src );

    separation = src->separation;
    separation_rate = src->separation_rate;
    max_separation = src->max_separation;
    min_separation = src->min_separation;
    axis = src->axis;
  }
}

// UniversalJoint
PhysicsEngineParameters::UniversalJointParameters::UniversalJointParameters() :
  anchor_point( Vec3f( 0, 0, 0 ) ),
  axis1( Vec3f( 0, 0, 0 ) ),
  axis2( Vec3f( 0, 0, 0 ) ),
  stop1_bounce( 0 ),
  stop1_error_correction( (H3DFloat)0.8 ),
  stop2_bounce( 0 ),
  stop2_error_correction( (H3DFloat)0.8 ),
  body1_anchor_point( Vec3f( 0, 0, 0 ) ),
  body1_axis( Vec3f( 0, 0, 0 ) ),
  body2_anchor_point( Vec3f( 0, 0, 0 ) ),
  body2_axis( Vec3f( 0, 0, 0 ) ) {
  type = "UniversalJoint";
  all_output = BODY1_ANCHOR_POINT | BODY1_AXIS | BODY2_ANCHOR_POINT | BODY2_AXIS;
}

void PhysicsEngineParameters::UniversalJointParameters::copyInputParameters( ConstraintParameters& s ) {
  UniversalJointParameters *src = dynamic_cast<UniversalJointParameters *>(&s);
  if( src ) {
    JointParameters::copyInputParameters( *src );

    anchor_point = src->anchor_point;
    axis1 = src->axis1;
    axis2 = src->axis2;
    stop1_bounce = src->stop1_bounce;
    stop1_error_correction = src->stop1_error_correction;
    stop2_bounce = src->stop2_bounce;
    stop2_error_correction = src->stop2_error_correction;
  }
}

void PhysicsEngineParameters::UniversalJointParameters::copyOutputParameters( ConstraintParameters& s ) {
  UniversalJointParameters *src = dynamic_cast<UniversalJointParameters *>(&s);
  if( src ) {
    JointParameters::copyOutputParameters( *src );

    body1_anchor_point = src->body1_anchor_point;
    body1_axis = src->body1_axis;
    body2_anchor_point = src->body2_anchor_point;
    body2_axis = src->body2_axis;
  }
}

Generic6DOFJointParameters::Generic6DOFJointParameters() :
  anchor_point( Vec3f( 0, 0, 0 ) ),
  desired_angular_velocity1( 0.0f ),
  desired_angular_velocity2( 0.0f ),
  desired_angular_velocity3( 0.0f ),
  min_angle1( -1.0f ),
  min_angle2( -1.0f ),
  min_angle3( -1.0f ),
  max_angle1( 1.0f ),
  max_angle2( 1.0f ),
  max_angle3( 1.0f ),
  max_torque1( 0.0f ),
  max_torque2( 0.0f ),
  max_torque3( 0.0f ),
  desired_linear_velocity1( 0.0f ),
  desired_linear_velocity2( 0.0f ),
  desired_linear_velocity3( 0.0f ),
  min_limit1( 0.0f ),
  min_limit2( 0.0f ),
  min_limit3( 0.0f ),
  max_limit1( 0.0f ),
  max_limit2( 0.0f ),
  max_limit3( 0.0f ),
  max_force1( 0.0f ),
  max_force2( 0.0f ),
  max_force3( 0.0f ),
  hinge1_angle( 0 ),
  hinge1_angle_rate( 0 ),
  hinge2_angle( 0 ),
  hinge2_angle_rate( 0 ),
  hinge3_angle( 0 ),
  hinge3_angle_rate( 0 ),
  body1_anchor_point( Vec3f( 0, 0, 0 ) ),
  body1_axis( Vec3f( 0, 0, 0 ) ),
  body2_anchor_point( Vec3f( 0, 0, 0 ) ),
  body2_axis( Vec3f( 0, 0, 0 ) ),
  generic6dof_output_bit_mask( 0 ) {
  type = "Generic6DOFJoint";
  all_output = HINGE1_ANGLE | HINGE2_ANGLE | HINGE3_ANGLE | HINGE1_ANGLE_RATE | HINGE2_ANGLE_RATE |
    HINGE3_ANGLE_RATE | BODY1_ANCHOR_POINT | BODY2_ANCHOR_POINT | BODY1_AXIS | BODY2_AXIS;
}

void Generic6DOFJointParameters::copyInputParameters( ConstraintParameters& s ) {
  Generic6DOFJointParameters *src = dynamic_cast<Generic6DOFJointParameters *>(&s);
  if( src ) {
    unsigned long int update_bit_mask_old = update_bit_mask;
    JointParameters::copyInputParameters( *src );
    update_bit_mask = src->update_bit_mask;
    copyGeneric6DOFOutputBitMask( src->generic6dof_output_bit_mask );

    anchor_point = src->anchor_point;
    axis1 = src->axis1;
    axis2 = src->axis2;
    axis3 = src->axis3;

    desired_angular_velocity1 = src->desired_angular_velocity1;
    desired_angular_velocity2 = src->desired_angular_velocity2;
    desired_angular_velocity3 = src->desired_angular_velocity3;

    min_angle1 = src->min_angle1;
    min_angle2 = src->min_angle2;
    min_angle3 = src->min_angle3;

    max_angle1 = src->max_angle1;
    max_angle2 = src->max_angle2;
    max_angle3 = src->max_angle3;

    max_torque1 = src->max_torque1;
    max_torque2 = src->max_torque2;
    max_torque3 = src->max_torque3;

    desired_linear_velocity1 = src->desired_linear_velocity1;
    desired_linear_velocity2 = src->desired_linear_velocity2;
    desired_linear_velocity3 = src->desired_linear_velocity3;

    min_limit1 = src->min_limit1;
    min_limit2 = src->min_limit2;
    min_limit3 = src->min_limit3;

    max_limit1 = src->max_limit1;
    max_limit2 = src->max_limit2;
    max_limit3 = src->max_limit3;

    max_force1 = src->max_force1;
    max_force2 = src->max_force2;
    max_force3 = src->max_force3;
  }
}

void Generic6DOFJointParameters::copyOutputParameters( ConstraintParameters& s ) {
  Generic6DOFJointParameters *src = dynamic_cast<Generic6DOFJointParameters *>(&s);
  if( src ) {
    unsigned long int update_bit_mask_old = update_bit_mask;
    JointParameters::copyOutputParameters( *src );
    update_bit_mask = src->update_bit_mask;
    copyGeneric6DOFOutputBitMask( src->generic6dof_output_bit_mask );

    body1_anchor_point = src->body1_anchor_point;
    body2_anchor_point = src->body2_anchor_point;
    body1_axis = src->body1_axis;
    body2_axis = src->body2_axis;
    hinge1_angle = src->hinge1_angle;
    hinge2_angle = src->hinge2_angle;
    hinge3_angle = src->hinge3_angle;
    hinge1_angle_rate = src->hinge1_angle_rate;
    hinge2_angle_rate = src->hinge2_angle_rate;
    hinge3_angle_rate = src->hinge3_angle_rate;
    max_angle1 = src->max_angle1;
    min_angle1 = src->min_angle1;
    axis1 = src->axis1;
    axis2 = src->axis2;
  }
}

