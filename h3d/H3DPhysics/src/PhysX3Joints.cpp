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
/// \file PhysX3Joints.cpp
/// \brief Source file for BulletJoint classes, which maintain a link between 
/// RigidBodyPhysics joint types and their Bullet implementations
///
//
//////////////////////////////////////////////////////////////////////////////
#include <H3D/H3DPhysics/PhysX3Joints.h>
#include <H3D/H3DPhysics/PhysX3Callbacks.h>
#include <H3D/H3DPhysics/PhysX3JointParameters.h>

#ifdef HAVE_PHYSX3

using namespace H3D;
using namespace physx;
using namespace PhysicsEngineParameters;

#ifdef min
#undef min
#endif

#ifdef max
#undef max
#endif

#if PX_PHYSICS_VERSION_MAJOR >= 3 && PX_PHYSICS_VERSION_MINOR >= 3
#define PX_PHYSICS_3_3_OR_LATER
#endif

PhysX3Joint::JointDatabase::JointMap PhysX3Joint::JointDatabase::map;

PhysX3Joint::PhysX3Joint( ConstraintParameters& jointParameters ) :
  joint( NULL ) {
  jointParameters.setConstraintId( (H3DConstraintId)this );
}

PhysX3Joint::~PhysX3Joint() {
  remove();
}

// Create a joint instance of the specified type name
PhysX3Joint* PhysX3Joint::createJoint( ConstraintParameters& jointParameters ) {
  CreateFunc f = JointDatabase::map[jointParameters.getType()];

  if( f )
    return (*f)(jointParameters);
  else
    return NULL;
}

// Remove joint from simulation
void PhysX3Joint::remove() {
  if( joint ) {
    joint->release();
    joint = NULL;
  }
}

void PhysX3Joint::setJointFrame( const Vec3f& position, const Rotation& orientation, Body::e bodies ) {
  global_frame = Matrix4f( position, orientation );

  PxTransform globalFrame = PxTransform( toPxVec3( position ), toPxQuat( orientation ) );
  PxTransform localFrame1 = globalFrame;
  PxTransform localFrame2 = globalFrame;

  PxRigidActor* actor1, *actor2;
  joint->getActors( actor1, actor2 );

  if( actor1 ) {
    localFrame1 = actor1->getGlobalPose().getInverse()*localFrame1;
  }
  if( actor2 ) {
    localFrame2 = actor2->getGlobalPose().getInverse()*localFrame2;
  }

  if( bodies == Body::BOTH || bodies == Body::B )
    joint->setLocalPose( PxJointActorIndex::eACTOR0, localFrame1 );

  if( bodies == Body::BOTH || bodies == Body::A )
    joint->setLocalPose( PxJointActorIndex::eACTOR1, localFrame2 );
}

Matrix4f PhysX3Joint::getJointFrame( Body::e body ) {
  PxRigidActor* actor1, *actor2;
  joint->getActors( actor1, actor2 );

  PxTransform localFrame = joint->getLocalPose(
    (body == Body::A) ? PxJointActorIndex::eACTOR1 : PxJointActorIndex::eACTOR0 );
  PxTransform globalFrame = localFrame;

  PxRigidActor* actor = (body == Body::A) ? actor2 : actor1;
  if( actor ) {
    globalFrame = actor->getGlobalPose() * globalFrame;
  }

  return toMatrix4f( globalFrame );
}

H3DFloat PhysX3Joint::getRotation( Axis::e axis, Body::e axisRelativeTo ) {
  Vec3f localY, localX;
  switch( axis ) {
  case Axis::X:
    localY = Vec3f( 0, 1, 0 );
    localX = Vec3f( 1, 0, 0 );
    break;
  case Axis::Y:
    localY = Vec3f( 1, 0, 0 );
    localX = Vec3f( 0, 1, 0 );
    break;
  case Axis::Z:
    localY = Vec3f( 0, 1, 0 );
    localX = Vec3f( 0, 0, 1 );
    break;
  }

  Vec3f bodyA_Y = getJointFrame( Body::B ).getRotationPart()*localY;
  Vec3f bodyB_Y = getJointFrame( Body::A ).getRotationPart()*localY;
  Vec3f bodyB_X = getJointFrame( axisRelativeTo ).getRotationPart()*localX;

  // Project bodyA_Y on to plane with normal bodyB_X
  Vec3f projBodyA_Y = bodyA_Y - bodyA_Y.dotProduct( bodyB_X ) * bodyB_X;
  projBodyA_Y.normalizeSafe();

  // Get the angle between the two Y axes now in the same plane
  H3DFloat dot = projBodyA_Y.dotProduct( bodyB_Y );
  dot = std::max( -1.0f, std::min( 1.0f, dot ) );
  H3DFloat angle = acos( dot );

  // Get the correct sign depending on the direction of rotation
  Vec3f crossProd = projBodyA_Y.crossProduct( bodyB_Y );
  crossProd.normalizeSafe();
  if( crossProd.dotProduct( bodyB_X ) < 0 ) {
    angle = -angle;
  }

  return angle;
}

// Fixed joint
//

PhysX3FixedJoint::JointDatabase
PhysX3FixedJoint::database( "FixedJoint", &newInstance<PhysX3FixedJoint> );

PhysX3FixedJoint::PhysX3FixedJoint( ConstraintParameters& jointParameters ) :
  PhysX3Joint( jointParameters ) {
  PhysX3Callbacks::PhysX3SpecificData *physx_data =
    static_cast<PhysX3Callbacks::PhysX3SpecificData *>(jointParameters.getEngine()->getEngineSpecificData());
  FixedJointParameters* params = static_cast <FixedJointParameters*> (&jointParameters);

  PhysX3RigidBody* body1 = (PhysX3RigidBody*)params->getBody1();
  PhysX3RigidBody* body2 = (PhysX3RigidBody*)params->getBody2();

  PxRigidActor* actor1 = body1 ? body1->getActor() : NULL;
  PxRigidActor* actor2 = body2 ? body2->getActor() : NULL;

  joint = PxFixedJointCreate(
    *physx_data->sdk,
    actor1, PxTransform::createIdentity(),
    actor2, PxTransform::createIdentity() );

  // Anchor point is in between body 1 and body 2
  Vec3f anchorPoint;
  if( actor1 ) {
    anchorPoint = toVec3f( actor1->getGlobalPose().p );
  }
  if( actor2 ) {
    anchorPoint += toVec3f( actor2->getGlobalPose().p );
  }
  if( actor1 && actor2 ) {
    anchorPoint /= 2;
  }

  setJointFrame( anchorPoint );

  setParameters( jointParameters );
}

/// Set the PhysX3 joint's parameters using the parameter values specified in jointParameters
void PhysX3FixedJoint::setParameters( ConstraintParameters& jointParameters ) {
  FixedJointParameters* params = static_cast<FixedJointParameters*> (&jointParameters);

  if( params->haveEngineOptions() ) {
    if( PhysX3JointParameters* options = dynamic_cast<PhysX3JointParameters*>(params->getEngineOptions()) ) {

      PxFixedJoint* physx_joint = static_cast<PxFixedJoint*>(joint);

      if( options->haveProjectionTolerance() ) {
        float linear_tolerance = options->getProjectionTolerance()[0];
        float angular_tolerance = options->getProjectionTolerance()[1];
        if( linear_tolerance > 0.0 && angular_tolerance > 0.0 ) {
          physx_joint->setProjectionLinearTolerance( PxReal( linear_tolerance ) );
          physx_joint->setProjectionAngularTolerance( PxReal( angular_tolerance ) );
        }
      }

      if( options->haveConstraintFlag() || options->haveEnabledProjection() ) {
        string constraint_flag = options->getConstraintFlag();
        if( constraint_flag == "ePROJECTION" ) {
          physx_joint->setConstraintFlag( PxConstraintFlag::ePROJECTION, options->getEnabledProjection() );
        } else if( constraint_flag == "ePROJECT_TO_ACTOR0" ) {
          physx_joint->setConstraintFlag( PxConstraintFlag::ePROJECT_TO_ACTOR0, options->getEnabledProjection() );
        } else if( constraint_flag == "ePROJECT_TO_ACTOR1" ) {
          physx_joint->setConstraintFlag( PxConstraintFlag::ePROJECT_TO_ACTOR1, options->getEnabledProjection() );
        }
      }
    }
  }
}


// Ball joint
//

PhysX3BallJoint::JointDatabase
PhysX3BallJoint::database( "BallJoint", &newInstance<PhysX3BallJoint> );

PhysX3BallJoint::PhysX3BallJoint( ConstraintParameters& jointParameters ) :
  PhysX3Joint( jointParameters ) {
  PhysX3Callbacks::PhysX3SpecificData *physx_data =
    static_cast<PhysX3Callbacks::PhysX3SpecificData *>(jointParameters.getEngine()->getEngineSpecificData());
  BallJointParameters* params = static_cast <BallJointParameters*> (&jointParameters);

  PhysX3RigidBody* body1 = (PhysX3RigidBody*)params->getBody1();
  PhysX3RigidBody* body2 = (PhysX3RigidBody*)params->getBody2();

  joint = PxSphericalJointCreate(
    *physx_data->sdk,
    body1 ? body1->getActor() : NULL, PxTransform::createIdentity(),
    body2 ? body2->getActor() : NULL, PxTransform::createIdentity() );

  setParameters( jointParameters );
}

// Set the bullet joint's parameters using the parameter values specified in jointParameters
void PhysX3BallJoint::setParameters( ConstraintParameters& jointParameters ) {
  BallJointParameters* params = static_cast <BallJointParameters*> (&jointParameters);

  // Anchor point
  if( params->haveAnchorPoint() ) {
    setJointFrame( params->getAnchorPoint() );
  }

  if( params->haveEngineOptions() ) {
    if( PhysX3Joint6DOFLimitParameters* options = dynamic_cast<PhysX3Joint6DOFLimitParameters*>(params->getEngineOptions()) ) {

      PxSphericalJoint* s_joint = static_cast<PxSphericalJoint*>(joint);

      if( options->haveContactDistance() ) {
        limit_contact_distance = options->getContactDistance();
        limit_contact_distance.y *= (H3DFloat)(H3DUtil::Constants::pi / 180.0);
      }

      // Cone Limit
      if( options->haveAngularX() ) {
        Vec2f limit = (H3DUtil::Constants::pi / 180.0)*options->getAngularX();
        if( limit != Vec2f( 0.0, 0.0 ) && limit.x > 0.0 && limit.y > 0.0 ) {
          s_joint->setLimitCone( PxJointLimitCone( limit.x, limit.y, PxReal( limit_contact_distance.y ) ) );
          s_joint->setSphericalJointFlag( PxSphericalJointFlag::eLIMIT_ENABLED, true );
        } else {
          s_joint->setSphericalJointFlag( PxSphericalJointFlag::eLIMIT_ENABLED, false );
        }

      }

      // Projection for all locked dofs
      if( options->haveProjectionTolerance() ) {
        float linear_tolerance = options->getProjectionTolerance()[0];
        if( linear_tolerance > 0.0 ) {
          s_joint->setProjectionLinearTolerance( PxReal( linear_tolerance ) );
        }
      }

      if( options->haveConstraintFlag() || options->haveEnabledProjection() ) {
        string constraint_flag = options->getConstraintFlag();
        if( constraint_flag == "ePROJECTION" ) {
          s_joint->setConstraintFlag( PxConstraintFlag::ePROJECTION, options->getEnabledProjection() );
        } else if( constraint_flag == "ePROJECT_TO_ACTOR0" ) {
          s_joint->setConstraintFlag( PxConstraintFlag::ePROJECT_TO_ACTOR0, options->getEnabledProjection() );
        } else if( constraint_flag == "ePROJECT_TO_ACTOR1" ) {
          s_joint->setConstraintFlag( PxConstraintFlag::ePROJECT_TO_ACTOR1, options->getEnabledProjection() );
        }
      }
    }
  }
}

// Modify the jointParameters argument to reflect the bullet joint's current parameters
void PhysX3BallJoint::getParameters( ConstraintParameters& jointParameters ) {
  BallJointParameters* params = static_cast <BallJointParameters*> (&jointParameters);

  // Body 1 anchor point
  if( params->haveBody1AnchorPoint() ) {
    params->setBody1AnchorPoint( getJointFrame( Body::B ).getTranslationPart() );
  }

  // Body 2 anchor point
  if( params->haveBody2AnchorPoint() ) {
    params->setBody2AnchorPoint( getJointFrame( Body::A ).getTranslationPart() );
  }
}


// Single axis hinge joint
//

PhysX3SingleAxisHingeJoint::JointDatabase
PhysX3SingleAxisHingeJoint::database( "SingleAxisHingeJoint", &newInstance<PhysX3SingleAxisHingeJoint> );

PhysX3SingleAxisHingeJoint::PhysX3SingleAxisHingeJoint( ConstraintParameters& jointParameters ) :
  PhysX3Joint( jointParameters ) {
  PhysX3Callbacks::PhysX3SpecificData *physx_data =
    static_cast<PhysX3Callbacks::PhysX3SpecificData *>(jointParameters.getEngine()->getEngineSpecificData());
  SingleAxisHingeJointParameters* params = static_cast <SingleAxisHingeJointParameters*> (&jointParameters);

  PhysX3RigidBody* body1 = (PhysX3RigidBody*)params->getBody1();
  PhysX3RigidBody* body2 = (PhysX3RigidBody*)params->getBody2();

  joint = PxRevoluteJointCreate(
    *physx_data->sdk,
    body1 ? body1->getActor() : NULL, PxTransform::createIdentity(),
    body2 ? body2->getActor() : NULL, PxTransform::createIdentity() );

  setParameters( jointParameters );
}

// Set the bullet joint's parameters using the parameter values specified in jointParameters
void PhysX3SingleAxisHingeJoint::setParameters( ConstraintParameters& jointParameters ) {
  SingleAxisHingeJointParameters* params = static_cast <SingleAxisHingeJointParameters*> (&jointParameters);
  PxRevoluteJoint* revoluteJoint = static_cast<PxRevoluteJoint*>(joint);

  // Anchor point
  if( params->haveAnchorPoint() ) {
    setJointFrame( params->getAnchorPoint(), Rotation( getJointFrame().getRotationPart() ) );
  }

  // Axis
  if( params->haveAxis() ) {
    Vec3f axis = params->getAxis();
    axis.normalizeSafe();
    setJointFrame( getJointFrame().getTranslationPart(), Rotation( Vec3f( 1, 0, 0 ), axis ) );
  }

  // Limit angles
  if( params->haveMaxAngle() || params->haveMinAngle() ) {
#ifdef PX_PHYSICS_3_3_OR_LATER
    PxJointAngularLimitPair limit = revoluteJoint->getLimit();
#else
    PxJointLimitPair limit = revoluteJoint->getLimit();
#endif
    limit.lower = -params->getMaxAngle();
    limit.upper = -params->getMinAngle();

    // Physx limits need to be less than pi
    if( limit.lower <= -physx::PxPi )
      limit.lower = -physx::PxPi + Constants::f_epsilon;
    if( limit.upper >= physx::PxPi )
      limit.upper = physx::PxPi - Constants::f_epsilon;

    if( limit.upper > limit.lower )
      revoluteJoint->setLimit( limit );

    revoluteJoint->setRevoluteJointFlag(
      PxRevoluteJointFlag::eLIMIT_ENABLED, limit.upper > limit.lower );
  }

  // Stop bounce
  if( params->haveStopBounce() ) {
#ifdef PX_PHYSICS_3_3_OR_LATER
    PxJointAngularLimitPair limit = revoluteJoint->getLimit();
#else
    PxJointLimitPair limit = revoluteJoint->getLimit();
#endif
    limit.restitution = params->getStopBounce();
    revoluteJoint->setLimit( limit );
  }

  if( params->haveEngineOptions() ) {
    if( PhysX3JointParameters* options = dynamic_cast<PhysX3JointParameters*>(params->getEngineOptions()) ) {

      // Projection for all locked dofs
      if( options->haveProjectionTolerance() ) {
        float linear_tolerance = options->getProjectionTolerance()[0];
        float angular_tolerance = options->getProjectionTolerance()[1];
        if( linear_tolerance > 0.0 && angular_tolerance > 0.0 ) {
          revoluteJoint->setProjectionLinearTolerance( PxReal( linear_tolerance ) );
          revoluteJoint->setProjectionAngularTolerance( PxReal( angular_tolerance ) );
        }
      }

      if( options->haveConstraintFlag() || options->haveEnabledProjection() ) {
        string constraint_flag = options->getConstraintFlag();
        if( constraint_flag == "ePROJECTION" ) {
          revoluteJoint->setConstraintFlag( PxConstraintFlag::ePROJECTION, options->getEnabledProjection() );
        } else if( constraint_flag == "ePROJECT_TO_ACTOR0" ) {
          revoluteJoint->setConstraintFlag( PxConstraintFlag::ePROJECT_TO_ACTOR0, options->getEnabledProjection() );
        } else if( constraint_flag == "ePROJECT_TO_ACTOR1" ) {
          revoluteJoint->setConstraintFlag( PxConstraintFlag::ePROJECT_TO_ACTOR1, options->getEnabledProjection() );
        }
      }
    }
  }
}

// Modify the jointParameters argument to reflect the bullet joint's current parameters
void PhysX3SingleAxisHingeJoint::getParameters( ConstraintParameters& jointParameters ) {
  SingleAxisHingeJointParameters* params = static_cast <SingleAxisHingeJointParameters*> (&jointParameters);

  // Body 1 anchor point
  if( params->haveBody1AnchorPoint() ) {
    params->setBody1AnchorPoint( getJointFrame( Body::A ).getTranslationPart() );
  }

  // Body 2 anchor point
  if( params->haveBody2AnchorPoint() ) {
    params->setBody2AnchorPoint( getJointFrame( Body::B ).getTranslationPart() );
  }

  if( params->haveAngle() || params->haveAngleRate() ) {
    // Note: Reversed to match Bullet and ODE convension for single axis hinge
    H3DFloat angle = -getRotation( Axis::X );

    // Angle
    if( params->haveAngle() ) {
      params->setAngle( angle );
    }

    // Angle rate
    if( params->haveAngleRate() ) {
      hingeAngleRate.setCurrentValue( angle );
      params->setAngleRate( hingeAngleRate );
    }
  }
}


// Abstract base for 6 DoF joints
//

PhysX36DoFJoint::PhysX36DoFJoint( ConstraintParameters& jointParameters ) :
  PhysX3Joint( jointParameters ) {
  PhysX3Callbacks::PhysX3SpecificData *physx_data =
    static_cast<PhysX3Callbacks::PhysX3SpecificData *>(jointParameters.getEngine()->getEngineSpecificData());
  JointParameters* params = static_cast <JointParameters*> (&jointParameters);

  PhysX3RigidBody* body1 = (PhysX3RigidBody*)params->getBody1();
  PhysX3RigidBody* body2 = (PhysX3RigidBody*)params->getBody2();

  joint = PxD6JointCreate(
    *physx_data->sdk,
    body2 ? body2->getActor() : NULL, PxTransform::createIdentity(),
    body1 ? body1->getActor() : NULL, PxTransform::createIdentity() );
}

void PhysX36DoFJoint::makeAxesPerpendicular() {
  axis3 = axis1.crossProduct( axis2 );
  axis3.normalizeSafe();

  Vec3f newAxis2 = axis3.crossProduct( axis1 );
  newAxis2.normalizeSafe();

  if( axis2.dotProduct( newAxis2 ) < 1 - H3DUtil::Constants::f_epsilon ) {
    Console(4) << "Warning: PhysX3 implemetation of joints only supports perpendicular axes!" << endl;
    axis2 = newAxis2;
  }
}


// Double axis hinge joint
//

PhysX3DoubleAxisHingeJoint::JointDatabase
PhysX3DoubleAxisHingeJoint::database( "DoubleAxisHingeJoint", &newInstance<PhysX3DoubleAxisHingeJoint> );

PhysX3DoubleAxisHingeJoint::PhysX3DoubleAxisHingeJoint( ConstraintParameters& jointParameters ) :
  PhysX36DoFJoint( jointParameters ) {
  PhysX3Callbacks::PhysX3SpecificData *physx_data =
    static_cast<PhysX3Callbacks::PhysX3SpecificData *>(jointParameters.getEngine()->getEngineSpecificData());
  DoubleAxisHingeJointParameters* params = static_cast <DoubleAxisHingeJointParameters*> (&jointParameters);

  PxD6Joint* d6Joint = static_cast<PxD6Joint*>(joint);
  d6Joint->setMotion( PxD6Axis::eTWIST, PxD6Motion::eFREE );
  d6Joint->setMotion( PxD6Axis::eSWING1, PxD6Motion::eFREE );

  setParameters( jointParameters );
}

// Set the bullet joint's parameters using the parameter values specified in jointParameters
void PhysX3DoubleAxisHingeJoint::setParameters( ConstraintParameters& jointParameters ) {
  DoubleAxisHingeJointParameters* params = static_cast <DoubleAxisHingeJointParameters*> (&jointParameters);
  PxD6Joint* d6Joint = static_cast<PxD6Joint*>(joint);

  // Anchor point
  if( params->haveAnchorPoint() ) {
    setJointFrame( params->getAnchorPoint(), Rotation( getJointFrame().getRotationPart() ) );
  }

  if( params->haveAxis1() || params->haveAxis2() ) {
    // Axis 1
    if( params->haveAxis1() ) {
      axis1 = params->getAxis1();
      axis1.normalizeSafe();
    }

    // Axis 2
    if( params->haveAxis2() ) {
      axis2 = params->getAxis2();
      axis2.normalizeSafe();
    }

    // Update axes
    makeAxesPerpendicular();
    setJointFrame(
      getJointFrame().getTranslationPart(),
      Rotation( Matrix3f( axis1.x, axis2.x, axis3.x,
                          axis1.y, axis2.y, axis3.y,
                          axis1.z, axis2.z, axis3.z ) ) );
  }

  // Limit max angle 1
  if( params->haveMaxAngle1() || params->haveMinAngle1() ) {
#ifdef PX_PHYSICS_3_3_OR_LATER
    PxJointAngularLimitPair limit = d6Joint->getTwistLimit();
#else
    PxJointLimitPair limit = d6Joint->getTwistLimit();
#endif

    limit.upper = params->getMaxAngle1();
    limit.lower = params->getMinAngle1();
    d6Joint->setTwistLimit( limit );
    d6Joint->setMotion( PxD6Axis::eTWIST, (limit.upper > limit.lower) ? PxD6Motion::eLIMITED : PxD6Motion::eLOCKED );
  }

  // Stop bounce
  if( params->haveStopBounce1() ) {
#ifdef PX_PHYSICS_3_3_OR_LATER
    PxJointAngularLimitPair limit = d6Joint->getTwistLimit();
#else
    PxJointLimitPair limit = d6Joint->getTwistLimit();
#endif
    limit.restitution = params->getStopBounce1();
    d6Joint->setTwistLimit( limit );
  }

  if( params->haveEngineOptions() ) {
    if( PhysX3JointParameters* options = dynamic_cast<PhysX3JointParameters*>(params->getEngineOptions()) ) {

      // Projection for all locked dofs
      if( options->haveProjectionTolerance() ) {
        float linear_tolerance = options->getProjectionTolerance()[0];
        float angular_tolerance = options->getProjectionTolerance()[1];
        if( linear_tolerance > 0.0 && angular_tolerance > 0.0 ) {
          d6Joint->setProjectionLinearTolerance( PxReal( linear_tolerance ) );
          d6Joint->setProjectionAngularTolerance( PxReal( angular_tolerance ) );
        }
      }

      if( options->haveConstraintFlag() || options->haveEnabledProjection() ) {
        string constraint_flag = options->getConstraintFlag();
        if( constraint_flag == "ePROJECTION" ) {
          d6Joint->setConstraintFlag( PxConstraintFlag::ePROJECTION, options->getEnabledProjection() );
        } else if( constraint_flag == "ePROJECT_TO_ACTOR0" ) {
          d6Joint->setConstraintFlag( PxConstraintFlag::ePROJECT_TO_ACTOR0, options->getEnabledProjection() );
        } else if( constraint_flag == "ePROJECT_TO_ACTOR1" ) {
          d6Joint->setConstraintFlag( PxConstraintFlag::ePROJECT_TO_ACTOR1, options->getEnabledProjection() );
        }
      }
    }
  }

}

// Modify the jointParameters argument to reflect the bullet joint's current parameters
void PhysX3DoubleAxisHingeJoint::getParameters( ConstraintParameters& jointParameters ) {
  DoubleAxisHingeJointParameters* params = static_cast <DoubleAxisHingeJointParameters*> (&jointParameters);

  // Body 1 anchor point
  if( params->haveBody1AnchorPoint() ) {
    params->setBody1AnchorPoint( getJointFrame( Body::B ).getTranslationPart() );
  }

  // Body 2 anchor point
  if( params->haveBody2AnchorPoint() ) {
    params->setBody2AnchorPoint( getJointFrame( Body::A ).getTranslationPart() );
  }

  // Hinge 1
  if( params->haveHinge1Angle() || params->haveHinge1AngleRate() ) {
    H3DFloat angle = getRotation( Axis::X );

    // Angle
    if( params->haveHinge1Angle() ) {
      params->setHinge1Angle( angle );
    }

    // Angle rate
    if( params->haveHinge1AngleRate() ) {
      hinge1AngleRate.setCurrentValue( angle );
      params->setHinge1AngleRate( hinge1AngleRate );
    }
  }

  // Hinge 2
  if( params->haveHinge2Angle() || params->haveHinge2AngleRate() ) {
    H3DFloat angle = getRotation( Axis::Y, Body::B );

    // Angle
    if( params->haveHinge2Angle() ) {
      params->setHinge2Angle( angle );
    }

    // Angle rate
    if( params->haveHinge2AngleRate() ) {
      hinge2AngleRate.setCurrentValue( angle );
      params->setHinge2AngleRate( hinge2AngleRate );
    }
  }
}


// Universal joint
//

PhysX3UniversalJoint::JointDatabase
PhysX3UniversalJoint::database( "UniversalJoint", &newInstance<PhysX3UniversalJoint> );

PhysX3UniversalJoint::PhysX3UniversalJoint( ConstraintParameters& jointParameters ) :
  PhysX36DoFJoint( jointParameters ) {
  PhysX3Callbacks::PhysX3SpecificData *physx_data =
    static_cast<PhysX3Callbacks::PhysX3SpecificData *>(jointParameters.getEngine()->getEngineSpecificData());
  UniversalJointParameters* params = static_cast <UniversalJointParameters*> (&jointParameters);

  PxD6Joint* d6Joint = static_cast<PxD6Joint*>(joint);

  // eTWIST is x, eSWING1 is y, eSWING2 is z axis in the joints' local coordinate system in PhysX3.
  // If any dof will be unlocked, first choice is eTWIST due to optimization at the PhysX3 side.
  // By setting axis1 & axis2, the joints' local coordinate system is transformed to the coordinate
  // system formed by the axis1 & axis2.
  // Therefore axis1 becomes eTWIST, axis2 becomes eSWING1.
  d6Joint->setMotion( PxD6Axis::eTWIST, PxD6Motion::eFREE );
  d6Joint->setMotion( PxD6Axis::eSWING1, PxD6Motion::eFREE );

  setParameters( jointParameters );
}

// Set the bullet joint's parameters using the parameter values specified in jointParameters
void PhysX3UniversalJoint::setParameters( ConstraintParameters& jointParameters ) {
  UniversalJointParameters* params = static_cast <UniversalJointParameters*> (&jointParameters);
  PxD6Joint* d6Joint = static_cast<PxD6Joint*>(joint);

  // Anchor point
  if( params->haveAnchorPoint() ) {
    setJointFrame( params->getAnchorPoint(), Rotation( getJointFrame().getRotationPart() ) );
  }

  if( params->haveAxis1() || params->haveAxis2() ) {
    // Axis 1
    if( params->haveAxis1() ) {
      axis1 = params->getAxis1();
      axis1.normalizeSafe();
    }

    // Axis 2
    if( params->haveAxis2() ) {
      axis2 = params->getAxis2();
      axis2.normalizeSafe();
    }

    // Update axes
    makeAxesPerpendicular();
    setJointFrame(
      getJointFrame().getTranslationPart(),
      Rotation( Matrix3f( axis1.x, axis2.x, axis3.x,
                          axis1.y, axis2.y, axis3.y,
                          axis1.z, axis2.z, axis3.z ) ) );
  }

  // Stop 1 bounce
  if( params->haveStop1Bounce() ) {
#ifdef PX_PHYSICS_3_3_OR_LATER
    PxJointAngularLimitPair limit = d6Joint->getTwistLimit();
#else
    PxJointLimitPair limit = d6Joint->getTwistLimit();
#endif
    limit.restitution = params->getStop1Bounce();
    d6Joint->setTwistLimit( limit );
  }

  // Stop 2 bounce
  if( params->haveStop2Bounce() ) {
#ifdef PX_PHYSICS_3_3_OR_LATER
    PxJointAngularLimitPair limit = d6Joint->getTwistLimit();
#else
    PxJointLimitPair limit = d6Joint->getTwistLimit();
#endif
    limit.restitution = params->getStop2Bounce();
    d6Joint->setTwistLimit( limit );
  }

  if( params->haveEngineOptions() ) {
    if( PhysX3JointParameters* options = dynamic_cast<PhysX3JointParameters*>(params->getEngineOptions()) ) {

      // Projection for all locked dofs
      if( options->haveProjectionTolerance() ) {
        float linear_tolerance = options->getProjectionTolerance()[0];
        float angular_tolerance = options->getProjectionTolerance()[1];
        if( linear_tolerance > 0.0 && angular_tolerance > 0.0 ) {
          d6Joint->setProjectionLinearTolerance( PxReal( linear_tolerance ) );
          d6Joint->setProjectionAngularTolerance( PxReal( angular_tolerance ) );
        }
      }

      if( options->haveConstraintFlag() || options->haveEnabledProjection() ) {
        string constraint_flag = options->getConstraintFlag();
        if( constraint_flag == "ePROJECTION" ) {
          d6Joint->setConstraintFlag( PxConstraintFlag::ePROJECTION, options->getEnabledProjection() );
        } else if( constraint_flag == "ePROJECT_TO_ACTOR0" ) {
          d6Joint->setConstraintFlag( PxConstraintFlag::ePROJECT_TO_ACTOR0, options->getEnabledProjection() );
        } else if( constraint_flag == "ePROJECT_TO_ACTOR1" ) {
          d6Joint->setConstraintFlag( PxConstraintFlag::ePROJECT_TO_ACTOR1, options->getEnabledProjection() );
        }
      }
    }
  }

}

// Modify the jointParameters argument to reflect the bullet joint's current parameters
void PhysX3UniversalJoint::getParameters( ConstraintParameters& jointParameters ) {
  UniversalJointParameters* params = static_cast <UniversalJointParameters*> (&jointParameters);

  // Body 1 anchor point
  if( params->haveBody1AnchorPoint() ) {
    params->setBody1AnchorPoint( getJointFrame( Body::B ).getTranslationPart() );
  }

  // Body 2 anchor point
  if( params->haveBody2AnchorPoint() ) {
    params->setBody2AnchorPoint( getJointFrame( Body::A ).getTranslationPart() );
  }

  // body1Axis output
  if( params->haveBody1Axis() ) {
    params->setBody1Axis( getJointFrame( Body::A ).getRotationPart()*Vec3f( 1, 0, 0 ) );
  }

  // body2Axis output
  if( params->haveBody2Axis() ) {
    params->setBody2Axis( getJointFrame( Body::B ).getRotationPart()*Vec3f( 0, 1, 0 ) );
  }
}


// Slider joint
//

PhysX3SliderJoint::JointDatabase
PhysX3SliderJoint::database( "SliderJoint", &newInstance<PhysX3SliderJoint> );

PhysX3SliderJoint::PhysX3SliderJoint( ConstraintParameters& jointParameters ) :
  PhysX3Joint( jointParameters ),
  simulationCallbackId( -1 ),
  physicsEngineThread( jointParameters.getEngine() ),
  initialized( false ),
  slider_force_mode( PxForceMode::eFORCE ),
  body2_slider_force_scale( 1.0 ) {
  PhysX3Callbacks::PhysX3SpecificData *physx_data =
    static_cast<PhysX3Callbacks::PhysX3SpecificData *>(jointParameters.getEngine()->getEngineSpecificData());
  SliderJointParameters* params = static_cast <SliderJointParameters*> (&jointParameters);

  PhysX3RigidBody* body2 = (PhysX3RigidBody*)params->getBody1();
  PhysX3RigidBody* body1 = (PhysX3RigidBody*)params->getBody2();

  joint = PxPrismaticJointCreate(
    *physx_data->sdk,
    body1 ? body1->getActor() : NULL, PxTransform::createIdentity(),
    body2 ? body2->getActor() : NULL, PxTransform::createIdentity() );

  setParameters( jointParameters );
  initialized = true;
}

PhysX3SliderJoint::~PhysX3SliderJoint() {
  remove();
}

// Set the bullet joint's parameters using the parameter values specified in jointParameters
void PhysX3SliderJoint::setParameters( ConstraintParameters& jointParameters ) {
  SliderJointParameters* params = static_cast <SliderJointParameters*> (&jointParameters);
  PxPrismaticJoint* prismaticJoint = static_cast<PxPrismaticJoint*>(joint);

  // Axis
  if( params->haveAxis() ) {
    Vec3f axis = params->getAxis();
    axis.normalizeSafe();

    // Anchor point is in between body 1 and body 2
    PxRigidActor* actor1, *actor2;
    joint->getActors( actor1, actor2 );
    Vec3f anchorPoint;

    bool use_explicit_anchor_point = false;
    bool have_offset_1 = false;
    bool have_offset_2 = false;
    Vec3f offset_1;
    Vec3f offset_2;

    // First time the joint is added. Use mid point of the bodies as anchor point
    // else check the engine options.
    if( initialized && params->haveEngineOptions() ) {
      if( PhysX3SliderJointParameters* options =
          dynamic_cast<PhysX3SliderJointParameters*>(params->getEngineOptions()) ) {

        if( options->haveExplicitAnchorPoint() ) {
          use_explicit_anchor_point = true;
          anchorPoint = options->getExplicitAnchorPoint();
        }

        if( options->haveBody1Offset() ) {
          have_offset_1 = true;
          offset_1 = options->getBody1Offset();
        }

        if( options->haveBody2Offset() ) {
          have_offset_2 = true;
          offset_2 = options->getBody2Offset();
        }

      }
    }

    if( !use_explicit_anchor_point ) {
      if( actor1 ) {
        anchorPoint = toVec3f( actor1->getGlobalPose().p );
      }
      if( actor2 ) {
        anchorPoint += toVec3f( actor2->getGlobalPose().p );
      }
      if( actor1 && actor2 ) {
        anchorPoint /= 2;
      }
    }

    setJointFrame( anchorPoint, Rotation( Vec3f( 1, 0, 0 ), axis ) );

    // WARNING: body1/body2 are reversed in the constructor. body1 in scene graph is body2 here.
    if( have_offset_1 ) {

      Vec3f ideal_rb_pos = toVec3f( actor2->getGlobalPose().p ) + offset_1;
      Vec3f ideal_anchor_point_global = anchorPoint - ideal_rb_pos;
      PxVec3 ideal_anchor_point_local = actor2->getGlobalPose().rotateInv( toPxVec3( ideal_anchor_point_global ) );

      if( actor2 ) {
        PxTransform localFrame = joint->getLocalPose( PxJointActorIndex::eACTOR1 );
        localFrame = PxTransform( ideal_anchor_point_local, localFrame.q );
        joint->setLocalPose( PxJointActorIndex::eACTOR1, localFrame );
      }

    }

    if( have_offset_2 ) {

      Vec3f ideal_rb_pos = toVec3f( actor1->getGlobalPose().p ) + offset_2;
      Vec3f ideal_anchor_point_global = anchorPoint - ideal_rb_pos;
      PxVec3 ideal_anchor_point_local = actor1->getGlobalPose().rotateInv( toPxVec3( ideal_anchor_point_global ) );

      if( actor1 ) {
        PxTransform localFrame = joint->getLocalPose( PxJointActorIndex::eACTOR0 );
        localFrame = PxTransform( ideal_anchor_point_local, localFrame.q );
        joint->setLocalPose( PxJointActorIndex::eACTOR0, localFrame );
      }
    }

  }

  if( params->haveEngineOptions() ) {

    if( PhysX3SliderJointParameters* options =
        dynamic_cast<PhysX3SliderJointParameters*>(params->getEngineOptions()) ) {

      if( options->haveBody2ForceScale() ) {
        body2_slider_force_scale = options->getBody2ForceScale();
      }

      if( options->haveForceType() ) {
        string ft = options->getForceType();
        if( ft == "eIMPULSE" ) {
          slider_force_mode = PxForceMode::eIMPULSE;
        } else if( ft == "eVELOCITY_CHANGE" ) {
          slider_force_mode = PxForceMode::eVELOCITY_CHANGE;
        } else if( ft == "eACCELERATION" ) {
          slider_force_mode = PxForceMode::eACCELERATION;
        } else
          slider_force_mode = PxForceMode::eFORCE;
      }
      if( options->haveProjectionTolerance() ) {
        float linear_tolerance = options->getProjectionTolerance()[0];
        float angular_tolerance = options->getProjectionTolerance()[1];
        if( linear_tolerance > 0.0 && angular_tolerance > 0.0 ) {
          prismaticJoint->setProjectionLinearTolerance( PxReal( linear_tolerance ) );
          prismaticJoint->setProjectionAngularTolerance( PxReal( angular_tolerance ) );
        }
      }

      if( options->haveConstraintFlag() || options->haveEnabledProjection() ) {
        string constraint_flag = options->getConstraintFlag();
        if( constraint_flag == "ePROJECTION" ) {
          prismaticJoint->setConstraintFlag( PxConstraintFlag::ePROJECTION, options->getEnabledProjection() );
        } else if( constraint_flag == "ePROJECT_TO_ACTOR0" ) {
          prismaticJoint->setConstraintFlag( PxConstraintFlag::ePROJECT_TO_ACTOR0, options->getEnabledProjection() );
        } else if( constraint_flag == "ePROJECT_TO_ACTOR1" ) {
          prismaticJoint->setConstraintFlag( PxConstraintFlag::ePROJECT_TO_ACTOR1, options->getEnabledProjection() );
        }
      }
    }
  }

  if( params->haveMaxSeparation() || params->haveMinSeparation() ) {
#ifdef PX_PHYSICS_3_3_OR_LATER
    PxJointLinearLimitPair limit = prismaticJoint->getLimit();
#else
    PxJointLimitPair limit = prismaticJoint->getLimit();
#endif

    limit.upper = params->getMaxSeparation();
    limit.lower = params->getMinSeparation();

    if( limit.upper > limit.lower ) {
      prismaticJoint->setLimit( limit );
    }

    prismaticJoint->setPrismaticJointFlag( PxPrismaticJointFlag::eLIMIT_ENABLED, limit.upper > limit.lower );
  }

  if( params->haveSliderForce() ) {
    slider_force = params->getSliderForce();
    if( simulationCallbackId < 0 ) {
      if( slider_force != 0 )
        simulationCallbackId = params->getEngine()->asynchronousCallback( &PhysX3SliderJoint::updateSlider, this );
    } else if( slider_force == 0 ) {
      params->getEngine()->removeAsynchronousCallbackNoLock( simulationCallbackId );
      simulationCallbackId = -1;
    }
  }// else if( simulationCallbackId >= 0 ) {
   // params->getEngine()->removeAsynchronousCallbackNoLock( simulationCallbackId );
   // simulationCallbackId = -1;
  //}
}

// Modify the jointParameters argument to reflect the bullet joint's current parameters
void PhysX3SliderJoint::getParameters( ConstraintParameters& jointParameters ) {
  SliderJointParameters* params = static_cast <SliderJointParameters*> (&jointParameters);

  if( params->haveSeparation() ) {

    Vec3f diff = getJointFrame( Body::B ).getTranslationPart() - getJointFrame( Body::A ).getTranslationPart();
    H3DFloat sep = -diff.length();

    Vec3f globalAxis = getJointFrame( Body::A ).getRotationPart()*Vec3f( 1, 0, 0 );
    if( diff.dotProduct( globalAxis ) < 0 ) {
      sep = -sep;
    }

    params->setSeparation( sep );
  }
}

// Callback used to update slider simulation
H3DUtil::PeriodicThread::CallbackCode PhysX3SliderJoint::updateSlider( void *data ) {
  static_cast <PhysX3SliderJoint*> (data)->update();
  return H3DUtil::PeriodicThread::CALLBACK_CONTINUE;
}

// Member function called by static callback, used to update motor simulation
void PhysX3SliderJoint::update() {
  PxVec3 the_force( slider_force, 0, 0 );
  PxRigidActor* actor1, *actor2;
  joint->getActors( actor1, actor2 );
  // WARNING: body1/body2 are reversed in the constructor. body1 in scene graph is body2 here.
  if( actor1 ) {
    PxTransform localFrame = joint->getLocalPose( PxJointActorIndex::eACTOR0 );
    PxTransform globalFrame = localFrame;
    globalFrame = actor1->getGlobalPose() * globalFrame;
    static_cast<PxRigidDynamic *>(actor1)->addForce( globalFrame.q.rotate( -the_force * body2_slider_force_scale ), slider_force_mode );
  }
  if( actor2 ) {
    PxTransform localFrame = joint->getLocalPose( PxJointActorIndex::eACTOR1 );
    PxTransform globalFrame = localFrame;
    globalFrame = actor2->getGlobalPose() * globalFrame;
    static_cast<PxRigidDynamic *>(actor2)->addForce( globalFrame.q.rotate( the_force ), slider_force_mode );
  }

}

// Remove joint from simulation
// Override to remove simulation callback
void PhysX3SliderJoint::remove() {
  if( simulationCallbackId >= 0 ) {
    physicsEngineThread->removeAsynchronousCallbackNoLock( simulationCallbackId );
    simulationCallbackId = -1;
  }
}

// Distance joint
//

PhysX3DistanceJoint::JointDatabase
PhysX3DistanceJoint::database( "DistanceJoint", &newInstance<PhysX3DistanceJoint> );

PhysX3DistanceJoint::PhysX3DistanceJoint( ConstraintParameters& jointParameters ) :
  PhysX3Joint( jointParameters ) {
  PhysX3Callbacks::PhysX3SpecificData *physx_data =
    static_cast<PhysX3Callbacks::PhysX3SpecificData *>(jointParameters.getEngine()->getEngineSpecificData());
  DistanceJointParameters* params = static_cast <DistanceJointParameters*> (&jointParameters);

  PhysX3RigidBody* body1 = (PhysX3RigidBody*)params->getBody1();
  PhysX3RigidBody* body2 = (PhysX3RigidBody*)params->getBody2();

  joint = PxDistanceJointCreate(
    *physx_data->sdk,
    body1 ? body1->getActor() : NULL, PxTransform::createIdentity(),
    body2 ? body2->getActor() : NULL, PxTransform::createIdentity() );

  setParameters( jointParameters );
}

// Set the distance joint's parameters using the parameter values specified in jointParameters
void PhysX3DistanceJoint::setParameters( ConstraintParameters& jointParameters ) {
  DistanceJointParameters* params = static_cast <DistanceJointParameters*> (&jointParameters);
  PxDistanceJoint* distanceJoint = static_cast<PxDistanceJoint*>(joint);
  PxDistanceJointFlags distance_joint_flags = distanceJoint->getDistanceJointFlags();

  // Max distance
  if( params->haveMaxDistance() ) {
    if( params->getMaxDistance() >= 0 ) {
      distanceJoint->setMaxDistance( params->getMaxDistance() );
      distance_joint_flags = distance_joint_flags | PxDistanceJointFlag::eMAX_DISTANCE_ENABLED;
    } else {
      distance_joint_flags = distance_joint_flags & ~PxDistanceJointFlag::eMAX_DISTANCE_ENABLED;
    }
  }

  // Min distance
  if( params->haveMinDistance() ) {
    H3DFloat min_distance = params->getMinDistance();
    if( min_distance >= 0 ) {
      if( min_distance > distanceJoint->getMaxDistance() ) {
        min_distance = distanceJoint->getMaxDistance();
      }
      distanceJoint->setMinDistance( min_distance );
      distance_joint_flags = distance_joint_flags | PxDistanceJointFlag::eMIN_DISTANCE_ENABLED;
    } else {
      distance_joint_flags = distance_joint_flags & ~PxDistanceJointFlag::eMIN_DISTANCE_ENABLED;
    }
  }

  // Stiffness
  if( params->haveStiffness() ) {
    if( params->getStiffness() >= 0 ) {
#ifdef PX_PHYSICS_3_3_OR_LATER
      distanceJoint->setStiffness( params->getStiffness() );
#else
      distanceJoint->setSpring( params->getStiffness() );
#endif
      distance_joint_flags = distance_joint_flags | PxDistanceJointFlag::eSPRING_ENABLED;
    } else {
      distance_joint_flags = distance_joint_flags & ~PxDistanceJointFlag::eSPRING_ENABLED;
    }
  }

  // Damping
  if( params->haveDamping() ) {
    if( params->getDamping() >= 0 ) {
      distanceJoint->setDamping( params->getDamping() );
    }
  }

  if( params->haveTolerance() ) {
    distanceJoint->setTolerance( params->getTolerance() );
  }

  distanceJoint->setDistanceJointFlags( distance_joint_flags );
}

// Modify the jointParameters argument to reflect the bullet joint's current parameters
void PhysX3DistanceJoint::getParameters( ConstraintParameters& jointParameters ) {
  DistanceJointParameters* params = static_cast <DistanceJointParameters*> (&jointParameters);
  PxDistanceJoint* distanceJoint = static_cast<PxDistanceJoint*>(joint);

  // distance
  if( params->haveDistance() ) {
    // After version 3.3.1 getDistance 
#if ( PX_PHYSICS_VERSION_MAJOR >= 3 ) && ( PX_PHYSICS_VERSION_MINOR >= 3 ) && ( PX_PHYSICS_VERSION_BUGFIX >= 1 )
    // I (Markus) can't find a notice about this in the documentation but the reported value
    // by physx3 is the squared distance.
    params->setDistance( H3DSqrt( distanceJoint->getDistance() ) );
#endif
  }

}

// Generic6DOFJoint
//

PhysX3Generic6DofJoint::JointDatabase
PhysX3Generic6DofJoint::database( "Generic6DOFJoint", &newInstance<PhysX3Generic6DofJoint> );

PhysX3Generic6DofJoint::PhysX3Generic6DofJoint( ConstraintParameters& jointParameters ) :
  PhysX36DoFJoint( jointParameters ),
  l_limit( PxJointLinearLimit( (PxReal)0.1, PxSpring( 100, 0 ) ) ),
  c_limit( PxJointLimitCone( (PxReal)0.1, (PxReal)0.1, PxSpring( 100, 0 ) ) ),
  t_limit( PxJointAngularLimitPair( 0, (PxReal)1.8, PxSpring( 100, 0 ) ) ) {

  PxD6Joint* d6Joint = static_cast<PxD6Joint*>(joint);

  // eTWIST is x, eSWING1 is y, eSWING2 is z axis in the joints' local coordinate system in PhysX3.
  // If any dof will be unlocked, first choice is eTWIST due to optimization at the PhysX3 side.
  // By setting axis1 & axis2, the joints' local coordinate system is transformed to the coordinate
  // system formed by the axis1 & axis2.
  // Therefore axis1 becomes eTWIST, axis2 becomes eSWING1.
  //d6Joint->setMotion ( PxD6Axis::eTWIST, PxD6Motion::eFREE );
  //d6Joint->setMotion ( PxD6Axis::eSWING1, PxD6Motion::eFREE );

  setParameters( jointParameters );
}

// Set the distance joint's parameters using the parameter values specified in jointParameters
void PhysX3Generic6DofJoint::setParameters( ConstraintParameters& jointParameters ) {
  Generic6DOFJointParameters* generic6DOFJointParameters = static_cast <Generic6DOFJointParameters*> (&jointParameters);
  PxD6Joint* d6Joint = static_cast<PxD6Joint*>(joint);


  // set anchorPoint
  if( generic6DOFJointParameters->haveAnchorPoint() ) {
    anchorPoint = generic6DOFJointParameters->getAnchorPoint();
    setJointFrame( anchorPoint, Rotation( getJointFrame().getRotationPart() ) );
  }

  // Axis3 should be changed through axis1 and axis2
  if( generic6DOFJointParameters->haveAxis1() || generic6DOFJointParameters->haveAxis2() ) {

    // Axis 1
    if( generic6DOFJointParameters->haveAxis1() ) {
      axis1 = generic6DOFJointParameters->getAxis1();
      axis1.normalizeSafe();
    }

    // Axis 2
    if( generic6DOFJointParameters->haveAxis2() ) {
      axis2 = generic6DOFJointParameters->getAxis2();
      axis2.normalizeSafe();
    }

    // Update axes
    makeAxesPerpendicular();
    setJointFrame(
      getJointFrame().getTranslationPart(),
      Rotation( Matrix3f( axis1.x, axis2.x, axis3.x,
                          axis1.y, axis2.y, axis3.y,
                          axis1.z, axis2.z, axis3.z ) ) );
  }

  // maxTorque1
  if( generic6DOFJointParameters->haveMaxTorque1() ) {
    //generic6DOFJoint->getRotationalLimitMotor(0)->m_maxMotorForce= generic6DOFJointParameters->getMaxTorque1()*bullet_data->m_worldScale*bullet_data->m_worldScale;
  }

  // maxTorque2
  if( generic6DOFJointParameters->haveMaxTorque2() ) {
    //generic6DOFJoint->getRotationalLimitMotor(1)->m_maxMotorForce= generic6DOFJointParameters->getMaxTorque2()*bullet_data->m_worldScale*bullet_data->m_worldScale;
  }

  // maxTorque3
  if( generic6DOFJointParameters->haveMaxTorque3() ) {
    //generic6DOFJoint->getRotationalLimitMotor(2)->m_maxMotorForce= generic6DOFJointParameters->getMaxTorque3()*bullet_data->m_worldScale*bullet_data->m_worldScale;
  }

  // maxForce1
  if( generic6DOFJointParameters->haveMaxForce1() ) {
    //generic6DOFJoint->getTranslationalLimitMotor()->m_maxMotorForce[0]= generic6DOFJointParameters->getMaxForce1();
  }

  // maxForce2
  if( generic6DOFJointParameters->haveMaxForce2() ) {
    //generic6DOFJoint->getTranslationalLimitMotor()->m_maxMotorForce[1]= generic6DOFJointParameters->getMaxForce2();
  }

  // maxForce3
  if( generic6DOFJointParameters->haveMaxForce3() ) {
    //generic6DOFJoint->getTranslationalLimitMotor()->m_maxMotorForce[2]= generic6DOFJointParameters->getMaxForce3();
  }

  // desiredAngularVelocity1
  // Enable or disable rotational motor on generic 6dof joint based on velocity value
  if( generic6DOFJointParameters->haveDesiredAngularVelocity1() ) {
    //generic6DOFJoint->getRotationalLimitMotor(0)->m_targetVelocity= generic6DOFJointParameters->getDesiredAngularVelocity1();
    //generic6DOFJoint->getRotationalLimitMotor(0)->m_enableMotor= abs(generic6DOFJointParameters->getDesiredAngularVelocity1()) > Constants::f_epsilon;
  }

  // desiredAngularVelocity2
  // Enable or disable rotational motor on generic 6dof joint based on velocity value
  if( generic6DOFJointParameters->haveDesiredAngularVelocity2() ) {
    //generic6DOFJoint->getRotationalLimitMotor(1)->m_targetVelocity= generic6DOFJointParameters->getDesiredAngularVelocity2();
    //generic6DOFJoint->getRotationalLimitMotor(1)->m_enableMotor= abs(generic6DOFJointParameters->getDesiredAngularVelocity2()) > Constants::f_epsilon;
  }

  // desiredAngularVelocity3
  // Enable or disable rotational motor on generic 6dof joint based on velocity value
  if( generic6DOFJointParameters->haveDesiredAngularVelocity3() ) {
    //generic6DOFJoint->getRotationalLimitMotor(2)->m_targetVelocity= generic6DOFJointParameters->getDesiredAngularVelocity3();
    //generic6DOFJoint->getRotationalLimitMotor(2)->m_enableMotor= abs(generic6DOFJointParameters->getDesiredAngularVelocity3()) > Constants::f_epsilon;
  }

  // desiredLinearVelocity1
  // Enable or disable rotational motor on generic 6dof joint based on velocity value
  if( generic6DOFJointParameters->haveDesiredLinearVelocity1() ) {
    //generic6DOFJoint->getTranslationalLimitMotor()->m_targetVelocity[0]= generic6DOFJointParameters->getDesiredLinearVelocity1();
    //generic6DOFJoint->getTranslationalLimitMotor()->m_enableMotor[0]= abs(generic6DOFJointParameters->getDesiredLinearVelocity1()) > Constants::f_epsilon;
  }

  // desiredLinearVelocity2
  // Enable or disable rotational motor on generic 6dof joint based on velocity value
  if( generic6DOFJointParameters->haveDesiredLinearVelocity2() ) {
    //generic6DOFJoint->getTranslationalLimitMotor()->m_targetVelocity[1]= generic6DOFJointParameters->getDesiredLinearVelocity2();
    //generic6DOFJoint->getTranslationalLimitMotor()->m_enableMotor[1]= abs(generic6DOFJointParameters->getDesiredLinearVelocity2()) > Constants::f_epsilon;
  }

  // desiredLinearVelocity3
  // Enable or disable rotational motor on generic 6dof joint based on velocity value
  if( generic6DOFJointParameters->haveDesiredLinearVelocity3() ) {
    //generic6DOFJoint->getTranslationalLimitMotor()->m_targetVelocity[2]= generic6DOFJointParameters->getDesiredLinearVelocity3();
    //generic6DOFJoint->getTranslationalLimitMotor()->m_enableMotor[2]= abs(generic6DOFJointParameters->getDesiredLinearVelocity3()) > Constants::f_epsilon;
  }

  // max limits
  if( generic6DOFJointParameters->haveMaxLimit1() ) {
    maxLimit1 = generic6DOFJointParameters->getMaxLimit1();
  }
  if( generic6DOFJointParameters->haveMaxLimit2() ) {
    maxLimit2 = generic6DOFJointParameters->getMaxLimit2();
  }
  if( generic6DOFJointParameters->haveMaxLimit3() ) {
    maxLimit3 = generic6DOFJointParameters->getMaxLimit3();
  }

  // min limits
  if( generic6DOFJointParameters->haveMinLimit1() ) {
    minLimit1 = generic6DOFJointParameters->getMinLimit1();
  }
  if( generic6DOFJointParameters->haveMinLimit2() ) {
    minLimit2 = generic6DOFJointParameters->getMinLimit2();
  }
  if( generic6DOFJointParameters->haveMinLimit3() ) {
    minLimit3 = generic6DOFJointParameters->getMinLimit3();
  }

  // max angles
  if( generic6DOFJointParameters->haveMaxAngle1() ) {
    maxAngle1 = generic6DOFJointParameters->getMaxAngle1();
  }
  if( generic6DOFJointParameters->haveMaxAngle2() ) {
    maxAngle2 = generic6DOFJointParameters->getMaxAngle2();
  }
  if( generic6DOFJointParameters->haveMaxAngle3() ) {
    maxAngle3 = generic6DOFJointParameters->getMaxAngle3();
  }

  // min angles
  if( generic6DOFJointParameters->haveMinAngle1() ) {
    minAngle1 = generic6DOFJointParameters->getMinAngle1();
  }
  if( generic6DOFJointParameters->haveMinAngle2() ) {
    minAngle2 = generic6DOFJointParameters->getMinAngle2();
  }
  if( generic6DOFJointParameters->haveMinAngle3() ) {
    minAngle3 = generic6DOFJointParameters->getMinAngle3();
  }

  // Limitations of limits in PhysX3: v3.3.1:
  //   - There is only one linear limit for all three axis.
  //   - setLinearLimit, gets PxJointLinearLimit, not PxJointLinearLimitPair
  //
  // Current implementation:
  //   - If max > min limit the axis, if max==min lock the axis, if max<min
  //     let the axis free
  //
  //   - The limit of the linear axis is axis1(if locked) else
  //     axis2(if locked) else axis3
  //     

  bool l_spring_updated = false;
  bool a_spring_updated = false;
  if( generic6DOFJointParameters->haveEngineOptions() ) {
    if( PhysX3Joint6DOFLimitParameters* options =
        dynamic_cast<PhysX3Joint6DOFLimitParameters*>(generic6DOFJointParameters->getEngineOptions()) ) {

      if( options->haveLinearSpring() ) {
        l_limit.stiffness = options->getLinearSpring().x;
        l_limit.damping = options->getLinearSpring().y;
        l_spring_updated = true;
      }

      if( options->haveAngularSpring() ) {
        c_limit.stiffness = options->getAngularSpring().x;
        c_limit.damping = options->getAngularSpring().y;
        t_limit.stiffness = options->getAngularSpring().x;
        t_limit.damping = options->getAngularSpring().y;
        a_spring_updated = true;
      }

      if( options->haveProjectionTolerance() ) {
        float linear_tolerance = options->getProjectionTolerance()[0];
        float angular_tolerance = options->getProjectionTolerance()[1];
        if( linear_tolerance > 0.0 ) {
          d6Joint->setProjectionLinearTolerance( PxReal( linear_tolerance ) );
          d6Joint->setProjectionAngularTolerance( PxReal( angular_tolerance ) );
        }
      }

      if( options->haveConstraintFlag() || options->haveEnabledProjection() ) {
        string constraint_flag = options->getConstraintFlag();
        if( constraint_flag == "ePROJECTION" ) {
          d6Joint->setConstraintFlag( PxConstraintFlag::ePROJECTION, options->getEnabledProjection() );
        } else if( constraint_flag == "ePROJECT_TO_ACTOR0" ) {
          d6Joint->setConstraintFlag( PxConstraintFlag::ePROJECT_TO_ACTOR0, options->getEnabledProjection() );
        } else if( constraint_flag == "ePROJECT_TO_ACTOR1" ) {
          d6Joint->setConstraintFlag( PxConstraintFlag::ePROJECT_TO_ACTOR1, options->getEnabledProjection() );
        }
      }
    }
  }


  // Axis1 - linear
  if( generic6DOFJointParameters->haveMinLimit1() ||
      generic6DOFJointParameters->haveMaxLimit1() ) {

    if( maxLimit1 == minLimit1 ) {
      d6Joint->setMotion( PxD6Axis::eX, PxD6Motion::eLOCKED );
      //Console(4) << "Trans1 locked " << endl; 
    } else if( maxLimit1 > minLimit1 ) {
      d6Joint->setMotion( PxD6Axis::eX, PxD6Motion::eLIMITED );
      l_limit.value = maxLimit1 - minLimit1;
      //Console(4) << "Trans1 limited " << endl; 
    } else if( maxLimit1 < minLimit1 ) {
      d6Joint->setMotion( PxD6Axis::eX, PxD6Motion::eFREE );
      //Console(4) << "Trans1 freed " << endl; 
    }
  }

  // Axis2 - linear
  if( generic6DOFJointParameters->haveMinLimit2() ||
      generic6DOFJointParameters->haveMaxLimit2() ) {

    if( maxLimit2 == minLimit2 ) {
      //Console(4) << "Trans2 locked" << endl; 
      d6Joint->setMotion( PxD6Axis::eY, PxD6Motion::eLOCKED );
    } else if( maxLimit2 > minLimit2 ) {
      //Console(4) << "Trans2 limited" << endl; 
      d6Joint->setMotion( PxD6Axis::eY, PxD6Motion::eLIMITED );
      if( d6Joint->getMotion( PxD6Axis::eX ) != PxD6Motion::eLIMITED )
        l_limit.value = maxLimit2 - minLimit2;
    } else if( maxLimit2 < minLimit2 ) {
      //Console(4) << "Trans2 freed" << endl; 
      d6Joint->setMotion( PxD6Axis::eY, PxD6Motion::eFREE );
    }
  }

  // Axis3 - linear
  if( generic6DOFJointParameters->haveMinLimit3() ||
      generic6DOFJointParameters->haveMaxLimit3() ) {

    if( maxLimit3 == minLimit3 ) {
      //Console(4) << "Trans3 locked" << endl; 
      d6Joint->setMotion( PxD6Axis::eZ, PxD6Motion::eLOCKED );
    } else if( maxLimit3 > minLimit3 ) {
      //Console(4) << "Trans3 limited" << endl; 
      d6Joint->setMotion( PxD6Axis::eZ, PxD6Motion::eLIMITED );
      if( d6Joint->getMotion( PxD6Axis::eX ) != PxD6Motion::eLIMITED &&
          d6Joint->getMotion( PxD6Axis::eY ) != PxD6Motion::eLIMITED )
        l_limit.value = maxLimit3 - minLimit3;
    } else if( maxLimit3 < minLimit3 ) {
      //Console(4) << "Trans3 freed" << endl; 
      d6Joint->setMotion( PxD6Axis::eZ, PxD6Motion::eFREE );
    }
  }

  if( d6Joint->getMotion( PxD6Axis::eX ) == PxD6Motion::eLIMITED ||
      d6Joint->getMotion( PxD6Axis::eY ) == PxD6Motion::eLIMITED ||
      d6Joint->getMotion( PxD6Axis::eZ ) == PxD6Motion::eLIMITED ||
      l_spring_updated ) {

    //Console(4) << "Translation limit set" << endl;
    d6Joint->setLinearLimit( l_limit );

  }


  // Axis1 - angular
  if( generic6DOFJointParameters->haveMinAngle1() ||
      generic6DOFJointParameters->haveMaxAngle1() ) {

    if( maxAngle1 == minAngle1 ) {
      //Console(4) << "Rot1 locked" << endl; 
      d6Joint->setMotion( PxD6Axis::eTWIST, PxD6Motion::eLOCKED );
    } else if( maxAngle1 > minAngle1 ) {
      //Console(4) << "Rot1 limited " << minAngle1 << "  " << maxAngle1 << endl; 
      d6Joint->setMotion( PxD6Axis::eTWIST, PxD6Motion::eLIMITED );
      t_limit.lower = minAngle1;
      t_limit.upper = maxAngle1;

    } else if( maxAngle1 < minAngle1 ) {
      d6Joint->setMotion( PxD6Axis::eTWIST, PxD6Motion::eFREE );
      //Console(4) << "Rot1 freed" << endl; 
    }

  }

  if( d6Joint->getMotion( PxD6Axis::eTWIST ) == PxD6Motion::eLIMITED ||
      a_spring_updated ) {
    //Console(4) << "Angular limit set" << endl;
    d6Joint->setTwistLimit( t_limit );
  }

  // Axis2 - angular
  if( generic6DOFJointParameters->haveMinAngle2() ||
      generic6DOFJointParameters->haveMaxAngle2() ) {

    if( maxAngle2 == minAngle2 ) {
      d6Joint->setMotion( PxD6Axis::eSWING1, PxD6Motion::eLOCKED );
      //Console(4) << "Rot2 locked" << endl; 
    } else if( maxAngle2 > minAngle2 ) {
      d6Joint->setMotion( PxD6Axis::eSWING1, PxD6Motion::eLIMITED );
      c_limit.yAngle = maxAngle2 - minAngle2;
      c_limit.zAngle = maxAngle2 - minAngle2;
      //Console(4) << "Rot2 limited " << c_limit.yAngle << endl; 
    } else if( maxAngle2 < minAngle2 ) {
      d6Joint->setMotion( PxD6Axis::eSWING1, PxD6Motion::eFREE );
      //Console(4) << "Rot2 freed" << endl; 
    }

  }

  // Axis3 - angular
  if( generic6DOFJointParameters->haveMinAngle3() ||
      generic6DOFJointParameters->haveMaxAngle3() ) {

    if( maxAngle3 == minAngle3 ) {
      d6Joint->setMotion( PxD6Axis::eSWING2, PxD6Motion::eLOCKED );
      Console( 4 ) << "Rot3 locked" << endl;
    } else if( maxAngle3 > minAngle3 ) {
      d6Joint->setMotion( PxD6Axis::eSWING2, PxD6Motion::eLIMITED );
      if( d6Joint->getMotion( PxD6Axis::eSWING1 ) != PxD6Motion::eLIMITED ) {
        c_limit.yAngle = maxAngle3 - minAngle3;
        c_limit.zAngle = maxAngle3 - minAngle3;
      }
      //Console(4) << "Rot3 limited" << endl; 
    } else if( maxAngle3 < minAngle3 ) {
      d6Joint->setMotion( PxD6Axis::eSWING2, PxD6Motion::eFREE );
      //Console(4) << "Rot3 freed" << endl; 
    }

  }

  if( d6Joint->getMotion( PxD6Axis::eSWING1 ) == PxD6Motion::eLIMITED ||
      d6Joint->getMotion( PxD6Axis::eSWING2 ) == PxD6Motion::eLIMITED ||
      a_spring_updated ) {
    //Console(4) << "Swing limit set" << endl;
    d6Joint->setSwingLimit( c_limit );
  }

}

// Modify the jointParameters argument to reflect the bullet joint's current parameters
void PhysX3Generic6DofJoint::getParameters( ConstraintParameters& jointParameters ) {
  Generic6DOFJointParameters* params = static_cast <Generic6DOFJointParameters*> (&jointParameters);
  PxD6Joint* d6Joint = static_cast<PxD6Joint*>(joint);



}


#endif

