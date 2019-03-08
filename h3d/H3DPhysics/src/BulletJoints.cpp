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
/// \file BulletJoints.cpp
/// \brief Source file for BulletJoint classes, which maintain a link between 
/// RigidBodyPhysics joint types and their Bullet implementations
///
//
//////////////////////////////////////////////////////////////////////////////
#include <H3D/H3DPhysics/BulletJoints.h>
#include <H3D/H3DPhysics/BulletCallbacks.h>
#include <cmath>

using namespace H3D;

#ifdef HAVE_BULLET

BulletJoint::JointDatabase::JointMap BulletJoint::JointDatabase::map;

const H3DFloat BulletJoint::maxMotorForce = 1e5;
const H3DFloat BulletJoint::maxMotorTorque = 1e5;

BulletJoint::BulletJoint ( PhysicsEngineParameters::JointParameters& jointParameters ) :
physicsEngineThread ( jointParameters.getEngine() ),
  joint ( NULL ),
  body1 ( NULL ),
  body2 ( NULL )
{
  // Get bullet representations of rigid bod(y/ies)
  if ( BulletBody* b= (BulletBody*)jointParameters.getBody1() ) {

    if ( BulletRigidBody* rb= dynamic_cast<BulletRigidBody*>(b) ) {
      body1= &rb->getRigidBody();
    }
  }

  if ( BulletBody* b= (BulletBody*)jointParameters.getBody2() ) {

    if ( BulletRigidBody* rb= dynamic_cast<BulletRigidBody*>(b) ) {
      body2= &rb->getRigidBody();
    }
  }  
}

// Create a joint instance of the specified type name
BulletJoint* BulletJoint::createJoint ( PhysicsEngineParameters::JointParameters& jointParameters )
{
  CreateFunc f= JointDatabase::map[jointParameters.getType()];

  if ( f )
    return (*f)( jointParameters );
  else
    return NULL;    
}

// Add joint to simulation
void BulletJoint::add ()
{
  BulletCallbacks::BulletSpecificData* bullet_data= 
    static_cast<BulletCallbacks::BulletSpecificData*>(physicsEngineThread->getEngineSpecificData());

  bullet_data->m_dynamicsWorld->addConstraint( joint.get(), true);
}

// Remove joint from simulation
void BulletJoint::remove ()
{
  BulletCallbacks::BulletSpecificData* bullet_data= 
    static_cast<BulletCallbacks::BulletSpecificData*>(physicsEngineThread->getEngineSpecificData());

  bullet_data->m_dynamicsWorld->removeConstraint( joint.get() );
}

btRigidBody& BulletJoint::getFixedBody()
{
  static btRigidBody s_fixed(0, 0,0);
  s_fixed.setMassProps(btScalar(0.),btVector3(btScalar(0.),btScalar(0.),btScalar(0.)));
  return s_fixed;
}

BulletBallJoint::JointDatabase
  BulletBallJoint::database ( "BallJoint", &newInstance<BulletBallJoint> );

BulletBallJoint::BulletBallJoint ( PhysicsEngineParameters::JointParameters& jointParameters ) :
BulletJoint ( jointParameters ),
  ballJoint ( NULL )
{

  PhysicsEngineParameters::BallJointParameters* ballJointParameters=
    dynamic_cast<PhysicsEngineParameters::BallJointParameters*> ( &jointParameters );

  BulletCallbacks::BulletSpecificData* bullet_data= 
    static_cast<BulletCallbacks::BulletSpecificData*>(ballJointParameters->getEngine()->getEngineSpecificData());

  // Replace any missing body with fixed body
  if ( !body1 )
    body1= &getFixedBody();
  if ( !body2 )
    body2= &getFixedBody();

  // Anchor point in world coordinates
  btVector3 anchorPoint= tobtVector3 ( ballJointParameters->getAnchorPoint() )*bullet_data->m_worldScale;

  // Create 2-body ball joint
  // Use fixed bodies rather than single body constructor to emulate 1-body joint, otherwise
  // setting the anchor point later doesn't seem to work properly.
  ballJoint= new btPoint2PointConstraint ( *body1, *body2, 
    body1->getWorldTransform().inverse()*anchorPoint, 
    body2->getWorldTransform().inverse()*anchorPoint );

  joint.reset ( ballJoint );

}

// Set the bullet joint's parameters using the parameter values specified in jointParameters
void BulletBallJoint::setParameters ( PhysicsEngineParameters::JointParameters& jointParameters )
{
  PhysicsEngineParameters::BallJointParameters* ballJointParameters= 
    dynamic_cast < PhysicsEngineParameters::BallJointParameters* > ( &jointParameters );
  BulletCallbacks::BulletSpecificData* bullet_data= 
    static_cast<BulletCallbacks::BulletSpecificData*>(ballJointParameters->getEngine()->getEngineSpecificData());

  if( ballJointParameters->haveAnchorPoint() )
  {
    // Anchor point in world coordinates
    btVector3 anchorPoint= tobtVector3 ( ballJointParameters->getAnchorPoint() )*bullet_data->m_worldScale;

    // Set pivot points relative to each body
    ballJoint->setPivotA ( body1->getWorldTransform().inverse()*anchorPoint );
    ballJoint->setPivotB ( body2->getWorldTransform().inverse()*anchorPoint );
  }
}

// Modify the jointParameters argument to reflect the bullet joint's current parameters
void BulletBallJoint::getParameters ( PhysicsEngineParameters::JointParameters& jointParameters )
{
  PhysicsEngineParameters::BallJointParameters* ballJointParameters= 
    dynamic_cast < PhysicsEngineParameters::BallJointParameters* > ( &jointParameters );
  BulletCallbacks::BulletSpecificData* bullet_data= 
    static_cast<BulletCallbacks::BulletSpecificData*>(ballJointParameters->getEngine()->getEngineSpecificData());

  if( ballJointParameters->haveBody1AnchorPoint() && body1 )
  {
    ballJointParameters->setBody1AnchorPoint( toVec3f ( body1->getWorldTransform()*ballJoint->getPivotInA()
      / bullet_data->m_worldScale ) );
  }

  if( ballJointParameters->haveBody2AnchorPoint() )
  {
    if ( body1 && body2 )
      ballJointParameters->setBody2AnchorPoint( toVec3f ( body2->getWorldTransform()*ballJoint->getPivotInB()
      / bullet_data->m_worldScale ) );
    else
      // Only one body specified, so it's in A not B
      ballJointParameters->setBody2AnchorPoint( toVec3f ( body2->getWorldTransform()*ballJoint->getPivotInA()
      / bullet_data->m_worldScale ) );
  }
}

BulletSingleAxisHingeJoint::JointDatabase
  BulletSingleAxisHingeJoint::database ( "SingleAxisHingeJoint", &newInstance<BulletSingleAxisHingeJoint> );

BulletSingleAxisHingeJoint::BulletSingleAxisHingeJoint ( PhysicsEngineParameters::JointParameters& jointParameters ) :
BulletJoint ( jointParameters ),
  hingeJoint ( NULL ),
  anchorPoint ( btVector3 ( 0.f, 0.f, 0.f ) ),
  bounce ( 0.f ),
  bias( 0.3f ),
  softness( 0.4f ) 
{
  PhysicsEngineParameters::SingleAxisHingeJointParameters* hingeJointParameters= 
    dynamic_cast < PhysicsEngineParameters::SingleAxisHingeJointParameters* > ( &jointParameters );
  BulletCallbacks::BulletSpecificData* bullet_data= 
    static_cast<BulletCallbacks::BulletSpecificData*>(hingeJointParameters->getEngine()->getEngineSpecificData());

  // Replace any missing body with fixed body
  if ( !body1 )
    body1= &getFixedBody();
  if ( !body2 )
    body2= &getFixedBody();

  // Anchor point in world coordinates
  anchorPoint= tobtVector3 ( hingeJointParameters->getAnchorPoint() )*bullet_data->m_worldScale;

  // Hinge axis in world coordinates
  btVector3 axis= tobtVector3 ( hingeJointParameters->getAxis() );
  axis.normalize();

  // Anchor point and hinge axis in body1's local coordinates
  btVector3 localAnchorPoint1= body1->getWorldTransform().inverse()*anchorPoint;
  btVector3 localAxis1= btTransform(body1->getWorldTransform().inverse().getRotation())*axis;

  // Anchor point and hinge axis in body2's local coordinates
  btVector3 localAnchorPoint2= body2->getWorldTransform().inverse()*anchorPoint;
  btVector3 localAxis2= btTransform(body2->getWorldTransform().inverse().getRotation())*axis;

  // Create 2-body hinge joint
  hingeJoint= new btHingeConstraint ( *body1, *body2, 
    localAnchorPoint1, localAnchorPoint2,
    localAxis1, localAxis2 );
  joint.reset ( hingeJoint );
}

// Set the bullet joint's parameters using the parameter values specified in jointParameters
void BulletSingleAxisHingeJoint::setParameters ( PhysicsEngineParameters::JointParameters& jointParameters )
{
  PhysicsEngineParameters::SingleAxisHingeJointParameters* hingeJointParameters= 
    dynamic_cast < PhysicsEngineParameters::SingleAxisHingeJointParameters* > ( &jointParameters );
  BulletCallbacks::BulletSpecificData* bullet_data= 
    static_cast<BulletCallbacks::BulletSpecificData*>(hingeJointParameters->getEngine()->getEngineSpecificData());

  // Motor velocity
  if( hingeJointParameters->haveMotorTarget() ) {
    hingeJoint->enableAngularMotor( true, hingeJointParameters->getMotorTarget(), maxMotorTorque );
  }

  // Set hinge anchor point
  if( hingeJointParameters->haveAnchorPoint() ) 
  {
    anchorPoint= tobtVector3 ( hingeJointParameters->getAnchorPoint() )*bullet_data->m_worldScale;

    hingeJoint->getAFrame().setOrigin ( body1->getWorldTransform().inverse()*anchorPoint );
    hingeJoint->getBFrame().setOrigin ( body2->getWorldTransform().inverse()*anchorPoint );
  }

  // Set joint limit (min angle)
  btScalar minAngle= hingeJoint->getLowerLimit();
  btScalar maxAngle= hingeJoint->getUpperLimit();
  if( hingeJointParameters->haveMinAngle() ) {
    minAngle= hingeJointParameters->getMinAngle();
  }
  if( hingeJointParameters->haveMaxAngle() ) {
    maxAngle= hingeJointParameters->getMaxAngle();
  }
  if ( hingeJointParameters->haveMinAngle() || hingeJointParameters->haveMaxAngle() ) {
    hingeJoint->setLimit( minAngle,maxAngle, softness, bias, bounce );
  }

  // Set bounce value for joint limits (note negation required to convert to CFM)
  if( hingeJointParameters->haveStopBounce() )
  {
    // _softness used by obsolete version of solver and left for compatibility;
    // _biasFactor works as a multiplier to constraint error, constraint becomes more "soft" when this factor is close to zero;
    // _relaxationFactor used to control bounce on limits : 1.0 means full bounce, 0.0 means no bounce
    bounce= btScalar(hingeJointParameters->getStopBounce());
    hingeJoint->setLimit( hingeJoint->getLowerLimit(), hingeJoint->getUpperLimit(), softness, bias, bounce );
  }

  if( hingeJointParameters->haveBias() )
  {
    bias = btScalar(hingeJointParameters->getBias());
    hingeJoint->setLimit( hingeJoint->getLowerLimit(), hingeJoint->getUpperLimit(), softness, bias, bounce);
  }

  if( hingeJointParameters->haveSoftness() )
  {
    softness = btScalar(hingeJointParameters->getSoftness());
    hingeJoint->setLimit( hingeJoint->getLowerLimit(), hingeJoint->getUpperLimit(), softness, bias, bounce);
  }

  // Set error correction value for joint limits
  if( hingeJointParameters->haveStopErrorCorrection() )
    hingeJoint->setParam ( BT_CONSTRAINT_STOP_ERP, hingeJointParameters->getStopErrorCorrection() );

  // axis
  // The setAxis() function does not appear to work and joint becomes unstable. Instead recalculate 
  // frameA and frameB from constraintSpace (a transformation with the hinge axis as the local z-axis)
  if( hingeJointParameters->haveAxis() )
  {
    // Hinge axis in world coordinates
    btVector3 axis= tobtVector3 ( hingeJointParameters->getAxis() );
    axis.normalize();

    btTransform constraintSpace= getAxisTransformZ ( axis );
    constraintSpace.setOrigin ( anchorPoint );

    btTransform frameA= body1->getWorldTransform().inverse() * constraintSpace;
    hingeJoint->getAFrame()= frameA;

    btTransform frameB= body2->getWorldTransform().inverse() * constraintSpace;
    hingeJoint->getBFrame()= frameB;
}
}

// Modify the jointParameters argument to reflect the bullet joint's current parameters
void BulletSingleAxisHingeJoint::getParameters ( PhysicsEngineParameters::JointParameters& jointParameters )
{
  PhysicsEngineParameters::SingleAxisHingeJointParameters* hingeJointParameters= 
    dynamic_cast < PhysicsEngineParameters::SingleAxisHingeJointParameters* > ( &jointParameters );
  BulletCallbacks::BulletSpecificData* bullet_data= 
    static_cast<BulletCallbacks::BulletSpecificData*>(hingeJointParameters->getEngine()->getEngineSpecificData());

  // Get anchor point relative to body1
  if ( hingeJointParameters->haveBody1AnchorPoint() && body1 )
    hingeJointParameters->setBody1AnchorPoint ( toVec3f ( body1->getWorldTransform()*hingeJoint->getAFrame().getOrigin()
    / bullet_data->m_worldScale ) );

  // Get anchor point relative to body2
  if ( hingeJointParameters->haveBody2AnchorPoint() ) {
    if ( body1 && body2 ) {
      hingeJointParameters->setBody2AnchorPoint ( toVec3f ( body2->getWorldTransform()*hingeJoint->getBFrame().getOrigin()
        / bullet_data->m_worldScale ) );
    } else {
      hingeJointParameters->setBody2AnchorPoint ( toVec3f ( body2->getWorldTransform()*hingeJoint->getAFrame().getOrigin()
        / bullet_data->m_worldScale ) );
    }
  }

  //********************************************************************************//
  btVector3 localY = btVector3(1,0,0);
  btVector3 localX = btVector3(0,1,0);

  btVector3 bodyA_Y = hingeJoint->getRigidBodyB().getWorldTransform().getBasis().inverse() * localY;
  btVector3 bodyB_Y = hingeJoint->getRigidBodyA().getWorldTransform().getBasis().inverse() * localY;
  btVector3 bodyB_X = hingeJoint->getRigidBodyA().getWorldTransform().getBasis().inverse() * localX;

  // Project bodyA_Y on to plane with normal bodyB_X
  btVector3 projBodyA_Y= bodyA_Y - bodyA_Y.dot( bodyB_X ) * bodyB_X;
  projBodyA_Y.normalize();
  
  // Get the angle between the two Y axes now in the same plane
  // Using the fact that for unit vectors the following is true atan2( sin(angle), cos(angle) ) = atan2( ||a.cross(b)||, a.dot(b) )
  btVector3 bodyB_Y_norm = bodyB_Y; 
  bodyB_Y_norm.normalize();
  btScalar angle= H3DAtan2( ( projBodyA_Y.cross( bodyB_Y_norm ) ).length(), projBodyA_Y.dot( bodyB_Y_norm ) );

  //********************************************************************************//
  
  // Get hinge angle
  if( hingeJointParameters->haveAngle() )
  {
    hingeJointParameters->setAngle( hingeJoint->getHingeAngle() );
  }

  // angleRate
  if ( hingeJointParameters->haveAngleRate() )
  {
    hingeAngleRate.setCurrentValue ( hingeJoint->getHingeAngle() );
    hingeJointParameters->setAngleRate ( hingeAngleRate );
  }
}

BulletUniversalJoint::JointDatabase
  BulletUniversalJoint::database ( "UniversalJoint", &newInstance<BulletUniversalJoint> );

BulletUniversalJoint::BulletUniversalJoint ( PhysicsEngineParameters::JointParameters& jointParameters ) :
BulletJoint ( jointParameters ),
  universalJoint ( NULL ),
  anchorPoint ( btVector3 ( 0, 0, 0 ) ),
  axis1 ( btVector3 ( 1, 0, 0 ) ),
  axis2 ( btVector3 ( 0, 1, 0 ) )
{
  PhysicsEngineParameters::UniversalJointParameters* universalJointParameters=
    dynamic_cast<PhysicsEngineParameters::UniversalJointParameters*> ( &jointParameters );
  BulletCallbacks::BulletSpecificData* bullet_data= 
    static_cast<BulletCallbacks::BulletSpecificData*>(universalJointParameters->getEngine()->getEngineSpecificData());

  if ( !body1 )
    body1= &getFixedBody();
  if ( !body2 )
    body2= &getFixedBody();

  // Anchor point and axes in world coordinates
  anchorPoint= tobtVector3 ( universalJointParameters->getAnchorPoint() )*bullet_data->m_worldScale;
  axis1= tobtVector3 ( universalJointParameters->getAxis1() );
  axis2= tobtVector3 ( universalJointParameters->getAxis2() );
  axis1.normalize();
  axis2.normalize();

  btVector3 localAxis1= btTransform(body1->getWorldTransform().inverse().getRotation())*axis1;
  btVector3 localAxis2= btTransform(body2->getWorldTransform().inverse().getRotation())*axis2;

  universalJoint= new btUniversalConstraint ( *body1, *body2, anchorPoint, localAxis1, localAxis2 );

  joint.reset ( universalJoint );
}

// Set the bullet joint's parameters using the parameter values specified in jointParameters
void BulletUniversalJoint::setParameters ( PhysicsEngineParameters::JointParameters& jointParameters )
{
  PhysicsEngineParameters::UniversalJointParameters* universalJointParameters= 
    dynamic_cast < PhysicsEngineParameters::UniversalJointParameters* > ( &jointParameters );
  BulletCallbacks::BulletSpecificData* bullet_data= 
    static_cast<BulletCallbacks::BulletSpecificData*>(universalJointParameters->getEngine()->getEngineSpecificData());

  // anchorPoint
  if ( universalJointParameters->haveAnchorPoint() )
  {
    anchorPoint= tobtVector3 ( universalJointParameters->getAnchorPoint() )*bullet_data->m_worldScale;

    universalJoint->getFrameOffsetA().setOrigin ( body1->getWorldTransform().inverse()*anchorPoint );
    universalJoint->getFrameOffsetB().setOrigin ( body2->getWorldTransform().inverse()*anchorPoint );
  }

  // axis1
  if ( universalJointParameters->haveAxis1() )
  {
    axis1= tobtVector3 ( universalJointParameters->getAxis1() );
    axis1.normalize();

    updateFrames();
  }

  // axis2
  if ( universalJointParameters->haveAxis2() )
  {
    axis2= tobtVector3 ( universalJointParameters->getAxis2() );
    axis2.normalize();

    updateFrames();
  }
}

// Modify the jointParameters argument to reflect the bullet joint's current parameters
void BulletUniversalJoint::getParameters ( PhysicsEngineParameters::JointParameters& jointParameters )
{
  PhysicsEngineParameters::UniversalJointParameters* universalJointParameters= 
    dynamic_cast < PhysicsEngineParameters::UniversalJointParameters* > ( &jointParameters );
  BulletCallbacks::BulletSpecificData* bullet_data= 
    static_cast<BulletCallbacks::BulletSpecificData*>(universalJointParameters->getEngine()->getEngineSpecificData());

  // Note: The getAnchorX() functions seems to return in global coords, while getAxisX() are in local

  // body1AnchorPoint output
  if( universalJointParameters->haveBody1AnchorPoint() )
    universalJointParameters->setBody1AnchorPoint( toVec3f ( universalJoint->getAnchor()
    / bullet_data->m_worldScale ) );

  // body2AnchorPoint output
  if( universalJointParameters->haveBody2AnchorPoint() )
    universalJointParameters->setBody2AnchorPoint( toVec3f ( universalJoint->getAnchor2()
    / bullet_data->m_worldScale ) );

  // body1Axis output
  if( universalJointParameters->haveBody1Axis() )
  {
    btVector3 axis1Local= universalJoint->getFrameOffsetA().getBasis().getColumn ( 2 );
    universalJointParameters->setBody1Axis( toVec3f ( body1->getWorldTransform().getBasis()*axis1Local ) );
  }

  // body2Axis output
  if( universalJointParameters->haveBody2Axis() )
  {
    btVector3 axis2Local= universalJoint->getFrameOffsetB().getBasis().getColumn ( 1 );
    universalJointParameters->setBody2Axis( toVec3f ( body2->getWorldTransform().getBasis()*axis2Local ) );
  }
}

// Update the frame transforms based on current anchor and axes
void BulletUniversalJoint::updateFrames ()
{
  // axis1 must be perpendicular to axis2
  btScalar angle= axis1.angle ( axis2 );
  if ( angle < Constants::pi/2 + Constants::f_epsilon &&
    angle > Constants::pi/2 - Constants::f_epsilon )
  {
    btVector3 zAxis = axis1;
    btVector3 yAxis = axis2;
    btVector3 xAxis= yAxis.cross(zAxis); // we want right coordinate system

    btTransform constraintSpace ( btMatrix3x3 (  xAxis.x(), yAxis.x(), zAxis.x(),
      xAxis.y(), yAxis.y(), zAxis.y(),
      xAxis.z(), yAxis.z(), zAxis.z() ) );
    constraintSpace.setOrigin ( anchorPoint );

    universalJoint->getFrameOffsetA()= body1->getWorldTransform().inverse()*constraintSpace;
    universalJoint->getFrameOffsetB()= body2->getWorldTransform().inverse()*constraintSpace; 
  }
  else
    Console(3) << "Warning: axis1 must be perpendicular to axis2!" << endl;
}

BulletDoubleAxisHingeJoint::JointDatabase
  BulletDoubleAxisHingeJoint::database ( "DoubleAxisHingeJoint", &newInstance<BulletDoubleAxisHingeJoint> );

BulletDoubleAxisHingeJoint::BulletDoubleAxisHingeJoint ( PhysicsEngineParameters::JointParameters& jointParameters ) :
BulletJoint ( jointParameters ),
  doubleHingeJoint ( NULL ),
  anchorPoint ( btVector3 ( 0, 0, 0 ) ),
  axis1 ( btVector3 ( 1, 0, 0 ) ),
  axis2 ( btVector3 ( 0, 1, 0 ) ),
  suspensionERP ( 0 ),
  suspensionCFM ( 0 )
{
  PhysicsEngineParameters::DoubleAxisHingeJointParameters* doubleHingeJointParameters=
    dynamic_cast<PhysicsEngineParameters::DoubleAxisHingeJointParameters*> ( &jointParameters );
  BulletCallbacks::BulletSpecificData* bullet_data= 
    static_cast<BulletCallbacks::BulletSpecificData*>(doubleHingeJointParameters->getEngine()->getEngineSpecificData());

  if ( !body1 )
    body1= &getFixedBody();
  if ( !body2 )
    body2= &getFixedBody();

  // Anchor point is in world coordinates, axes are in local coordinates
  anchorPoint= tobtVector3(doubleHingeJointParameters->getAnchorPoint())*bullet_data->m_worldScale;
  axis1= btTransform(body1->getWorldTransform().inverse().getRotation())*tobtVector3(doubleHingeJointParameters->getAxis1());
  axis2= btTransform(body2->getWorldTransform().inverse().getRotation())*tobtVector3(doubleHingeJointParameters->getAxis2());

  doubleHingeJoint= new btHinge2Constraint ( *body1, *body2, anchorPoint, axis1, axis2 );

  // To better emulate the ODE joint, lock the z-axis using limits. Makes suspension less springy.
  //doubleHingeJoint->getTranslationalLimitMotor()->m_lowerLimit.setZ ( 0 );
  //doubleHingeJoint->getTranslationalLimitMotor()->m_upperLimit.setZ ( 0 );

  joint.reset ( doubleHingeJoint );
}

// Set the bullet joint's parameters using the parameter values specified in jointParameters
void BulletDoubleAxisHingeJoint::setParameters ( PhysicsEngineParameters::JointParameters& jointParameters )
{
  PhysicsEngineParameters::DoubleAxisHingeJointParameters* doubleHingeJointParameters=
    dynamic_cast<PhysicsEngineParameters::DoubleAxisHingeJointParameters*> ( &jointParameters );
  BulletCallbacks::BulletSpecificData* bullet_data= 
    static_cast<BulletCallbacks::BulletSpecificData*>(doubleHingeJointParameters->getEngine()->getEngineSpecificData());

  // set anchorPoint
  if ( doubleHingeJointParameters->haveAnchorPoint() )
  {
    anchorPoint= tobtVector3 ( doubleHingeJointParameters->getAnchorPoint() )*bullet_data->m_worldScale;

    doubleHingeJoint->getFrameOffsetA().setOrigin ( body1->getWorldTransform().inverse()*anchorPoint );
    doubleHingeJoint->getFrameOffsetB().setOrigin ( body2->getWorldTransform().inverse()*anchorPoint );
  }

  // axis1
  if ( doubleHingeJointParameters->haveAxis1() )
  {
    axis1= tobtVector3 ( doubleHingeJointParameters->getAxis1() );
    axis1.normalize();
  }

  // axis2
  if ( doubleHingeJointParameters->haveAxis2() )
  {
    axis2= tobtVector3 ( doubleHingeJointParameters->getAxis2() );
    axis2.normalize();
  }

  // update frames if any axis has changed
  if ( doubleHingeJointParameters->haveAxis1() || doubleHingeJointParameters->haveAxis2() )
    updateFrames();

  // maxTorque1 (axis1 is the local z-axis, see btHinge2Constraint.cpp)
  if ( doubleHingeJointParameters->haveMaxTorque1() )
    doubleHingeJoint->getRotationalLimitMotor(2)->m_maxMotorForce= doubleHingeJointParameters->getMaxTorque1()*bullet_data->m_worldScale*bullet_data->m_worldScale;

  // maxTorque2 (axis1 is the local x-axis)
  if ( doubleHingeJointParameters->haveMaxTorque2() )
    doubleHingeJoint->getRotationalLimitMotor(0)->m_maxMotorForce= doubleHingeJointParameters->getMaxTorque2()*bullet_data->m_worldScale*bullet_data->m_worldScale;

  // desiredAngularVelocity1 (axis1 is the local z-axis, see btHinge2Constraint.cpp)
  // Enable or disable rotational motor on generic 6dof joint based on velocity value
  if ( doubleHingeJointParameters->haveDesiredAngularVelocity1() )
  {
    doubleHingeJoint->getRotationalLimitMotor(2)->m_targetVelocity= doubleHingeJointParameters->getDesiredAngularVelocity1();
    doubleHingeJoint->getRotationalLimitMotor(2)->m_enableMotor= abs(doubleHingeJointParameters->getDesiredAngularVelocity1()) > Constants::f_epsilon;
  }

  // desiredAngularVelocity1 (axis2 is the local x-axis)
  // Enable or disable rotational motor on generic 6dof joint based on velocity value
  if ( doubleHingeJointParameters->haveDesiredAngularVelocity2() )
  {
    doubleHingeJoint->getRotationalLimitMotor(0)->m_targetVelocity= doubleHingeJointParameters->getDesiredAngularVelocity2();
    doubleHingeJoint->getRotationalLimitMotor(0)->m_enableMotor= abs(doubleHingeJointParameters->getDesiredAngularVelocity2()) > Constants::f_epsilon;
  }

  // maxAngle1 - upper limit of axis 1
  if ( doubleHingeJointParameters->haveMaxAngle1() )
    doubleHingeJoint->setUpperLimit ( btScalar(doubleHingeJointParameters->getMaxAngle1()) );

  // minAngle1 - lower limit of axis 1
  if ( doubleHingeJointParameters->haveMinAngle1() )
    doubleHingeJoint->setLowerLimit ( btScalar(doubleHingeJointParameters->getMinAngle1()) );

  // suspensionForce
  if ( doubleHingeJointParameters->haveSuspensionForce() )
    suspensionCFM= doubleHingeJointParameters->getSuspensionForce();

  // suspensionErrorCorrection
  if ( doubleHingeJointParameters->haveSuspensionErrorCorrection() )
    suspensionERP= doubleHingeJointParameters->getSuspensionErrorCorrection();

  // Update spring and damper suspension from CFM and ERP specified
  if ( doubleHingeJointParameters->haveSuspensionForce() ||
    doubleHingeJointParameters->haveSuspensionErrorCorrection() )
    updateSuspension ( *doubleHingeJointParameters );

#if BT_BULLET_VERSION < 283
  // stopBounce1 - restitution
  if ( doubleHingeJointParameters->haveStopBounce1() )
    doubleHingeJoint->getRotationalLimitMotor(2)->m_limitSoftness= btScalar ( doubleHingeJointParameters->getStopBounce1() );
#else
#ifdef _MSC_VER
#pragma message("Bullet version is equal to or above 2.83. stopBounce1 field of DoubleAxisHingeJoint will not be used.") 
#else
#ifdef __GNUC__
#warning Bullet version is equal to or above 2.83. stopBounce1 field of DoubleAxisHingeJoint will not be used.
#endif
#endif
#endif

  // stopConstantForceMix1
  if ( doubleHingeJointParameters->haveStopConstantForceMix1() )
    doubleHingeJoint->getRotationalLimitMotor(2)->m_stopCFM= btScalar ( doubleHingeJointParameters->getStopConstantForceMix1() );

  // stopErrorCorrection1
  if ( doubleHingeJointParameters->haveStopErrorCorrection1() )
    doubleHingeJoint->getRotationalLimitMotor(2)->m_stopERP= btScalar ( doubleHingeJointParameters->getStopErrorCorrection1() );
}

// Modify the jointParameters argument to reflect the bullet joint's current parameters
void BulletDoubleAxisHingeJoint::getParameters ( PhysicsEngineParameters::JointParameters& jointParameters )
{
  PhysicsEngineParameters::DoubleAxisHingeJointParameters* doubleHingeJointParameters=
    dynamic_cast<PhysicsEngineParameters::DoubleAxisHingeJointParameters*> ( &jointParameters );
  BulletCallbacks::BulletSpecificData* bullet_data= 
    static_cast<BulletCallbacks::BulletSpecificData*>(doubleHingeJointParameters->getEngine()->getEngineSpecificData());

  // hinge1Angle
  if( doubleHingeJointParameters->haveHinge1Angle() )
    doubleHingeJointParameters->setHinge1Angle( (H3DFloat)doubleHingeJoint->getAngle1() );

  // hinge2Angle
  if( doubleHingeJointParameters->haveHinge2Angle() )
    doubleHingeJointParameters->setHinge2Angle( (H3DFloat)doubleHingeJoint->getAngle2() );

  // Rate of change of hinge 1 angle
  if( doubleHingeJointParameters->haveHinge1AngleRate() )
  {
    hinge1AngleRate.setCurrentValue ( (H3DFloat)doubleHingeJoint->getAngle1() );
    doubleHingeJointParameters->setHinge1AngleRate( hinge1AngleRate );
  }

  // Rate of change of hinge 2 angle
  if( doubleHingeJointParameters->haveHinge2AngleRate() )
  {
    hinge2AngleRate.setCurrentValue ( (H3DFloat)doubleHingeJoint->getAngle2() );
    doubleHingeJointParameters->setHinge2AngleRate( hinge2AngleRate );
  }

  // body1AnchorPoint
  if( doubleHingeJointParameters->haveBody1AnchorPoint() )
    doubleHingeJointParameters->setBody1AnchorPoint( toVec3f ( doubleHingeJoint->getAnchor() 
    / bullet_data->m_worldScale ) );

  // body2AnchorPoint
  if( doubleHingeJointParameters->haveBody2AnchorPoint() )
    doubleHingeJointParameters->setBody2AnchorPoint( toVec3f ( doubleHingeJoint->getAnchor2()
    / bullet_data->m_worldScale ) );

  // body1Axis output
  if( doubleHingeJointParameters->haveBody1Axis() )
  {
    btVector3 axis1Local= doubleHingeJoint->getFrameOffsetA().getBasis().getColumn ( 2 );
    doubleHingeJointParameters->setBody1Axis( toVec3f ( body1->getWorldTransform().getBasis()*axis1Local ) );
  }

  // body2Axis output
  if( doubleHingeJointParameters->haveBody2Axis() )
  {
    btVector3 axis2Local= doubleHingeJoint->getFrameOffsetB().getBasis().getColumn ( 0 );
    doubleHingeJointParameters->setBody2Axis( toVec3f ( body2->getWorldTransform().getBasis()*axis2Local ) );
  }
}

// Update the frame transforms based on current anchor and axes
void BulletDoubleAxisHingeJoint::updateFrames ()
{
  // axis1 must be perpendicular to axis2
  btScalar angle= axis1.angle ( axis2 );
  if ( angle < Constants::pi/2 + Constants::f_epsilon &&
    angle > Constants::pi/2 - Constants::f_epsilon )
  {
    btVector3 zAxis = axis1;
    btVector3 xAxis = axis2;
    btVector3 yAxis = zAxis.cross(xAxis); // we want right coordinate system

    btTransform constraintSpace ( btMatrix3x3 (  xAxis.x(), yAxis.x(), zAxis.x(),
      xAxis.y(), yAxis.y(), zAxis.y(),
      xAxis.z(), yAxis.z(), zAxis.z() ) );
    constraintSpace.setOrigin ( anchorPoint );

    doubleHingeJoint->getFrameOffsetA()= body1->getWorldTransform().inverse()*constraintSpace;
    doubleHingeJoint->getFrameOffsetB()= body2->getWorldTransform().inverse()*constraintSpace; 
  }
  else
    Console(3) << "Warning: axis1 ( " << toVec3f ( axis1 ) << " ) must be perpendicular to axis2 ( " << toVec3f ( axis2 ) << " )!" << endl;
}

// Update spring and damper constants based on current suspensionERP and suspensionCFM
void BulletDoubleAxisHingeJoint::updateSuspension ( PhysicsEngineParameters::DoubleAxisHingeJointParameters& hingeJointParameters )
{
  // This value determines the stiffest possible constraint (when CFM=0)
  // Too small and joint will be unstable
  btScalar eps= 0.001f;

  // Avoid division by zero at extremes, i.e. CFM= 0 and ERP= 1
  btScalar CFM= suspensionCFM > eps ? suspensionCFM : eps;
  btScalar ERP= suspensionERP < 1-eps ? suspensionERP : 1-eps;

  // Calculate spring and damper constants from CFM and ERP
  // Based on manipulation of equations on ODE Wiki here: http://opende.sourceforge.net/wiki/index.php/Manual_%28Concepts%29#How_To_Use_ERP_and_CFM
  // When give ERP and CFM in terms of spring and damping constants (and time step)
  btScalar stepSize= hingeJointParameters.getEngine()->getStepSize();
  btScalar springConstant= ERP / (stepSize*CFM);
  btScalar damperConstant= 1 /
    (CFM*(-ERP/(ERP-1)+1));

  // Set spring and damper constants
  doubleHingeJoint->setStiffness( 2, springConstant );
  doubleHingeJoint->setDamping( 2, damperConstant );
}

BulletGeneric6DOFJoint::JointDatabase
  BulletGeneric6DOFJoint::database ( "Generic6DOFJoint", &newInstance<BulletGeneric6DOFJoint> );

BulletGeneric6DOFJoint::BulletGeneric6DOFJoint ( PhysicsEngineParameters::JointParameters& jointParameters ) :
BulletJoint ( jointParameters ),
  generic6DOFJoint ( NULL ),
  anchorPoint ( btVector3 ( 0, 0, 0 ) ),
  axis1 ( btVector3 ( 1, 0, 0 ) ),
  axis2 ( btVector3 ( 0, 1, 0 ) ),
  axis3 ( btVector3 ( 0, 0, 1 ) ),
  maxAngle1 ( 0.0f ),
  maxAngle2 ( 0.0f ),
  maxAngle3 ( 0.0f ),
  minAngle1 ( 0.0f ),
  minAngle2 ( 0.0f ),
  minAngle3 ( 0.0f ),
  maxLimit1 ( 0.0f ),
  maxLimit2 ( 0.0f ),
  maxLimit3 ( 0.0f ),
  minLimit1 ( 0.0f ),
  minLimit2 ( 0.0f ),
  minLimit3 ( 0.0f )
{
  PhysicsEngineParameters::Generic6DOFJointParameters* generic6DOFJointParameters=
    dynamic_cast<PhysicsEngineParameters::Generic6DOFJointParameters*> ( &jointParameters );
  BulletCallbacks::BulletSpecificData* bullet_data= 
    static_cast<BulletCallbacks::BulletSpecificData*>(generic6DOFJointParameters->getEngine()->getEngineSpecificData());

  if ( !body1 )
    body1= &getFixedBody();
  if ( !body2 )
    body2= &getFixedBody();

  generic6DOFJoint= new btGeneric6DofConstraint ( *body1, *body2, btTransform::getIdentity(), btTransform::getIdentity(), true );
  joint.reset ( generic6DOFJoint );
}

// Set the bullet joint's parameters using the parameter values specified in jointParameters
void BulletGeneric6DOFJoint::setParameters ( PhysicsEngineParameters::JointParameters& jointParameters )
{
  PhysicsEngineParameters::Generic6DOFJointParameters* generic6DOFJointParameters=
    dynamic_cast<PhysicsEngineParameters::Generic6DOFJointParameters*> ( &jointParameters );
  BulletCallbacks::BulletSpecificData* bullet_data= 
    static_cast<BulletCallbacks::BulletSpecificData*>(generic6DOFJointParameters->getEngine()->getEngineSpecificData());

  // set anchorPoint
  if ( generic6DOFJointParameters->haveAnchorPoint() )
  {
    anchorPoint= tobtVector3 ( generic6DOFJointParameters->getAnchorPoint() )*bullet_data->m_worldScale;

    generic6DOFJoint->getFrameOffsetA().setOrigin ( body1->getWorldTransform().inverse()*anchorPoint );
    generic6DOFJoint->getFrameOffsetB().setOrigin ( body2->getWorldTransform().inverse()*anchorPoint );
  }

  // axis1
  if ( generic6DOFJointParameters->haveAxis1() )
  {
    axis1= tobtVector3 ( generic6DOFJointParameters->getAxis1() );
    axis1.normalize();
  }

  // axis2
  if ( generic6DOFJointParameters->haveAxis2() )
  {
    axis2= tobtVector3 ( generic6DOFJointParameters->getAxis2() );
    axis2.normalize();
  }

  // axis3
  if ( generic6DOFJointParameters->haveAxis3() )
  {
    axis3= tobtVector3 ( generic6DOFJointParameters->getAxis3() );
    axis3.normalize();
  }

  // update frames if any axis has changed
  if ( generic6DOFJointParameters->haveAxis1() || generic6DOFJointParameters->haveAxis2() || generic6DOFJointParameters->haveAxis3() )
    updateFrames();

  // maxTorque1
  if ( generic6DOFJointParameters->haveMaxTorque1() )
    generic6DOFJoint->getRotationalLimitMotor(0)->m_maxMotorForce= generic6DOFJointParameters->getMaxTorque1()*bullet_data->m_worldScale*bullet_data->m_worldScale;

  // maxTorque2
  if ( generic6DOFJointParameters->haveMaxTorque2() )
    generic6DOFJoint->getRotationalLimitMotor(1)->m_maxMotorForce= generic6DOFJointParameters->getMaxTorque2()*bullet_data->m_worldScale*bullet_data->m_worldScale;

  // maxTorque3
  if ( generic6DOFJointParameters->haveMaxTorque3() )
    generic6DOFJoint->getRotationalLimitMotor(2)->m_maxMotorForce= generic6DOFJointParameters->getMaxTorque3()*bullet_data->m_worldScale*bullet_data->m_worldScale;

  // maxForce1
  if ( generic6DOFJointParameters->haveMaxForce1() ) {
    generic6DOFJoint->getTranslationalLimitMotor()->m_maxMotorForce[0]= generic6DOFJointParameters->getMaxForce1();
  }

  // maxForce2
  if ( generic6DOFJointParameters->haveMaxForce2() )
    generic6DOFJoint->getTranslationalLimitMotor()->m_maxMotorForce[1]= generic6DOFJointParameters->getMaxForce2();

  // maxForce3
  if ( generic6DOFJointParameters->haveMaxForce3() )
    generic6DOFJoint->getTranslationalLimitMotor()->m_maxMotorForce[2]= generic6DOFJointParameters->getMaxForce3();

  // desiredAngularVelocity1
  // Enable or disable rotational motor on generic 6dof joint based on velocity value
  if ( generic6DOFJointParameters->haveDesiredAngularVelocity1() )
  {
    generic6DOFJoint->getRotationalLimitMotor(0)->m_targetVelocity= generic6DOFJointParameters->getDesiredAngularVelocity1();
    generic6DOFJoint->getRotationalLimitMotor(0)->m_enableMotor= abs(generic6DOFJointParameters->getDesiredAngularVelocity1()) > Constants::f_epsilon;
  }

  // desiredAngularVelocity2
  // Enable or disable rotational motor on generic 6dof joint based on velocity value
  if ( generic6DOFJointParameters->haveDesiredAngularVelocity2() )
  {
    generic6DOFJoint->getRotationalLimitMotor(1)->m_targetVelocity= generic6DOFJointParameters->getDesiredAngularVelocity2();
    generic6DOFJoint->getRotationalLimitMotor(1)->m_enableMotor= abs(generic6DOFJointParameters->getDesiredAngularVelocity2()) > Constants::f_epsilon;
  }

  // desiredAngularVelocity3
  // Enable or disable rotational motor on generic 6dof joint based on velocity value
  if ( generic6DOFJointParameters->haveDesiredAngularVelocity3() )
  {
    generic6DOFJoint->getRotationalLimitMotor(2)->m_targetVelocity= generic6DOFJointParameters->getDesiredAngularVelocity3();
    generic6DOFJoint->getRotationalLimitMotor(2)->m_enableMotor= abs(generic6DOFJointParameters->getDesiredAngularVelocity3()) > Constants::f_epsilon;
  }

  // desiredLinearVelocity1
  // Enable or disable rotational motor on generic 6dof joint based on velocity value
  if ( generic6DOFJointParameters->haveDesiredLinearVelocity1() )
  {
    generic6DOFJoint->getTranslationalLimitMotor()->m_targetVelocity[0]= generic6DOFJointParameters->getDesiredLinearVelocity1();
    generic6DOFJoint->getTranslationalLimitMotor()->m_enableMotor[0]= abs(generic6DOFJointParameters->getDesiredLinearVelocity1()) > Constants::f_epsilon;
  }

  // desiredLinearVelocity2
  // Enable or disable rotational motor on generic 6dof joint based on velocity value
  if ( generic6DOFJointParameters->haveDesiredLinearVelocity2() )
  {
    generic6DOFJoint->getTranslationalLimitMotor()->m_targetVelocity[1]= generic6DOFJointParameters->getDesiredLinearVelocity2();
    generic6DOFJoint->getTranslationalLimitMotor()->m_enableMotor[1]= abs(generic6DOFJointParameters->getDesiredLinearVelocity2()) > Constants::f_epsilon;
  }

  // desiredLinearVelocity3
  // Enable or disable rotational motor on generic 6dof joint based on velocity value
  if ( generic6DOFJointParameters->haveDesiredLinearVelocity3() )
  {
    generic6DOFJoint->getTranslationalLimitMotor()->m_targetVelocity[2]= generic6DOFJointParameters->getDesiredLinearVelocity3();
    generic6DOFJoint->getTranslationalLimitMotor()->m_enableMotor[2]= abs(generic6DOFJointParameters->getDesiredLinearVelocity3()) > Constants::f_epsilon;
  }

  // max angles
  if ( generic6DOFJointParameters->haveMaxAngle1() )
    maxAngle1= generic6DOFJointParameters->getMaxAngle1();
  if ( generic6DOFJointParameters->haveMaxAngle2() )
    maxAngle2= generic6DOFJointParameters->getMaxAngle2();
  if ( generic6DOFJointParameters->haveMaxAngle3() )
    maxAngle3= generic6DOFJointParameters->getMaxAngle3();

  // min angle
  if ( generic6DOFJointParameters->haveMinAngle1() )
    minAngle1= generic6DOFJointParameters->getMinAngle1();
  if ( generic6DOFJointParameters->haveMinAngle2() )
    minAngle2= generic6DOFJointParameters->getMinAngle2();
  if ( generic6DOFJointParameters->haveMinAngle3() )
    minAngle3= generic6DOFJointParameters->getMinAngle3();

  if ( generic6DOFJointParameters->haveMaxAngle1() ||
    generic6DOFJointParameters->haveMaxAngle2() ||
    generic6DOFJointParameters->haveMaxAngle3() )
    generic6DOFJoint->setAngularUpperLimit ( btVector3 ( maxAngle1, maxAngle2, maxAngle3 ) );

  if ( generic6DOFJointParameters->haveMinAngle1() ||
    generic6DOFJointParameters->haveMinAngle2() ||
    generic6DOFJointParameters->haveMinAngle3() )
    generic6DOFJoint->setAngularLowerLimit ( btVector3 ( minAngle1, minAngle2, minAngle3 ) );

  // max limits
  if ( generic6DOFJointParameters->haveMaxLimit1() )
    maxLimit1= generic6DOFJointParameters->getMaxLimit1();
  if ( generic6DOFJointParameters->haveMaxLimit2() )
    maxLimit2= generic6DOFJointParameters->getMaxLimit2();
  if ( generic6DOFJointParameters->haveMaxLimit3() )
    maxLimit3= generic6DOFJointParameters->getMaxLimit3();

  // min limits
  if ( generic6DOFJointParameters->haveMinLimit1() )
    minLimit1= generic6DOFJointParameters->getMinLimit1();
  if ( generic6DOFJointParameters->haveMinLimit2() )
    minLimit2= generic6DOFJointParameters->getMinLimit2();
  if ( generic6DOFJointParameters->haveMinLimit3() )
    minLimit3= generic6DOFJointParameters->getMinLimit3();

  if ( generic6DOFJointParameters->haveMaxLimit1() ||
    generic6DOFJointParameters->haveMaxLimit2() ||
    generic6DOFJointParameters->haveMaxLimit3() )
    generic6DOFJoint->setLinearUpperLimit ( btVector3 ( maxLimit1, maxLimit2, maxLimit3 ) );

  if ( generic6DOFJointParameters->haveMinLimit1() ||
    generic6DOFJointParameters->haveMinLimit2() ||
    generic6DOFJointParameters->haveMinLimit3() )
    generic6DOFJoint->setLinearLowerLimit ( btVector3 ( minLimit1, minLimit2, minLimit3 ) );
}

// Modify the jointParameters argument to reflect the bullet joint's current parameters
/*void BulletDoubleAxisHingeJoint::getParameters ( PhysicsEngineParameters::JointParameters& jointParameters )
{
PhysicsEngineParameters::DoubleAxisHingeJointParameters* doubleHingeJointParameters=
dynamic_cast<PhysicsEngineParameters::DoubleAxisHingeJointParameters*> ( &jointParameters );
BulletCallbacks::BulletSpecificData* bullet_data= 
static_cast<BulletCallbacks::BulletSpecificData*>(doubleHingeJointParameters->engine_thread->getEngineSpecificData());

}*/

// Update the frame transforms based on current anchor and axes
void BulletGeneric6DOFJoint::updateFrames ()
{
  // axis1 must be perpendicular to axis2
  //btScalar angle= axis1.angle ( axis2 );
  //if ( angle < Constants::pi/2 + Constants::f_epsilon &&
  //     angle > Constants::pi/2 - Constants::f_epsilon )
  //{
  btVector3 xAxis = axis1;
  btVector3 yAxis = axis2;
  btVector3 zAxis = axis3;
  btTransform constraintSpace ( btMatrix3x3 (  xAxis.x(), yAxis.x(), zAxis.x(),
    xAxis.y(), yAxis.y(), zAxis.y(),
    xAxis.z(), yAxis.z(), zAxis.z() ) );
  constraintSpace.setOrigin ( anchorPoint );

  generic6DOFJoint->getFrameOffsetA()= body1->getWorldTransform().inverse()*constraintSpace;
  generic6DOFJoint->getFrameOffsetB()= body2->getWorldTransform().inverse()*constraintSpace; 
  //}
  //else
  //  Console(3) << "Warning: axis1 ( " << toVec3f ( axis1 ) << " ) must be perpendicular to axis2 ( " << toVec3f ( axis2 ) << " )!" << endl;
}

BulletSliderJoint::JointDatabase
  BulletSliderJoint::database ( "SliderJoint", &newInstance<BulletSliderJoint> );

BulletSliderJoint::BulletSliderJoint ( PhysicsEngineParameters::JointParameters& jointParameters ) :
BulletJoint ( jointParameters ),
  sliderJoint ( NULL ),
  minSeparation ( 0 ),
  maxSeparation ( 0 ),
  slider_force( 0 ),
  simulationCallbackId( -1 )
{
  PhysicsEngineParameters::SliderJointParameters* sliderJointParameters=
    dynamic_cast<PhysicsEngineParameters::SliderJointParameters*> ( &jointParameters );

  // If a body is missing, replace it with the fixed body
  // This method is used instead of the single body btSliderConstraint constructor
  // because the resulting slider direction is consistant with the ODE implementation
  if ( !body1 )
    body1= &getFixedBody();
  if ( !body2 )
    body2= &getFixedBody();

  btTransform frameA, frameB;
  setAxis ( sliderJointParameters->getAxis(), frameA, frameB );

  // Create slider joint (frame a and b are set later in setParameters)

  //sliderJoint= new btGeneric6DofConstraint ( *body1, *body2, frameA, frameB, false );
  //sliderJoint->setAngularLowerLimit ( btVector3 ( 0, 0, 0 ) );
  //sliderJoint->setAngularUpperLimit ( btVector3 ( 0, 0, 0 ) );
  //sliderJoint->setLinearUpperLimit ( btVector3 ( 0, 0, 0 ) );
  //sliderJoint->setLinearLowerLimit ( btVector3 ( 0, 0, 0 ) );
  ////sliderJoint->getRigidBodyA().setAngularFactor(btVector3(1,0,0));
  ////sliderJoint->getRigidBodyB().setAngularFactor(btVector3(1,0,0));

  sliderJoint= new btSliderConstraint ( *body1, *body2, frameA, frameB, true );

  joint.reset ( sliderJoint );
}

// Set the bullet joint's parameters using the parameter values specified in jointParameters
void BulletSliderJoint::setParameters ( PhysicsEngineParameters::JointParameters& jointParameters )
{
  PhysicsEngineParameters::SliderJointParameters* sliderJointParameters=
    dynamic_cast<PhysicsEngineParameters::SliderJointParameters*> ( &jointParameters );
  BulletCallbacks::BulletSpecificData* bullet_data= 
    static_cast<BulletCallbacks::BulletSpecificData*>(sliderJointParameters->getEngine()->getEngineSpecificData());

  // Motor velocity
  if( sliderJointParameters->haveMotorTarget() ) {
    sliderJoint->setPoweredLinMotor( true );
    sliderJoint->setMaxLinMotorForce( maxMotorForce );
    sliderJoint->setTargetLinMotorVelocity ( -sliderJointParameters->getMotorTarget() );
  }

  // axis
  if ( sliderJointParameters->haveAxis() )
    setAxis ( sliderJointParameters->getAxis(), sliderJoint->getFrameOffsetA(), sliderJoint->getFrameOffsetB() );

  // minSeparation
  if( sliderJointParameters->haveMinSeparation() )
    minSeparation= sliderJointParameters->getMinSeparation()*bullet_data->m_worldScale;


  // maxSepartion
  if( sliderJointParameters->haveMaxSeparation() )
    maxSeparation= sliderJointParameters->getMaxSeparation()*bullet_data->m_worldScale;

  if ( sliderJointParameters->haveMinSeparation() || sliderJointParameters->haveMaxSeparation() ) {
    //sliderJoint->setLinearUpperLimit ( btVector3 ( -minSeparation, 0, 0 ) );
    //sliderJoint->setLinearLowerLimit ( btVector3 ( -maxSeparation, 0, 0 ) );
    sliderJoint->setUpperLinLimit (-minSeparation );
    sliderJoint->setLowerLinLimit (-maxSeparation );
    //sliderJoint->setPoweredLinMotor(true);

  }

  // Set bounce value for joint limits, a.k.a restitution
  if( sliderJointParameters->haveStopBounce() )
    //sliderJoint->getTranslationalLimitMotor()->m_restitution= sliderJointParameters->getStopBounce();
    sliderJoint->setRestitutionLimLin(btScalar(sliderJointParameters->getStopBounce()));

  // Set error correction value for joint limits
  if( sliderJointParameters->haveStopErrorCorrection() )
    sliderJoint->setParam ( BT_CONSTRAINT_STOP_ERP, sliderJointParameters->getStopErrorCorrection() );

  if( sliderJointParameters->haveSliderForce() ) {
    slider_force = sliderJointParameters->getSliderForce()*bullet_data->m_worldScale;
    if( simulationCallbackId < 0 ) {
      if( slider_force != 0 )
        simulationCallbackId= physicsEngineThread->asynchronousCallback ( &BulletSliderJoint::updateSlider, this );
    } else if( slider_force == 0 ) {
      physicsEngineThread->removeAsynchronousCallbackNoLock( simulationCallbackId );
      simulationCallbackId = -1;
}
  } else if( simulationCallbackId >= 0 ) {
    physicsEngineThread->removeAsynchronousCallbackNoLock( simulationCallbackId );
    simulationCallbackId = -1;
  }
}

// Modify the jointParameters argument to reflect the bullet joint's current parameters
void BulletSliderJoint::getParameters ( PhysicsEngineParameters::JointParameters& jointParameters )
{
  PhysicsEngineParameters::SliderJointParameters* sliderJointParameters=
    dynamic_cast<PhysicsEngineParameters::SliderJointParameters*> ( &jointParameters );
  BulletCallbacks::BulletSpecificData* bullet_data= 
    static_cast<BulletCallbacks::BulletSpecificData*>(sliderJointParameters->getEngine()->getEngineSpecificData());

  btVector3 diff = sliderJoint->getRigidBodyB().getCenterOfMassPosition() - sliderJoint->getRigidBodyA().getCenterOfMassPosition();
  H3DFloat sep = diff.length();
  btVector3 globalAxis = body1->getWorldTransform().inverse() * btVector3(1,0,0);
  if ( diff.dot(globalAxis) < 0) {
    sep = -sep;
  }
  if( sliderJointParameters->haveSeparation() ) {
    sliderJointParameters->setSeparation ( -sliderJoint->getLinearPos() / bullet_data->m_worldScale );
  }

  // SeparationRate rate of change of slider separation
  if( sliderJointParameters->haveSeparationRate() ) {
    separationRate.setCurrentValue ( -sliderJoint->getLinearPos() / bullet_data->m_worldScale );
    sliderJointParameters->setSeparationRate( separationRate );
  }
}

// Set the slider joint axis (in world coords)
// Point between current body positions taken to be a point on the axis
void BulletSliderJoint::setAxis ( const Vec3f& axis, btTransform& frameA, btTransform& frameB )
{
  // Calculate the origin of the slider constraint
  // To be consistant with ODE implementation, this is the point in the middle of the two
  // body's starting positions. If only one body is present, then use its position only.
  btVector3 center ( 0, 0, 0 );
  if ( body1 != &getFixedBody() )
    center+= body1->getWorldTransform().getOrigin();
  if ( body2 != &getFixedBody() )
    center+= body2->getWorldTransform().getOrigin();

  if ( body1 != &getFixedBody() && body2 != &getFixedBody() )
    center/= 2;

  // Reverse engineered Bullet documentation!:
  // * Slider axis is local x-axis
  // * frameA is transformation from BodyA space to the constraint space
  // * frameB is transformation from BodyB space to the constraint space

  // Axis of slider in world coordinates
  btVector3 xAxis= tobtVector3 ( axis );
  xAxis.normalize();

  // Create the slider transform in world coordinates
  // Slider axis should be local x-axis
  btTransform constraintSpace = getAxisTransformX ( xAxis );
  constraintSpace.setOrigin ( center );

  // frameA is transformation from BodyA space to the constraint space. Markus disagrees, he think it is from constraint space to bodyA space.
  frameA= body1->getWorldTransform().inverse() * constraintSpace;

  // frameB is transformation from BodyB space to the constraint space. Markus disagrees, he think it is from constraint space to bodyA space.
  frameB= body2->getWorldTransform().inverse() * constraintSpace;
}

// Callback used to update slider simulation
H3DUtil::PeriodicThread::CallbackCode BulletSliderJoint::updateSlider ( void *data )
{
  static_cast < BulletSliderJoint* > ( data )->update ();
  return H3DUtil::PeriodicThread::CALLBACK_CONTINUE;
}

// Member function called by static callback, used to update motor simulation
void BulletSliderJoint::update ()
{
  btVector3 the_force = btVector3( slider_force, 0, 0 );
  if( body1 ) {
    body1->applyCentralForce( body1->getWorldTransform().getBasis() * sliderJoint->getFrameOffsetA().getBasis() * the_force );
  }
  if( body2 ) {
    body2->applyCentralForce( -(body2->getWorldTransform().getBasis() * sliderJoint->getFrameOffsetB().getBasis() * the_force) );
  }
}

// Remove joint from simulation
// Override to remove simulation callback
void BulletSliderJoint::remove ()
{
  if( simulationCallbackId >= 0 ) {
    physicsEngineThread->removeAsynchronousCallbackNoLock ( simulationCallbackId );
    simulationCallbackId = -1;
  }

  BulletJoint::remove();
}

BulletMotorJoint::JointDatabase
  BulletMotorJoint::database ( "MotorJoint", &newInstance<BulletMotorJoint> );

BulletMotorJoint::BulletMotorJoint ( PhysicsEngineParameters::JointParameters& jointParameters ) :
BulletJoint ( jointParameters ),
  simulationCallbackId ( -1 ),
  torque1 ( 0 ),
  torque2 ( 0 ),
  torque3 ( 0 ),
  enabledAxes ( 0 )
{
}

// Set the bullet joint's parameters using the parameter values specified in jointParameters
void BulletMotorJoint::setParameters ( PhysicsEngineParameters::JointParameters& jointParameters )
{
  PhysicsEngineParameters::MotorJointParameters* motorJointParameters=
    dynamic_cast<PhysicsEngineParameters::MotorJointParameters*> ( &jointParameters );
  BulletCallbacks::BulletSpecificData* bullet_data= 
    static_cast<BulletCallbacks::BulletSpecificData*>(motorJointParameters->getEngine()->getEngineSpecificData());

  if( motorJointParameters->haveEnabledAxes() )
    enabledAxes= motorJointParameters->getEnabledAxes();

  // motor1Axis
  if( motorJointParameters->haveMotor1Axis() )
    axis1= motorJointParameters->getMotor1Axis();

  // motor2Axis
  if( motorJointParameters->haveMotor2Axis() && body1 )
    axis2= toMatrix4f ( body1->getWorldTransform().inverse() ).getRotationPart() *
    motorJointParameters->getMotor2Axis();

  // motor3Axis
  if( motorJointParameters->haveMotor3Axis() && body2 )
    axis3= toMatrix4f ( body2->getWorldTransform().inverse() ).getRotationPart() *
    motorJointParameters->getMotor3Axis();

  // axis1Torque
  if( motorJointParameters->haveAxis1Torque() )
    torque1= motorJointParameters->getAxis1Torque()*bullet_data->m_worldScale*bullet_data->m_worldScale;

  // axis2Torque
  if( motorJointParameters->haveAxis2Torque() )
    torque2= motorJointParameters->getAxis2Torque()*bullet_data->m_worldScale*bullet_data->m_worldScale;

  // axis3Torque
  if( motorJointParameters->haveAxis3Torque() )
    torque3= motorJointParameters->getAxis3Torque()*bullet_data->m_worldScale*bullet_data->m_worldScale;
}

// Modify the jointParameters argument to reflect the bullet joint's current parameters
void BulletMotorJoint::getParameters ( PhysicsEngineParameters::JointParameters& jointParameters )
{
  //PhysicsEngineParameters::MotorJointParameters* motorJointParameters=
  //  dynamic_cast<PhysicsEngineParameters::MotorJointParameters*> ( &jointParameters );

}

// Add joint to simulation
// Override to add simulation callback
void BulletMotorJoint::add ()
{
  simulationCallbackId= physicsEngineThread->asynchronousCallback ( &BulletMotorJoint::updateMotor, this );
}

// Remove joint from simulation
// Override to remove simulation callback
void BulletMotorJoint::remove ()
{
  if ( simulationCallbackId != -1 ) {
    physicsEngineThread->removeAsynchronousCallbackNoLock ( simulationCallbackId );
  }
}

// Member function called by static callback, used to update motor simulation
void BulletMotorJoint::update ()
{
  if ( enabledAxes > 0 )
  {
    if ( axis1.length() > btScalar(0) )
    {
      // Apply torque on axis1
      if ( body1 )
      {
        H3DUtil::Quaternion axisTorque1 ( Rotation ( axis1, torque1 ) );
        body1->applyTorque ( tobtVector3 ( axisTorque1.toEulerAngles () ) );
      }

      if ( body2 )
      {
        H3DUtil::Quaternion axisTorque2 ( Rotation ( axis1, -torque1 ) );
        body2->applyTorque ( tobtVector3 ( axisTorque2.toEulerAngles () ) );
      }
    }

    if ( enabledAxes > 1 && body1 )
    {
      if ( axis2.length() > btScalar(0) )
      {
        // Apply torque on axis2 - anchored to body1
        Vec3f axis= toMatrix4f ( body1->getWorldTransform() ).getRotationPart() * axis2;

        H3DUtil::Quaternion axisTorque1 ( Rotation ( axis, torque2 ) );
        body1->applyTorque ( tobtVector3 ( axisTorque1.toEulerAngles () ) );

        if ( body2 )
        {
          H3DUtil::Quaternion axisTorque2 ( Rotation ( axis, -torque2 ) );
          body2->applyTorque ( tobtVector3 ( axisTorque2.toEulerAngles () ) );
        }
      }

      if ( enabledAxes > 2 && body2 )
      {
        if ( axis3.length() > btScalar(0) )
        {
          // Apply toque on axis3 - anchored to body2
          Vec3f axis= toMatrix4f ( body2->getWorldTransform() ).getRotationPart() * axis3;

          if ( body1 )
          {
            H3DUtil::Quaternion axisTorque1 ( Rotation ( axis, torque3 ) );
            body1->applyTorque ( tobtVector3 ( axisTorque1.toEulerAngles () ) );
          }

          H3DUtil::Quaternion axisTorque2 ( Rotation ( axis, -torque3 ) );
          body2->applyTorque ( tobtVector3 ( axisTorque2.toEulerAngles () ) );
        }
      }
    }
  }
}

// Callback used to update motor simulation
H3DUtil::PeriodicThread::CallbackCode BulletMotorJoint::updateMotor ( void *data )
{
  static_cast < BulletMotorJoint* > ( data )->update ();
  return H3DUtil::PeriodicThread::CALLBACK_CONTINUE;
}

BulletSoftBodyLinearJoint::JointDatabase
  BulletSoftBodyLinearJoint::database ( "SoftBodyLinearJoint", &newInstance<BulletSoftBodyLinearJoint> );

// A class to manage the link between a SoftBodyLinearJoint and its Bullet implementation
BulletSoftBodyLinearJoint::BulletSoftBodyLinearJoint ( JointParameters& jointParameters ) 
  : BulletJoint ( jointParameters ),
  softBody1 ( NULL ), softBody2 ( NULL ), bt_joint ( NULL )
{
  SoftBodyLinearJointParameters* sbLinearJointParameters=
    dynamic_cast<SoftBodyLinearJointParameters*> ( &jointParameters );
  BulletCallbacks::BulletSpecificData* bullet_data= 
    static_cast<BulletCallbacks::BulletSpecificData*>(sbLinearJointParameters->getEngine()->getEngineSpecificData());

  // Body 1 should always be a soft body
  if ( BulletSoftBody* sb= (BulletSoftBody*)jointParameters.getBody1() ) {
    softBody1= &sb->getSoftBody();
  }

  // Body 2 may be soft or rigid
  if ( BulletBody* b= (BulletBody*)jointParameters.getBody2() ) {
    if ( BulletSoftBody* sb= dynamic_cast<BulletSoftBody*>(b) ) {
      softBody2= &sb->getSoftBody();
    }
  }

  if ( sbLinearJointParameters->haveAnchorPoint() )
    anchorPoint= sbLinearJointParameters->getAnchorPoint() * bullet_data->m_worldScale;
}

// Set the bullet joint's parameters using the parameter values specified in jointParameters
void BulletSoftBodyLinearJoint::setParameters ( JointParameters& jointParameters ) {
  SoftBodyLinearJointParameters* sbLinearJointParameters=
    dynamic_cast<SoftBodyLinearJointParameters*> ( &jointParameters );
  BulletCallbacks::BulletSpecificData* bullet_data= 
    static_cast<BulletCallbacks::BulletSpecificData*>(sbLinearJointParameters->getEngine()->getEngineSpecificData());

  if ( sbLinearJointParameters->haveAnchorPoint() ) {
    anchorPoint= sbLinearJointParameters->getAnchorPoint() * bullet_data->m_worldScale;
    if ( bt_joint != NULL ) {
      // If currently added, re-add to refresh anchor point
      remove();
      add();
    }
  }
}

// Modify the jointParameters argument to reflect the bullet joint's current parameters
void BulletSoftBodyLinearJoint::getParameters ( JointParameters& jointParameters ) {

}

void BulletSoftBodyLinearJoint::add () {
  btSoftBody::LJoint::Specs specs;
  specs.position= tobtVector3 ( anchorPoint );

  // Save the joint index for removal
  int nr_joints_before = softBody1->m_joints.size();

  // Create and add the joint
  if ( softBody2 ) {
    // Soft body to soft body
    softBody1->appendLinearJoint ( specs, softBody2 );
  } else if ( body2 ) {
    // Soft body to rigid body
    softBody1->appendLinearJoint ( specs, btSoftBody::Body ( body2 ) );
  } else {
    // Second body omitted: Soft body to static rigid body
    softBody1->appendLinearJoint ( specs, btSoftBody::Body ( &getFixedBody() ) );
  }

  if( softBody1->m_joints.size() > nr_joints_before ) {
    bt_joint = softBody1->m_joints[nr_joints_before];
  }
}

void BulletSoftBodyLinearJoint::remove () {
  if( bt_joint ) {
    if( softBody1->m_joints.findLinearSearch( bt_joint ) != softBody1->m_joints.size() ) {
      // This will set the joint up for deletion if the soft body simulation is still running
      // If the simulation is not running anymore then the joint will be cleaned up
      // when the soft body is destroyed.
      bt_joint->m_delete = true;
      bt_joint = NULL;
    }
  }
}

BulletSoftBodyAngularJoint::JointDatabase
  BulletSoftBodyAngularJoint::database ( "SoftBodyAngularJoint", &newInstance<BulletSoftBodyAngularJoint> );

// A class to manage the link between a BulletSoftBodyAngularJoint and its Bullet implementation
BulletSoftBodyAngularJoint::BulletSoftBodyAngularJoint ( JointParameters& jointParameters ) 
  : BulletJoint ( jointParameters ),
  softBody1 ( NULL ), softBody2 ( NULL ), bt_joint ( NULL )
{
  SoftBodyAngularJointParameters* sbAngularJointParameters=
    dynamic_cast<SoftBodyAngularJointParameters*> ( &jointParameters );

  // Body 1 should always be a soft body
  if ( BulletSoftBody* sb= (BulletSoftBody*)jointParameters.getBody1() ) {
    softBody1= &sb->getSoftBody();
  }

  // Body 2 may be soft or rigid
  if ( BulletBody* b= (BulletBody*)jointParameters.getBody2() ) {
    if ( BulletSoftBody* sb= dynamic_cast<BulletSoftBody*>(b) ) {
      softBody2= &sb->getSoftBody();
    }
  }

  axis= sbAngularJointParameters->getAxis();

}

// Set the bullet joint's parameters using the parameter values specified in jointParameters
void BulletSoftBodyAngularJoint::setParameters ( JointParameters& jointParameters ) {
  SoftBodyAngularJointParameters* sbAngularJointParameters=
    dynamic_cast<SoftBodyAngularJointParameters*> ( &jointParameters );

  if ( sbAngularJointParameters->haveAxis() ) {
    axis= sbAngularJointParameters->getAxis();
    if ( bt_joint != NULL ) {
      // If currently added, re-add to refresh anchor point
      remove();
      add();
    }
  }
}

// Modify the jointParameters argument to reflect the bullet joint's current parameters
void BulletSoftBodyAngularJoint::getParameters ( JointParameters& jointParameters ) {

}

void BulletSoftBodyAngularJoint::add () {
  btSoftBody::AJoint::Specs specs;
  specs.axis= tobtVector3 ( axis );

  // Save the joint index for removal
  int nr_joints_before = softBody1->m_joints.size();

  // Create and add the joint
  if ( softBody2 ) {
    // Soft body to soft body
    softBody1->appendAngularJoint ( specs, softBody2 );
  } else if ( body2 ) {
    // Soft body to rigid body
    softBody1->appendAngularJoint ( specs, btSoftBody::Body ( body2 ) );
  } else {
    // Second body omitted: Soft body to static rigid body
    softBody1->appendAngularJoint ( specs, btSoftBody::Body ( &getFixedBody() ) );
  }

  if( softBody1->m_joints.size() > nr_joints_before ) {
    bt_joint = softBody1->m_joints[nr_joints_before];
  }
}

void BulletSoftBodyAngularJoint::remove () {
  if( bt_joint ) {
    if( softBody1->m_joints.findLinearSearch( bt_joint ) != softBody1->m_joints.size() ) {
      // This will set the joint up for deletion if the soft body simulation is still running
      // If the simulation is not running anymore then the joint will be cleaned up
      // when the soft body is destroyed.
      bt_joint->m_delete = true;
      bt_joint = NULL;
    }
  }
}

#endif

