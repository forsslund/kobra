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
/// \file BulletJoints.h
/// \brief Header file for BulletJoint classes, which maintain a link between 
/// RigidBodyPhysics joint types and their Bullet implementations
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __BULLETJOINTS__
#define __BULLETJOINTS__

#include <H3D/H3DPhysics/PhysicsEngineThread.h>
#include <H3DUtil/Vec3f.h>

#ifdef HAVE_BULLET

#if _MSC_VER >= 1500
#pragma warning( disable : 4305 )
#endif

#include <btBulletDynamicsCommon.h>
#include <BulletSoftBody/btSoftBody.h>

#if _MSC_VER >= 1500
#pragma warning( default : 4305 )
#endif

namespace H3D {

#ifdef DOXYGEN_IFDEF_BECAUSE_COND_STATEMENT_WONT_WORK_IN_THIS_CASE
  /// \brief Base for classes managing the link between a RigidBodyPhysics joint type
  /// and its corresponding implementation in Bullet
  class BulletJoint
#else
  /// \brief Base for classes managing the link between a RigidBodyPhysics joint type
  /// and its corresponding implementation in Bullet
  ATTRIBUTE_ALIGNED16( class ) BulletJoint
#endif
  {
  public:
    BT_DECLARE_ALIGNED_ALLOCATOR();
    /// Constructor
    BulletJoint ( PhysicsEngineParameters::JointParameters& jointParameters );

    /// Set the bullet joint's parameters using the parameter values specified in jointParameters
    virtual void setParameters ( PhysicsEngineParameters::JointParameters& jointParameters ) {}

    /// Modify the jointParameters argument to reflect the bullet joint's current parameters
    virtual void getParameters ( PhysicsEngineParameters::JointParameters& jointParameters ) {}

    /// Create a joint instance of the type and parameters specified
    static BulletJoint* createJoint ( PhysicsEngineParameters::JointParameters& jointParameters );

    /// Add joint to simulation
    virtual void add ();

    /// Remove joint from simulation
    virtual void remove ();

  protected:

    /// Return a reference to a shared static rigid body to emulate joints fixed to space
    static btRigidBody& getFixedBody();  

    /// Template function to construct a joint of the specified type and parameters
    template < typename T > 
    static BulletJoint* newInstance( PhysicsEngineParameters::JointParameters& jointParameters ) { return new T ( jointParameters ); }
    typedef BulletJoint* (*CreateFunc)( PhysicsEngineParameters::JointParameters& jointParameters );

    /// Used to register BulletJoint class types by type name
    /// Add a static instance to each joint subclass and specify type name and create function
    struct JointDatabase
    {
      JointDatabase ( std::string name, CreateFunc createFunc )
      {
        map[name]= createFunc;
      }

      typedef std::map<std::string,CreateFunc> JointMap;
      static JointMap map;
    };

    /// Ptr to physics engine thread
    PhysicsEngineThread* physicsEngineThread;

    /// Bullet rigid body corresponding to H3DJointNode's body1
    btRigidBody* body1;

    /// Bullet rigid body corresponding to H3DJointNode's body2
    btRigidBody* body2;

    /// Bullet representation of joint
    auto_ptr < btTypedConstraint > joint;

    static const H3DFloat maxMotorForce;
    static const H3DFloat maxMotorTorque;
  };

  /// A class to manage the link between a BallJoint and its Bullet implementation
  class BulletBallJoint : public BulletJoint
  {
  public:
    /// Constructor
    BulletBallJoint ( PhysicsEngineParameters::JointParameters& jointParameters );

    /// Set the bullet joint's parameters using the parameter values specified in jointParameters
    virtual void setParameters ( PhysicsEngineParameters::JointParameters& jointParameters );

    /// Modify the jointParameters argument to reflect the bullet joint's current parameters
    virtual void getParameters ( PhysicsEngineParameters::JointParameters& jointParameters );

  protected:

    /// Bullet representation of joint
    btPoint2PointConstraint* ballJoint;

    /// Register the joint type
    static JointDatabase database;
  };

  /// A class to manage the link between a SingleAxisHingeJoint and its Bullet implementation
  class BulletSingleAxisHingeJoint : public BulletJoint
  {
  public:
    /// Constructor
    BulletSingleAxisHingeJoint ( PhysicsEngineParameters::JointParameters& jointParameters );

    /// Set the bullet joint's parameters using the parameter values specified in jointParameters
    virtual void setParameters ( PhysicsEngineParameters::JointParameters& jointParameters );

    /// Modify the jointParameters argument to reflect the bullet joint's current parameters
    virtual void getParameters ( PhysicsEngineParameters::JointParameters& jointParameters );

  protected:

    /// Bullet representation of joint
    btHingeConstraint* hingeJoint;

    /// The current anchor point, used to rebuild the frame offsets when axis is set
    btVector3 anchorPoint;

    /// The current bounce factor for the stops/limits
    /// Specifies how much the joint resists velocities which violate the angular limits. 
    /// The lower the value, the less the constraint resists the violation. Recommended value near 1.
    btScalar bounce;

    /// Rate of change of hinge angle
    RateOfChange hingeAngleRate;

    /// Register the joint type
    static JointDatabase database;

    /// The current bias factor for the stops/limits
    /// Controls the strength with which the joint resists angular limit violation.
    btScalar bias;

    /// The current softness factor for the stops/limits
    /// The percentage at which velocity error corrections starts operating.
    /// If you're above the sofness, velocities that would shoot through the limit are slowed down.
    btScalar softness;
  };

  /// A class to manage the link between a DoubleAxisHingeJoint and its Bullet implementation
  class BulletDoubleAxisHingeJoint : public BulletJoint
  {
  public:
    /// Constructor
    BulletDoubleAxisHingeJoint ( PhysicsEngineParameters::JointParameters& jointParameters );

    /// Set the bullet joint's parameters using the parameter values specified in jointParameters
    virtual void setParameters ( PhysicsEngineParameters::JointParameters& jointParameters );

    /// Modify the jointParameters argument to reflect the bullet joint's current parameters
    virtual void getParameters ( PhysicsEngineParameters::JointParameters& jointParameters );

  protected:

    /// Update the frame transforms based on current anchor and axes
    void updateFrames ();

    /// Update spring and damper constants based on current suspensionERP and suspensionCFM
    void updateSuspension ( PhysicsEngineParameters::DoubleAxisHingeJointParameters& hingeJointParameters );

    /// Bullet representation of joint
    btHinge2Constraint* doubleHingeJoint;

    /// The current anchor point, used to rebuild the frame offsets when axis is set
    btVector3 anchorPoint;

    /// The current axis1, used to rebuild the frame offsets when an axis is set
    btVector3 axis1;

    /// The current axis2, used to rebuild the frame offsets when an axis is set
    btVector3 axis2;

    /// Current error correction for suspension
    btScalar suspensionERP;

    /// Current constant force mix for suspension
    btScalar suspensionCFM;

    /// Rate of change of hinge 1 angle
    RateOfChange hinge1AngleRate;

    /// Rate of change of hinge 2 angle
    RateOfChange hinge2AngleRate;

    /// Register the joint type
    static JointDatabase database;
  };

  /// A class to manage the link between a UniversalJoint and its Bullet implementation
  class BulletUniversalJoint : public BulletJoint
  {
  public:
    /// Constructor
    BulletUniversalJoint ( PhysicsEngineParameters::JointParameters& jointParameters );

    /// Set the bullet joint's parameters using the parameter values specified in jointParameters
    virtual void setParameters ( PhysicsEngineParameters::JointParameters& jointParameters );

    /// Modify the jointParameters argument to reflect the bullet joint's current parameters
    virtual void getParameters ( PhysicsEngineParameters::JointParameters& jointParameters );

  protected:

    /// Update the frame transforms based on current anchor and axes
    void updateFrames ();

    /// Bullet representation of joint
    btUniversalConstraint* universalJoint;

    /// The current anchor point, used to rebuild the frame offsets when axis is set
    btVector3 anchorPoint;

    /// The current axis1, used to rebuild the frame offsets when an axis is set
    btVector3 axis1;

    /// The current axis2, used to rebuild the frame offsets when an axis is set
    btVector3 axis2;

    /// Register the joint type
    static JointDatabase database;
  };

  /// A class to manage the link between a SliderJoint and its Bullet implementation
  class BulletSliderJoint : public BulletJoint
  {
  public:
    /// Constructor
    BulletSliderJoint ( PhysicsEngineParameters::JointParameters& jointParameters );

    /// Set the bullet joint's parameters using the parameter values specified in jointParameters
    virtual void setParameters ( PhysicsEngineParameters::JointParameters& jointParameters );

    /// Modify the jointParameters argument to reflect the bullet joint's current parameters
    virtual void getParameters ( PhysicsEngineParameters::JointParameters& jointParameters );

    /// Remove joint from simulation
    /// Override to remove simulation callback
    virtual void remove ();

  protected:
    /// Member function called by static callback, used to update simulation if sliderForce is added.
    void update ();

    /// Callback used to update sliderForce motor simulation.
    static H3DUtil::PeriodicThread::CallbackCode updateSlider( void *data );

    /// Handle for simulation callback
    int simulationCallbackId;

    /// Set the slider joint axis (in world coords)
    /// Point between current body positions taken to be a point on the axis
    void setAxis ( const Vec3f& axis,
      btTransform& frameA, btTransform& frameB );

    /// Bullet representation of joint
    btSliderConstraint* sliderJoint;

    /// Rate of change of slider separation
    RateOfChange separationRate;

    /// Minimum separation of slider
    btScalar minSeparation;

    /// Maximum separation of slider
    btScalar maxSeparation;

    /// Force to be applied if non-zero.
    btScalar slider_force;

    /// Register the joint type
    static JointDatabase database;
  };

  /// A class to manage the link between a MotorJoint and its Bullet implementation
  class BulletMotorJoint : public BulletJoint
  {
  public:
    /// Constructor
    BulletMotorJoint ( PhysicsEngineParameters::JointParameters& jointParameters );

    /// Set the bullet joint's parameters using the parameter values specified in jointParameters
    virtual void setParameters ( PhysicsEngineParameters::JointParameters& jointParameters );

    /// Modify the jointParameters argument to reflect the bullet joint's current parameters
    virtual void getParameters ( PhysicsEngineParameters::JointParameters& jointParameters );

    /// Add joint to simulation
    /// Override to add simulation callback
    virtual void add ();

    /// Remove joint from simulation
    /// Override to remove simulation callback
    virtual void remove ();

  protected:

    /// Member function called by static callback, used to update motor simulation
    void update ();

    /// Callback used to update motor simulation
    static H3DUtil::PeriodicThread::CallbackCode updateMotor ( void *data );

    /// Handle for simulation callback
    int simulationCallbackId;

    Vec3f axis1;

    H3DFloat torque1;

    Vec3f axis2;

    H3DFloat torque2;

    Vec3f axis3;

    H3DFloat torque3;

    int enabledAxes;

    /// Register the joint type
    static JointDatabase database;
  };

  /// A class to manage the link between a BulletGeneric6DOFJoint and its Bullet implementation
  class BulletGeneric6DOFJoint : public BulletJoint
  {
  public:
    /// Constructor
    BulletGeneric6DOFJoint ( PhysicsEngineParameters::JointParameters& jointParameters );

    /// Set the bullet joint's parameters using the parameter values specified in jointParameters
    virtual void setParameters ( PhysicsEngineParameters::JointParameters& jointParameters );

    /// Modify the jointParameters argument to reflect the bullet joint's current parameters
    //virtual void getParameters ( PhysicsEngineParameters::JointParameters& jointParameters );

  protected:

    /// Update the frame transforms based on current anchor and axes
    void updateFrames ();

    /// Bullet representation of joint
    btGeneric6DofConstraint* generic6DOFJoint;

    /// The current anchor point, used to rebuild the frame offsets when axis is set
    btVector3 anchorPoint;

    /// The current axis1, used to rebuild the frame offsets when an axis is set
    btVector3 axis1;

    /// The current axis2, used to rebuild the frame offsets when an axis is set
    btVector3 axis2;

    btVector3 axis3;

    H3DFloat maxAngle1;
    H3DFloat maxAngle2;
    H3DFloat maxAngle3;

    H3DFloat minAngle1;
    H3DFloat minAngle2;
    H3DFloat minAngle3;

    H3DFloat maxLimit1;
    H3DFloat maxLimit2;
    H3DFloat maxLimit3;

    H3DFloat minLimit1;
    H3DFloat minLimit2;
    H3DFloat minLimit3;

    /// Register the joint type
    static JointDatabase database;
  };

  // Soft body joint types

  /// A class to manage the link between a SoftBodyLinearJoint and its Bullet implementation
  class BulletSoftBodyLinearJoint : public BulletJoint
  {
  public:
    /// Constructor
    BulletSoftBodyLinearJoint ( PhysicsEngineParameters::JointParameters& jointParameters );

    /// Set the bullet joint's parameters using the parameter values specified in jointParameters
    virtual void setParameters ( PhysicsEngineParameters::JointParameters& jointParameters );

    /// Modify the jointParameters argument to reflect the bullet joint's current parameters
    virtual void getParameters ( PhysicsEngineParameters::JointParameters& jointParameters );

    /// Add joint to simulation
    /// Override to append joint to soft body
    virtual void add ();

    /// Remove joint to simulation
    /// Override to remove joint from soft body
    virtual void remove ();

  protected:

    /// The soft body involved in the joint
    /// Always present
    btSoftBody* softBody1;

    /// The 2nd soft body involved in the joint
    /// NULL if the second body is rigid
    btSoftBody* softBody2;

    /// The bullet joint that were added to the softbody
    btSoftBody::Joint * bt_joint;

    /// The current anchor point of the joint
    Vec3f anchorPoint;

    /// Register the joint type
    static JointDatabase database;
  };

  /// A class to manage the link between a BulletSoftBodyAngularJoint and its Bullet implementation
  class BulletSoftBodyAngularJoint : public BulletJoint
  {
  public:
    /// Constructor
    BulletSoftBodyAngularJoint ( PhysicsEngineParameters::JointParameters& jointParameters );

    /// Set the bullet joint's parameters using the parameter values specified in jointParameters
    virtual void setParameters ( PhysicsEngineParameters::JointParameters& jointParameters );

    /// Modify the jointParameters argument to reflect the bullet joint's current parameters
    virtual void getParameters ( PhysicsEngineParameters::JointParameters& jointParameters );

    /// Add joint to simulation
    /// Override to append joint to soft body
    virtual void add ();

    /// Remove joint to simulation
    /// Override to remove joint from soft body
    virtual void remove ();

  protected:

    /// The soft body involved in the joint
    /// Always present
    btSoftBody* softBody1;

    /// The 2nd soft body involved in the joint
    /// NULL if the second body is rigid
    btSoftBody* softBody2;

    /// The bullet joint that were added to the softbody
    btSoftBody::Joint * bt_joint;

    /// The current anchor point of the joint
    Vec3f axis;

    /// Register the joint type
    static JointDatabase database;
  };

}

#endif // HAVE_BULLET

#endif
