//////////////////////////////////////////////////////////////////////////////
//    Copyright 2019, SenseGraphics AB
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
/// \file PhysX4Joints.h
/// \brief Header file for PhysX4Joint classes, which maintain a link between 
/// RigidBodyPhysics joint types and their PhysX4 implementations
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __PHYSX4JOINTS__
#define __PHYSX4JOINTS__

#include <H3D/H3DPhysics/PhysicsEngineThread.h>

#ifdef HAVE_PHYSX4
#include <PxPhysicsAPI.h>

namespace H3D {

  /// \brief Base for classes managing the link between a RigidBodyPhysics joint type
  /// and its corresponding implementation in PhysX4
  class H3DPHYS_API PhysX4Joint {
  public:
    /// Constructor
    PhysX4Joint( PhysicsEngineParameters::ConstraintParameters& jointParameters );

    /// Destructor
    ///
    /// The joint is also removed when it is destroyed if it has not already
    /// been removed with remove()
    virtual ~PhysX4Joint();

    /// Set the PhysX4 joint's parameters using the parameter values specified in jointParameters
    virtual void setParameters( PhysicsEngineParameters::ConstraintParameters& jointParameters ) {}

    /// Modify the jointParameters argument to reflect the PhysX4 joint's current parameters
    virtual void getParameters( PhysicsEngineParameters::ConstraintParameters& jointParameters ) {}

    /// Create a joint instance of the type and parameters specified
    static PhysX4Joint* createJoint( PhysicsEngineParameters::ConstraintParameters& jointParameters );

    /// Remove joint from simulation
    virtual void remove();
  protected:
    /// Axis enumerations
    struct Axis {
      enum e { X, Y, Z };
    };

    /// Body enumerations
    struct Body {
      enum e { A, B, BOTH };
    };

    /// Set the position and orientation of the joint in world space
    ///
    /// The local joint frames are set based on the current position and orientation
    /// of the attached body/ies
    ///
    void setJointFrame( const Vec3f& position, const Rotation& orientation = Rotation(), Body::e bodies = Body::BOTH );

    /// Gets the position and orientation of the joint in world space relative
    /// to the specified body
    ///
    /// \param body Selects which body to get the matrix.
    ///
    Matrix4f getJointFrame( Body::e body );

    /// Gets the position and orientation of the joint in world space
    Matrix4f getJointFrame() {
      return global_frame;
    }

    /// Get the current joint rotation about a given axis
    H3DFloat getRotation( Axis::e axis, Body::e axisRelativeTo = Body::A );

    /// Template function to construct a joint of the specified type and parameters
    template < typename T >
    static PhysX4Joint* newInstance( PhysicsEngineParameters::ConstraintParameters& jointParameters ) { return new T( jointParameters ); }
    typedef PhysX4Joint* (*CreateFunc)(PhysicsEngineParameters::ConstraintParameters& jointParameters);

    /// Used to register PhysX4Joint class types by type name
    /// Add a static instance to each joint subclass and specify type name and create function
    struct JointDatabase {
      JointDatabase( std::string name, CreateFunc createFunc ) {
        map[name] = createFunc;
      }

      typedef std::map<std::string, CreateFunc> JointMap;
      static H3DPHYS_API JointMap map;
    };

    /// The PhysX4 joint
    physx::PxJoint* joint;

    /// The joint frame in global coordinates
    Matrix4f global_frame;

  };

  /// A class to manage the link between a FixedJoint and its PhysX4 implementation
  class PhysX4FixedJoint : public PhysX4Joint {
  public:
    /// Constructor
    PhysX4FixedJoint( PhysicsEngineParameters::ConstraintParameters& jointParameters );

    /// Set the PhysX4 joint's parameters using the parameter values specified in jointParameters
    virtual void setParameters( PhysicsEngineParameters::ConstraintParameters& jointParameters );

    /// Modify the jointParameters arg
  protected:

    /// Register the joint type
    static JointDatabase database;
  };

  /// A class to manage the link between a BallJoint and its PhysX4 implementation
  class PhysX4BallJoint : public PhysX4Joint {
  public:
    /// Constructor
    PhysX4BallJoint( PhysicsEngineParameters::ConstraintParameters& jointParameters );

    /// Set the PhysX4 joint's parameters using the parameter values specified in jointParameters
    virtual void setParameters( PhysicsEngineParameters::ConstraintParameters& jointParameters );

    /// Modify the jointParameters argument to reflect the PhysX4 joint's current parameters
    virtual void getParameters( PhysicsEngineParameters::ConstraintParameters& jointParameters );

  protected:

    Vec2f limit_contact_distance;

    /// Register the joint type
    static JointDatabase database;
  };

  /// A class to manage the link between a SingleAxisHingeJoint and its PhysX4 implementation
  class PhysX4SingleAxisHingeJoint : public PhysX4Joint {
  public:
    /// Constructor
    PhysX4SingleAxisHingeJoint( PhysicsEngineParameters::ConstraintParameters& jointParameters );

    /// Set the PhysX4 joint's parameters using the parameter values specified in jointParameters
    virtual void setParameters( PhysicsEngineParameters::ConstraintParameters& jointParameters );

    /// Modify the jointParameters argument to reflect the PhysX4 joint's current parameters
    virtual void getParameters( PhysicsEngineParameters::ConstraintParameters& jointParameters );

  protected:

    /// Rate of change of hinge angle
    RateOfChange hingeAngleRate;

    /// Register the joint type
    static JointDatabase database;
  };

  /// An abstract base class for joints that use a Px6DJoint
  class PhysX46DoFJoint : public PhysX4Joint {
  public:
    /// Constructor
    PhysX46DoFJoint( PhysicsEngineParameters::ConstraintParameters& jointParameters );

  protected:

    /// Force axis1 and axis2 to be perpendicular and set the 3rd perpendicular axis
    void makeAxesPerpendicular();

    /// Rate of change of hinge1 angle
    RateOfChange hinge1AngleRate;

    /// Rate of change of hinge2 angle
    RateOfChange hinge2AngleRate;

    /// Rate of change of hinge3 angle
    RateOfChange hinge3AngleRate;

    Vec3f axis1;
    Vec3f axis2;
    Vec3f axis3;
  };

  /// A class to manage the link between a DoubleAxisHingeJoint and its PhysX4 implementation
  class PhysX4DoubleAxisHingeJoint : public PhysX46DoFJoint {
  public:
    /// Constructor
    PhysX4DoubleAxisHingeJoint( PhysicsEngineParameters::ConstraintParameters& jointParameters );

    /// Set the PhysX4 joint's parameters using the parameter values specified in jointParameters
    virtual void setParameters( PhysicsEngineParameters::ConstraintParameters& jointParameters );

    /// Modify the jointParameters argument to reflect the PhysX4 joint's current parameters
    virtual void getParameters( PhysicsEngineParameters::ConstraintParameters& jointParameters );

  protected:

    /// Register the joint type
    static JointDatabase database;
  };

  /// A class to manage the link between a UniversalJoint and its PhysX4 implementation
  class PhysX4UniversalJoint : public PhysX46DoFJoint {
  public:
    /// Constructor
    PhysX4UniversalJoint( PhysicsEngineParameters::ConstraintParameters& jointParameters );

    /// Set the PhysX4 joint's parameters using the parameter values specified in jointParameters
    virtual void setParameters( PhysicsEngineParameters::ConstraintParameters& jointParameters );

    /// Modify the jointParameters argument to reflect the PhysX4 joint's current parameters
    virtual void getParameters( PhysicsEngineParameters::ConstraintParameters& jointParameters );

  protected:

    Vec2f limit_contact_distance;

    /// Register the joint type
    static JointDatabase database;
  };

  /// A class to manage the link between a SliderJoint and its PhysX4 implementation
  class PhysX4SliderJoint : public PhysX4Joint {
  public:
    /// Constructor
    PhysX4SliderJoint( PhysicsEngineParameters::ConstraintParameters& jointParameters );
    /// Destructor
    ~PhysX4SliderJoint();

    /// Set the PhysX4 joint's parameters using the parameter values specified in jointParameters
    virtual void setParameters( PhysicsEngineParameters::ConstraintParameters& jointParameters );

    /// Modify the jointParameters argument to reflect the PhysX4 joint's current parameters
    virtual void getParameters( PhysicsEngineParameters::ConstraintParameters& jointParameters );

    /// Remove joint from simulation
    /// Override to remove simulation callback
    virtual void remove();

    PhysicsEngineThread * physicsEngineThread;
  protected:

    /// Rate of change of separation
    RateOfChange separationRate;

    /// Member function called by static callback, used to update simulation if sliderForce is added.
    void update();

    /// Callback used to update sliderForce motor simulation.
    static H3DUtil::PeriodicThread::CallbackCode updateSlider( void *data );

    /// Handle for simulation callback
    int simulationCallbackId;

    /// Force to be applied if non-zero.
    physx::PxReal slider_force;

    /// Mode of the slider_force
    physx::PxForceMode::Enum slider_force_mode;

    /// Scale the slider force of body2.
    H3DFloat body2_slider_force_scale;

    /// Register the joint type
    static JointDatabase database;

    /// Set to True after the parameters are set.
    bool initialized;
  };

  /// A class to manage the link between a DistanceJoint and its PhysX4 implementation
  class PhysX4DistanceJoint : public PhysX4Joint {
  public:
    /// Constructor
    PhysX4DistanceJoint( PhysicsEngineParameters::ConstraintParameters& jointParameters );

    /// Set the PhysX4 joint's parameters using the parameter values specified in jointParameters
    virtual void setParameters( PhysicsEngineParameters::ConstraintParameters& jointParameters );

    /// Modify the jointParameters argument to reflect the PhysX4 joint's current parameters
    virtual void getParameters( PhysicsEngineParameters::ConstraintParameters& jointParameters );

  protected:

    /// Register the joint type
    static JointDatabase database;
  };

  /// A generic 6DOF joint
  class PhysX4Generic6DofJoint : public PhysX46DoFJoint {
  public:
    /// Constructor
    PhysX4Generic6DofJoint( PhysicsEngineParameters::ConstraintParameters& jointParameters );

    /// Set the PhysX4 joint's parameters using the parameter values specified in jointParameters
    virtual void setParameters( PhysicsEngineParameters::ConstraintParameters& jointParameters );

    /// Modify the jointParameters argument to reflect the PhysX4 joint's current parameters
    virtual void getParameters( PhysicsEngineParameters::ConstraintParameters& jointParameters );

  protected:

    /// The current anchor point, used to rebuild the frame offsets when axis is set
    Vec3f anchorPoint;

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

    physx::PxJointLinearLimit l_limit;
    physx::PxJointLimitCone c_limit;
    physx::PxJointAngularLimitPair t_limit;

    /// Register the joint type
    static JointDatabase database;
  };


}

#endif // HAVE_PHYSX4

#endif
