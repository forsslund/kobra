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
/// \file RigidBody.h
/// \brief Header file for RigidBody, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __RIGIDBODY__
#define __RIGIDBODY__

#include <H3D/X3DNode.h>
#include <H3D/SFFloat.h>
#include <H3D/Box.h>
#include <H3D/Sphere.h>
#include <H3D/Cylinder.h>
#include <H3D/SFVec3f.h>
#include <H3D/SFBool.h>
#include <H3D/MFNode.h>
#include <H3D/MFVec3f.h>
#include <H3D/Shape.h>
#include <H3D/SFMatrix3f.h>
#include <H3D/SFRotation.h>
#include <H3D/PeriodicUpdate.h>
#include <H3D/H3DPhysics/CollidableShape.h>
#include <H3D/H3DPhysics/H3DBodyNode.h>
#include <H3D/H3DPhysics/PhysicsEngineThread.h>
#include <H3D/H3DPhysics/H3DEngineOptions.h>

#ifdef DEBUG_RB_LAG
#include <fstream>
#endif

namespace H3D{  
  /// \ingroup X3DNodes
  /// \class RigidBody
  /// \brief The RigidBody node describes a body and its properties that 
  /// can be affected by the physics model. A body is modelled as a collection
  /// of shapes that describe mass distribution rather than renderable 
  /// geometry. Bodies are connected together using Joints and are 
  /// represented by geometry.
  ///
  /// The geometry field is used to connect the body modelled by the physics
  /// engine implementation to the real geometry of the scene through the use
  /// of collidable nodes. This allows the geometry to be connected directly
  /// to the physics model as well as collision detection. Collidable nodes 
  /// have their location set to the same location as the body instance in 
  /// which they are located. Their position and location are not relative
  /// to this object, unless otherwise defined.
  ///
  ///
  /// The massDensityModel field is used to describe the geometry type and 
  /// dimensions used to calculate the mass density in the physics model. 
  /// This geometry has no renderable property, other than for defining 
  /// the model of the mass density. It is not rendered, nor modified by
  /// the physics model.
  ///
  /// The finiteRotationAxis field specifies a vector around which the 
  /// object rotates when useFiniteRotation is true.
  ///
  /// The useFiniteRotation field is used to influence the way the body's
  /// rotation is calculated. In very fast rotating objects, such as a wheel
  /// of a car, an infinitely small time step can cause the modelling to
  /// explode. The default value is to use the faster infinite mode. Setting
  /// the field value to TRUE uses the finite calculation model. Using the 
  /// finite model is more costly to compute but will be more accurate for
  /// high rotation speed bodies.
  ///
  /// The useGlobalGravity field is used to indicate whether this particular
  /// body should be influenced by the containing RigidBodyCollection's 
  /// gravity setting. A value of TRUE indicates that the gravity is used, 
  /// a value of FALSE indicates that it is not used. This only applies to
  /// this body instance. Contained sub-bodies shall not be affected by this
  /// setting.
  ///
  /// The inertia field represents a inertia tensor matrix. This will be used
  /// if the massDensityModel is NULL.
  ///
  /// The fixed field is used to indicate that this body does not move. Any
  /// calculations involving collisions with this body should take into account
  /// that this body does not move. This is useful for representing objects 
  /// such as the ground, walls etc that can be collided with, have an effect
  /// on other objects, but are not capable of moving themselves.
  ///
  /// The mass field indicates the mass of the body in kilograms. All bodies
  /// shall have a non-zero mass, with the default value of 1 kilogram.
  ///
  /// The damping factor fields allow the user to instruct the implementation
  /// to automatically damp the motion of the body over time. The value of the
  /// field is used to take a multiple of the value calculated in the last 
  /// frame and apply it in opposition to the current motion for this frame. 
  /// Damping is useful to provide an appearance of frictional forces and also
  /// to prevent the body from exploding due to numerical instability of the 
  /// physics model calculations. Damping is proportional to the current 
  /// velocity and/or rotation of the object. The application of damping is
  /// controlled through the use of the autoDamp field. When the value is 
  /// FALSE, no damping is applied. When the value is TRUE, rotational and 
  /// translational damping is calculated and applied. 
  ///
  /// The torques and forces fields define zero or more sets of torque and 
  /// force values that are applied to the object every frame. These are 
  /// continuously applied until reset to zero by the user.
  ///
  /// The velocity fields are used to provide a constant velocity value to
  /// the object every frame. If both forces and velocity are defined, the
  /// velocity is used only on the first frame that the node is active, and
  /// then the forces are applied. The velocity fields then report the changed
  /// values as a result of the application of the physics model in each frame.
  /// Setting a new value to the appropriate field will reset the body's 
  /// velocity for the next frame. Caution should be used in doing this as
  /// the underlying physics models may assume some amount of caching between
  /// time step evaluations and instantaneous velocity changes may lead to 
  /// numerical instability.
  ///
  /// The position and orientation fields are used to set the initial 
  /// conditions of this body's location in world space. After the initial
  /// conditions have been set, these fields are used to report the current
  /// information based on the most recent physics model evaluation. Setting
  /// new values will cause the objects to be moved to the new location and
  /// orientation for the start of the next evaluation cycle. Care should be
  /// used in manually changing the position and orientation as the underlying
  /// physics models may cache information between time step evaluations and 
  /// sudden instantaneous changes may lead to numerical instability.
  ///
  /// The disable fields define conditions for when the body ceases to 
  /// considered as part of the rigid body calculations and should be 
  /// considered as at rest. Due to the numerical instability of physics
  /// models, even bodies initially declared to be at rest may gain some
  /// amount of movement, even when not effected by an external forces. 
  /// These values define tolerances for which the physics model should
  /// start to ignore this object in any calculation, thus resulting in them
  /// being actually at rest and not subject to these instability conditions. 
  /// Once any one of these values is achieved, the body is considered as being
  /// at rest unless acted upon by an external force (e. g., collision or 
  /// action of connected joint). By default, this automatic disabling is 
  /// turned off. It may be enabled by setting the autoDisable field to TRUE.
  ///
  /// The enabled field controls whether the information in this node is 
  /// submitted to the physics engine for processing. If the enabled field
  /// is set TRUE, the node is submitted to the physics engine. If the enabled
  /// field is set FALSE, the node is not submitted to the physics engine for 
  /// processing.
  ///
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../examples/RigidBody/BallJoint.x3d">BallJoint.x3d</a>
  ///     ( <a href="examples/BallJoint.x3d.html">Source</a> )
  ///
  /// \par Internal routes:
  /// \dotfile RigidBody.dot
  class H3DPHYS_API RigidBody : public H3DBodyNode {
  public:

    typedef TypedMFNode< X3DNBodyCollidableNode > MFCollidableNode;

    /// The ValueUpdater field is used to update values in the
    /// PhysicsEngineThread according to changes of fields in the
    /// RigidBody node. More specifically it calls 
    /// PhysicsEngineThread::setRigidBodyParameters with the new values.
    class H3DPHYS_API ValueUpdater: 
      public EventCollectingField< PeriodicUpdate< Field > > {
        virtual void update();
    };

    typedef MFH3DEngineOptions < RigidBody > MFEngineOptions;
    friend class MFH3DEngineOptions < RigidBody >;

    typedef TypedSFNode < MatrixTransform > SFTransformNode;

    /// Constructor.
    RigidBody(
      Inst< SFNode > _metadata = 0,
      Inst< SFFloat > _angularDampingFactor = 0,
      Inst< SFVec3f > _angularVelocity = 0,
      Inst< SFBool > _autoDamp = 0,
      Inst< SFBool > _autoDisable = 0,
      Inst< SFVec3f > _centerOfMass = 0,
      Inst< SFFloat > _disableAngularSpeed = 0,
      Inst< SFFloat > _disableLinearSpeed = 0,
      Inst< SFFloat > _disableTime = 0,
      Inst< SFBool > _enabled = 0,
      Inst< SFVec3f > _finiteRotationAxis = 0,
      Inst< SFBool > _fixed = 0,
      Inst< MFVec3f > _forces = 0,
      Inst< MFCollidableNode > _geometry = 0,
      Inst< SFMatrix3f > _inertia = 0,
      Inst< SFFloat > _linearDampingFactor = 0,
      Inst< SFVec3f > _linearVelocity = 0,
      Inst< SFFloat > _mass = 0,
      Inst< SFNode > _massDensityModel = 0,
      Inst< SFRotation > _orientation = 0,
      Inst< SFVec3f > _position = 0,
      Inst< MFVec3f > _torques = 0,
      Inst< SFBool > _useFiniteRotation = 0,
      Inst< SFBool > _useGlobalGravity = 0,
      Inst< SFBool > _kinematicControl = 0,
      Inst< ValueUpdater > _valueUpdater = 0,
      Inst< MFEngineOptions > _engineOptions = 0,
      Inst< SFTransformNode > _transform = 0);

    /// Returns the default xml containerField attribute value.
    /// For this node it is "bodies".
    virtual string defaultXMLContainerField() {
      return "bodies";
    }

    /// Traverse the scene graph.
    virtual void traverseSG(H3D::TraverseInfo &ti);

    /// Render the collidables of the body.
    virtual void renderCollidable( bool render_only_enabled_collidables );

    /// The angularDampingFactor field allow the user
    /// to automatically damp the motion of the body over time. 
    /// Specifically it adds a torque to the rigid body that is 
    /// -angularVelocity * factor.
    /// Damping is useful to provide an appearance of frictional forces and also
    /// to prevent the body from exploding due to numerical instability of the 
    /// physics model calculations. This field is only used if the 
    /// autoDamp field is set to TRUE. 
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value: </b> 0.001
    /// <b>Value range: </b> [0,1]
    ///
    /// \dotfile RigidBody_angularDampingFactor.dot
    auto_ptr< SFFloat > angularDampingFactor;

    /// The angularVelocity field is the angular velocity of the rigid mody in
    /// around each axis in radians / s. It is used to provide a constant 
    /// velocity value to the object every frame. If both torques and velocity
    /// are defined, the velocity is used only on the first frame that the
    /// node is active, and then the forces are applied. The velocity fields
    /// then report the changed values as a result of the application of 
    /// the physics model in each frame.
    /// Setting a new value to the appropriate field will reset the body's 
    /// velocity for the next frame. Caution should be used in doing this as
    /// the underlying physics models may assume some amount of caching between
    /// time step evaluations and instantaneous velocity changes may lead to 
    /// numerical instability.
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value: </b> Vec3f( 0, 0, 0 )
    ///
    /// \dotfile RigidBody_angularVelocity.dot
    auto_ptr< SFVec3f > angularVelocity;

    /// The autoDamp field determines if automatic damping should be used
    /// or not. If TRUE, the angularDampingFactor and linearDampingFactor
    /// will be used to apply damping to the rigid body.
    /// 
    /// <b>Default value: </b> FALSE
    ///
    /// \dotfile RigidBody_autoDamp.dot
    auto_ptr< SFBool > autoDamp;

    /// The autoDisable field determines if automatic disabling of rigid
    /// bodies should be used or not. A disabled body will be considered 
    /// at rest and not move unless acted upon by an external force (
    /// e. g., collision or action of connected joint).
    /// If TRUE, the disableAngularSpeed, disableLinearSpeed, and
    /// disableTime fields will be used as threshold values to determine
    /// if a body should be disabled. If FALSE, the body is always active.
    /// 
    /// <b>Default value: </b> TRUE
    ///
    /// \dotfile RigidBody_autoDisable.dot
    auto_ptr< SFBool > autoDisable;

    /// The center of mass for this rigid body.
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value: </b> Vec3f( 0, 0, 0 )
    ///
    /// \dotfile RigidBody_centerOfMass.dot
    auto_ptr< SFVec3f > centerOfMass;


    /// The disable threshold for angular speed. If autoDisable is TRUE
    /// the body will be disable if the angular speed of the body goes
    /// below this value for longer than the value of disableTime in
    /// simulation time.
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value: </b> 0
    ///
    /// \dotfile RigidBody_disableAngularSpeed.dot  
    auto_ptr< SFFloat > disableAngularSpeed;

    /// The disable threshold for linear speed. If autoDisable is TRUE
    /// the body will be disable if the linear speed of the body goes
    /// below this value for longer than the value of disableTime in
    /// simulation time.
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value: </b> 0
    ///
    /// \dotfile RigidBody_disableLinearSpeed.dot  
    auto_ptr< SFFloat > disableLinearSpeed;

    /// The time in seconds of simulation time the body velocities has to be
    /// below any of the disableLinearSpeed or disableAngularSpeed field 
    /// values in order to be become disabled. It is only used if the 
    /// autoDisable is TRUE.
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value: </b> 0
    ///
    /// \dotfile RigidBody_disableTime.dot  
    auto_ptr< SFFloat > disableTime;

    /// The enabled field controls whether the information in this node is 
    /// submitted to the physics engine for processing. If the enabled field
    /// is set TRUE, the node is submitted to the physics engine. If the enabled
    /// field is set FALSE, the node is not submitted to the physics engine for 
    /// processing.
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value: </b> TRUE
    ///
    /// \dotfile RigidBody_enabled.dot  
    auto_ptr< SFBool > enabled;

    /// The finiteRotationAxis field specifies a vector around which the 
    /// object rotates when useFiniteRotation is TRUE. If this axis is 
    /// zero (0,0,0), full finite rotations are performed on the body. 
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value: </b> Vec3f( 0, 0, 0 )
    ///
    /// \dotfile RigidBody_finiteRotationAxis.dot  
    auto_ptr< SFVec3f > finiteRotationAxis;

    /// The fixed field is used to indicate that this body does not move. Any
    /// calculations involving collisions with this body should take into 
    /// account that this body does not move. This is useful for representing
    /// objects such as the ground, walls etc that can be collided with, have
    /// an effect on other objects, but are not capable of moving themselves.
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value: </b> FALSE
    ///
    /// \dotfile RigidBody_fixed.dot  
    auto_ptr< SFBool > fixed;

    /// The forces field define zero or more sets of external 
    /// force values that are applied to the object every frame. These are 
    /// continuously applied until reset to zero by the user.
    /// 
    /// <b>Access type: </b> inputOutput
    ///
    /// \dotfile RigidBody_forces.dot  
    auto_ptr< MFVec3f > forces;

    /// The geometry field is used to connect the body modelled by the physics
    /// engine implementation to the real geometry of the scene through the use
    /// of collidable nodes. This allows the geometry to be connected directly
    /// to the physics model as well as collision detection.
    /// 
    /// <b>Access type: </b> inputOutput
    ///
    /// \dotfile RigidBody_geometry.dot  
    auto_ptr< MFCollidableNode > geometry;

    /// The inertia tensor for the rigid body. This will be used if the
    /// massDensityModel is NULL.
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>DefaultValue: </b> Matrix3f( 1, 0, 0,
    ///                                 0, 1, 0,
    ///                                 0, 0, 1 )
    /// 
    /// \dotfile RigidBody_inertia.dot  
    auto_ptr< SFMatrix3f > inertia;

    /// The linearDampingFactor field allow the user
    /// to automatically damp the motion of the body over time. 
    /// Specifically it adds a force to the rigid body that is 
    /// -linearVelocity * factor.
    /// Damping is useful to provide an appearance of frictional forces and also
    /// to prevent the body from exploding due to numerical instability of the 
    /// physics model calculations. This field is only used if the 
    /// autoDamp field is set to TRUE. 
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value: </b> 0.001
    /// <b>Value range: </b> [0,1]
    ///
    /// \dotfile RigidBody_linearDampingFactor.dot
    auto_ptr< SFFloat > linearDampingFactor;

    /// The linearVelocity field is the linear velocity of the rigid mody in
    /// around each axis in m/s. It is used to provide a constant 
    /// velocity value to the object every frame. If both forces and velocity
    /// are defined, the velocity is used only on the first frame that the
    /// node is active, and then the forces are applied. The velocity fields
    /// then report the changed values as a result of the application of 
    /// the physics model in each frame.
    /// Setting a new value to the appropriate field will reset the body's 
    /// velocity for the next frame. Caution should be used in doing this as
    /// the underlying physics models may assume some amount of caching between
    /// time step evaluations and instantaneous velocity changes may lead to 
    /// numerical instability.
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value: </b> Vec3f( 0, 0, 0 )
    ///
    /// \dotfile RigidBody_linearVelocity.dot
    auto_ptr< SFVec3f > linearVelocity;

    /// The mass of the rigid body in kilograms.
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value: </b> 1
    ///
    /// \dotfile RigidBody_mass.dot
    auto_ptr< SFFloat > mass;

    /// The massDensityModel field is used to describe the geometry type and 
    /// dimensions used to calculate the mass density in the physics model. 
    /// This geometry has no renderable property, other than for defining 
    /// the model of the mass density. If NULL, the inertia field will be 
    /// used instead.
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Valid values: </b> Box, Sphere, Cone
    /// 
    ///  \dotfile RigidBody_massDensityModel.dot
    auto_ptr< SFNode > massDensityModel;

    /// The orientation and field is used to set the initial 
    /// conditions of this body's location in world space. After the initial
    /// conditions have been set, these fields are used to report the current
    /// information based on the most recent physics model evaluation. Setting
    /// new values will cause the objects to be moved to the new location 
    /// for the start of the next evaluation cycle. Care should be
    /// used in manually changing the orientation as the underlying
    /// physics models may cache information between time step evaluations 
    /// and sudden instantaneous changes may lead to numerical instability.
    ///
    /// <b>Access type: </b> inputOutput
    /// <b>Default value: </b> Rotation( 0, 0, 1, 0 )
    ///
    /// \dotfile RigidBody_orientation.dot
    auto_ptr< SFRotation > orientation;

    /// The position field is used to set the initial 
    /// conditions of this body's location in world space. After the initial
    /// conditions have been set, these fields are used to report the current
    /// information based on the most recent physics model evaluation. Setting
    /// new values will cause the objects to be moved to the new location 
    /// for the start of the next evaluation cycle. Care should be
    /// used in manually changing the position as the underlying
    /// physics models may cache information between time step evaluations 
    /// and sudden instantaneous changes may lead to numerical instability.
    ///
    /// <b>Access type: </b> inputOutput
    /// <b>Default value: </b> Vec3f( 0, 0, 0 )
    ///
    /// \dotfile RigidBody_position.dot
    auto_ptr< SFVec3f > position;

    /// The torques field define zero or more sets of external 
    /// torque values that are applied to the object every frame. These are 
    /// continuously applied until reset to zero by the user.
    /// 
    /// <b>Access type: </b> inputOutput
    ///
    /// \dotfile RigidBody_torques.dot  
    auto_ptr< MFVec3f > torques;

    /// The useFiniteRotation field is used to influence the way the body's
    /// rotation is calculated. In very fast rotating objects, such as a wheel
    /// of a car, an infinitely small time step can cause the modelling to
    /// explode. The default value is to use the faster infinite mode. Setting
    /// the field value to TRUE uses the finite calculation model. Using the 
    /// finite model is more costly to compute but will be more accurate for
    /// high rotation speed bodies.
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value: </b> FALSE
    ///
    /// \dotfile RigidBody_useFiniteRotation.dot
    /// <b>Access type: </b>
    auto_ptr< SFBool > useFiniteRotation;

    /// The useGlobalGravity field is used to indicate whether this particular
    /// body should be influenced by the containing RigidBodyCollection's 
    /// gravity setting. A value of TRUE indicates that the gravity is used, 
    /// a value of FALSE indicates that it is not used. This only applies to
    /// this body instance. Contained sub-bodies shall not be affected by this
    /// setting.
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value: </b> TRUE
    ///
    /// \dotfile RigidBody_useGlobalGravity.dot
    auto_ptr< SFBool > useGlobalGravity;

    /// If true, then the body can be moved explicitly by changing its position
    /// and orientation. If false, then the object will be controlled using force
    /// and torque. 
    ///
    /// kinematicControl will automatically be set to true if position or orientation
    /// fields are set after the body has been initialized.
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value: </b> FALSE
    ///
    /// \dotfile RigidBody_kinematicControl.dot
    auto_ptr< SFBool > kinematicControl;

    /// An initial transformation applied to all joint parameters
    /// 
    /// <b>Access type: </b> inputOutput
    ///
    /// \dotfile RigidBody_transform.dot
    auto_ptr< SFTransformNode > transform;

    /// Field used when debugging the code.
    ///
    /// <b>Access type: </b> inputOutput
    /// <b>Default value: </b> FALSE
    /// \dotfile RigidBody_debug.dot
    auto_ptr< SFBool > debug;

  protected:
    /// The valueUpdater field is used to update values in the
    /// PhysicsEngineThread according to changes of fields in the
    /// RigidBody node. More specifically it calls 
    /// PhysicsEngineThread::setRigidBodyParameters with the new values.
    /// Every field that has a corresponding value in 
    /// PhysicsEngineThreadParameters::RigidBodyParameters is routed 
    /// to this field.
    /// C++ only field.
    auto_ptr< ValueUpdater > valueUpdater; 

  public:

    /// Physics engine options specific for this RigidBody.
    /// 
    /// <b>Access type: </b> inputOutput
    /// 
    /// \dotfile RigidBody_engineOptions.dot
    auto_ptr< MFEngineOptions > engineOptions;

    /// Initialize the rigid body for the given PhysicsEngineThread. I.e. 
    /// create a new rigid body in the physics engine with the parameters
    /// of the rigid body fields. Returns 0 on success.
    virtual bool initializeBody( H3D::PhysicsEngineThread& pt );

    /// Deletes this rigid body node from the given PhysicsEngineThread.
    virtual bool deleteBody();

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:

    /// Returns a new concrete instance of RigidBodyParameters appropriate for this subtype of H3DBodyNode
    virtual PhysicsEngineParameters::RigidBodyParameters *createRigidBodyParameters();

#ifdef DEBUG_RB_LAG
    std::ofstream lagDebugFile;
#endif

    /// Returns a RigidBodyParameter to describe the rigid body. By default
    /// the function returns a RigidBodyParameter with values that have changed
    /// since the last loop.
    //// \param all_params If true then it returns all field values regardless 
    /// of whether the values have changed
    virtual PhysicsEngineParameters::RigidBodyParameters *getRigidBodyParameters( bool all_params = false );

    /// Apply external forces from the force and torque field.
    void applyForces( H3D::PhysicsEngineThread *pt );  

    /// A counter to check if all the collidable shapes have been given an id or not
    std::vector< std::pair<unsigned int, std::string> > collidables_without_ids;
    unsigned int nbframes_collidables_without_ids;
  };
}
#endif
