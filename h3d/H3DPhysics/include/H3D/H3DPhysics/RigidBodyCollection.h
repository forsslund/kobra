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
/// \file RigidBodyCollection.h
/// \brief Header file for RigidBodyCollection, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __RIGIDBODYCOLLECTION__
#define __RIGIDBODYCOLLECTION__

#include <H3D/X3DChildNode.h>
#include <H3D/MFNode.h>
#include <H3D/SFBool.h>
#include <H3D/SFFloat.h>
#include <H3D/SFVec3f.h>
#include <H3D/SFInt32.h>
#include <H3D/H3DPhysics/H3DPhysics.h>
#include <H3D/H3DPhysics/CollisionCollection.h>
#include <H3D/H3DPhysics/RigidBody.h>
#include <H3D/H3DPhysics/H3DJointNode.h>
#include <H3D/H3DPhysics/PhysicsEngineThread.h>
#include <H3D/H3DPhysics/H3DEngineOptions.h>
#include <H3DUtil/Threads.h>

namespace H3D{
  /// \class RigidBodyCollection
  /// \brief The RigidBodyCollection node represents a system of bodies 
  /// that will interact within a single physics model. The collection is not
  /// a renderable part of the scene graph nor are its children as a typical
  /// model may need to represent the geometry for physics separately, and 
  /// in less detail, than those needed for visuals.
  ///
  /// The rigidBodies field contains a collection of the top-level nodes that
  /// comprise a set of rigid bodies that should be evaluated as a single set
  /// of interactions. 
  ///
  /// The joints field is used to register all the joints between the
  /// bodies contained in this collection. If a joint is connected between 
  /// bodies in two different collections, the result is 
  /// implementation-dependent. If a joint instance is registered with more
  /// than one collection, the results are implementation dependent. Joints
  /// not registered with any collection are not evaluated.
  ///
  /// The enabled field is used to control whether the physics model for 
  /// this collection should be run this frame.
  ///
  /// The contactSurfaceThickness field represents how far bodies may 
  /// interpenetrate after a collision. This allows simulation of softer
  /// bodies that may deform somewhat during collision. The default value
  /// is zero. 
  ///
  /// NOTE  Since a value of 0 may cause jittering due to floating point
  /// inaccuracy, a very small value of 0.001 may be useful.
  /// 
  /// The gravity field indicates direction and strength of the local 
  /// gravity vector for this collection of bodies. The default gravity 
  /// is standard earth gravity of 9.8m/s2 downwards.
  ///
  /// The set_contacts input field is used to provide per-frame sets of 
  /// information about contacts between bodies in this frame. These 
  /// contacts are then used to modify the location of the bodies within
  /// the scene graph when the physics model is evaluated at the end of the
  /// frame. For efficiency, a user may reuse instances of the Contact node
  /// for each frame rather than allocating a new instance per frame. A
  /// browser implementation shall not make assumptions about the same 
  /// object instance having the same values each frame.
  ///
  /// The preferAccuracy field is used to provide a performance hint to
  /// the underlying evaluation about whether the user prefers to have very
  /// accurate models or fast models. Accuracy comes at a large penalty in
  /// both speed and memory usage, but may not be needed most of the time. 
  /// The default setting is to optimize for speed rather than accuracy.
  /// 
  /// The iterations field is used to control how many iterations over the
  /// collections of joints and bodies are to be performed each time the
  /// model is evaluated. Rigid body physics is a process of iterative 
  /// refinement in order to maintain reasonable performance. As the number
  /// of iterations grow, the more stable the final results are at the cost
  /// of increasing evaluation time. Since maintaining real-time performance
  /// is a trade off between accuracy and frame rate, this setting allows 
  /// the user to control that trade off to a limited extent.
  ///
  /// The errorCorrection field describes how quickly the system should 
  /// resolve intersection errors due to floating point inaccuracies. This
  /// value ranges between 0 and 1. A value of 0 means no correction at all
  /// while a value of 1 indicates that all errors should be corrected in
  /// a single step.
  ///
  /// The constantForceMix field can be used to apply damping to the 
  /// calculations by violating the normal constraints by applying a small, 
  /// constant force to those calculations. This allows joints and bodies 
  /// to be a fraction springy, as well as helping to eliminate numerical
  /// instability. The larger the value, the more soft each of the constraints
  /// being evaluated. A value of zero indicates hard constraints so that
  /// everything is exactly honoured. By combining the errorCorrection and
  /// constantForceMix fields, various effects, such as spring-driven or 
  /// spongy connections, can be emulated.
  ///
  /// The collider field associates a collision collection with this rigid
  /// body collection allowing seamless updates and integration without
  /// the need to use the X3D event model.
  ///
  /// The disable fields define conditions for when the body ceases to 
  /// considered as part of the rigid body calculations and should be 
  /// considered as at rest. Due to the numerical instability of physics 
  /// models, even bodies initially declared to be at rest may gain some
  /// amount of movement, even when not effected by an external forces.
  /// These values define tolerances for which the physics model should 
  /// start to ignore this object in any calculation, thus resulting in
  /// them being actually at rest and not subject to these instability
  /// conditions. Once any one of these values is achieved, the body is
  /// considered as being at rest, unless acted upon by an external force
  /// (e. g., collision or action of connected joint). By default, this
  /// automatic disabling is turned off. It may be enabled by setting 
  /// the autoDisable field to TRUE.
  ///
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../examples/RigidBody/BallJoint.x3d">BallJoint.x3d</a>
  ///     ( <a href="examples/BallJoint.x3d.html">Source</a> )
  ///
  /// \par Internal routes:
  /// \dotfile RigidBodyCollection.dot
  class H3DPHYS_API RigidBodyCollection : 
    public X3DChildNode,
    public H3DDisplayListObject {
  public:

    /// Specifies that only the CollisionCollection node type may be
    /// the value for this field
    class H3DPHYS_API SFCollisionCollection: public TypedSFNode< CollisionCollection > {
    public:
      /// On field destruction, call deleteCollidables on its value to
      /// remove the collidables from the PhysicsEngineThread
      ~SFCollisionCollection() {
        if ( value.get() )
          static_cast< CollisionCollection * >( value.get() )->deleteCollidables();
      }

      virtual void onAdd( Node *n) {
        TypedSFNode< CollisionCollection >::onAdd( n );
        if( n ){
          RigidBodyCollection* rbc = static_cast< RigidBodyCollection* >( owner );
          CollisionCollection* cc = static_cast< CollisionCollection* >( n );
          cc->setRigidBodyCollection( rbc );
        }
      }

      virtual void onRemove( Node *n) {
        if( n ){
          RigidBodyCollection* rbc = static_cast< RigidBodyCollection* >( owner );
          CollisionCollection* cc = static_cast< CollisionCollection* >( n );
          cc->setRigidBodyCollection();
        }
        TypedSFNode< CollisionCollection >::onRemove( n );
      }
    };

    /// The ValueUpdater field is used to update values in the
    /// PhysicsEngineThread according to changes of fields in the
    /// RigidBodyCollection node. More specifically it calls 
    /// PhysicsEngineThread::setWorldParameters with the new values.
    /// Every field that has a corresponding value in 
    /// PhysicsEngineThreadParameters::WorldParameters is routed 
    /// to this field.
    class H3DPHYS_API ValueUpdater: 
      public EventCollectingField< PeriodicUpdate< Field > > {
    protected:
      virtual void update();
    };

    typedef MFH3DEngineOptions < RigidBodyCollection > MFEngineOptions;
    friend class MFH3DEngineOptions < RigidBodyCollection >;

    /// MFRigidBody is specialized to remove the rigid body from the
    /// PhysicsEngineThread of the RigidBodyCollection when it is
    /// removed from the field. Initialization of rigid bodies
    /// is handled in the traverseSG function of RigidBody though.
    /// The onAdd function cannot be used to do this since the
    /// PhysicsEngineThread might not exist when it is called.
    class H3DPHYS_API MFRigidBody: public TypedMFNode< RigidBody > {
    public:
      /// Destructor. Needed for the correct version of onRemove to be 
      /// called.
      virtual ~MFRigidBody() {
        clear();
      }
    protected:
      /// Removes the rigid body from the PhysicsEngineThread of the 
      /// RigidBodyCollection when removed from the field.
      virtual void onRemove( Node *n );
    };

    /// MFJoint is specialized to remove the joint from the
    /// PhysicsEngineThread of the RigidBodyCollection when it is
    /// removed from the field. Initialization of rigid bodies
    /// is handled in the traverseSG function of H3DJointNode though.
    /// The onAdd function cannot be used to do this since the
    /// PhysicsEngineThread might not exist when it is called.  
    class H3DPHYS_API MFBodyConstraint: public TypedMFNode< H3DBodyConstraintNode > {
    public:
      /// Destructor. Needed for the correct version of onRemove to be 
      /// called.
      virtual ~MFBodyConstraint() {
        clear();
      }
    protected:
      /// Removes the joint from the PhysicsEngineThread of the 
      /// RigidBodyCollection when removed from the field.
      virtual void onRemove( Node *n );
    };

    /// For a RigidBodyCollection, the joints field may only contain H3DJointNodes
    ///
    /// For the subclass PhysicsBodyCollection, the same field may contain any kind of
    /// H3DBodyConstraintNode.
    typedef SpecializedMFNode < H3DJointNode, MFBodyConstraint > MFJoint;

    class H3DPHYS_API EnableDisable: 
                    public AutoUpdate< SFBool > {
    protected:
      virtual void update();
    };

    class H3DPHYS_API EnableUseMainThread: 
                    public PeriodicUpdate< OnValueChangeSField < SFBool > > {
    protected:
      virtual void onValueChange ( const bool& b );
    };

    /// Constructor.
    RigidBodyCollection(
      Inst< SFNode  > _metadata = 0,
      Inst< MFNode  > _set_contacts = 0,
      Inst< SFBool  > _autoDisable = 0,
      Inst< MFRigidBody > _bodies = 0,
      Inst< SFFloat > _constantForceMix = 0,
      Inst< SFFloat > _contactSurfaceThickness  = 0,
      Inst< SFFloat > _disableAngularSpeed = 0,
      Inst< SFFloat > _disableLinearSpeed = 0,
      Inst< SFFloat > _disableTime = 0,
      Inst< SFBool  > _enabled = 0,
      Inst< SFFloat > _errorCorrection = 0,
      Inst< SFVec3f > _gravity = 0,
      Inst< SFInt32 > _iterations = 0,
      Inst< MFJoint > _joints = 0,
      Inst< SFFloat > _maxCorrectionSpeed = 0,
      Inst< SFBool  > _preferAccuracy = 0,
      Inst< SFCollisionCollection  > _collider = 0,
      Inst< ValueUpdater > _valueUpdater = 0,
      Inst< EnableDisable > _enableDisable = 0,
      Inst< SFString > _physicsEngine = 0,
      Inst< SFInt32 > _desiredUpdateRate = 0,
      Inst< SFInt32 > _updateRate = 0,
      Inst< SFTime  > _stepUpdateTime = 0,
      Inst< EnableUseMainThread > _useMainThread = 0,
      Inst< MFEngineOptions > _engineOptions = 0,
      Inst< DisplayList > _displayList = 0,
      Inst< SFBool > _renderCollidables = 0,
      Inst< SFBool > _renderOnlyEnabledCollidables = 0,
      Inst< SFBool  > _useStaticTimeStep = 0,
      Inst< SFInt32 > _syncGraphicsFrames = 0,
      Inst< SFInt32 > _syncPhysicsFrames = 0 );

    /// Destructor.
    virtual ~RigidBodyCollection();

    /// Initialize the node. A new PhysicsEngineThread with the underlying
    /// physics engine specified by the physicsEngine is created.
    virtual void initialize();

    /// Traverse the scene graph. 
    virtual void traverseSG(TraverseInfo &ti);

    /// Render the collision objects.
    virtual void render();

    /// Not implemented in H3D API. 
    ///
    /// The set_contacts input field is used to provide per-frame sets of 
    /// information about contacts between bodies in this frame. These 
    /// contacts are then used to modify the location of the bodies within
    /// the scene graph when the physics model is evaluated at the end of the
    /// frame. For efficiency, a user may reuse instances of the Contact node
    /// for each frame rather than allocating a new instance per frame. 
    /// 
    /// <b>Access type:</b>inputOnly
    ///
    /// \dotfile RigidBodyCollection_set_contacts.dot
    auto_ptr< MFNode  > set_contacts;

    /// If true the disable.. fields are used to define conditions for when
    /// the body ceases to considered as part of the rigid body calculations
    /// and should be considered as at rest.  Due to the numerical instability
    /// of physics 
    /// models, even bodies initially declared to be at rest may gain some
    /// amount of movement, even when not effected by an external forces.
    /// These values define tolerances for which the physics model should 
    /// start to ignore this object in any calculation, thus resulting in
    /// them being actually at rest and not subject to these instability
    /// conditions. Once any one of these values is achieved, the body is
    /// considered as being at rest, unless acted upon by an external force
    /// (e. g., collision or action of connected joint). 
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> RGB( 0, 0, 0 ) \n
    /// 
    /// \dotfile RigidBodyCollection_autoDisable.dot
    auto_ptr< SFBool  > autoDisable;

    /// The constantForceMix field can be used to apply damping to the 
    /// calculations by violating the normal constraints by applying a small, 
    /// constant force to those calculations. This allows joints and bodies 
    /// to be a fraction springy, as well as helping to eliminate numerical
    /// instability. The larger the value, the more soft each of the constraints
    /// being evaluated. A value of zero indicates hard constraints so that
    /// everything is exactly honoured. By combining the errorCorrection and
    /// constantForceMix fields, various effects, such as spring-driven or 
    /// spongy connections, can be emulated.
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 0.0001
    /// 
    /// \dotfile RigidBodyCollection_constantForceMix.dot
    auto_ptr< SFFloat > constantForceMix;

    /// The contactSurfaceThickness field represents how far bodies may 
    /// interpenetrate after a collision. This allows simulation of softer
    /// bodies that may deform somewhat during collision. 
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 0
    /// 
    /// \dotfile RigidBodyCollection_contactSurfaceThickness.dot
    auto_ptr< SFFloat > contactSurfaceThickness;

    /// The disable threshold for angular speed to use when the autoDisable
    /// field is true. 
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 0
    /// 
    /// \dotfile RigidBodyCollection_disableAngularSpeed.dot
    auto_ptr< SFFloat > disableAngularSpeed;

    /// The disable threshold for linear speed to use when the autoDisable
    /// field is true. 
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 0
    /// 
    /// \dotfile RigidBodyCollection_disableLinearSpeed.dot
    auto_ptr< SFFloat > disableLinearSpeed;

    /// The time a body has to be below the disable thresholds for it to be disabled
    /// when autoDisable is true. 
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 0
    /// 
    /// \dotfile RigidBodyCollection_disableTime.dot
    auto_ptr< SFFloat > disableTime;

    ///  The enabled field is used to control whether the physics model for 
    /// this collection should be advanced or not.
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> true
    /// 
    /// \dotfile RigidBodyCollection_enabled.dot
    auto_ptr< SFBool  > enabled;

    /// The errorCorrection field describes how quickly the system should 
    /// resolve intersection errors due to floating point inaccuracies. This
    /// value ranges between 0 and 1. A value of 0 means no correction at all
    /// while a value of 1 indicates that all errors should be corrected in
    /// a single step.
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 0.8
    /// 
    /// \dotfile RigidBodyCollection_errorCorrection.dot
    auto_ptr< SFFloat > errorCorrection;

    /// The gravity field indicates direction and strength of the local 
    /// gravity vector for this collection of bodies. The default gravity 
    /// is standard earth gravity of 9.8m/s2 downwards.
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 0 -9.8 0
    /// 
    /// \dotfile RigidBodyCollection_gravity.dot
    auto_ptr< SFVec3f > gravity;

    /// The iterations field is used to control how many iterations over the
    /// collections of joints and bodies are to be performed each time the
    /// model is evaluated. Rigid body physics is a process of iterative 
    /// refinement in order to maintain reasonable performance. As the number
    /// of iterations grow, the more stable the final results are at the cost
    /// of increasing evaluation time. Since maintaining real-time performance
    /// is a trade off between accuracy and frame rate, this setting allows 
    /// the user to control that trade off to a limited extent.   
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 10
    /// 
    /// \dotfile RigidBodyCollection_iterations.dot
    auto_ptr< SFInt32 > iterations;


    /// The X3D specification does not comment this field at all.
    /// What should it do?
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> -1
    /// 
    /// \dotfile RigidBodyCollection_maxCorrectionSpeed.dot
    auto_ptr< SFFloat > maxCorrectionSpeed;

    /// The preferAccuracy field is used to provide a performance hint to
    /// the underlying evaluation about whether the user prefers to have very
    /// accurate models or fast models. Accuracy comes at a large penalty in
    /// both speed and memory usage, but may not be needed most of the time. 
    /// The default setting is to optimize for speed rather than accuracy.
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> -1
    /// 
    /// \dotfile RigidBodyCollection_preferAccuracy.dot
    auto_ptr< SFBool  > preferAccuracy;

    /// The collider field associates a collision collection with this rigid
    /// body collection
    /// 
    /// <b>Access type:</b> inputOutput
    /// 
    /// \dotfile RigidBodyCollection_collider.dot
    auto_ptr< SFCollisionCollection  > collider;

    /// The physics engine to use for rigid body simulations. Valid
    /// values are "ODE", "PhysX" and "Bullet".
    ///
    /// <b>Access type:</b> initializeOnly
    /// <b>Default value:</b> "ODE"
    /// 
    /// \dotfile RigidBodyCollection_physicsEngine.dot
    auto_ptr< SFString > physicsEngine;

    /// The desired update rate(Hz) for the rigid body simulation thread
    /// to run at.
    ///
    /// <b>Access type:</b> initializeOnly
    /// <b>Default value:</b> 100
    /// 
    /// \dotfile RigidBodyCollection_desiredUpdateRate.dot
    auto_ptr< SFInt32 > desiredUpdateRate;

    /// The actual update rate(Hz) the rigid body simulation thread
    /// runs in.
    ///
    /// <b>Access type:</b> outputOnly
    /// 
    /// \dotfile RigidBodyCollection_updateRate.dot
    auto_ptr< SFInt32 > updateRate;

    /// The time(in seconds) the last step in the rigid body simulation
    /// took.
    ///
    /// <b>Access type:</b> outputOnly
    /// 
    /// \dotfile RigidBodyCollection_stepUpdateTime.dot
    auto_ptr< SFTime > stepUpdateTime;

    /// Use the main/graphics thread for physics simulation.
    /// Otherwise a separate simulation thread is used.
    ///
    /// <b>Access type:</b> outputOnly
    /// <b>Default value:</b> FALSE
    /// 
    /// \dotfile RigidBodyCollection_useMainThread.dot
    auto_ptr< EnableUseMainThread > useMainThread;

    /// Additional physics engine specific parameters for the simulation.
    ///
    /// <b>Access type:</b> inputOutput
    /// 
    /// \dotfile RigidBodyCollection_engineOptions.dot
    auto_ptr< MFEngineOptions > engineOptions;

    /// The rigid bodies that should be evaluated in this RigidBodyCollection.
    ///
    /// <b>Access type:</b>inputOutput
    /// 
    /// \dotfile RigidBodyCollection_bodies.dot
    auto_ptr< MFRigidBody  > bodies;

    /// The joints that should be evaluated in this RigidBodyCollection.
    ///
    /// <b>Access type:</b>inputOutput
    /// 
    /// \dotfile RigidBodyCollection_joints.dot
    auto_ptr< MFBodyConstraint  > joints;

    /// If true the collision object for all rigid bodies will be visualized.
    ///
    /// <b>Access type:</b>inputOutput
    /// <b>Default value:</b>false
    ///
    /// \dotfile RigidBodyCollection_renderCollidables.dot
    auto_ptr< SFBool > renderCollidables;

    /// If true the collision object for all rigid bodies will be visualized.
    ///
    /// <b>Access type:</b>inputOutput
    /// <b>Default value:</b>false
    ///
    /// \dotfile RigidBodyCollection_renderOnlyEnabledCollidables.dot
    auto_ptr< SFBool > renderOnlyEnabledCollidables;

    /// If true then a constant time step is used, which is based on the desired
    /// update rate. Otherwise the real time elapsed is used.
    ///
    /// <b>Access type:</b>inputOutput
    /// <b>Default value:</b>false
    ///
    /// \dotfile RigidBodyCollection_useStaticTimeStep.dot
    auto_ptr < SFBool > useStaticTimeStep;

    /// The number of graphics frames to execute between synchronization of graphics and physics
    ///
    /// If 0 then graphics and physics will run independently with non-deterministic synchronization
    ///
    /// <b>Access type:</b>initializeOnly
    /// <b>Default value:</b>0
    ///
    /// \dotfile RigidBodyCollection_syncGraphicsFrames.dot
    auto_ptr< SFInt32 > syncGraphicsFrames;

    /// The number of physics frames to execute between synchronization of graphics and physics
    ///
    /// If 0 then graphics and physics will run independently with non-deterministic synchronization
    ///
    /// <b>Access type:</b>initializeOnly
    /// <b>Default value:</b>0
    ///
    /// \dotfile RigidBodyCollection_syncPhysicsFrames.dot
    auto_ptr< SFInt32 > syncPhysicsFrames;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

    /// Get the PhysicsEngineThread that is used in the RigidBodyCollection.
    H3D::PhysicsEngineThread *getSimulationThread() {
      return simulationThread.get();
    }

 protected:
    /// The valueUpdater field is used to update values in the
    /// PhysicsEngineThread according to changes of fields in the
    /// RigidBodyCollection node. More specifically it calls 
    /// PhysicsEngineThread::setWorldParameters with the new values.
    /// Every field that has a corresponding value in 
    /// PhysicsEngineThreadParameters::WorldParameters is routed 
    /// to this field.
    /// C++ only field.
    ///
    /// \dotfile RigidBodyCollection_valueUpdater.dot
    auto_ptr< ValueUpdater > valueUpdater; 

    /// The PhysicsEngineThread handling the physics simulation for 
    /// this RigidBodyCollection.
    auto_ptr< H3D::PhysicsEngineThread > simulationThread;

    auto_ptr < EnableDisable > enableDisable;

    /// Creates a new instance of a subclass of WorldParameters appropriate for the
    /// subclass of H3DPhysicsBodyCollection. Provides a default implementation and
    /// supposed to be extended in case new world parameters are used by the subclass.
    virtual PhysicsEngineParameters::WorldParameters* createWorldParameters () {
      return new PhysicsEngineParameters::WorldParameters();
    }

    /// Traverse the nodes of the physics simulation
    ///
    /// If using deterministic synchronization the simulation outputs are guaranteed 
    /// not to change during the execution of this method (see syncGraphicsFrames)
    virtual void traverseSimulation( TraverseInfo& ti );

    /// If we should synchronize graphics and physics on the current graphics frame
    /// (see syncGraphicsFrames), then block until the required number of physics frames
    /// are executed (see syncPhysicsFrames) and return true. Otherwise return false and
    /// return immediately.
    ///
    /// If deterministic synchronization is not enabled (e.g., syncGraphicsFrames == 0), then
    /// the method returns true immediately without waiting. 
    bool waitForSimulationSteps();

    /// If using deterministic synchronization, begin running the required number of physics
    /// simulation steps now (see syncPhysicsFrames). Otherwise do nothing.
    void beginSimulationSteps();

    /// If using deterministic synchronization, begin copying out the results of the previous
    /// physics simulation steps now. Otherwise do nothing.
    void beginSynchronizeSteps();

    /// Returns a WorldParameter to describe the collection. By default
    /// the function returns a WorldParameter with values that have changed
    /// since the last loop.
    //// \param all_params If true then it returns all field values regardless 
    /// of whether the values have changed
    virtual PhysicsEngineParameters::WorldParameters *getWorldParameters( bool all_params = false );

    /// The number of graphics frames since the last physics synchronization
    std::size_t nr_graphics_steps;
  };


#ifdef USE_PROFILER
  static H3DTime lastProfileTime;
  static H3DTime profileInterval;
#endif
}

#endif
