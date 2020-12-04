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
/// \file PhysicsEngineThread.h
/// \brief Header file for PhysicsEngineThread.
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __PHYSICSENGINETHREAD__
#define __PHYSICSENGINETHREAD__

#include <H3D/H3DPhysics/PhysicsEngineParameters.h>
#include <H3DUtil/Threads.h>
#include <memory>

#ifdef DEBUG_RB_LAG
#include <fstream>
#endif

// Use a mutex lock to get contacts
// otherwise use a synchronous callback
#define USE_CONTACTS_LOCK

namespace H3D {

  using namespace PhysicsEngineParameters;

  /// Used to emulate rate of change fields that are not supported by 
  /// natively supported by the engine
  /// e.g. SingleAxisHingeJoint::angleRate, separationRate etc.
  class RateOfChange {
  public:
    /// Constructor
    RateOfChange ( H3DDouble _resetTime= 2.0 ) :
        calls ( 0 ), value ( 0.0f ), prevValue ( 0.0f ), resetTime ( _resetTime )
        {}

        /// Supply a current value of the variable to calculate the rate of change for
        /// Should be called reguarly and frequently
        void setCurrentValue ( H3DFloat v ) {
          prevTime= time;
          time= TimeStamp();

          if ( time-prevTime < resetTime ) {
            prevValue= value;
            value= v;

            if ( calls < 2 )
              ++calls;
          }
          else
            // If longer than reset time, then start again
            calls= 0;
        }

        /// Return the current rate of change for the value
        /// Returns 0 in case of insufficient values supplied, or other errors
        operator H3DFloat() {
          H3DDouble deltaT= time-prevTime;
          if ( calls >= 2 && deltaT > 0.0f )
            return (H3DFloat)((value-prevValue)/deltaT);
          else
            return 0.0f;
        }

  protected:

    H3DFloat value, prevValue;
    TimeStamp time, prevTime;
    unsigned int calls;

    /// Time after which previous value will be scrapped and will start over
    H3DDouble resetTime;
  };

  /// \defgroup PhysicsEngineCallbackStructs Physics engines callback structs
  /// This group contains structs which contains callbacks to different
  /// implementation of physics engines.

  /// \class PhysicsEngineThread
  /// \brief The PhysicsEngineThread contains functions for transferring data
  /// to a physics engine.
  ///
  /// The functions calls callbacks of the chosen physics engine. In order for
  /// a physics engine to be valid for use by the PhysicsEngineThread
  /// library the physics engine has to be registered through the
  /// PhysicsEngineRegistration class.
  class H3DPHYS_API PhysicsEngineThread :  public H3DUtil::PeriodicThread {
  public:

    /// \brief Struct that can be inherited from in order to specify data
    /// which is specific to a physics engine implementation.
    struct EngineSpecificData {
      /// Destructor. Making the class a polymorphic type.
      virtual ~EngineSpecificData() {}
    };

    /// Constructor.
    /// \param engine The name of the physics engine to use.
    /// \param thread_priority The priority of the thread.
    /// \param thread_frequency The frequence of the thread loop. -1 means
    /// run as fast as possible.
    /// \param _useMainThread If true execute all callbacks in the main thread.
    /// \param _use_synchronization If true, use deterministic synchronization of the physics and graphics threads.
    /// \param _solver_type Chooses which solver to use. This option has different meaning for different physics engines,
    /// but has no effect if only one solver option is available for a certain solver.
    PhysicsEngineThread( const string &engine = "ODE",
                         Priority thread_priority = NORMAL_PRIORITY,
                         int thread_frequency = 100,
                         bool _useMainThread = false,
                         bool _use_synchronization = false,
                         const H3DUInt32 _solver_type = 0 );

    /// Destructor.
    virtual ~PhysicsEngineThread();

    /// If true all callbacks will be executed in the thread of the caller rather
    /// than in the physics engine thread
    void setUseMainThread ( bool _useMainThread );

    /// Should be called once each cycle of the main thread
    /// If using main thread for callbacks, queued asynchronous callbacks will
    /// be executed by this function
    void mainThreadStep ();

    /// Return name of engine used
    string getEngine () {
      return engine;
    }

    /// This alternative to removeAsynchronousCallback() should be used to remove
    /// a callback from within a blocking callback of the same thread. It assumes
    /// that any threads that might add or remove callbacks are already blocked
    bool removeAsynchronousCallbackNoLock( int callback_handle );

    // Override base thread callback functions to provide the option
    // of executing in the main thread instead.

    /// Add a callback function to be executed in this thread. The calling
    /// thread will wait until the callback function has returned before 
    /// continuing.
    virtual void synchronousCallback( CallbackFunc func, void *data );

    /// Add a callback function to be executed in this thread. The calling
    /// thread will continue executing after adding the callback and will 
    /// not wait for the callback function to execute.
    /// Returns a handle to the callback that can be used to remove
    /// the callback.
    virtual int asynchronousCallback( CallbackFunc func, void *data );

    /// Attempts to remove a callback. returns true if succeded. returns
    /// false if the callback does not exist. This function should be handled
    /// with care. It can remove the wrong callback if the callback that
    /// returned the callback_handle id is removed and a new callback is added.
    /// Callbacks are removed if they return CALLBACK_DONE or a call to this
    /// function is made.
    virtual bool removeAsynchronousCallback( int callback_handle );

    // Shape functions

    /// Create a new shape with the given parameters. 
    /// An id to the new shape is returned or 0 if the operation could not be
    /// performed. The ShapeParameters pointer is owned by the caller(operation
    /// will be performed with a synchronous callback in the physics engine 
    /// thread).
    H3DCollidableId addCollidable( CollidableParameters& p );

    /// Remove a shape from the physics engine. Returns 0 on success.
    bool removeCollidable( H3DCollidableId id );

    /// Set the parameters for a given shape. Only the paramaters in the
    /// ShapeParameters structure that has actually been set will be changed.
    /// Returns 0 on success.
    /// The ShapeParameters pointer will be owned by the physics engine thread
    /// after this call and will be deleted by the physics engine when it has
    /// been used(operation will be performed with an asynchronous callback in
    /// the physics engine thread).
    bool setCollidableParameters( H3DCollidableId id,
      CollidableParameters& params );

    /// Get the parameters for a given shape. The ShapeParameters structure will
    /// be updated with the values from the specified shape.
    /// Returns 0 on success.
    bool getCollidableParameters( H3DCollidableId id,
      CollidableParameters& params );

    // Rigid body functions
    /// Create a new rigid body with the given parameters.
    /// An id to the new shape is returned or 0 if the operation could not be
    /// performed. The ShapeParameters pointer is owned by the caller(operation
    /// will be performed with a synchronous callback in the physics engine 
    /// thread).
    H3DBodyId addRigidBody( RigidBodyParameters& params );

    /// Remove a rigid body from the physics engine. Returns 0 on success.
    bool removeRigidBody( H3DBodyId id );

    /// Set the parameters for a given rigid body. Only the parameters in the
    /// RigidBodyParameters structure that has actually been set will be
    /// changed.
    /// Returns 0 on success.
    /// The RigidBodyParameters pointer will be owned by the physics engine
    /// thread after this call and will be deleted by the physics engine when
    /// it has been used(operation will be performed with an asynchronous
    /// callback in the physics engine thread).
    bool setRigidBodyParameters( H3DBodyId body, 
      RigidBodyParameters& p );

    /// Get the parameters for a given rigid body. The RigidBodyParameters
    /// structure will be updated with the values from the specified rigid
    /// body.
    /// Returns 0 on success.
    bool getRigidBodyParameters( H3DBodyId body,
      RigidBodyParameters& params );

    // Constraint functions
    /// Create a new constraint with the given parameters. 
    /// An id to the new constraint is returned or 0 if the operation could not be
    /// performed. The ConstraintParameters pointer is owned by the caller(operation
    /// will be performed with a synchronous callback in the physics engine 
    /// thread).
    H3DConstraintId addConstraint( ConstraintParameters& params );

    /// Remove an constraint from the physics engine. Returns true on success.
    bool removeConstraint( H3DConstraintId constraint );

    /// Set the parameters for a given constraint. Only the paramaters in the
    /// ConstraintParameters structure that has actually been set will be
    /// changed.
    /// Returns 0 on success.
    /// The ConstraintParameters pointer will be owned by the physics engine
    /// thread after this call and will be deleted by the physics engine when
    /// it has been used(operation will be performed with an asynchronous
    /// callback in the physics engine thread).
    bool setConstraintParameters( H3DConstraintId constraint, 
      ConstraintParameters& params );

    /// Get the parameters for a given constraint. The ConstraintParameters
    /// structure will be updated with the values from the specified constraint.
    /// Returns 0 on success.
    bool getConstraintParameters( H3DConstraintId constraint, 
      ConstraintParameters& params );

    // Space functions
    /// Create a new collision space with the given parameters.
    /// An id to the new collision space is returned or 0 if the operation
    /// could not be performed. The SpaceParameters pointer is owned by the
    /// caller(operation will be performed with a synchronous callback in the
    /// physics engine  thread).
    H3DSpaceId addSpace( SpaceParameters& p );

    /// Remove a collision space from the physics engine. Returns 0 on success.
    bool removeSpace( H3DSpaceId id );

    // Implement these functions.
    /*bool setSpaceParameters( H3DSpaceId space_id,
    SpaceParameters& p );

    bool getSpaceParameters( H3DSpaceId space_id,
    SpaceParameters& p );*/

    /// Add a force applied to the center of mass of the rigid body 
    /// where all values are defined in the global  coordinate system of the world.
    /// Returns 0 on success.
    int setGlobalExternalForce( H3DBodyId body,
      const Vec3f &force ); 

    /// Add a torque to a rigid body where the torque is specified around the global
    /// axis of the world coordinate system.
    /// Returns 0 on success.
    int setGlobalExternalTorque( H3DBodyId body,
      const Vec3f &torque );

    /// Set the global world parameters of the physics engine.
    void setWorldParameters( WorldParameters& params );    

    /// Synchronise with the scene graph
    void synchroniseWithSceneGraph();

    /// Adds the contacts from the last collision detection phase to the given list.
    /// Returns 0 on success.
    int getCurrentContacts( list< ContactParameters > &contacts );

    /// Get the internal data specific for the underying physics engine.
    inline EngineSpecificData* getEngineSpecificData() {
      return engine_data.get();
    }

    /// Set data specific for the underlying physics engine that is used. Should
    /// not be set by users directly but only by the physics engine implementation.
    inline void setEngineSpecificData( EngineSpecificData *d) {
      engine_data.reset( d );
    }

    /// Start stepping the simulation.
    virtual void startSimulation();

    /// Stop stepping the simulation.
    virtual void stopSimulation();

    /// Returns true if the simulation is currently running and advancing 
    /// the simulation.
    inline bool isSimulationRunning() {
      return simulation_running;
    }

    /// If the simulation is still executing steps requested by a previous call to
    /// beginSimulationSteps() then this function will block, otherwise the function 
    /// will return immediately.
    void waitForSimulationSteps();

    /// Begin executing a fixed number of simulation steps
    ///
    /// When called periodically from the graphics thread this allows for a deterministic
    /// physics/graphics thread interaction while still allowing parallel execution.
    ///
    /// \param nr_steps The number of physics steps to execute
    void beginSimulationSteps( std::size_t nr_steps );

    /// Begin copying out the results of the previous physics simulation steps
    void beginSynchronizeSteps();

    /// Set the global contact parameters for the engine, i.e. the parameters
    /// that will be used per default when two shapes collide.
    inline void setGlobalContactParameters( const GlobalContactParameters &params ) {
      global_contact_params_lock.lock();
      global_contact_params = params;
      global_contact_params_lock.unlock();
    }

    /// Get the global contact parameters for the engine, i.e. the parameters
    /// that will be used per default when two shapes collide.
    inline GlobalContactParameters getGlobalContactParameters( ) {
      global_contact_params_lock.lock();
      GlobalContactParameters r = global_contact_params;
      global_contact_params_lock.unlock();
      return r;
    }

    /// Get the world parameters for the engine, i.e. the parameters
    /// that will be used per default when two shapes collide.
    inline WorldParameters getWorldParameters( ) {
      world_params_lock.lock();
      WorldParameters r = *world_params;
      world_params_lock.unlock();
      return r;
    }

    /// Return the update rate(in Hz) that the thread of the physics engine
    /// is running in.
    inline int getUpdateRate() {
      return update_rate;
    }

    /// Return the time(in seconds) spent in the last loop of the physics
    /// engine update thread.
    inline H3DTime getLastLoopTime() {
      return time_in_last_loop;
    }

    /// Set the time(in seconds) spent in the last loop of the physics
    /// engine update thread(only to be used by physics engine implementations_.
    inline void setLastLoopTime( H3DTime t) {
      time_in_last_loop = t;
    }

    /// Set the time interval (seconds) over which to average the frame rate calculation
    inline void setFramerateInterval ( H3DTime t ) {
      frame_rate_interval= t;
    }

    /// Get the step size(in secondes) for the physics engine to advance
    /// each loop.
    inline H3DFloat getStepSize() {
      return step_size;
    }

    /// Returns the number of rigid bodies currently in the simulator.
    inline unsigned int getCurrentNrRigidBodies() {
      return (unsigned int )rigid_bodies.size();
    }

    /// Get the the ids of all current rigid bodies. The paramater c
    /// must be an stl container class containing objects of H3DBodyId
    /// and supporting the push_back function. E.g. list< H3DBodyId > or
    /// vector< H3DBodyId >
    template< class Container >
    inline void getCurrentRigidBodies( Container &c ) {
      for( RigidBodyMap::iterator i = rigid_bodies.begin(); 
        i != rigid_bodies.end(); ++i ) {
          c.push_back( (*i).first );
      }
    } 

    /// Returns the number of shapes currently in the simulator.
    inline unsigned int getCurrentNrCollidables() {
      return (unsigned int) collidables.size();
    }

    /// Get the the ids of all current shapes. The paramater c
    /// must be an stl container class containing objects of H3DShapeId
    /// and supporting the push_back function. E.g. list< H3DShapeId > or
    /// vector< H3DShapeId >
    template< class Container >
    inline void getCurrentCollidables( Container &c ) {
      for( CollidableMap::iterator i = collidables.begin(); 
        i != collidables.end(); ++i ) {
          c.push_back( (*i).first );
      }
    } 

    /// Returns the number of constraint currently in the simulator.
    inline unsigned int getCurrentConstraints() {
      return (unsigned int) constraints.size();
    }

    /// Returns the solver type set by the user.
    inline H3DUInt32 getSolverType() const {
      return solver_type;
    }

    /// Get the the ids of all current constraints. The paramater c
    /// must be an stl container class containing objects of H3DConstraintId
    /// and supporting the push_back function. E.g. list< H3DConstraintId > or
    /// vector< H3DConstraintId >
    template< class Container >
    inline void getCurrentConstraints( Container &c ) {
      for( ConstraintMap::iterator i = constraints.begin(); 
        i != constraints.end(); ++i ) {
          c.push_back( (*i).first );
      }
    } 

    /// This function returns true if the physics engine with the given
    /// name has been registered and can be used. E.g. "ODE", "PhysX"
    /// or "Bullet"
    static bool supportsPhysicsEngine( const string &engine );

    /// This function returns a list of strings with names of supported
    /// physics engines.
    static vector< string > getSupportedPhysicsEngineNames();

    /// The PhysicsEngineCallbacks struct is a container for callback functions
    /// for an implementation of rigid body physics in a specific physics engine.
    /// When creating support for a new physics engine each of the functions
    /// in this struct must be implemented and collected in a PhysicsEngineCallbacks
    /// struct and then registered via the PhysicsEngineRegistration class
    /// (or registerPhysicsEngine function).
    struct PhysicsEngineCallbacks {
      virtual ~PhysicsEngineCallbacks() {}

      // global engine callback functions
      H3DUtil::PeriodicThread::CallbackCode (*initEngine)(void *data);
      H3DUtil::PeriodicThread::CallbackCode (*deInitEngine)(void *data);
      H3DUtil::PeriodicThread::CallbackCode (*doSimulationSteps)(void *data);
      H3DUtil::PeriodicThread::CallbackCode (*setWorldParameters)(void *data);
      H3DUtil::PeriodicThread::CallbackCode (*synchroniseWithSceneGraph)(void *data);
      H3DUtil::PeriodicThread::CallbackCode (*getCurrentContacts)(void *data);
      
      // shape callback functions
      H3DUtil::PeriodicThread::CallbackCode (*addCollidable)(void *data);
      H3DUtil::PeriodicThread::CallbackCode (*removeCollidable)(void *data);
      H3DUtil::PeriodicThread::CallbackCode (*setCollidableParameters)(void *data);
      H3DUtil::PeriodicThread::CallbackCode (*getCollidableParameters)(void *data);

      // space callback functions
      H3DUtil::PeriodicThread::CallbackCode (*addSpace)(void *data);
      H3DUtil::PeriodicThread::CallbackCode (*removeSpace)(void *data);
      H3DUtil::PeriodicThread::CallbackCode (*setSpaceParameters)(void *data);
      H3DUtil::PeriodicThread::CallbackCode (*getSpaceParameters)(void *data);

      // rigid body callback functions
      H3DUtil::PeriodicThread::CallbackCode (*addRigidBody)(void *data);
      H3DUtil::PeriodicThread::CallbackCode (*removeRigidBody)(void *data);
      H3DUtil::PeriodicThread::CallbackCode (*setRigidBodyParameters)(void *data);
      H3DUtil::PeriodicThread::CallbackCode (*getRigidBodyParameters)(void *data);

      // constraint callback functions
      H3DUtil::PeriodicThread::CallbackCode (*addConstraint)(void *data);
      H3DUtil::PeriodicThread::CallbackCode (*removeConstraint)(void *data);
      H3DUtil::PeriodicThread::CallbackCode (*setConstraintParameters)(void *data);
      H3DUtil::PeriodicThread::CallbackCode (*getConstraintParameters)(void *data);

      // external force callback functions
      H3DUtil::PeriodicThread::CallbackCode (*addGlobalExternalForceAndTorque)(void *data);
    };

    /// Class used to register a class to the registered physics engines.
    /// If you want to register a physics engine implementation create a static
    /// instance of this class. It will then be registered upon loading
    /// the dll it resides in automatically.
    struct PhysicsEngineRegistration{
    public:
      /// Constructor.
      PhysicsEngineRegistration( const string &name,
        PhysicsEngineCallbacks *callbacks ) {
          PhysicsEngineThread::registerPhysicsEngine( name, callbacks );
      }
    };

    /// Register a physics engine implementation.
    static void registerPhysicsEngine( const string &name,
      PhysicsEngineCallbacks *callbacks ) {
        registered_physics_engines()[name] = callbacks;
    }   


    /// The createPhysicsEngineCallbacks is a convinience function to 
    /// initialize a PhysicsEngineCallbacks class for use when registering
    /// a physics engine. Given a class with static functions named the same
    /// as the callback functions in PhysicsEngineCallbacks it will create
    /// a new PhysicsEngineCallbacks instance with the callbacks initialized
    /// to the callbacks from the class.
    /// E.g. createPhysicsEngineCallbacks< ODECallbacks >();
    template< class A > 
    static PhysicsEngineCallbacks *createPhysicsEngineCallbacks() {
      PhysicsEngineCallbacks *cb = new PhysicsEngineCallbacks;
      cb->initEngine = A::initEngine;
      cb->deInitEngine = A::deInitEngine;
      cb->doSimulationSteps = A::doSimulationSteps;
      cb->getCurrentContacts = A::getCurrentContacts;
      cb->setWorldParameters = A::setWorldParameters;
      cb->synchroniseWithSceneGraph= A::synchroniseWithSceneGraph;

      // shape callback functions
      cb->addCollidable = A::addCollidable;
      cb->removeCollidable = A::removeCollidable;
      cb->setCollidableParameters = A::setCollidableParameters;
      cb->getCollidableParameters = A::getCollidableParameters;

      // space callback functions
      cb->addSpace = A::addSpace;
      cb->removeSpace = A::removeSpace;
      cb->setSpaceParameters = A::setSpaceParameters;
      cb->getSpaceParameters = A::getSpaceParameters;

      // rigid body callback functions
      cb->addRigidBody = A::addRigidBody;
      cb->removeRigidBody = A::removeRigidBody;
      cb->setRigidBodyParameters = A::setRigidBodyParameters;
      cb->getRigidBodyParameters = A::getRigidBodyParameters;

      // constraint callback functions
      cb->addConstraint = A::addConstraint;
      cb->removeConstraint = A::removeConstraint;
      cb->setConstraintParameters = A::setConstraintParameters;
      cb->getConstraintParameters = A::getConstraintParameters;

      // external force callback functions
      cb->addGlobalExternalForceAndTorque = A::addGlobalExternalForceAndTorque;
      return cb;
    }

    /// Convinience function to get the callback functions for the underlying
    /// physics engine implementation.
    virtual PhysicsEngineCallbacks* callbacks() {
      return registered_physics_engines()[engine];
    }

    /// Set a list of current haptics devices used to apply haptic interaction forces
    /// to rigid bodies.
    void setHapticsDevices ( const NodeVector& _hapticsDevices );

    /// Adds a node to be deleted in synch with the graphics thread.
    /// One example use is to add a node which is not needed anymore but deleting it
    /// in the physics thread would cause problems( due to routing etc. ) in the scene-graph.
    /// Added nodes are deleted at the end of BodyCollection::traverseSG()
    void addNodeToDeleteInSynch( Node *n );

    /// Tells the physics engine thread to syncronise physics and graphics threads at the end
    /// of the graphics frame.
    void synchroniseAtTheEndOfTheFrame();

    /// True if using deterministic synchronization of the physics and graphics threads
    bool useSynchronization() {
      return use_synchronization;
    }

  protected:

    /// Nodes populated from the physics thread but deleted in the graphics thread.
    AutoRefVector < Node > nodesToDeleteInSync;
    MutexLock nodesToDeleteInSync_lock;

    /// If it is true synchronises the physics and graphics thread at the end of the graphics frame.
    bool synchroniseTheThreads;
    MutexLock synchroniseTheThreads_lock;

    PhysicsEngineParameters::ExternalForceTorqueParameters force_torque_params;

    NodeVector hapticsDevices;
    MutexLock hapticsDevices_lock;

    struct CallbacksMap : public std::map< string, PhysicsEngineCallbacks * > {
      ~CallbacksMap() {
        for( iterator i = begin(); i != end(); ++i ) {
          delete (*i).second;
          (*i).second = NULL;
        }
      }

      vector< string > getAllKeys() {
        vector< string > physics_engine_names;
        for( iterator i = begin(); i != end(); ++i )
          physics_engine_names.push_back( (*i).first );
        return physics_engine_names;
      }
    };

    /// A static map of all registered physics engines. It is a function with
    /// a local static instance in order to work around the static initialization
    /// order fiasco. Local static variables are initialized first time they are
    /// used while global static variables are initialized at program startup
    /// without any insurance of the order they are initialized. This is a problem
    /// since we need the CallbackMap to be created first. This way it is.
    static CallbacksMap &registered_physics_engines() {
      static CallbacksMap ans;
      return ans;
    }

    /// Callback function executed after stepping the simulation to finalize and copy out data
    ///
    /// Simply calls the method updateSimulation()
    static H3DUtil::PeriodicThread::CallbackCode updateSimulationCB(void *data);

    /// Callback function to execute a fixed number of physics frames
    static H3DUtil::PeriodicThread::CallbackCode executeSimulationSteps( void* data );

    /// Adds external haptic interaction forces to a rigid body
    void addHapticInteractionForces ( 
      PhysicsEngineParameters::RigidBodyParameters& params, 
      PhysicsEngineParameters::ExternalForceTorqueParameters& forceParams );

    /// A method executed after stepping the simulation to finalize and copy out data
    virtual void updateSimulation();

    /// This function will be called once per cycle in the physics engine thread
    /// in order to update the output parameters of shapes, bodies and constraints.
    void updateRigidBodyParameters();

    /// Do pre-processing of the CollidableParameters before it is sent to the physics thread.
    ///
    /// This includes cloning the geometry and scaling it by the Collidable's scale field.
    ///
    void preUpdateCollidable ( CollidableParameters& p );

    /// Applies the specified scale parameter to the geometry
    ///
    void scaleGeometry ( X3DGeometryNode& geometry, const Vec3f& scale );

    /// Helper function which returns true if the scale factor is uniform
    static bool isUniformScale ( const Vec3f& scale );

    /// Makes the scale parameter uniform and prints a warning message if it
    /// is not already uniform
    static Vec3f makeUniformScale ( const Vec3f& scale, X3DGeometryNode& geometry );

    /// The underlying physics engine that we currently use.
    string engine;

    /// Internal data specific for the underlying physics engine.
    auto_ptr< EngineSpecificData > engine_data;

    /// True if the simulation is running.
    bool simulation_running;

    /// Thread callback function handle for the callback function that performs 
    /// the simulation step.
    int simulation_cb_handle;

    /// Thread callback function handle for updateSimulationCB callback function.
    int update_cb_handle;

    /// Global contact parameters to be used by default on all contacts.
    GlobalContactParameters global_contact_params;

    /// Global world parameters for the physics engine.
    auto_ptr < WorldParameters > world_params;

    /// Mutex lock for access to the global_contact_params member.
    MutexLock global_contact_params_lock;

    /// Mutex lock for access to world_params member.
    MutexLock world_params_lock;

    /// The update rate of the physics engine thread(in Hz)
    int update_rate;

    /// the time(in seconds) spent in the last loop of the physics
    /// engine update thread.
    H3DTime time_in_last_loop;

    /// The time the update_rate variale was last updated.
    H3DTime time_of_last_update;

    /// Number of physics frames since last frame rate calculation
    int frame_count;

    /// Time interval (seconds) over which to average the frame rate calculation
    H3DTime frame_rate_interval;

    /// The step size(in seconds) to take in each step in the simulation.
    H3DFloat step_size;

    /// A map of all rigid bodies and their parameters currently in the
    /// simulation.
    typedef  map< H3DBodyId, RigidBodyParameters* > RigidBodyMap;
    RigidBodyMap rigid_bodies;

    /// Mutex lock for access to the rigid_bodies member variable.
    MutexLock rigid_body_lock;

    /// A map of all shapes and their parameters currently in the
    /// simulation.
    typedef  map< H3DCollidableId, CollidableParameters* > CollidableMap;
    CollidableMap collidables;

    /// Mutex lock for access to the shapes member variable.
    MutexLock collidable_lock;

   /// If true then all callbacks should be executed in the main/caller thread
   bool useMainThread;

   /// A list of async callbacks to execute in the main thread
   CallbackList mainThreadCallbacks;

    /// A map of all constraints and their parameters currently in the
    /// simulation.
    typedef map< H3DConstraintId, ConstraintParameters* > ConstraintMap;
    ConstraintMap constraints;

    /// Mutex lock for access to the constraint member variable.
    MutexLock constraint_lock;

    /// A map of all spaces and their parameters currently in the
    /// simulation.
    typedef map< H3DSpaceId, SpaceParameters* > SpaceMap;
    SpaceMap spaces;

    /// Mutex lock for access to the space member variable.
    MutexLock space_lock;

    /// The type of solver to be used for the physics engine.
    const H3DUInt32 solver_type;

#ifdef USE_CONTACTS_LOCK
    typedef list< ContactParameters > ContactList;
    ContactList currentContacts;

    MutexLock currentContacts_lock;
#endif

#ifdef DEBUG_RB_LAG
   typedef std::map<H3DBodyId,FILE*> DebugFileMap;
   DebugFileMap lagDebugFiles;
#endif

    /// A condition lock to synchronize physics and graphics threads in the
    /// case that deterministic synchronization is used (see beginSimulationSteps())
    ConditionLock stepping_simulation_lock;

    /// The condition variable associated with stepping_simulation_lock. True if simulation
    /// steps are currently executing.
    bool stepping_simulation;

    /// True if using deterministic synchronization of the physics and graphics threads
    bool use_synchronization;

    /// The number of simulation steps requested by beginSimulationSteps()
    std::size_t nr_physics_steps;
  };


}
#endif
