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
/// \file SoftBodyPhysicsEngineThread.h
/// \brief Header file for SoftBodyPhysicsEngineThread.
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __SOFTBODYPHYSICSENGINETHREAD__
#define __SOFTBODYPHYSICSENGINETHREAD__

#include <H3D/H3DPhysics/PhysicsEngineThread.h>
#include <H3D/H3DPhysics/SoftBodyParameters.h>

namespace H3D {

  using namespace PhysicsEngineParameters;

  /// \ingroup SoftBody
  /// A specialization of PhysicsEngineThread that provides support 
  /// for soft body physics simulation in addition to rigid body
  class H3DPHYS_API SoftBodyPhysicsEngineThread : public PhysicsEngineThread {
  public:

    /// Constructor.
    /// \param engine The name of the physics engine to use.
    /// \param thread_priority The priority of the thread.
    /// \param thread_frequency The frequence of the thread loop. -1 means
    /// run as fast as possible.
    /// \param _useMainThread If true execute all callbacks in the main thread.
    /// \param _use_synchronization If true, use deterministic synchronization of the physics and graphics threads.
    SoftBodyPhysicsEngineThread( const string &engine = "Bullet",
                                 Priority thread_priority = NORMAL_PRIORITY,
                                 int thread_frequency = 100,
                                 bool _useMainThread = false,
                                 bool _use_synchronization = false );

    /// Create a new soft body with the given parameters.
    H3DBodyId addSoftBody( H3DSoftBodyNodeParameters& params );

    /// Remove a soft body from the physics engine. Returns true on success.
    bool removeSoftBody( H3DBodyId body );

    /// Set the parameters for a given rigid body.
    /// Only the paramaters in the SoftBodyParameters
    /// structure that has actually been set will be changed.
    bool setSoftBodyParameters( H3DBodyId body, 
      H3DSoftBodyNodeParameters& params );

    /// Get the parameters for a given rigid body. The RigidBodyParameters
    /// structure will be updated with the values from the specified rigid
    /// body.
    /// Returns true on success.
    bool getSoftBodyParameters( H3DBodyId body,
      H3DSoftBodyNodeParameters& params );

    // Modifier functions
    /// Create a new modifier with the given parameters. 
    /// An id to the new modifier is returned or -1 if the operation could not be
    /// performed.
    H3DModifierId addModifier( ModifierParameters& params );

    /// Remove an modifier from the physics engine. Returns true on success.
    bool removeModifier( H3DModifierId modifier );

    /// Set the parameters for a given modifier.
    /// Only the paramaters in the ModifierParameters
    /// structure that has actually been set will be changed.
    bool setModifierParameters( H3DModifierId modifier, 
      ModifierParameters& params );

    /// Get the parameters for a given modifier. The ModifierParameters
    /// structure will be updated with the values from the specified modifier.
    /// Returns true on success.
    bool getModifierParameters( H3DModifierId modifier,
      ModifierParameters& params );


    /// Structure containing callback functions that implement the engine
    struct SoftBodyPhysicsEngineCallbacks : public PhysicsEngineCallbacks {
      // soft body callback functions
      H3DUtil::PeriodicThread::CallbackCode (*addSoftBody)(void *data);
      H3DUtil::PeriodicThread::CallbackCode (*removeSoftBody)(void *data);
      H3DUtil::PeriodicThread::CallbackCode (*setSoftBodyParameters)(void *data);
      H3DUtil::PeriodicThread::CallbackCode (*getSoftBodyParameters)(void *data);
      H3DUtil::PeriodicThread::CallbackCode (*applyExternalForces)(void *data);

    };

    /// Class used to register a class to the registered physics engines.
    /// If you want to register a physics engine implementation create a static
    /// instance of this class. It will then be registered upon loading
    /// the dll it resides in automatically.
    struct SoftBodyPhysicsEngineRegistration{
    public:
      /// Constructor.
      SoftBodyPhysicsEngineRegistration( const string &name,
        SoftBodyPhysicsEngineCallbacks* callbacks ) {
          PhysicsEngineThread::registerPhysicsEngine( name, callbacks );
          SoftBodyPhysicsEngineThread::registerSoftBodyPhysicsEngine( name, callbacks );
      }
    };

    /// Register a physics engine implementation.
    static void registerSoftBodyPhysicsEngine( const string &name,
      SoftBodyPhysicsEngineCallbacks* callbacks ) {
        registered_softbody_physics_engines()[name] = callbacks;
    }

    template< class A > 
    static SoftBodyPhysicsEngineCallbacks *createSoftBodyPhysicsEngineCallbacks() {
      SoftBodyPhysicsEngineCallbacks *cb = new SoftBodyPhysicsEngineCallbacks;
      cb->initEngine = A::initEngine;
      cb->deInitEngine = A::deInitEngine;
      cb->doSimulationSteps = A::doSimulationSteps;
      cb->getCurrentContacts = A::getCurrentContacts;
      cb->setWorldParameters = A::setWorldParameters;

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

      // soft body simulation callbacks
      cb->addSoftBody = A::addSoftBody;
      cb->removeSoftBody = A::removeSoftBody;
      cb->setSoftBodyParameters = A::setSoftBodyParameters;
      cb->getSoftBodyParameters = A::getSoftBodyParameters;
      cb->applyExternalForces = A::applyExternalForces;

      return cb;
    }

    /// Convinience function to get the callback functions for the underlying
    /// physics engine implementation.
    virtual SoftBodyPhysicsEngineCallbacks* callbacks() {
      return registered_softbody_physics_engines()[engine];
    }

    struct SoftBodyCallbacksMap :
      public std::map< string, SoftBodyPhysicsEngineCallbacks * > {
      // The reason for not having a destructor in this Map is because the
      // engine is properly removed by the normal CallbacksMap struct that
      // is used for rigid body physics engines.
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
    static SoftBodyCallbacksMap& registered_softbody_physics_engines() {
      static SoftBodyCallbacksMap ans;
      return ans;
    }

    /// This function returns true if the physics engine with the given
    /// name has been registered and can be used. E.g. "ODE", "PhysX"
    /// or "Bullet"
    static bool supportsPhysicsEngine( const string &engine );

    /// This function returns a list of strings with names of supported
    /// physics engines.
    static vector< string > getSupportedPhysicsEngineNames();

  protected:

    /// Per simulation tick updates to the simulation
    virtual void updateSimulation ();

    /// This function will be called once per cycle in the physics engine thread
    /// in order to update the output parameters of shapes, bodies and constraints.
    void updateSoftBodyParameters();

    /// Apply forces from haptic interaction with soft bodies
    void applyManipulationForces ();

    /// Apply external forces to soft bodies
    void applyExternalForces ();

    /// Clear all forces accumulated on the soft bodies
    void clearAllForces ();

    /// A map of all soft bodies and their parameters currently in the
    /// simulation.
    typedef  map< H3DBodyId, H3DSoftBodyNodeParameters* > SoftBodyMap;
    SoftBodyMap soft_bodies;

    /// Mutex lock for access to the soft_bodies member variable.
    MutexLock soft_body_lock;

    /// A map of all modifiers and their parameters currently in the
    /// simulation.
    typedef  map< H3DModifierId, ModifierParameters* > ModifierMap;
    ModifierMap modifiers;

    /// Mutex lock for access to the modifiers member variable.
    MutexLock modifier_lock;
  };

}
#endif
