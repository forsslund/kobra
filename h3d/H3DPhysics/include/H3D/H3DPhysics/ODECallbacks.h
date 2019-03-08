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
/// \file ODECallbacks.h
/// \brief Header file for ODECallbacks, struct with callbacks using ODE
/// as the physics engine.
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __ODECALLBACKS__
#define __ODECALLBACKS__

#include <H3D/H3DPhysics/H3DPhysics.h>

#ifdef HAVE_ODE

#include <H3DUtil/Threads.h>
#include <H3D/H3DPhysics/PhysicsEngineThread.h>
#include <ode/ode.h>

namespace H3D {
  /// \ingroup PhysicsEngineCallbackStructs
  /// \brief The ODECallbacks struct contains callback functions and
  /// variables needed to interface with ODE (Open Dynamics Engine ).
  ///
  /// ODE is an open source physics engine which
  /// can be found <a href="http://www.ode.org/">here</a>.
  struct ODECallbacks {

    struct ODESpecificData: public H3D::PhysicsEngineThread::EngineSpecificData {
      ODESpecificData( int _max_nr_contacts = 50 ) :
        max_nr_contacts( _max_nr_contacts ),
        contact_group( NULL ),
        space_id( NULL ),
        world_id( NULL ) {
        contacts = new dContact[ max_nr_contacts ];
      }
      
      ~ODESpecificData() {
        delete [] contacts;
      }

      struct CollidableOffset {
        Vec3f translation;
        Rotation rotation;
      };

      dWorldID world_id;
      dSpaceID space_id;
      dJointGroupID contact_group;
      
      /// Map of geometries that are associated with triangle meshes. The first item
      /// is id for transform of the second item. The second item is the triangle mesh.
      std::map< dGeomID, dGeomID > tri_meshes;

      // Map of offsets applied to a geometry (collidable shape).
      // The offsets can only be applied when the rigidbody they belong to
      // has been updated and the collidable has been attached to the body
      std::map< dGeomID, std::vector< CollidableOffset > > pending_offsets;
      
      /// The maximum number of contacts to collect per geometry-geometry pair
      int max_nr_contacts;
      
      /// Array of contacts, used to obtain contacts from ODE for a single geometry-geometry pair
      dContact *contacts;

      /// All the contacts detected during previous collision detection frame
      vector< dContact > allContacts;

      /// Map of joint id to force callback (used by slider joint only atm).
      map< dJointID, dReal > slider_joint_forces;
    };

   ///////// Physics engine registration callbacks //////////////

    // global engine callback functions
    static H3DUtil::PeriodicThread::CallbackCode initEngine(void *data);
    static H3DUtil::PeriodicThread::CallbackCode deInitEngine(void *data);
    static H3DUtil::PeriodicThread::CallbackCode doSimulationSteps(void *data);
    static H3DUtil::PeriodicThread::CallbackCode setWorldParameters(void *data);
    static H3DUtil::PeriodicThread::CallbackCode synchroniseWithSceneGraph(void *data);
    static H3DUtil::PeriodicThread::CallbackCode getCurrentContacts(void *data);
    // shape callback functions
    // implemented for Box and Sphere
    static H3DUtil::PeriodicThread::CallbackCode addCollidable(void *data);
    static H3DUtil::PeriodicThread::CallbackCode removeCollidable(void *data);
    static H3DUtil::PeriodicThread::CallbackCode setCollidableParameters(void *data);
    static H3DUtil::PeriodicThread::CallbackCode getCollidableParameters(void *data);

    // Collision space callback functions
    static H3DUtil::PeriodicThread::CallbackCode addSpace(void *data);
    static H3DUtil::PeriodicThread::CallbackCode removeSpace(void *data);
    static H3DUtil::PeriodicThread::CallbackCode setSpaceParameters(void *data);
    static H3DUtil::PeriodicThread::CallbackCode getSpaceParameters(void *data);

    // Rigid body callback functions
    static H3DUtil::PeriodicThread::CallbackCode addRigidBody(void *data);
    static H3DUtil::PeriodicThread::CallbackCode removeRigidBody(void *data);
    static H3DUtil::PeriodicThread::CallbackCode setRigidBodyParameters(void *data);    
    static H3DUtil::PeriodicThread::CallbackCode getRigidBodyParameters(void *data);    

    // Joint functions
    static H3DUtil::PeriodicThread::CallbackCode addConstraint(void *data);
    static H3DUtil::PeriodicThread::CallbackCode removeConstraint(void *data);
    static H3DUtil::PeriodicThread::CallbackCode setConstraintParameters(void *data);
    static H3DUtil::PeriodicThread::CallbackCode getConstraintParameters(void *data);

    // external force callback functions
    static H3DUtil::PeriodicThread::CallbackCode addGlobalExternalForceAndTorque(void *data);  

    ////////////////////////////////////////////////////////////

  protected:

    // internal callback functions
    static void nearCallback(void *data, dGeomID o1, dGeomID o2);

    static void addShape( ShapeParameters *params );
    static void addOffset( OffsetParameters * params );
 
    static void attachJoint(void *data);
    static void setSingleAxisHingeJointParameters(void *data);
    static void getSingleAxisHingeJointParameters(void *data);

    static void setBallJointParameters(void *data);
    static void getBallJointParameters(void *data);

    static void setDoubleAxisHingeJointParameters(void *data);
    static void getDoubleAxisHingeJointParameters(void *data);

    static void setMotorJointParameters(void *data);
    static void getMotorJointParameters(void *data);

    static void setSliderJointParameters(void *data);
    static void getSliderJointParameters(void *data);

    static void setUniversalJointParameters(void *data);
    static void getUniversalJointParameters(void *data);

     // registration of physics engine callback functions.
    static H3D::PhysicsEngineThread::PhysicsEngineRegistration registration;

    // Internal setRigidBodyParameters function.
    static void setRigidBodyParameters( PhysicsEngineParameters::RigidBodyParameters *params,
                                        ODESpecificData *ode_data );

    // Internal setConstaintParameters function.
    static void setConstraintParameters( PhysicsEngineParameters::JointParameters * params );

    // Offsets can only be applied if there's a rigid body set for the collidable.
    // This function handles pending offsets when there is a body attached. 
    static void handlePendingOffsets( RigidBodyParameters* params );

  };

}

#endif // HAVE_ODE
#endif // __ODECALLBACKS__
