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
/// \file PhysX3Callbacks.h
/// \brief Header file for PhysX3Callbacks, struct with callbacks using PhysX3
/// as the physics engine.
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __PHYSX3CALLBACKS__
#define __PHYSX3CALLBACKS__
#define NOMINMAX

#include <H3D/H3DApi.h>
#include <H3DUtil/Threads.h>
#include <H3D/H3DPhysics/PhysicsEngineThread.h>

#ifdef HAVE_PHYSX3

// the PhysX3 include files requires a define with LINUX set to 1 when
// compiling in Linux.
#ifdef H3D_LINUX
// check that exactly one of NDEBUG and _DEBUG is defined
#if !(defined NDEBUG ^ defined _DEBUG)
  #define _DEBUG 1
  
#endif
#endif

#include <PxPhysicsAPI.h>

using namespace physx;

namespace H3D {

  /// Convert H3D vector to PhysX3 vector
  static PxVec3 toPxVec3 ( const Vec3f& v ) { return PxVec3 ( v.x, v.y, v.z ); }

  static Vec3f toVec3f ( const PxVec3& v ) { return Vec3f ( v.x, v.y, v.z ); }

  static Rotation toRotation ( const PxQuat& q ) { return Rotation(Quaternion(q.x,q.y,q.z,q.w)); }

  static PxQuat toPxQuat ( const Rotation& r ) { 
    Quaternion q ( r );
    q.normalize();
    return PxQuat ( q.v.x, q.v.y, q.v.z, q.w );
  }

  static Matrix4f toMatrix4f ( const PxTransform& t ) {
    Matrix4f m ( toRotation ( t.q ) );
    m[0][3]= t.p.x;
    m[1][3]= t.p.y;
    m[2][3]= t.p.z;
    return m;
  }

  /// \ingroup PhysicsEngineCallbackStructs
  /// \brief The PhysX3Callbacks struct contains callback functions and
  /// variables needed to interface with PhysX3 SDK.
  ///
  /// PhysX3 SDK is a close source physics engine library which can be
  /// can be found <a href="http://supportcenteronline.com/ics/support/default.asp?deptID=1949">here</a>.
  struct PhysX3Callbacks {

    class SimulationEventCallback : public PxSimulationEventCallback {
    public:   
      // This is called when a breakable constraint breaks. 
      virtual void onConstraintBreak (PxConstraintInfo *constraints, PxU32 count) {}
 
      // This is called during PxScene::fetchResults with the actors which have just been woken up. 
      virtual void onWake (PxActor **actors, PxU32 count) {}
 
      // This is called during PxScene::fetchResults with the actors which have just been put to sleep. 
      virtual void onSleep (PxActor **actors, PxU32 count) {}
 
      // The user needs to implement this interface class in order to be notified when certain contact events occur. 
      virtual void onContact (const PxContactPairHeader &pairHeader, const PxContactPair *pairs, PxU32 nbPairs);
 
      virtual void onTrigger (PxTriggerPair *pairs, PxU32 count) {}

      void clearContacts ();

      void getContacts ( list<PhysicsEngineParameters::ContactParameters>& contacts );
      void setContactReportingMode( string _mode = "DEFAULT" ) {
        contact_reporting_mode = _mode;
      };

    protected:
      list<PhysicsEngineParameters::ContactParameters> contacts;
      std::string contact_reporting_mode;
    };

    class ContactModifyCallback : public PxContactModifyCallback{
    public:   
      
      // This is called for each pair of collidables PxPairFlag::eMODIFY_CONTACTS
      // flag is set for in the physX3FilterShader filter.
      virtual void onContactModify(PxContactModifyPair* const pairs, PxU32 count);
    };

    struct PhysX3SpecificData: public PhysicsEngineThread::EngineSpecificData {
      PhysX3SpecificData() : 
        foundation ( NULL ), 
        sdk( NULL ), 
        scene( NULL ), 
        simulation_events ( NULL ), 
        cpu_dispatcher ( NULL ),
        last_update_time ( 0 ),
        default_material( NULL ),
        contact_modifier( NULL ),
        cooking( NULL ) {
      }

      PxFoundation* foundation;
      PxPhysics * sdk;
      PxScene * scene;
      PxCooking* cooking;
      PxMaterial* default_material;
      SimulationEventCallback* simulation_events;
      ContactModifyCallback* contact_modifier;
      PxDefaultCpuDispatcher* cpu_dispatcher;

      H3DTime last_update_time;
    };

   ///////// Physics engine registration callbacks //////////////

    // Global engine callback functions
    static H3DUtil::PeriodicThread::CallbackCode initEngine(void *data);
    static H3DUtil::PeriodicThread::CallbackCode deInitEngine(void *data);
    static H3DUtil::PeriodicThread::CallbackCode doSimulationSteps(void *data);
    static H3DUtil::PeriodicThread::CallbackCode setWorldParameters(void *data);
    static H3DUtil::PeriodicThread::CallbackCode synchroniseWithSceneGraph(void *data);
    static H3DUtil::PeriodicThread::CallbackCode getCurrentContacts(void *data);
    
    // Shape callback functions
    static H3DUtil::PeriodicThread::CallbackCode addCollidable(void *data);
    static H3DUtil::PeriodicThread::CallbackCode removeCollidable(void *data);
    static H3DUtil::PeriodicThread::CallbackCode setCollidableParameters(void *data);
    static H3DUtil::PeriodicThread::CallbackCode getCollidableParameters(void *data);

    // Collision space callback functions
    static H3DUtil::PeriodicThread::CallbackCode addSpace( void *data );
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

    // soft body callback functions
    static H3DUtil::PeriodicThread::CallbackCode addSoftBody ( void *data );
    static H3DUtil::PeriodicThread::CallbackCode removeSoftBody ( void *data );
    static H3DUtil::PeriodicThread::CallbackCode setSoftBodyParameters ( void *data );
    static H3DUtil::PeriodicThread::CallbackCode getSoftBodyParameters ( void *data );
    static H3DUtil::PeriodicThread::CallbackCode applyExternalForces ( void *data );

  protected:

    /// registration of physics engine callback functions.
    static H3D::PhysicsEngineThread::PhysicsEngineRegistration registration;
  };

  class PhysX3CollidableShape;

  /// PhysX implementation of a rigid body
  class PhysX3RigidBody {
  public:
    PhysX3RigidBody ( PhysicsEngineParameters::RigidBodyParameters& _params );

    virtual ~PhysX3RigidBody ();

    virtual void setParameters ( PhysicsEngineParameters::RigidBodyParameters& _params );

    virtual void getParameters ( PhysicsEngineParameters::RigidBodyParameters& _params );

    virtual void createRigidBody( PhysX3Callbacks::PhysX3SpecificData *physx_data );

    /// Apply sleeping parameters to body
    virtual void setSleeping ( PhysicsEngineParameters::RigidBodyParameters& _params );

    virtual PxRigidActor* getActor() {
      return rigid_body;
    }

    void removeCollidableShape( PhysX3CollidableShape* s );

    virtual void createRigidBodyStatic( PhysX3Callbacks::PhysX3SpecificData *physx_data );

    /// The list of all bodies
    static std::vector< PhysX3RigidBody* > all_physX3_bodies;

  protected:

    void setPxRigidBodyParameters( PxRigidBody* _body, PhysicsEngineParameters::RigidBodyParameters& _params );
    
    PxRigidActor* rigid_body;

    typedef std::vector < PhysX3CollidableShape* > ShapeVector;
    ShapeVector shapes;

    /// The mass of the body
    H3DFloat mass;

    /// The geometry describing the mass density of the body
    Node* massDensityModel;

    /// Local offset for center of mass
    Vec3f centerOfMass;

    /// User defined inertia tensor
    Matrix3f inertia;

    /// True if the body is fixed in space
    bool fixed;

    /// True if the body can be controlled kinematically
    bool kinematic;

    /// Should the body be put to sleep if inactive
    bool autoDisable;

    /// Linear speed below which body should sleep
    ///
    /// Note: For PhysX it is the total mass normalized kinetic energy that is tested
    ///
    H3DFloat disableLinearSpeed;

    /// Angular speed below which body should sleep
    ///
    /// Note: For PhysX it is the total mass normalized kinetic energy that is tested
    ///
    H3DFloat disableAngularSpeed;

    /// Tells whether the body is static or not
    bool    isStatic;
  };

  /// PhysX implementation of an PxArticulationLink
  class PhysX3ArticulationLink : public PhysX3RigidBody {
  public:

    PhysX3ArticulationLink ( PhysicsEngineParameters::RigidBodyParameters& _params,
      PxArticulation* _articulation, PxArticulationLink* _link );

    virtual ~PhysX3ArticulationLink ();

    virtual void setParameters ( PhysicsEngineParameters::RigidBodyParameters& _params ){};

    virtual void getParameters ( PhysicsEngineParameters::RigidBodyParameters& _params ){};

    virtual void createRigidBody( PhysX3Callbacks::PhysX3SpecificData *physx_data ){};

    /// Apply sleeping parameters to body
    void setSleeping ( PhysicsEngineParameters::RigidBodyParameters& _params ){};

    PxArticulation* getArticulation() {
      return articulation;
    }
    
    virtual PxRigidBody* getActor() {
      return link;
    }

  protected:

    PxArticulation* articulation;
    PxArticulationLink* link;

  };


  /// PhysX implementation of an articulated rigid body
  class PhysX3ArticulatedRigidBody : public PhysX3RigidBody {
  public:

    typedef std::vector<PxRigidBody*> PxRigidBodyList;
    typedef std::vector < PhysX3ArticulationLink* > ArticulationLinkList;
    
    ShapeVector shapes;

    PhysX3ArticulatedRigidBody ( PhysicsEngineParameters::RigidBodyParameters& _params );

    virtual ~PhysX3ArticulatedRigidBody ();

    virtual void setParameters ( PhysicsEngineParameters::RigidBodyParameters& _params );

    virtual void getParameters ( PhysicsEngineParameters::RigidBodyParameters& _params );

    virtual void createRigidBody( PhysX3Callbacks::PhysX3SpecificData *physx_data );

    /// Apply sleeping parameters to body
    void setSleeping ( PhysicsEngineParameters::RigidBodyParameters& _params );

    PxArticulation* getArticulation() {
      return articulation;
    }

    PxRigidBodyList& getActors() {
      return rigid_bodies;
    }

    virtual PxRigidBody* getActor() {
      if( rigid_bodies.size() > 0 )
        return rigid_bodies[0];
      return NULL;
    }

  protected:
    

    PxArticulation* articulation;

    PxRigidBodyList rigid_bodies;

    ArticulationLinkList articulation_links;

  };

  /// PhysX implementation of a collidable shape
  class PhysX3CollidableShape {
  public:
    PhysX3CollidableShape ( PhysicsEngineParameters::ShapeParameters& _params );

    virtual ~PhysX3CollidableShape ();

    void setParameters ( PhysicsEngineParameters::ShapeParameters& _params );

    void getParameters ( PhysicsEngineParameters::ShapeParameters& _params );

    void addToBody ( PhysX3RigidBody& _body, PhysX3Callbacks::PhysX3SpecificData& physx_data );

    void removeFromBody ( bool removeFromBodyShapes = false );

    vector<PxGeometry*> getGeometry () {
      return geometry;
    }

    std::vector<Vec4d> clipPlanes;
    MutexLock clipPlanes_mutex;

  protected:
      
    void addToBodyInternal ( PhysX3RigidBody& _body, PhysX3Callbacks::PhysX3SpecificData& physx_data );

    void removeFromBodyInternal ();

    // Update the PhysX filter data used for collision filtering in the shader.
    void updateFilterData( PhysX3Callbacks::PhysX3SpecificData * physx_data );

    PhysX3RigidBody* body;
    bool collisionEnabled;
    bool suppressDisabledContacts;
    bool setFlagsForAll;
    vector<PxShape*> shape;
    vector<PxGeometry*> geometry;
    Vec3f translation;
    Rotation rotation;
    Rotation rotationOffset;
    H3DDouble restOffset;
    H3DDouble contactOffset;

    unsigned int exceptionGroupMask;
    unsigned int selectionGroupMask;
    unsigned int contactFilterOptionsMask;
  };
}

#endif // HAVE_PHYSX3
#endif // __PHYSX3CALLBACKS__
