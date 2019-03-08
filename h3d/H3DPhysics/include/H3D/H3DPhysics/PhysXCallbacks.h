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
/// \file PhysXCallbacks.h
/// \brief Header file for PhysXCallbacks, struct with callbacks using PhysX
/// as the physics engine.
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __PHYSXCALLBACKS__
#define __PHYSXCALLBACKS__
#define NOMINMAX

#include <H3D/H3DApi.h>
#include <H3D/H3DPhysics/H3DPhysics.h>
#include <H3DUtil/Threads.h>
#include <H3D/H3DPhysics/SoftBodyPhysicsEngineThread.h>

#ifdef HAVE_PHYSX

// the PhysX include files requires a define with LINUX set to 1 when
// compiling in Linux.
#ifdef H3D_LINUX
#define LINUX 1
#endif

#include <NxPhysics.h>
#include <NxFoundation.h>
#include <PhysXLoader.h>

namespace H3D {

  /// Convert H3D vector to PhysX vector
  static NxVec3 toNxVec3 ( const Vec3f& v ) { return NxVec3 ( v.x, v.y, v.z ); }

  static Vec3f toVec3f ( const NxVec3& v ) { return Vec3f ( v.x, v.y, v.z ); }

  /// \ingroup PhysicsEngineCallbackStructs
  /// \brief The PHYSXCallbacks struct contains callback functions and
  /// variables needed to interface with PhysX SDK.
  ///
  /// PhysX SDK is a close source physics engine library which can be
  /// can be found <a href="http://supportcenteronline.com/ics/support/default.asp?deptID=1949">here</a>.
  struct PhysXCallbacks {

    class ContactReport : public NxUserContactReport {        
      void onContactNotify(NxContactPair& pair, NxU32 events);

    public:
      void clearContacts ();

      void getContacts ( list<PhysicsEngineParameters::ContactParameters>& contacts );

    protected:
      list<PhysicsEngineParameters::ContactParameters> contacts;
    };

    struct PhysXSpecificData: public PhysicsEngineThread::EngineSpecificData {
      PhysXSpecificData() : sdk( NULL ), scene( NULL ) {
      }

      NxPhysicsSDK * sdk;
      NxScene * scene;
      NxMaterial * default_material;

      ContactReport contactReport;
      map< NxJoint *, NxReal > slider_joint_forces;
    };

    // One RigidBodyInfo object corresponds to one H3D RigidBody node
    struct RigidBodyInfo {
      RigidBodyInfo() : fixed( false ) {}

      // denotes if this body is fixed or not.
      bool fixed;
    };

    // One ShapeInfo object corresponds to one H3D CollidableShape node
    struct ShapeInfo {
      static int created_shape_id;
      ShapeInfo() : shape(NULL), actor(NULL) {
        id = created_shape_id;
        created_shape_id += 1;
      }

      H3DCollidableId id;
      NxShape * shape;
      NxActor * actor;
    };
    typedef map< H3DCollidableId, ShapeInfo * > ShapeInfoMap;
    static ShapeInfoMap shapeInfoMap;

    // One ConstraintInfo object corresponds to one H3DConstraintNode
    // but the NxJoint object in this struct may change during
    // simulation (PhysX requires deletion and recreation with
    // changes in actors).
    // Since it is likely that we may need to use the 6DOF PhysX 
    // joints to implement more than one joint in X3D, there could 
    // be a need for more member variables later.
    struct ConstraintInfo {      
      static int created_joint_id;
      ConstraintInfo () : joint( NULL ) {
        id = created_joint_id;
        created_joint_id += 1;
      }

      H3DConstraintId id;
      NxJoint * joint;
    };

    typedef map< H3DConstraintId, ConstraintInfo * > ConstraintInfoMap;
    static ConstraintInfoMap constraintInfoMap;

    // Keeps information of shapes in each body
    // Used for easy reference in setRigidBodyParameters()
    typedef map< H3DBodyId, vector< ShapeInfo *> > BodyShapeMap;
    static BodyShapeMap bodyShapeMap;

   ///////// Physics engine registration callbacks //////////////

    // Global engine callback functions
    static H3DUtil::PeriodicThread::CallbackCode initEngine(void *data);
    static H3DUtil::PeriodicThread::CallbackCode deInitEngine(void *data);
    static H3DUtil::PeriodicThread::CallbackCode doSimulationSteps(void *data);
    static H3DUtil::PeriodicThread::CallbackCode setWorldParameters(void *data);
    static H3DUtil::PeriodicThread::CallbackCode synchroniseWithSceneGraph(void *data);
    static H3DUtil::PeriodicThread::CallbackCode getCurrentContacts(void *data);

    // Shape callback functions
    // implemented for Box and Sphere
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

    // internal callback functions
    static void setBallJointParameters(void *data);
    static void getBallJointParameters(void *data);
    static void setSingleAxisHingeJointParameters(void *data);
    static void getSingleAxisHingeJointParameters(void *data);
    static void setSliderJointParameters(void *data);
    static void getSliderJointParameters(void *data);
    static void setDoubleAxisHingeJointParameters(void *data);
    static void getDoubleAxisHingeJointParameters(void *data);
    static void setUniversalJointParameters(void *data);
    static void getUniversalJointParameters(void *data);
    static void setMotorJointParameters(void *data);
    static void getMotorJointParameters(void *data);

    // attach joint functions
    static ConstraintInfo* attachBallJoint( ConstraintParameters *params );
    static ConstraintInfo* attachSingleAxisHingeJoint( ConstraintParameters *params );
    static ConstraintInfo* attachSliderJoint( ConstraintParameters *params );
    /// \note attachDoubleAxisHingeJoint not yet implemented
    static ConstraintInfo* attachDoubleAxisHingeJoint( ConstraintParameters *params );
    static ConstraintInfo* attachUniversalJoint( ConstraintParameters *params );
    static ConstraintInfo* attachMotorJoint( ConstraintParameters *params );

    /// registration of physics engine callback functions.
    static SoftBodyPhysicsEngineThread::SoftBodyPhysicsEngineRegistration registration;

    // Internal setRigidBodyParameters function.
    static void setRigidBodyParameters( PhysicsEngineParameters::RigidBodyParameters *params,
                                        PhysXSpecificData *physx_data );

    // Internal functions that should be used instead of
    // physx_data->scene->createActor/releaseActor commands.
    static NxActor * createActor( PhysXSpecificData *physx_data,
                                  NxActorDesc &actor_desc,
                                  NxActor *replace_actor = NULL );

    static void releaseActor( PhysXSpecificData *physx_data,
                              NxActor *actor,
                              bool release_user_data = true );

    // Internal setConstaintParameters function.
    static void setConstraintParameters( PhysicsEngineParameters::JointParameters * params );
  };

  /// Handles the link between H3DSoftBodyNode representation and PhysX
  class PhysXH3DSoftBody { 
  public:
    /// Constructor
    PhysXH3DSoftBody ( PhysXCallbacks::PhysXSpecificData& physx_data );

    /// Destructor
    virtual ~PhysXH3DSoftBody () {};

    /// Get current properties of the simulated soft body
    virtual void getParameters ( PhysicsEngineParameters::H3DSoftBodyNodeParameters& params ) {};

    /// Set current properties of the simulated soft body
    virtual void setParameters ( PhysicsEngineParameters::H3DSoftBodyNodeParameters& params ) {};

    /// Apply external forces to the simulated soft body
    virtual void applyExternalForces ( PhysicsEngineParameters::H3DSoftBodyNodeParameters& bodyParameters ) {};
  protected:

    /// Ptr to PhysX scene
    /// Used to release soft body on destruction
    NxScene* scene;

    /// This describes the buffers that are used to recieve the modified mesh information from the simulation engine
    NxMeshData receiveBuffers;

    // Structure for the rendering buffer
    struct RenderBufferVertexElement
    {
      RenderBufferVertexElement() {
        texCoord[0] = .0f;
        texCoord[1] = .0f;
      }
      NxVec3 position;
      NxVec3 normal;
      float texCoord[2];
    };

    // The rendering buffers
    RenderBufferVertexElement* mVertexRenderBuffer;
    NxU32* mIndexRenderBuffer;

    NxU32 mMaxVertices;
    NxU32 mMaxIndices;
    NxU32 mNumIndices;
    NxU32 mNumParentIndices;
    NxU32 mNumVertices;
    NxU32 mLastNumVertices;

    NxU32 mMeshDirtyFlags;

    /// Indices of vertices currently fixed
    PhysicsEngineParameters::H3DSoftBodyNodeParameters::IndexList fixedVertices;
    bool waitForNonZeromNumVertices;
  };

  /// Handles the link between H3D SoftBody representation and PhysX
  class PhysXSoftBody : public PhysXH3DSoftBody  {
  public:

    /// Constructor
    PhysXSoftBody ( PhysicsEngineParameters::SoftBodyParameters& params,
                    PhysXCallbacks::PhysXSpecificData& physx_data );

    /// Destructor
    virtual ~PhysXSoftBody ();

    /// Get current properties of the simulation cloth
    virtual void getParameters ( PhysicsEngineParameters::H3DSoftBodyNodeParameters& params );

    /// Set current properties of the simulation cloth
    virtual void setParameters ( PhysicsEngineParameters::H3DSoftBodyNodeParameters& params );

    /// Apply external forces to the cloth
    virtual void applyExternalForces ( PhysicsEngineParameters::H3DSoftBodyNodeParameters& bodyParameters );

  protected:

    /// Generates a physX cloth mesh description from the specified geometry node
    /// The contents of vertices and indices should be deleted when the NxSoftBodyMeshDesc is
    /// no longer useful.
    static NxSoftBodyMeshDesc generateMeshDesc ( PhysicsEngineParameters::SoftBodyParameters& params,
                                                 NxVec3*& vertices_ptr, NxU32 *&indices_ptr );

    /// Cook a mesh from the mesh description
    bool cookMesh ( NxSoftBodyMeshDesc& desc );

    /// Allocate memory to recieve mesh data from the simulation engine
    void allocateReceiveBuffers( size_t numVertices, size_t numTriangles);

    void applyDampingParameters ( DampingParameters& dampingParams );

    void applyFrictionParameters ( FrictionParameters& frictionParams );

    void applyStiffnessParameters ( StiffnessParameters& stiffnessParams );

    /// PhysX soft body representation
    NxSoftBody* softBody;

    NxSoftBodyMesh* softBodyMesh;

    PhysicsEngineParameters::MaterialPropertyParameters::UnitType stiffnessUnitType;
  };

  /// Handles the link between H3D Cloth representation and PhysX
  class PhysXCloth : public PhysXH3DSoftBody {
  public:

    /// Constructor
    PhysXCloth ( PhysicsEngineParameters::ClothParameters& params,
                 PhysXCallbacks::PhysXSpecificData& physx_data );

    /// Destructor
    virtual ~PhysXCloth ();

    /// Get current properties of the simulation cloth
    virtual void getParameters ( PhysicsEngineParameters::H3DSoftBodyNodeParameters& params );

    /// Set current properties of the simulation cloth
    virtual void setParameters ( PhysicsEngineParameters::H3DSoftBodyNodeParameters& params );

    /// Apply external forces to the cloth
    virtual void applyExternalForces ( PhysicsEngineParameters::H3DSoftBodyNodeParameters& bodyParameters );

    /// Returns the wrapped NxCloth
    NxCloth* getCloth () { return cloth; }

  protected:

    /// Generates a physX cloth mesh description from the specified geometry node
    static NxClothMeshDesc generateMeshDesc ( PhysicsEngineParameters::ClothParameters& params,
                                              NxVec3*& vertices_ptr, NxU32 *&indices_ptr );

    /// Cook a mesh from the mesh description
    bool cookMesh ( NxClothMeshDesc& desc );

    /// Allocate memory to recieve mesh data from the simulation engine
    void allocateReceiveBuffers( size_t numVertices, size_t numTriangles);

    /// Fix the specified vertices unfixing others
    void fixVertices ( PhysicsEngineParameters::H3DSoftBodyNodeParameters& params );

    void applyDampingParameters ( DampingParameters& dampingParams );

    void applyFrictionParameters ( FrictionParameters& frictionParams );

    void applyStiffnessParameters ( StiffnessParameters& stiffnessParams );

    /// The PhysX soft body
    NxCloth* cloth;

    /// The PhysX cloth mesh
    NxClothMesh* clothMesh;

    PhysicsEngineParameters::MaterialPropertyParameters::UnitType stiffnessUnitType;
  };
}

#endif // HAVE_PHYSX
#endif // __PHYSXCALLBACKS__
