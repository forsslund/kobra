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
/// \file BulletCallbacks.h
/// \brief Header file for BulletCallbacks, struct with callbacks using bullet
/// as the physics engine.
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __BULLETCALLBACKS__
#define __BULLETCALLBACKS__

#include <H3D/H3DPhysics/SoftBodyPhysicsEngineThread.h>
#include <H3D/H3DPhysics/BulletSoftBodyOptions.h>
#include <H3DUtil/Threads.h>

#ifdef HAVE_BULLET

#if _MSC_VER >= 1500
#pragma warning( disable : 4305 )
#endif

#include <btBulletDynamicsCommon.h>
#include <BulletSoftBody/btSoftBody.h>
#include <BulletSoftBody/btSoftRigidDynamicsWorld.h>
#include <BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h>
#include <BulletSoftBody/btDefaultSoftBodySolver.h>

#if _MSC_VER >= 1500
#pragma warning( default : 4305 )
#endif

// Define to use std::vector for fast edge building
// otherwise standard c arrays are used
//
// Can't see any time performance difference here
// so use STL vector for safety.
//
#define FAST_EDGE_BUILDING_USE_STL_VECTOR

namespace H3D {

  /// Convert H3D vector to Bullet vector
  static btVector3 tobtVector3 ( const Vec3f& v ) { return btVector3 ( v.x, v.y, v.z ); }

  /// Convert Bullet vector to H3D vector
  static Vec3f toVec3f ( const btVector3& v ) { return Vec3f ( v.x(), v.y(), v.z() ); }

  /// Construct a transform to a coordinate system with the specified axis as the local x-axis
  btTransform getAxisTransformX ( const btVector3& xAxis );

  /// Construct a transform to a coordinate system with the specified axis as the local z-axis
  btTransform getAxisTransformZ ( const btVector3& zAxis );

  /// Create an H3D matrix from a bullet transform
  Matrix4f toMatrix4f ( const btTransform& t );

  /// Create a 3x3 bullet matrix from a 3x3 H3D matrix
  static btMatrix3x3 tobtMatrix3x3 ( const Matrix3f& m );

  class H3DSoftBody;
  class BulletAttachment;

  /// \ingroup PhysicsEngineCallbackStructs Bullet
  /// \brief The BulletCallbacks struct contains callback functions and
  /// variables needed to interface with the bullets physics library.
  ///
  /// The bullets physics library is an open source physics engine which
  /// can be found <a href="http://www.bulletphysics.com/wordpress/">here</a>.
  struct BulletCallbacks {

#ifdef DOXYGEN_IFDEF_BECAUSE_COND_STATEMENT_WONT_WORK_IN_THIS_CASE
    /// A struct containing information and settings specific to bullet.
    struct BulletSpecificData: public PhysicsEngineThread::EngineSpecificData {
#else
    ATTRIBUTE_ALIGNED16( struct ) BulletSpecificData: public PhysicsEngineThread::EngineSpecificData {
#endif

      BT_DECLARE_ALIGNED_ALLOCATOR();

      auto_ptr< btBroadphaseInterface > m_broadphase;
      auto_ptr< btCollisionDispatcher > m_dispatcher;
      auto_ptr< btConstraintSolver > m_solver;
      auto_ptr< btSoftBodyRigidBodyCollisionConfiguration > m_collisionConfiguration;
      auto_ptr< btSoftBodySolver > m_softBodySolver;
      auto_ptr< btSoftRigidDynamicsWorld > m_dynamicsWorld;
      btSoftBodyWorldInfo m_softBodyWorldInfo;

      typedef vector < H3DSoftBody* > H3DSoftBodyVector;
      typedef vector < BulletAttachment* > BulletAttachmentVector;

      H3DSoftBodyVector m_softBodies;

      BulletAttachmentVector m_attachments;


      btScalar m_fixedTimeStep;
      btScalar m_worldScale;
      btVector3 m_gravity;
      btScalar m_collisionMargin;
      btScalar m_maxVelocityLinear;
      btScalar m_maxVelocityAngular;

      BulletSpecificData () : 
        m_fixedTimeStep ( 1 / 1000.0f ), 
        m_worldScale ( 1.0f ),
        m_gravity ( btVector3 ( 0, 0, 0 ) ),
        m_collisionMargin ( (btScalar)0.01 ),
        m_maxVelocityLinear( (btScalar)-1 ),
        m_maxVelocityAngular( (btScalar)-1 ) {}

      ~BulletSpecificData();
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

    // Collision space callback functions
    static H3DUtil::PeriodicThread::CallbackCode addSpace(void *data);
    static H3DUtil::PeriodicThread::CallbackCode removeSpace(void *data);
    static H3DUtil::PeriodicThread::CallbackCode setSpaceParameters(void *data);
    static H3DUtil::PeriodicThread::CallbackCode getSpaceParameters(void *data);


    ////////////////////////////////////////////////////////////

    // soft body callback functions
    static H3DUtil::PeriodicThread::CallbackCode addSoftBody ( void *data );
    static H3DUtil::PeriodicThread::CallbackCode removeSoftBody ( void *data );
    static H3DUtil::PeriodicThread::CallbackCode setSoftBodyParameters ( void *data );
    static H3DUtil::PeriodicThread::CallbackCode getSoftBodyParameters ( void *data );
    static H3DUtil::PeriodicThread::CallbackCode applyExternalForces ( void *data );

  protected:

    /// registration of physics engine callback functions.
    static SoftBodyPhysicsEngineThread::SoftBodyPhysicsEngineRegistration registration;
  };

  /// Define our own soft body solver in order to filter collisions
  class btSoftBodySolverH3D : public btDefaultSoftBodySolver {
    virtual void processCollision( btSoftBody *, const btCollisionObjectWrapper* );
  };

  /// Define our own soft body in order to filter collisions
  class btSoftBodyH3D : public btSoftBody {
  public:
    btSoftBodyH3D( 
      btSoftBodyWorldInfo* worldInfo, int node_count, const btVector3* x, const btScalar* m ) 
      : btSoftBody ( worldInfo, node_count, x, m ),
      collisionGroup ( 0 ),
      collidesWith ( 0 )
    {}

    void filteredCollisionHandler( const btCollisionObjectWrapper* pcoWrap );

    /// Save cluster contacts here since they are removed immediately after solving
    std::vector < PhysicsEngineParameters::ContactParameters > cluster_contacts;

    int collisionGroup;
    int collidesWith;
  };

  class BulletCollidable;

  class H3DPHYS_API BulletBody {
  public:
    /// Base for user specific physics engine information attached to this body
    struct BulletBodyUserData {
      virtual ~BulletBodyUserData() {}
    };

    virtual ~BulletBody() {}

    /// Set user specific data associated with the physics engine implementation of this body
    ///
    /// The BulletBody assumes exclusive ownership and will delete the userdata on destruction or replacement
    ///
    inline void setUserData ( BulletBodyUserData& _userdata ) {
      userdata.reset ( &_userdata );
    }

    /// Get user specific data associated with the physics engine implementation of this body
    inline BulletBodyUserData* getUserData () {
      return userdata.get();
    }

    /// User specific data associated with the physics engine implementation of this body
    auto_ptr < BulletBodyUserData > userdata;
  };

  /// Handles link between H3D and bullet representations of a rigid body
  /// This makes management and clean-up of additional data easier and avoids the need for a map
  class H3DPHYS_API BulletRigidBody : public BulletBody
  {
  public:
    /// Constructor
    BulletRigidBody ( PhysicsEngineParameters::RigidBodyParameters& bodyParameters );

    /// Apply an external force and torque to the body
    void addGlobalExternalForceAndTorque ( PhysicsEngineParameters::ExternalForceTorqueParameters& forces );

    /// Update bullet representation of rigid body from specified parameters
    void setParameters ( PhysicsEngineParameters::RigidBodyParameters& bodyParameters );

    /// Get the current parameters of the rigid body
    void getParameters ( PhysicsEngineParameters::RigidBodyParameters& bodyParameters );

    /// Returns the bullet implementation of rigid body
    btRigidBody& getRigidBody () { return *rigidBody; }

    /// Returns the PhysicsEngineThread which the rigid body belongs to
    PhysicsEngineThread& getPhysicsThread () { return *physicsThread; }

    /// Add bullet collidables to rigid body based on current collidables list
    void updateCollidables ();

    /// Re-adds the rigid body to the world following changes to collidables
    void collisionShapeChanged ();

    /// Returns the collidable of the specified index
    BulletCollidable* getCollidable ( size_t index );

    /// Soft body collision flags
    int collisionSoft;

    int collisionGroup;
    int collidesWith;

  protected:
    typedef vector <BulletCollidable*> BulletCollidableList;

    /// \brief Check that the compound shape meets the requirements and output
    /// suitable warnings to the console if it does not
    void validateCollidables ( btCompoundShape& compoundShape );

    /// Bullet representation of rigid body
    auto_ptr < btRigidBody > rigidBody;

    /// \brief The bullet implementation of the collision geometry associated with this shape.
    ///
    /// A btCollisionShape which may be used either to represent no
    /// collision at all (btEmptyShape - as NULL causes crash), or a container for multiple geometries (btCompoundShape).
    /// This is used to ensure that the additional collision shape is deleted with the body
    auto_ptr < btCollisionShape > collisionShape;

    /// List of collidables or geometries used by this rigid body
    BulletCollidableList collidables;

    /// The physics thread to which the rigid body belongs
    PhysicsEngineThread* physicsThread;

    /// A node used as the current mass density model
    Node* massDensityModel;
  };

  /// Handles the link between H3D and bullet representations of collidable objects
  class H3DPHYS_API BulletCollidable
  {
  public:
    /// Constructor
    BulletCollidable ( PhysicsEngineParameters::CollidableParameters& collidableParameters );

    /// Destructor
    virtual ~BulletCollidable() {}

    /// Update bullet representation of collidable using specified parameters
    void setParameters ( PhysicsEngineParameters::CollidableParameters& collidableParameters );

    /// Add a rigid body that uses this collidable
    virtual void addBody ( BulletRigidBody& body );

    /// Remove a rigid body that no longer uses this collidable
    virtual void removeBody ( BulletRigidBody& body );

    /// For all rigid bodies that use this collidable, re-add the collidable hierachy
    virtual void updateCollidables ();

    /// Add the collidable hierachy to the specified compound shape
    /// The hierachy is flattened so that the index may be used to identify the
    /// shape during collision detection
    void addCollidableTo ( btCompoundShape& compoundShape );

    /// Recursively add the collidable hierachy to the specified compound shape
    /// Called by addCollidableTo() to recursively evaluate transform and flatten
    virtual void addCollidableToRecursive ( btCompoundShape& compoundShape,
      const btTransform& parentTransform,
      bool parentEnabled );

  protected:
    typedef vector < BulletRigidBody* > RigidBodyList;

    /// Updates the local transform to match the parameters specified
    void updateTransform ( PhysicsEngineParameters::CollidableParameters& collidableParameters,
      btTransform& transform );

    /// List of rigid bodies using this collidable
    /// Required to 'enable'/'disable' the collidable
    RigidBodyList bodies;

    /// Local transformation offset for this collidable
    btTransform localTransform;

    /// Bullet representation of collidable
    auto_ptr < btCollisionShape > collidableShape;

    /// An empty shape used in place of collidableShape when disabled
    /// This preserves the index used in collision detection
    auto_ptr < btEmptyShape > emptyShape;

    /// True if this collidable (and children) should be used in collision detection
    bool enabled;
  };

#ifdef DOXYGEN_IFDEF_BECAUSE_COND_STATEMENT_WONT_WORK_IN_THIS_CASE
  /// Handles the link between H3D CollidableOffset and bullet representation
  class H3DPHYS_API BulletCollidableOffset : public BulletCollidable
#else
  /// Handles the link between H3D CollidableOffset and bullet representation
  ATTRIBUTE_ALIGNED16( class ) H3DPHYS_API BulletCollidableOffset : public BulletCollidable
#endif
  {
  public:
    BT_DECLARE_ALIGNED_ALLOCATOR();
    /// Constructor
    BulletCollidableOffset ( PhysicsEngineParameters::OffsetParameters& offsetParameters );

    /// Recursively add the collidable hierachy to the specified compound shape
    /// Called by addCollidableTo() to recursively evaluate transform and flatten
    virtual void addCollidableToRecursive ( btCompoundShape& compoundShape,
      const btTransform& parentTransform, 
      bool parentEnabled );

    /// Add a rigid body that uses this collidable
    /// Add recursively
    virtual void addBody ( BulletRigidBody& body );

    /// Remove a rigid body that no longer uses this collidable
    /// Remove recursively
    virtual void removeBody ( BulletRigidBody& body );

  protected:
    BulletCollidable* childCollidable;
  };

#ifdef DOXYGEN_IFDEF_BECAUSE_COND_STATEMENT_WONT_WORK_IN_THIS_CASE
  /// Handles the link between H3D CollidableShape and bullet representation
 class H3DPHYS_API BulletCollidableShape : public BulletCollidable
#else
  /// Handles the link between H3D CollidableShape and bullet representation
  ATTRIBUTE_ALIGNED16( class ) H3DPHYS_API BulletCollidableShape : public BulletCollidable
#endif
  {
  public:
    BT_DECLARE_ALIGNED_ALLOCATOR();
    /// Constructor
    BulletCollidableShape ( PhysicsEngineParameters::ShapeParameters& shapeParameters );

  protected:

    /// Return an appropriate bullet btCollisionShape to match the specified X3DGeometryNode
    btCollisionShape* createCollisionShape ( PhysicsEngineParameters::ShapeParameters& shapeParameters,
      BulletCallbacks::BulletSpecificData& bullet_data );

    /// Mesh associated with the collision shape (if appropriate)
    auto_ptr < btStridingMeshInterface > mesh;
  };

  class BulletFixedConstraint;

  /// Handles the link between H3D SoftBody and bullet representation
  class H3DPHYS_API H3DSoftBody : public BulletBody
  {
  public:

    /// Destructor
    virtual ~H3DSoftBody();

    /// Base for specific tag data that can be used for m_tag field of
    /// a btSoftbody::Element.
    struct H3DPHYS_API BulletSoftBodyTagData {
      virtual ~BulletSoftBodyTagData() {}
    };

    /// Used to mark a link as attachment link.
    /// The information in it can be used to reference link
    /// and nodes in other softBody.
    struct H3DPHYS_API LinkAttachmentTagData : public BulletSoftBodyTagData {
      LinkAttachmentTagData( btSoftBody *_softBody0,
        int _link_index,
                             btSoftBody *_softBody1,
                             BulletAttachment * _bullet_attachment,
                             int _attachment_link_index )
                             : softBody0( _softBody0 ),
        link_index( _link_index ),
          softBody1( _softBody1 ),
          bullet_attachment( _bullet_attachment ),
          attachment_link_index ( _attachment_link_index ) {
            softBody0->m_links[link_index].m_tag = this;
      }

      /// softBody0 is the soft body in which the link exists.
      btSoftBody *softBody0;

      /// softBody1 is the other soft body.
      btSoftBody *softBody1;

      /// The attachment link index into softBody0.
      int link_index;

      // A pointer to the attachment that created this tag data.
      BulletAttachment * bullet_attachment;

      /// The link index from the point of view of this attachment
      /// Allows specific links within an attachment to be removed by index
      int attachment_link_index;
    };

    /// Update bullet representation of soft body from specified parameters
    virtual void setParameters ( PhysicsEngineParameters::H3DSoftBodyNodeParameters& bodyParameters,
      bool updateGeometry= true );

    /// Get the current parameters of the soft body
    virtual void getParameters ( PhysicsEngineParameters::H3DSoftBodyNodeParameters& bodyParameters );

    /// Apply external forces to the soft body
    virtual void applyExternalForces ( PhysicsEngineParameters::H3DSoftBodyNodeParameters& bodyParameters );

    /// Returns the bullet representation of the soft body
    btSoftBodyH3D& getSoftBody () { return *softBody; }

    /// Update the H3D geometry from the bullet representation
    ///
    /// If the application modifies the bullet representation of the mesh using native
    /// bullet functions, calling this function allows the changes to be reflected in the H3D
    /// geometry as well.
    ///
    /// The function assumes that the soft body mesh keeps the same type of elements (i.e. if the
    /// mesh started as an IndexedTriangleSet, the modified bullet mesh must contain only triangles).
    ///
    virtual void geometryFromBullet()= 0;

    /// Set the fixed vertices of the soft body.
    inline void setFixedVertices( const H3DSoftBodyNodeParameters::IndexList& indices ) {
      fixedVertices = indices;
    }

    /// Clear the fixed vertices of the soft body.
    inline void clearFixedVertices() {
      fixedVertices.clear();
    }

    /// Returns the fixed vertices of the soft body.
    inline H3DSoftBodyNodeParameters::IndexList& getFixedVertices() {
      return fixedVertices;
    }

    void addFixedConstraint ( BulletFixedConstraint& fixedConstraint );

    void removeFixedConstraint ( BulletFixedConstraint& fixedConstraint );

    void applyFixedConstraints ();

    typedef vector<BulletFixedConstraint*> FixedConstraintList;

    /// Helper function which returns the per link Material for the specified link
    /// 
    /// If it doesn't exist then it is created
    btSoftBody::Material* getLinkMaterial ( size_t linkIndex );

    /// Remove a link and the associated per-edge material if created
    ///
    /// All links should be removed this way to preserve the ordering of
    /// the materials
    void removeLinkAndMaterial ( size_t linkIndex );

  protected:
    /// Constructor
    H3DSoftBody () : mass ( 0.0 ),
      hasEdgeStiffness ( false ),
      coordPerElement ( -1 ),
      curEdgeLinkIndex ( -1 ),
      collisionOptions ( BulletSoftBodyParameters::SDF_RIGIDSOFT | 
      BulletSoftBodyParameters::CLUSTER_SOFTSOFT ),
#ifndef FAST_EDGE_BUILDING_USE_STL_VECTOR
                         edgeAdded ( NULL ),
      nrNodes ( 0 ),
#endif
      enablePerEdgeStiffness ( false ),
      enableFastEdgeBuilding ( true ),
      stiffnessUnitType ( PhysicsEngineParameters::MaterialPropertyParameters::UNIT_UNIFORM ),
      stiffnessAngularUnitType( PhysicsEngineParameters::MaterialPropertyParameters::UNIT_UNIFORM ),
      stiffnessVolumeUnitType( PhysicsEngineParameters::MaterialPropertyParameters::UNIT_UNIFORM ),
      nrClusters ( 64 ) {}

    void applyDampingParameters ( DampingParameters& dampingParams );

    void applyFrictionParameters ( FrictionParameters& frictionParams );

    void applyMassParameters ( MassParameters& massParams );

    void applyStiffnessParameters ( StiffnessParameters& stiffnessParams );
    void applyStiffnessAngularParameters( StiffnessParameters& stiffnessParams );
    void applyStiffnessVolumeParameters( StiffnessParameters& stiffnessParams );

    /// Set the mass of the soft body using appropriate Bullet function (e.g. volume or total)
    virtual void setMass ( H3DFloat _mass )= 0;

    /// Update the btSoftBody collision options based on specified parameters
    void setCollisionOptions ( PhysicsEngineParameters::H3DSoftBodyNodeParameters& bodyParameters );

    /// Update the enablePerEdgeStiffness and enableFastEdgeBuilding options based on the specified soft body parameters
    void setEdgeOptions ( PhysicsEngineParameters::H3DSoftBodyNodeParameters& bodyParameters );

    /// Update individual link stiffnesses, creating a material for each link
    void setStiffnessPerEdge ( StiffnessParameters& stiffnessParams );

    /// Remove per edge stiffnesses and revert all edges to use the default material
    void clearPerEdgeStiffness ();

    /// Remove all edges from the soft body
    ///
    /// Does not affect links that represent attachments 
    void clearEdgeLinks ();

    /// Call before adding any edges to the soft body
    ///
    /// Allocates memory for fast edge building and per edge stiffness feature
    /// if these options are enabled.
    ///
    void initAddEdgeLinks ( size_t _nrNodes );

    /// Add an edge between node0 index and node1 index
    void addEdgeLink ( int node0, int node1 );

    /// Call after adding the last edge to the soft body. After calling
    /// do not call addEdgeLink() again, without first calling initAddEdgeLinks()
    ///
    /// Deallocates memory for fast edge building.
    ///
    void deinitAddEdgeLinks ();

    /// Update the stiffness values for all edges.
    ///
    /// This should be called after modifying any of the materials
    void updateEdgeStiffness ();

    /// Helper template function for getting attributes from the soft body
    template < typename PropertyFunctor, typename AttributeParametersType >
    void setOutputAttribute ( AttributeParametersType& output, PropertyFunctor propertyAccessor ) {
      typename AttributeParametersType::AttributeVectorType values;
      const typename AttributeParametersType::IndexList& index= output.getIndex();
      if ( index.empty() ) {
        values.resize ( propertyAccessor.size(*softBody) );
        for ( int i= 0; i < propertyAccessor.size(*softBody); ++i ) {
          values[i]= propertyAccessor(*softBody, i);
        }
      } else {
        values.resize ( index.size() );
        for ( int i= 0; i < static_cast<int>(index.size()); ++i ) {
          int idx= index[i];
          if ( /*idx >= 0 && */idx < propertyAccessor.size(*softBody) ) {
            values[i]= propertyAccessor(*softBody, idx);
          }
        }
      }
      output.setValues ( values );
    }

    /// Bullet representation of soft body
    auto_ptr < btSoftBodyH3D > softBody;

    /// The mass of the soft body
    btScalar mass;

    /// Indices of vertices currently fixed
    PhysicsEngineParameters::H3DSoftBodyNodeParameters::IndexList fixedVertices;

    /// FixedConstraints that fix vertices of this soft body
    FixedConstraintList fixedConstraints;

    /// Cache of current indices into geometry coordinates
    /// May represent indexed triangles or tetra etc. depending on subtype
    PhysicsEngineParameters::H3DSoftBodyNodeParameters::IndexList indices;

    unsigned int collisionOptions;

    /// Have stiffnesses per edge been specified?
    /// If so these should override linearStiffness
    bool hasEdgeStiffness;

    vector<int> edgeToLink;

    int coordPerElement;

    size_t curEdgeLinkIndex;

    /// A 2D array of bools indicating whether an edge exists between the two indexed nodes
    ///
    /// Used when the enableFastEdgeBuilding options is true.
    ///
#ifdef FAST_EDGE_BUILDING_USE_STL_VECTOR
    vector < vector < int > > edgeAdded;
#else
    bool** edgeAdded;

    /// The number of nodes in the soft body. Required to deallocate fast edge building data
    ///
    /// Used when the enableFastEdgeBuilding options is true.
    ///
    size_t nrNodes;
#endif

    /// If enabled, it is possible to set the stiffness of the soft body per edge using
    /// the edgeStiffness of H3DSoftBodyNode.
    bool enablePerEdgeStiffness;

    /// If enabled, optimisations to the time complexity of adding edges to the
    /// soft body are enabled. 
    bool enableFastEdgeBuilding;

    PhysicsEngineParameters::MaterialPropertyParameters::UnitType stiffnessUnitType;
    PhysicsEngineParameters::MaterialPropertyParameters::UnitType stiffnessAngularUnitType;
    PhysicsEngineParameters::MaterialPropertyParameters::UnitType stiffnessVolumeUnitType;

    // Number of collision clusters
    int nrClusters;
  };

  class H3DPHYS_API BulletSoftBody : public H3DSoftBody
  {
  public:
    /// Constructor
    BulletSoftBody ( PhysicsEngineParameters::SoftBodyParameters& bodyParameters,
      BulletCallbacks::BulletSpecificData& bulletData );

    /// Update bullet representation of soft body from specified parameters
    virtual void setParameters ( PhysicsEngineParameters::H3DSoftBodyNodeParameters& bodyParameters,
      bool updateGeometry= true );

    /// Update the H3D geometry from the bullet representation
    ///
    /// If the application modifies the bullet representation of the mesh using native
    /// bullet functions, calling this function allows the changes to be reflected in the H3D
    /// geometry as well.
    ///
    /// The function assumes that the soft body mesh keeps the same type of elements. In this
    /// case, the bullet mesh must contain only tetra and the tetra must be listed in the m_tetras member.
    ///
    /// \note Not implemented yet. Implement as/when required
    ///
    virtual void geometryFromBullet() {};

  protected:
    /// Set the mass of the soft body using appropriate Bullet function (e.g. volume or total)
    virtual void setMass ( H3DFloat _mass );

    void appendTetra( H3DSoftBodyNodeParameters& bodyParameters );
    void appendOneTetra( const H3DSoftBodyNodeParameters::IndexList::value_type &i,
      const H3DSoftBodyNodeParameters::IndexList::value_type &j,
                         const H3DSoftBodyNodeParameters::IndexList::value_type &k,
                         const H3DSoftBodyNodeParameters::IndexList::value_type &l );
  };

  class H3DPHYS_API BulletCloth : public H3DSoftBody
  {
  public:
    /// Constructor
    BulletCloth ( PhysicsEngineParameters::ClothParameters& bodyParameters,
      BulletCallbacks::BulletSpecificData& bulletData );

    /// Update bullet representation of soft body from specified parameters
    virtual void setParameters ( PhysicsEngineParameters::H3DSoftBodyNodeParameters& bodyParameters,
      bool updateGeometry= true );

    /// Update the H3D geometry from the bullet representation
    ///
    /// If the application modifies the bullet representation of the mesh using native
    /// bullet functions, calling this function allows the changes to be reflected in the H3D
    /// geometry as well.
    ///
    /// The function assumes that the soft body mesh keeps the same type of elements. In this
    /// case, the bullet mesh must contain only triangles and they must be listed in the m_faces member.
    ///
    virtual void geometryFromBullet();

  protected:

    /// Set the mass of the soft body using appropriate Bullet function (e.g. volume or total)
    virtual void setMass ( H3DFloat _mass );
  };

  class H3DPHYS_API BulletRope : public H3DSoftBody
  {
  public:
    /// Constructor
    BulletRope ( PhysicsEngineParameters::RopeParameters& bodyParameters,
      BulletCallbacks::BulletSpecificData& bulletData );

    /// Update bullet representation of soft body from specified parameters
    virtual void setParameters ( PhysicsEngineParameters::H3DSoftBodyNodeParameters& bodyParameters,
      bool updateGeometry= true );

    /// Update the H3D geometry from the bullet representation
    ///
    /// If the application modifies the bullet representation of the mesh using native
    /// bullet functions, calling this function allows the changes to be reflected in the H3D
    /// geometry as well.
    ///
    /// The function assumes that the soft body mesh keeps the same type of elements. In this
    /// case, the bullet mesh must contain no cyclic links, just a 1-d line of links.
    ///
    /// \note Not implemented yet. Implement as/when required
    ///
    virtual void geometryFromBullet() {};

  protected:

    /// Set the mass of the soft body using appropriate Bullet function (e.g. volume or total)
    virtual void setMass ( H3DFloat _mass );
  };

  class H3DPHYS_API BulletAttachment {
  public:
    /// Update bullet representation of the attachment from specified parameters
    virtual void setParameters ( PhysicsEngineParameters::H3DAttachmentParameters& attachmentParameters ) {};

    /// Get the current parameters of the attachment
    virtual void getParameters ( PhysicsEngineParameters::H3DAttachmentParameters& attachmentParameters ) {};

    virtual void update () {}

    /// Remove the attachment from the simulation
    virtual void remove ()= 0;

    /// Destructor
    virtual ~BulletAttachment() {};

  protected:
    /// Constructor
    BulletAttachment () : collide ( true ) {};

    /// If true bodies will collide with each other
    bool collide;
  };

  class H3DPHYS_API BulletRigidBodyAttachment : public BulletAttachment {
  public:
    /// Constructor
    BulletRigidBodyAttachment ( PhysicsEngineParameters::H3DRigidBodyAttachmentParameters& attachmentParameters );

    /// Destructor
    virtual ~BulletRigidBodyAttachment ();

    /// Update bullet representation of the attachment from specified parameters
    virtual void setParameters ( PhysicsEngineParameters::H3DAttachmentParameters& attachmentParameters );

    /// Remove the attachment from the simulation
    virtual void remove ();

  protected:
    /// Remove all the bullet anchors added by this attachment
    void removeAnchors();

    /// The soft body involved in the attachment
    btSoftBody* softBody;

    /// The rigid body involved in the attachment
    btRigidBody* rigidBody;

    /// Indices of vertices to be attached
    PhysicsEngineParameters::H3DAttachmentParameters::IndexList indices;

    /// List of anchors added by this attachment, to enable removal
    btSoftBody::tAnchorArray addedAnchors;
  };

  class H3DPHYS_API BulletSoftBodyAttachment : public BulletAttachment {
  public:
    /// Constructor
    BulletSoftBodyAttachment ( PhysicsEngineParameters::SoftBodyAttachmentParameters& attachmentParameters );

    /// Update bullet representation of the attachment from specified parameters
    virtual void setParameters ( PhysicsEngineParameters::H3DAttachmentParameters& attachmentParameters );

    /// Destructor
    virtual ~BulletSoftBodyAttachment ();

    virtual void update ();

    /// Remove the attachment from the simulation
    virtual void remove ();

  protected:

    /// Adds a new link to the attachment with the given index. Does not
    /// check if the index already exists, for that see insertLink()
    ///
    /// The link will be between this->softBody (node index1) and this->softBody2 (node index2)
    ///
    void addLink ( size_t index, size_t index1, size_t index2 );

    /// Inserts a link into the attachment at index
    ///
    /// The link will be between this->softBody (node index1) and this->softBody2 (node index2)
    ///
    void insertLink ( size_t index, size_t index1, size_t index2 );

    /// Remove the specified link from the attachment
    ///
    /// \return True if link was removed successfully
    ///
    bool removeLink ( size_t index );

    /// Changes a specific link in the attachment at specified index
    ///
    /// The link will be between this->softBody (node index1) and this->softBody2 (node index2)
    ///
    /// \return true iff the link was found and updated
    ///
    bool updateLink ( size_t index, size_t index1, size_t index2 );

    /// Apply the stiffness values specified by the StiffnessParameters
    void applyStiffnessParameters ( StiffnessParameters& stiffnessParams );

    /// Set a uniform stiffness for all links in the attachment
    void setUniformStiffness ( H3DFloat stiffness );

    /// Set a different stiffness for each link in the attachment
    void setStiffnessPerLink ( StiffnessParameters& stiffnessParams );

    /// Set the stiffness of the link, indexed by the local attachment index
    bool setLinkStiffness ( size_t index, H3DFloat stiffness );

    /// The soft body involved in the attachment
    btSoftBody* softBody;

    /// The other soft body involved in the attachment
    btSoftBody* softBody2;

    /// The H3DSoftBody for soft body 1
    H3DSoftBody* h3dSoftBody;

    /// Indices of vertices to be attached
    PhysicsEngineParameters::H3DAttachmentParameters::IndexList indices;

    /// Indices of softBody2 vertices to be attached
    PhysicsEngineParameters::H3DAttachmentParameters::IndexList indices2;

    /// The units that the stiffness values refer to e.g., uniform or per edge
    PhysicsEngineParameters::MaterialPropertyParameters::UnitType stiffnessUnitType;
  };

  class BulletFixedConstraint {
  public:
    /// Constructor
    BulletFixedConstraint ( PhysicsEngineParameters::FixedConstraintParameters& fixedConstraintParameters );

    /// Update bullet representation of the fixed constraint from specified parameters
    virtual void setParameters ( PhysicsEngineParameters::FixedConstraintParameters& fixedConstraintParameters );

    const PhysicsEngineParameters::FixedConstraintParameters::IndexList& getIndices ();

    BulletSoftBody* getSoftBody ();

  protected:
    PhysicsEngineParameters::FixedConstraintParameters::IndexList indices;

    BulletSoftBody* softBody;
  };
}

#endif // HAVE_BULLET

#endif
