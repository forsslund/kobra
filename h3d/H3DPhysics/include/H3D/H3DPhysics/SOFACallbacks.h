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
/// \file SOFACallbacks.h
/// \brief Header file for SOFACallbacks, struct with callbacks using SOFA
/// as the physics engine.
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __SOFACALLBACKS__
#define __SOFACALLBACKS__

#include <H3D/H3DPhysics/H3DPhysics.h>

#ifdef HAVE_SOFA

#include <H3DUtil/Threads.h>
#include <H3D/H3DPhysics/PhysicsEngineThread.h>

#include <sofa/defaulttype/SolidTypes.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/component/typedef/MechanicalState_double.h>
#include <sofa/component/typedef/Mass_double.h>
//#include <sofa/component/typedef/MechanicalState_float.h>
//#include <sofa/component/typedef/Mass_float.h>
#include <sofa/simulation/tree/TreeSimulation.h>
#include <sofa/component/odesolver/EulerSolver.h>
#include <sofa/component/init.h>
#include <sofa/simulation/common/Node.h>
#include <sofa/simulation/common/xml/initXml.h>

namespace H3D {
  class SOFACollidable;

  /// \ingroup PhysicsEngineCallbackStructs
  /// \brief The SOFACallbacks struct contains callback functions and
  /// variables needed to interface with SOFA.
  struct SOFACallbacks {

    struct SOFASpecificData: public H3D::PhysicsEngineThread::EngineSpecificData {
    SOFASpecificData():
      scene_root( NULL ) {}
      
      ~SOFASpecificData() {
      }

      sofa::simulation::Node::SPtr scene_root;
      sofa::simulation::Simulation::SPtr simulationPtr;

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
    static H3DUtil::PeriodicThread::CallbackCode addJoint(void *data);
    static H3DUtil::PeriodicThread::CallbackCode removeJoint(void *data);
    static H3DUtil::PeriodicThread::CallbackCode setJointParameters(void *data);
    static H3DUtil::PeriodicThread::CallbackCode getJointParameters(void *data);

  static H3DUtil::PeriodicThread::CallbackCode addConstraint(void *data){return PeriodicThread::CALLBACK_DONE;};
  static H3DUtil::PeriodicThread::CallbackCode removeConstraint(void *data){return PeriodicThread::CALLBACK_DONE;};
  static H3DUtil::PeriodicThread::CallbackCode setConstraintParameters(void *data){return PeriodicThread::CALLBACK_DONE;};
  static H3DUtil::PeriodicThread::CallbackCode getConstraintParameters(void *data){return PeriodicThread::CALLBACK_DONE;};

    // external force callback functions
    static H3DUtil::PeriodicThread::CallbackCode addGlobalExternalForceAndTorque(void *data);    

    ////////////////////////////////////////////////////////////

  protected:

    /*
    // internal callback functions
    static void nearCallback(void *data, dGeomID o1, dGeomID o2);

    static void addShape( ShapeParameters *params );
    static void addOffset( OffsetParameters * params );
 
    static H3DUtil::PeriodicThread::CallbackCode attachJoint(void *data);
    static H3DUtil::PeriodicThread::CallbackCode setSingleAxisHingeJointParameters(void *data);
    static H3DUtil::PeriodicThread::CallbackCode getSingleAxisHingeJointParameters(void *data);

    static H3DUtil::PeriodicThread::CallbackCode setBallJointParameters(void *data);
    static H3DUtil::PeriodicThread::CallbackCode getBallJointParameters(void *data);

    static H3DUtil::PeriodicThread::CallbackCode setDoubleAxisHingeJointParameters(void *data);
    static H3DUtil::PeriodicThread::CallbackCode getDoubleAxisHingeJointParameters(void *data);

    static H3DUtil::PeriodicThread::CallbackCode setMotorJointParameters(void *data);
    static H3DUtil::PeriodicThread::CallbackCode getMotorJointParameters(void *data);

    static H3DUtil::PeriodicThread::CallbackCode setSliderJointParameters(void *data);
    static H3DUtil::PeriodicThread::CallbackCode getSliderJointParameters(void *data);

    static H3DUtil::PeriodicThread::CallbackCode setUniversalJointParameters(void *data);
    static H3DUtil::PeriodicThread::CallbackCode getUniversalJointParameters(void *data);
    */
     // registration of physics engine callback functions.
    static H3D::PhysicsEngineThread::PhysicsEngineRegistration registration;
  };


  class SOFARigidBody {
  public:
    
    /// Constructor
    SOFARigidBody ( PhysicsEngineParameters::RigidBodyParameters& bodyParameters );
    
    /// Apply an external force and torque to the body
    void addGlobalExternalForceAndTorque ( PhysicsEngineParameters::ExternalForceTorqueParameters& forces );

    /// Update SOFA representation of rigid body from specified parameters
    void setParameters ( PhysicsEngineParameters::RigidBodyParameters& bodyParameters );

    /// Get the current parameters of the rigid body
    void getParameters ( PhysicsEngineParameters::RigidBodyParameters& bodyParameters );

    /// Add SOFA collidables to rigid body based on current collidables list
    void updateCollidables ();///t

    /// Re-adds the rigid body to the world following changes to collidables
    void collisionShapeChanged ();///t

    sofa::simulation::Node::SPtr getNode () {
      return rigidBody;
    }

  protected:
    typedef vector <SOFACollidable*> SOFACollidableList;

    sofa::simulation::Node::SPtr rigidBody;
    MechanicalObjectRigid3::SPtr rigidDOF;
    UniformMassRigid3::SPtr rigidMass;

    /// List of collidables or geometries used by this rigid body
    SOFACollidableList collidables;

    /// The physics thread to which the rigid body belongs
    PhysicsEngineThread* physicsThread;

    Node* massDensityModel;
  };

 class SOFACollidable {
  public:
    /// Constructor
   SOFACollidable ( PhysicsEngineParameters::CollidableParameters& collidableParameters );
   //SOFACollidable ( PhysicsEngineParameters::ShapeParameters& collidableParameters );

   /// Update bullet representation of collidable using specified parameters
   void setParameters ( PhysicsEngineParameters::CollidableParameters& collidableParameters );

    /// Add a rigid body that uses this collidable
    virtual void addBody ( SOFARigidBody& body );

    /// Remove a rigid body that no longer uses this collidable
    virtual void removeBody ( SOFARigidBody& body );

    /// For all rigid bodies that use this collidable, re-add the collidable hierarchy
    virtual void updateCollidables ();

    /// Add the collidable hierarchy to the specified compound shape
    /// The hierarchy is flattened so that the index may be used to identify the
    /// shape during collision detection
    //void addCollidableTo ( btCompoundShape& compoundShape );

    /// Recursively add the collidable hierarchy to the specified compound shape
    /// Called by addCollidableTo() to recursively evaluate transform and flatten
    //virtual void addCollidableToRecursive ( btCompoundShape& compoundShape,
    //                                        btTransform& parentTransform,
    //                                        bool parentEnabled );

  protected:
    typedef vector < SOFARigidBody* > RigidBodyList;

    /// Updates the local transform to match the parameters specified
    //void updateTransform ( PhysicsEngineParameters::CollidableParameters& collidableParameters,
    //                       btTransform& transform );

    /// List of rigid bodies using this collidable
    /// Required to 'enable'/'disable' the collidable
    RigidBodyList bodies;

    /// Local transformation offset for this collidable
    sofa::defaulttype::Mat4x4f localTransform;

    /// An empty shape used in place of collidableShape when disabled
    /// This preserves the index used in collision detection
    //auto_ptr < btEmptyShape > emptyShape;

   /// SOFA representation of collidable
   sofa::core::CollisionModel::SPtr collidableShape;
   sofa::simulation::Node::SPtr collidableNode;

   
   /// True if this collidable (and children) should be used in collision detection
   bool enabled;
 };

  /// Handles the link between H3D CollidableOffset and bullet representation
  class SOFACollidableOffset : public SOFACollidable
  {
  public:
    /// Constructor
    SOFACollidableOffset ( PhysicsEngineParameters::OffsetParameters& offsetParameters );

    /// Recursively add the collidable hierarchy to the specified compound shape
    /// Called by addCollidableTo() to recursively evaluate transform and flatten
    //virtual void addCollidableToRecursive ( btCompoundShape& compoundShape,
    //                                        btTransform& parentTransform, 
    //                                        bool parentEnabled );

    /// Add a rigid body that uses this collidable
    /// Add recursively
    virtual void addBody ( SOFARigidBody& body );

    /// Remove a rigid body that no longer uses this collidable
    /// Remove recursively
    virtual void removeBody ( SOFARigidBody& body );

  protected:
    SOFACollidable* childCollidable;
  };

  /// Handles the link between H3D CollidableShape and bullet representation
  class SOFACollidableShape : public SOFACollidable
  {
  public:
    /// Constructor
    SOFACollidableShape ( PhysicsEngineParameters::ShapeParameters& shapeParameters );

  protected:

    /// Return an appropriate bullet btCollisionShape to match the specified X3DGeometryNode
    sofa::core::CollisionModel *createCollisionShape( X3DGeometryNode *geometry);
  };
}

#endif // HAVE_SOFA
#endif // __SOFACALLBACKS__
