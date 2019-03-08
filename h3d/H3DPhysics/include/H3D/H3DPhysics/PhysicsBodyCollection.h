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
/// \file PhysicsBodyCollection.h
/// \brief Header file for PhysicsBodyCollection, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __PHYSICSBODYCOLLECTION__
#define __PHYSICSBODYCOLLECTION__

#include <H3D/H3DPhysics/RigidBodyCollection.h>
#include <H3D/H3DPhysics/RigidBody.h>
#include <H3D/H3DPhysics/H3DSoftBodyNode.h>
#include <H3D/H3DPhysics/H3DBodyInteractorNode.h>
#include <H3D/H3DPhysics/H3DBodyModifierNode.h>
#include <H3DUtil/Threads.h>

namespace H3D{
  /// 
  /// \class PhysicsBodyCollection
  /// \brief The PhysicsBodyCollection node represents a system of bodies
  /// which could be both rigid or soft interacting within a single physics
  /// model. The collection is not
  /// a renderable part of the scene graph nor are its children as a typical
  /// model may need to represent the geometry for physics separately, and 
  /// in less detail, than those needed for visuals.
  ///
  /// The rigidBodies field contains a collection of the top-level nodes that
  /// comprise a set of rigid bodies that should be evaluated as a single set
  /// of interactions. 
  ///
  /// The softBodies field contains a collection of the top-level nodes that
  /// comprise a set of soft bodies that should be evaluated as a single set
  /// of interactions. 
  ///
  /// The interactors field is used to register all type of interactors acting
  /// on or between the bodies contained in this collection. If an interactor
  /// is connected between bodies in two different collections, the result is 
  /// implementation-dependent. If an interactor instance is registered with more
  /// than one collection, the results are implementation dependent. Interactor
  /// not registered with any collection are not evaluated.
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../examples/softbody/Cloth.x3d">Cloth.x3d</a>
  ///     ( <a href="examples/Cloth.x3d.html">Source</a> )
  ///
  /// \par Internal routes:
  /// \dotfile PhysicsBodyCollection.dot
  class H3DPHYS_API PhysicsBodyCollection : public RigidBodyCollection{
  public:

    /// MFRigidBody is specialized to remove the rigid body from the
    /// PhysicsEngineThread of the PhysicsBodyCollection when it is
    /// removed from the field. Initialization of rigid bodies
    /// is handled in the traverseSG function of RigidBody though.
    /// The onAdd function cannot be used to do this since the
    /// PhysicsEngineThread might not exist when it is called.
    //class H3DPHYS_API MFRigidBody: public TypedMFNode< RigidBody > {
    //public:
    //  /// Destructor. Needed for the correct version of onRemove to be 
    //  /// called.
    //  virtual ~MFRigidBody() {
    //    clear();
    //  }
    //protected:
    //  /// Removes the rigid body from the PhysicsEngineThread of the 
    //  /// PhysicsBodyCollection when removed from the field.
    //  virtual void onRemove( Node *n );
    //};

    /// MFSoftBody is specialized to remove the rigid body from the
    /// PhysicsEngineThread of the PhysicsBodyCollection when it is
    /// removed from the field. Initialization of soft bodies
    /// is handled in the traverseSG function of SoftBody though.
    /// The onAdd function cannot be used to do this since the
    /// PhysicsEngineThread might not exist when it is called.
    class H3DPHYS_API MFSoftBody: public TypedMFNode< H3DSoftBodyNode > {
    public:
      /// Destructor. Needed for the correct version of onRemove to be 
      /// called.
      virtual ~MFSoftBody() {
        clear();
      }
    protected:
      /// Removes the rigid body from the H3DRigidBodyPhysicsEngine of the 
      /// RigidBodyCollection when removed from the field.
      virtual void onRemove( Node *n );
    };

    /// MFBodyModifier is specialized to remove the modifier from the
    /// PhysicsEngineThread of the PhysicsBodyCollection when it is
    /// removed from the field. Initialization of modifier
    /// is handled in the traverseSG function of H3DBodyModifierNode
    /// though. The onAdd function cannot be used to do this since
    /// the PhysicsEngineThread might not exist when it is called.  
    class H3DPHYS_API MFBodyModifier : public TypedMFNode< H3DBodyModifierNode > {
    public:
      /// Destructor. Needed for the correct version of onRemove to be 
      /// called.
      virtual ~MFBodyModifier() {
        clear();
      }
    protected:
      /// Removes the modifier from the PhysicsEngineThread of the 
      /// PhysicsBodyCollection when removed from the field.
      virtual void onRemove( Node *n );
    };

    /// For a PhysicsBodyCollection we undo the specialization of the joints field, it can now
    /// take any kind of H3DBodyConstraintNode, not only H3DJointNode.
    typedef GeneralizedMFNode < H3DBodyConstraintNode, MFJoint, MFBodyConstraint > MFPhysicsBodyConstraint;

    /// Constructor.
    PhysicsBodyCollection(
      Inst< MFNode  > _set_contacts = 0,
      Inst< SFBool  > _autoDisable = 0,
      Inst< SFFloat > _constantForceMix = 0,
      Inst< SFFloat > _contactSurfaceThickness  = 0,
      Inst< SFFloat > _disableAngularSpeed = 0,
      Inst< SFFloat > _disableLinearSpeed = 0,
      Inst< SFFloat > _disableTime = 0,
      Inst< SFBool  > _enabled = 0,
      Inst< SFFloat > _errorCorrection = 0,
      Inst< SFVec3f > _gravity = 0,
      Inst< SFInt32 > _iterations = 0,
      Inst< SFFloat > _maxCorrectionSpeed = 0,
      Inst< SFNode  > _metadata = 0,
      Inst< SFBool  > _preferAccuracy = 0,
      Inst< SFCollisionCollection  > _collider = 0,
      Inst< ValueUpdater > _valueUpdater = 0,
      Inst< EnableDisable > _enableDisable = 0,
      Inst< SFString > _physicsEngine = 0,
      Inst< SFInt32 > _desiredUpdateRate = 0,
      Inst< SFInt32 > _updateRate = 0,
      Inst< SFTime  > _stepUpdateTime = 0,
      Inst< EnableUseMainThread  > _useMainThread = 0,
      Inst< MFEngineOptions > _engineOptions = 0,
      Inst< MFRigidBody > _bodies = 0,
      Inst< MFPhysicsBodyConstraint > _joints = 0,
      Inst< MFSoftBody > _softBodies = 0,
      Inst< MFBodyModifier  > _modifiers = 0);

    /// Destructor.
    virtual ~PhysicsBodyCollection();

    /// Initialize the node. A new PhysicsEngineThread with the underlying
    /// physics engine specified by the physicsEngine is created.
    virtual void initialize();

    /// The rigid bodies that should be evaluated in this PhysicsBodyCollection.
    ///
    /// This field is an alias of the parent RigidBodyCollection's bodies field.
    ///
    /// <b>Access type:</b>inputOutput
    /// 
    /// \dotfile PhysicsBodyCollection_bodies.dot
    MFRigidBody* rigidBodies;

    /// The soft bodies that should be evaluated in this PhysicsBodyCollection.
    ///
    /// <b>Access type:</b>inputOutput
    /// 
    /// \dotfile PhysicsBodyCollection_softBodies.dot
    auto_ptr< MFSoftBody > softBodies;

    /// The constraints that should be evaluated in this PhysicsBodyCollection.
    ///
    /// This field is an alias of the parent RigidBodyCollection's joints field. But in 
    /// a PhysicsBodyCollection the joints/constraints field may contain any type of 
    /// H3DBodyConstraintNode, not only H3DJointNode.
    ///
    /// <b>Access type:</b>inputOutput
    /// 
    /// \dotfile PhysicsBodyCollection_joints.dot
    MFBodyConstraint* constraints;

    /// The modifiers that should be evaluated in this PhysicsBodyCollection.
    ///
    /// <b>Access type:</b>inputOutput
    /// 
    /// \dotfile PhysicsBodyCollection_modifiers.dot
    auto_ptr< MFBodyModifier > modifiers;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:
    /// Traverse the nodes of the physics simulation
    ///
    /// If using deterministic synchronization the simulation outputs are guaranteed 
    /// not to change during the execution of this method (see syncGraphicsFrames)
    virtual void traverseSimulation( TraverseInfo& ti );
  };
}
#endif
