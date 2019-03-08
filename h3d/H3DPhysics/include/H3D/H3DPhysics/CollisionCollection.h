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
/// \file CollisionCollection.h
/// \brief Header file for CollisionCollection, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __COLLISIONCOLLECTION__
#define __COLLISIONCOLLECTION__

#include <H3D/X3DChildNode.h>
#include <H3D/SFBool.h>
#include <H3D/MFString.h>
#include <H3D/SFFloat.h>
#include <H3D/MFNode.h>
#include <H3D/SFNode.h>
#include <H3D/SFVec2f.h>
#include <H3D/PeriodicUpdate.h>
#include <H3D/H3DPhysics/H3DPhysics.h>
#include <H3D/H3DPhysics/PhysicsEngineThread.h>
#include <H3D/H3DPhysics/CollidableGroup.h>
#include <H3D/H3DPhysics/CollidableExceptionGroup.h>
#include <H3D/H3DPhysics/CollidableSelectionGroup.h>

namespace H3D{

  class RigidBodyCollection;

  /// \ingroup X3DNodes
  /// \class CollisionCollection
  /// \brief The CollisionCollection node holds a collection of objects
  /// in the collidables field that can be managed as a single entity for
  /// resolution of inter-object collisions with other groups of 
  /// collidable objects. A group consists of both collidable objects as
  /// well as spaces that may be collided against each other. A set of 
  /// parameters are provided that specify default values that will be 
  /// assigned to all Contact nodes generated from the CollisionSensor 
  /// node. A user may then override the individual Contact node by 
  /// inserting a script between the output of the sensor and the input 
  /// to the RigidBodyCollection node if it is desired to process the 
  /// contact stream.
  ///
  /// The enabled field is used to control whether the collision detection
  /// system for this collection should be run at the end of this frame. 
  /// A value of TRUE enables it while a value of FALSE disables it. A 
  /// CollisionSensor node watching this collection does not report any 
  /// outputs for this collection for this frame if it is not enabled.
  ///
  /// The bounce field indicates how bouncy the surface contact is. A 
  /// value of 0 indicates no bounce at all while a value of 1 indicates 
  /// maximum bounce.
  ///
  /// The minBounceSpeed field indicates the minimum speed, in metres per
  /// second, that an object shall have before an object will bounce. If 
  /// the object is below this speed, it will not bounce, effectively 
  /// having an equivalent value for the bounce field of zero.
  ///
  /// The surfaceSpeed field defines the speed in the two friction 
  /// directions in metres per second. This is used to indicate if the 
  /// contact surface is moving independently of the motion of the bodies.
  /// e.g. a conveyor belt.
  ///
  /// The softnessConstantForceMix value applies a constant force value 
  /// to make the colliding surfaces appear to be somewhat soft.
  ///
  /// The softnessErrorCorrection determines how much of the collision 
  /// error should be fixed in a set of evaluations. The value is limited
  /// to the range of [0,1]. A value of 0 specifies no error correction 
  /// while a value of 1 specifies that all errors should be corrected in 
  /// a single step.
  ///
  /// The appliedParameters indicates globally which parameters are to be
  /// applied to the collision outputs when passing information into the 
  /// the rigid body physics system. These parameters specify a series of 
  /// defaults that apply to all contacts generated. Individual contacts 
  /// may override which values are applicable, if needed, by setting the
  /// field of the same name in the contact itself.
  ///
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../examples/RigidBody/BallJoint.x3d">BallJoint.x3d</a>
  ///     ( <a href="examples/BallJoint.x3d.html">Source</a> )
  ///
  /// \par Internal routes:
  /// \dotfile CollisionCollection.dot
  class H3DPHYS_API CollisionCollection :
    public X3DChildNode{
  public:

    /// Node type of values in collidables field. The collidables field value
    /// should only be of type X3DNBodyCollisionSpaceNode or 
    /// X3DNBodyCollidableNode
    class H3DPHYS_API MFCollidable: public MFNode {
    public:
      /// Destructor. To make sure the correct onRemove function is called upon
      /// destruction.
      ~MFCollidable() {
        clear();
      }

    protected:
      /// On addition of a node to collidables field, add the node to the 
      /// nodes_changed list
      virtual void onAdd( Node *n );

      /// On removal of a node from collidables field, add the node to the 
      /// nodes_changed list
      virtual void onRemove( Node *n );
    };

    class H3DPHYS_API MFCollidableGroup : public TypedMFNode< CollidableGroup > {
    public:
      /// Destructor.
      ~MFCollidableGroup() {
        clear();
      }

    protected:
      /// On removal of a node from collidable field, delete the collidable
      virtual void onRemove( Node *n ) {
        CollidableGroup *cg = static_cast<CollidableGroup *>(n);
        cg->deleteCollidableGroup();
        TypedMFNode< CollidableGroup >::onRemove( n );
      }
    };

    class H3DPHYS_API MFCollidableExceptionGroup : public TypedMFNode< CollidableExceptionGroup > {
    public:
      /// Destructor.
      ~MFCollidableExceptionGroup() {
        clear();
      }

    protected:
      /// On removal of a node from collidable field, delete the collidable
      virtual void onRemove( Node *n ) {
        CollidableExceptionGroup *cg = static_cast<CollidableExceptionGroup *>(n);
        cg->deleteCollidableExceptionGroup();
        TypedMFNode< CollidableExceptionGroup >::onRemove( n );
      }
    };

    class H3DPHYS_API MFCollidableSelectionGroup : public TypedMFNode< CollidableSelectionGroup > {
    public:
      /// Destructor.
      ~MFCollidableSelectionGroup() {
        clear();
      }

    protected:
      /// On removal of a node from collidable field, delete the collidable
      virtual void onRemove( Node *n ) {
        CollidableSelectionGroup *cg = static_cast<CollidableSelectionGroup *>(n);
        cg->deleteCollidableSelectionGroup();        
        TypedMFNode< CollidableSelectionGroup >::onRemove( n );
      }
    };

    /// The ValueUpdater field is used to update values in the
    /// PhysicsEngineThread according to changes of fields in the
    /// CollisionCollection node. More specifically it calls 
    /// PhysicsEngineThread::setGlobalContactParameters with the new values.
    /// Every field that has a corresponding value in 
    /// PhysicsEngineThreadParameters::GlobalContactParameters is routed 
    /// to this field.
    class H3DPHYS_API ValueUpdater: 
      public EventCollectingField< PeriodicUpdate< Field > > {
        virtual void update();
    };


    /// Constructor.
    CollisionCollection(
      Inst< MFString  > _appliedParameters  = 0,
      Inst< SFFloat   > _bounce  = 0,
      Inst< MFCollidable > _collidables  = 0,
      Inst< MFCollidableGroup > _collidableGroups  = 0,
      Inst< MFCollidableExceptionGroup > _collidableExceptionGroups  = 0,
      Inst< SFBool    > _enabled = 0,
      Inst< SFVec2f   > _frictionCoefficients  = 0,
      Inst< SFNode    > _metadata = 0,
      Inst< SFFloat   > _minBounceSpeed  = 0,
      Inst< SFVec2f   > _slipFactors  = 0,
      Inst< SFFloat   > _softnessConstantForceMix  = 0,
      Inst< SFFloat   > _softnessErrorCorrection  = 0,
      Inst< SFVec2f   > _surfaceSpeed = 0,
      Inst< ValueUpdater > _valueUpdater = 0,
      Inst< MFCollidableSelectionGroup > _collidableSelectionGroups  = 0,
      Inst< SFString > _contactReportMode = 0 );

    /// Destructor. Deletes the collidables field value.
    ~CollisionCollection();

    /// traverseSG function of this node.
    virtual void traverseSG(H3D::TraverseInfo &ti);

    /// Initialize the collision collection
    void initializeCollisionCollection ();

    /// Deletes the collidables in this CollisionCollection
    void deleteCollidables();

    /// Deletes the collidableGroups in this CollisionCollection
    void deleteCollidableGroups();

    /// Deletes the collidableExceptionGroups in this CollisionCollection
    void deleteCollidableExceptionGroups();

    /// Deletes the collidableSelectionGroups in this CollisionCollection
    void deleteCollidableSelectionGroups();

    /// Deletes specified collidable node
    void deleteCollidable( Node * n );

    /// Initializes specified collidable node
    void initializeCollidable( Node * n );

    /// Indicates globally parameters to be applied to collision outputs 
    /// when passing information into the the rigid body physics system.
    /// The following are valid values:
    ///
    /// <ul><li>"BOUNCE": The bounce field value is used.</li>
    /// <li>"USER_FRICTION": The system will normally calculate the 
    /// friction direction vector that is perpendicular to the contact 
    /// normal. This setting indicates that the user-supplied value in
    /// this contact should be used.</li>
    /// <li>"FRICTION_COEFFICIENT-2": The frictionCoefficients field
    /// values are used.</li>
    /// <li>"ERROR_REDUCTION": The softnessErrorCorrection field value in 
    /// the contact evaluation should be used.</li>
    /// <li>"CONSTANT_FORCE": The softnessConstantForceMix field value in
    /// the contact evaluation should be used.</li>
    /// <li>"SPEED-1": The surfaceSpeed field value first component is
    /// used.</li>
    /// <li>"SPEED-2": The surfaceSpeed field value second component is
    /// used.</li>
    /// <li>"SLIP-1": The slipFactors field value first component is
    /// used.</li>
    /// <li>"SLIP-2": The slipFactors field value second component is
    /// used.</li></ul>
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value:</b> "BOUNCE" \n
    /// 
    /// \dotfile CollisionCollection_appliedParameters.dot
    auto_ptr< MFString > appliedParameters;

    /// Indicates bounciness of surface contact is. Zero indicates no 
    /// bounce at all while a value of 1 indicates maximum bounce.
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value:</b> 0 \n
    /// 
    /// \dotfile CollisionCollection_bounce.dot
    auto_ptr< SFFloat > bounce;

    /// The collection of objects used in inter-object collisions.
    /// Of value type X3DNbodyCollisionSpace or X3DNBodyCollidableNode
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value:</b> NULL \n
    /// 
    /// \dotfile CollisionCollection_collidables.dot
    auto_ptr< MFCollidable > collidables;

    /// Each collidable will be checked for collision with collidables
    /// in the same CollidableGroup. More than one CollidableGroups
    /// can be added in the CollisionCollection. In case no
    /// CollidableGroup exists collision is possible between all
    /// collidables. Collidables should be added directly to the
    /// collidables field for them in order to be initialized.
    /// collidableGroups should only be used for grouping purposes
    /// and has nothing to do with creating and deleting collidables.
    /// Same collidable can be added to more than one collidableGroup.
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value:</b> NULL \n
    /// 
    /// \dotfile CollisionCollection_collidableGroups.dot
    auto_ptr< MFCollidableGroup > collidableGroups;

    /// Collisions between the collidables in the same
    /// CollidableExceptionGroup will be ignored. More than one
    /// CollidableExceptionGroups can be added in the CollisionCollection.
    /// In case no CollidableExceptionGroup exists collision is possible
    /// between all collidables. Collidables should be added directly to
    /// the collidables field for them in order to be initialized.
    /// collidableExceptionGroups should only be used for grouping purposes
    /// and has nothing to do with creating and deleting collidables.
    /// Same collidable can be added to more than one collidableExceptionGroup.
    /// 
    /// <b>Access type: </b> inputOutput
    /// 
    /// \dotfile CollisionCollection_collidableGroups.dot
    auto_ptr< MFCollidableExceptionGroup > collidableExceptionGroups;

    /// A collidable object added to the collidables field will only collide with other collidables
    /// in the collidables field and in the selectedCollidables field. Collision relations of the
    /// objects in the selectedCollidables field with ohter objects in the scene will not be affected
    /// by the CollidableSelectionGroup.
    ///
    /// A collidable in any other CollidableSelectionGroup can not be added to the collidables field,
    /// neither a collidable in collidables field can be added to another CollidableSelectionGroup.
    /// A collidable can be added in selectedCollidables field of more than one CollidableSelectionGroup
    /// 
    /// <b>Access type: </b> inputOutput
    /// 
    /// \dotfile CollisionCollection_collidableGroups.dot
    auto_ptr< MFCollidableSelectionGroup > collidableSelectionGroups;

    /// Indicates whether the collision detection system for this 
    /// collection should be run at the end of this frame. A 
    /// CollisionSensor node watching this collection does not 
    /// report any outputs for this collection for this frame if it 
    /// is not enabled.
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value:</b> TRUE \n
    /// 
    /// \dotfile CollisionCollection_enabled.dot
    auto_ptr< SFBool > enabled;

    /// Not documented in the X3D specification, your guess is as good as mine.
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value:</b> Vec2f( 0, 0 ) \n
    /// 
    /// \dotfile CollisionCollection_frictionCoefficients.dot
    auto_ptr< SFVec2f > frictionCoefficients;

    /// The minimum speed, in metres per second, that an object shall
    /// have before an object will bounce. If the object is below this 
    /// speed, it will not bounce, effectively having an equivalent 
    /// value for the bounce field of zero.
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value:</b> 0.1 \n
    /// 
    /// \dotfile CollisionCollection_minBounceSpeed.dot
    auto_ptr< SFFloat > minBounceSpeed;

    /// Not documented in the X3D specification, your guess is as good as mine.
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value:</b> Vec2f( 0, 0 ) \n
    /// 
    /// \dotfile CollisionCollection_slipFactors.dot
    auto_ptr< SFVec2f > slipFactors;

    /// The constant force value to control apparent softness on 
    /// colliding surfaces.
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value:</b> 0.0001 \n
    /// 
    /// \dotfile CollisionCollection_softnessConstantForceMix.dot
    auto_ptr< SFFloat > softnessConstantForceMix;

    /// Indicates extent of collision error correction. Value limited 
    /// to the range [0,1]. Zero specifies no error correction while 1
    /// specifies that all errors should be corrected in a single step.
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value:</b> 0.8 \n
    /// 
    /// \dotfile CollisionCollection_softnessErrorCorrection.dot
    auto_ptr< SFFloat > softnessErrorCorrection;

    /// The speed in the two friction directions in metres per second.
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value:</b> 0 \n
    /// 
    /// \dotfile CollisionCollection_surfaceSpeed.dot
    auto_ptr< SFVec2f > surfaceSpeed;

    // Not in the X3D standard.
    /// Determines how to report( the filtering ) the contacts from the physics engine.
    /// 
    /// <b>Access type: </b> initializeOnly
    /// <b>Default value:</b> "DEFAULT" \n
    /// <b>Valid values:</b> "DEFAULT" "ALL" \n
    /// 
    /// \dotfile CollisionCollection_contactReportMode.dot
    auto_ptr< SFString > contactReportMode;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

    H3D::PhysicsEngineThread *getLastLoopPhysicsEngine() {
      return engine_thread;
    }

    void setRigidBodyCollection( RigidBodyCollection* _rbc = NULL ); 

  protected:

    PhysicsEngineParameters::GlobalContactParameters getGlobalContactParameters ();

    /// The valueUpdater field is used to update values in the
    /// PhysicsEngineThread according to changes of fields in the
    /// CollisionCollection node. More specifically it calls 
    /// PhysicsEngineThread::setGlobalContactParameters with the new values.
    /// Every field that has a corresponding value in 
    /// PhysicsEngineThreadParameters::GlobalContactParameters is routed 
    /// to this field.
    /// C++ only field.
    ///
    /// \dotfile CollisionCollection_valueUpdater.dot
    auto_ptr< ValueUpdater > valueUpdater; 

    /// The PhysicsEngineThread with which this CollisionCollection is created
    H3D::PhysicsEngineThread *engine_thread;

    bool collisionCollectionInitialized;

    RigidBodyCollection* rbc;
  };
}
#endif
