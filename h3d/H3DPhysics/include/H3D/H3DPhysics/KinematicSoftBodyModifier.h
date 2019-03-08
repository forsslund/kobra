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
/// \file KinematicSoftBodyModifier.h
/// \brief Header file for KinematicSoftBodyModifier, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __KINEMATICSOFTBODYMODIFIER__
#define __KINEMATICSOFTBODYMODIFIER__

#include <H3D/H3DPhysics/H3DDeviceSoftBodyModifierNode.h>
#include <H3D/H3DPhysics/RigidBody.h>
#include <HAPI/HAPIForceEffect.h>

namespace H3D{  

  /// Force effect used to render the reaction force from haptic interaction
  class H3DPHYS_API ReactionForce: public HAPI::HAPIForceEffect {
  public:

    /// Specify the center of mass of the proxy object
    void setCenterOfMass ( Vec3f p );

    /// Add a spring for rendering by the force effect
    void addSpring ( Vec3f p1, Vec3f p2, H3DFloat k );

    /// Add a torque spring for rendering by the force effect
    void addTorqueSpring ( Rotation r1, Rotation r2, H3DFloat k );

    /// Transfer springs to haptic thread
    void transferSprings ( PeriodicThreadBase& thread );

    /// The force of the EffectOutput will be the force of the force field. 
    EffectOutput virtual calculateForces( const EffectInput &input );

  protected:

    /// Callback to transfer springs to haptic thread
    static PeriodicThread::CallbackCode transferSpringsCB ( void* data );

    struct Spring {
      Spring ( Vec3f _p1, Vec3f _p2, H3DFloat _k ) :
    p1 ( _p1 ), p2 ( _p2 ), k ( _k ) {}
    Vec3f getForce () { return (p1-p2)*k; }
    Vec3f p1, p2;
    H3DFloat k;
    };

    struct TorqueSpring {
      TorqueSpring ( Rotation _r1, Rotation _r2, H3DFloat _k ) :
    r1 ( _r1 ), r2 ( _r2 ), k ( _k ) {}

    Vec3f getTorque () { 
      Quaternion cur ( r2 );
      Quaternion target ( r1 );
      Quaternion springTorque= (cur*target.inverse()).inverse();
      return springTorque.toEulerAngles()*k;
    }

    Rotation r1, r2;
    H3DFloat k;
    };

    typedef vector<Spring> SpringVector;
    SpringVector springsCurrent;
    SpringVector springs;

    typedef vector<TorqueSpring> TorqueSpringVector;
    TorqueSpringVector torqueSpringsCurrent;
    TorqueSpringVector torqueSprings;

    Vec3f centerOfMass, centerOfMassCurrent;
  };

  /// \ingroup SoftBody
  /// A node used to define how a haptic device will affect an H3DSoftBodyNode.
  /// The KinematicSoftBodyModifier node contains a RigidBody node which whose
  /// position will be linked to the haptic device. The geometry field of the 
  /// RigidBody performs the deformation of the soft body.
  ///
  /// The RigidBody node and contained CollidableShape (s) must be added to
  /// a RigidBodyCollection by the user in the usual way to ensure that they
  /// are correctly initialized.
  ///
  /// \par Limitations: 
  /// The underlying physics engine implementation must support kinematic rigid bodies,
  /// i.e., the ability to directly control a rigid body's position.
  ///
  /// \par
  /// Unlike the FunctionSoftBodyModifier, it is not possible to limit a KinematicSoftBodyModifier
  /// to a specific H3DSoftBodyNode. A KinematicSoftBodyModifier added to one H3DSoftBodyNode will
  /// affect all other H3DSoftBodyNode instances in the same PhysicsBodyCollection???.
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../examples/softbody/KinematicSoftBodyModifier_Cloth.x3d">KinematicSoftBodyModifier_Cloth.x3d</a>
  ///     ( <a href="examples/KinematicSoftBodyModifier_Cloth.x3d.html">Source</a> )
  ///   - <a href="../../examples/softbody/KinematicSoftBodyModifier_SoftBody.x3d">KinematicSoftBodyModifier_SoftBody.x3d</a>
  ///     ( <a href="examples/KinematicSoftBodyModifier_SoftBody.x3d.html">Source</a> )
  ///
  /// \par Internal routes:
  /// \dotfile KinematicSoftBodyModifier.dot
  class H3DPHYS_API KinematicSoftBodyModifier : public H3DDeviceSoftBodyModifierNode {
  public:

    class H3DPHYS_API UpdateForceFeedbackType : public AutoUpdate < SFString > {
    public:
      enum FeedbackType {  Feedback_None, 
        Feedback_SpringDamper,
        Feedback_SpringDamperProjected,
        Feedback_ContactDepth };

      UpdateForceFeedbackType () : feedbackType ( Feedback_None ) {}

      /// Get the type of feedback thread-safely
      FeedbackType getFeedbackType ();

    protected:
      virtual void update ();

      FeedbackType feedbackType;
      MutexLock mutex;
    };

    /// Constructor.
    KinematicSoftBodyModifier (
      Inst< SFNode > _metadata = 0,
      Inst< ValueUpdater > _valueUpdater = 0,
      Inst< SFH3DBodyNode > _body1 = 0,
      Inst< MFEngineOptions > _engineOptions = 0,
      Inst< SFGeometryNode > _hapticGeometry = 0,
      Inst< SFInt32 > _deviceIndex = 0,
      Inst< SFRigidBody > _rigidBody = 0,
      Inst< SFMatrix4f > _offset = 0,
      Inst< SFBool > _useProxyPosition = 0,
      Inst< SFString > _forceFeedbackType = 0,
      Inst< UpdateForceFeedbackType > _updateFeedbackType = 0 );

    /// Initialize the node
    virtual void initialize ();

    /// Traverse the scene graph
    virtual void traverseSG ( TraverseInfo& ti );

    /// Calculate forces for deformation
    virtual void calculateForces ( PhysicsEngineParameters::H3DSoftBodyNodeParameters& softBodyParams,
      HAPI::HAPIHapticsDevice& hd );

    /// The Rigid body that will deform the soft bodies
    ///
    /// Note: The RigidBody node and contained CollidableShape(s) must be added to
    /// a RigidBodyCollection by the user in the usual way to ensure that they
    /// are correctly initialized.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> NULL \n
    /// 
    /// \dotfile KinematicSoftBodyModifier_rigidBody.dot
    auto_ptr < SFRigidBody > rigidBody;

    /// A transformation that represents an offset between the haptic device and the rigid body
    /// used to deform the soft bodies.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> Matrix4f() \n
    /// 
    /// \dotfile KinematicSoftBodyModifier_offset.dot
    auto_ptr < SFMatrix4f > offset;

    /// If true, then the proxy position of the haptic device will be used to calculate
    /// the position of the rigid body deformer, otherwise the device position will be used.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> true \n
    /// 
    /// \dotfile KinematicSoftBodyModifier_useProxyPosition.dot
    auto_ptr < SFBool > useProxyPosition;

    /// EXPERIMENTAL: The method to use to determine the force and torque feedback
    /// This field may NOT part of the final interface!
    ///
    /// The force feedback is applied in addition to any existing force feedback from a haptics renderer
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> "NONE" \n
    /// <b>Valid values:</b> "NONE", "SPRING_DAMPER", "CONTACT_DEPTH" \n
    /// 
    /// \dotfile KinematicSoftBodyModifier_forceFeedbackType.dot
    auto_ptr < SFString > forceFeedbackType;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:

    /// Field used to update the force feedback type for the physics thread.
    /// C++ only field.
    auto_ptr < UpdateForceFeedbackType > updateFeedbackType;

    /// The id of the kinematic rigid body
    H3DBodyId bodyId;

    /// Force effect used to provide force feedback
    AutoRef < ReactionForce > reactionForce;

    bool kinematicControl;
    Vec3f lastNormal;

    Matrix4f bodyOffset;

    bool useProxyPos;

  };
}

#endif
