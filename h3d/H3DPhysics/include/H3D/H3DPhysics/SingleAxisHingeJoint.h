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
/// \file SingleAxisHingeJoint.h
/// \brief Header file for SingleAxisHingeJoint, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __SINGLEAXISHINGEJOINT__
#define __SINGLEAXISHINGEJOINT__

#include <H3D/X3DNode.h>
#include <H3D/SFVec3f.h>
#include <H3D/SFFloat.h>
#include <H3D/MFString.h>
#include <H3D/SFNode.h>
#include <H3D/H3DPhysics/H3DPhysics.h>
#include <H3D/H3DPhysics/H3DRigidBodyJointNode.h>

// H3DUtil includes
#include <H3DUtil/AutoPtrVector.h>

namespace H3D{

  /// \ingroup X3DNodes
  /// \class SingleAxisHingeJoint
  /// \brief This node represents a joint with a single axis about which to 
  /// rotate. As the name suggests, this is a joint that works like a 
  /// traditional door hinge. The axis of the hinge is defined to be along
  /// the unit vector described in the axis field and centered on the 
  /// anchorPoint described in world coordinates. The objects on each side
  /// of the hinge are specified by the body1 and body2 fields.
  ///
  /// The minAngle and maxAngle fields are used to control the maximum angles 
  /// through which the hinge is allowed to travel. 
  ///
  /// The stopBounce field describes how much the joint should bounce the body
  /// back if the joint limit has been reached or exceeded. A value of zero
  /// indicates no bounce at all, and a value of one says that it should
  /// bounce with velocity equal and opposite to the collision velocity of 
  /// the contact.
  ///
  /// The stopErrorCorrection field describes the amount of error correction to
  /// be performed in a time step when the joint reaches the limit. A value of
  /// zero means no error correction is to be performed and a value of one 
  /// means all error should be corrected in a single step.
  /// 
  /// The angle output field reports the current relative angle between the two 
  /// bodies in radians and the angleRate field describes the rate at which that
  /// angle is currently changing in radians per second. 
  ///
  /// The body anchor point output fields report the current location of the 
  /// anchor point relative to the corresponding body. This can be used to 
  /// determine if the joint has broken.
  /// 
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../examples/RigidBody/SingleAxisHingeJoint.x3d">SingleAxisHingeJoint.x3d</a>
  ///     ( <a href="examples/SingleAxisHingeJoint.x3d.html">Source</a> )
  ///   - <a href="../../examples/RigidBody/single_axis_hinge_1_body.x3d">single_axis_hinge_1_body.x3d</a>
  ///     ( <a href="examples/single_axis_hinge_1_body.x3d.html">Source</a> )
  ///
  /// \par Internal routes:
  /// \dotfile SingleAxisHingeJoint.dot
  class H3DPHYS_API SingleAxisHingeJoint :  public H3DRigidBodyJointNode {
  public:

    /// Constructor.
    SingleAxisHingeJoint( 
      Inst< SFNode > _metadata = 0,
      Inst< ValueUpdater > _valueUpdater = 0,
      Inst< SFRigidBody > _body1 = 0,
      Inst< SFRigidBody > _body2 = 0,
      Inst< MFString     > _forceOutput = 0,
      Inst< MFEngineOptions > _engineOptions = 0,
      Inst< SFTransformNode   > _transform = 0,
      Inst< SFVec3f  >  _anchorPoint = 0,
      Inst< SFVec3f  >  _axis = 0,
      Inst< SFFloat  >  _maxAngle = 0,
      Inst< SFFloat   >  _minAngle = 0,
      Inst< SFFloat   >  _stopBounce = 0,
      Inst< SFFloat   >  _stopErrorCorrection = 0,
      Inst< SFFloat   >  _angle = 0,
      Inst< SFFloat   >  _angleRate = 0,
      Inst< SFVec3f   >  _body1AnchorPoint = 0,
      Inst< SFVec3f   >  _body2AnchorPoint = 0,
      Inst< SFFloat   >  _bias = 0,
      Inst< SFFloat   >  _softness = 0);

    /// The point where the hinge is centered in world coordinates.
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> Vec3f( 0, 0, 0 ) \n
    /// 
    /// \dotfile SingleAxisHingeJoint_anchorPoint.dot
    auto_ptr< SFVec3f > anchorPoint;

    /// The axis of the hinge. Defined to be along the unit vector defined in
    /// this field (in world coordinates). 
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> Vec3f( 0, 0, 0 ) \n
    /// 
    /// \dotfile SingleAxisHingeJoint_axis.dot
    auto_ptr< SFVec3f > axis;

    /// The maximum angle(in radians) through which the hinge is allowed
    /// to travel.
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> pi \n
    /// 
    /// \dotfile SingleAxisHingeJoint_maxAngle.dot
    auto_ptr< SFFloat > maxAngle;

    /// The minimum angle(in radians) through which the hinge is allowed
    /// to travel.
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> -pi \n
    /// 
    /// \dotfile SingleAxisHingeJoint_minAngle.dot
    auto_ptr< SFFloat > minAngle;

    /// The stopBounce field describes how much the joint should bounce the body
    /// back if the joint limit has been reached or exceeded. A value of zero
    /// indicates no bounce at all, and a value of one says that it should
    /// bounce with velocity equal and opposite to the collision velocity of 
    /// the contact.
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 0 \n
    /// 
    /// \dotfile SingleAxisHingeJoint_stopBounce.dot
    auto_ptr< SFFloat > stopBounce;

    /// The stopErrorCorrection field describes the amount of error correction to
    /// be performed in a time step when the joint reaches the limit. A value of
    /// zero means no error correction is to be performed and a value of one 
    /// means all error should be corrected in a single step.
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 0.8 \n
    /// 
    /// \dotfile SingleAxisHingeJoint_stopErrorCorrection.dot
    auto_ptr< SFFloat > stopErrorCorrection;

    /// The angle output field reports the current relative angle(in radians) 
    /// between the two bodies in radians.
    /// 
    /// <b>Access type:</b> outputOnly
    /// 
    /// \dotfile SingleAxisHingeJoint_angle.dot
    auto_ptr< SFFloat > angle;

    /// The angleRate output field describes the rate at which the relative 
    /// angle between the two bodies change(in radians/s).
    /// 
    /// <b>Access type:</b> outputOnly
    /// 
    /// \dotfile SingleAxisHingeJoint_angleRate.dot
    auto_ptr< SFFloat > angleRate;

    /// The body1AnchorPoint output field reports the current location of the 
    /// anchor point relative to body1.
    ///
    /// <b>Access type:</b> outputOnly
    /// 
    /// \dotfile SingleAxisHingeJoint_body1AnchorPoint.dot
    auto_ptr< SFVec3f > body1AnchorPoint;

    /// The body2AnchorPoint output field reports the current location of the 
    /// anchor point relative to body2.
    ///
    /// <b>Access type:</b> outputOnly
    /// 
    /// \dotfile SingleAxisHingeJoint_body2AnchorPoint.dot
    auto_ptr< SFVec3f > body2AnchorPoint;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:
    /// Returns a new instance of 
    /// PhysicsEngineParameters::SingleAxisHingeJointParameters with the values
    /// corresponding to fields that have changed in the last frame will be set
    /// to the new values. All other values will be ignored.
    /// \param all_params if true, function returns all field values regardless of whether
    /// the values have changed.
    virtual PhysicsEngineParameters::ConstraintParameters * getConstraintParameters( bool all_params = false );

    /// Returns a new concrete instance of ConstraintParameters appropriate for this subtype of H3DBodyConstraintNode
    virtual PhysicsEngineParameters::ConstraintParameters* createConstraintParameters ();

    /// Updates the angle, angleRate, body1AnchorPoint and body2AnchorPoint
    /// fields if they are enabled by the forceOutput field.
    virtual void updateOutputFields();

    /// Apply the specifed transform to all joint parameters
    virtual void applyTransform ( const Matrix4f& _transform );

  public:
    /// The bias field describes the strength with which the joint resists angular
    /// limit violation. 
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 0.3 \n
    /// 
    /// \dotfile SingleAxisHingeJoint_bias.dot
    auto_ptr< SFFloat > bias;
    
    /// The softness field describes the percentage of limit where the movement 
    /// is free. Beyond this softness percentage, velocities that would shoot 
    /// through the limit are slowed down.
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 0.9 \n
    /// 
    /// \dotfile SingleAxisHingeJoint_softness.dot
    auto_ptr< SFFloat > softness;

  };
}
#endif
