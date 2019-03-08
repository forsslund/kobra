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
/// \file UniversalJoint.h
/// \brief Header file for UniversalJoint, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __UNIVERSALJOINT__
#define __UNIVERSALJOINT__

#include <H3D/X3DNode.h>
#include <H3D/SFNode.h>
#include <H3D/SFFloat.h>
#include <H3D/H3DPhysics/H3DPhysics.h>
#include <H3D/H3DPhysics/H3DRigidBodyJointNode.h>

namespace H3D{

  /// \ingroup X3DNodes
  /// \class UniversalJoint
  /// \brief A universal joint is like a BallJoint that constrains an extra 
  /// degree of rotational freedom. Given the axis specified by the axis1 
  /// field on the body specified by the body1 field, and the axis specified 
  /// by the axis2 field on body2 that is perpendicular to axis1, the 
  /// UniversalJoint node keeps the axes perpendicular to each other. Thus, 
  /// rotation of the two bodies about the direction perpendicular to the two 
  /// axes will be equal.
  ///
  /// The  vectors specified by the axis1 and axis2 fields shall be 
  /// perpendicular. If not, the interactions are undefined.
  ///
  /// The stop bounce fields describe how much the joint should bounce the 
  /// body back on the corresponding axis if the joint limit has been reached 
  /// or exceeded. A value of zero indicates no bounce at all, and a value of 
  /// one indicates that it should bounce with velocity equal and opposite to 
  /// the collision velocity of the contact.
  ///
  /// The stop error correction fields describe the amount of error correction 
  /// to be performed in a time step when the joint reaches the limit on the 
  /// corresponding axis. A value of zero means no error correction is to be 
  /// performed and a value of one means all error should be corrected in a 
  /// single step.
  ///
  /// The body anchor point and body axis output fields report the current 
  /// location of the anchor point relative to the corresponding body. This 
  /// can be used to determine if the joint has broken.
  ///
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../examples/RigidBody/UniversalJoint.x3d">UniversalJoint.x3d</a>
  ///     ( <a href="examples/UniversalJoint.x3d.html">Source</a> )
  ///
  /// \par Internal routes:
  /// \dotfile UniversalJoint.dot
  class H3DPHYS_API UniversalJoint :  public H3DRigidBodyJointNode {
  public:
    /// Constructor.
    UniversalJoint(
      Inst< SFNode > _metadata = 0,
      Inst< ValueUpdater > _valueUpdater = 0,
      Inst< SFRigidBody > _body1 = 0,
      Inst< SFRigidBody > _body2 = 0,
      Inst< MFString     > _forceOutput = 0,
      Inst< MFEngineOptions > _engineOptions = 0,
      Inst< SFTransformNode > _transform = 0,
      Inst< SFVec3f  >  _anchorPoint          = 0,
      Inst< SFVec3f  >  _axis1                = 0,
      Inst< SFVec3f  >  _axis2                = 0,
      Inst< SFFloat  >  _stop1Bounce          = 0,
      Inst< SFFloat  >  _stop1ErrorCorrection = 0,
      Inst< SFFloat  >  _stop2Bounce          = 0,
      Inst< SFFloat  >  _stop2ErrorCorrection = 0,
      Inst< SFVec3f  >  _body1AnchorPoint      = 0,
      Inst< SFVec3f  >  _body1Axis            = 0,
      Inst< SFVec3f  >  _body2AnchorPoint      = 0,
      Inst< SFVec3f  >  _body2Axis            = 0 );

    /// The point where the joint is centered in world coordinates.
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> Vec3f( 0, 0, 0 ) \n
    /// 
    /// \dotfile UniversalJoint_anchorPoint.dot
    auto_ptr< SFVec3f  > anchorPoint;

    /// The joint axis on body1.
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> Vec3f( 0, 0, 0 ) \n
    /// 
    /// \dotfile UniversalJoint_axis1.dot
    auto_ptr< SFVec3f  > axis1;

    /// The joint axis on body2.
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> Vec3f( 0, 0, 0 ) \n
    /// 
    /// \dotfile UniversalJoint_axis2.dot
    auto_ptr< SFVec3f  > axis2;

    /// The amount at which the joint should bounce the body1 back on 
    /// the axis1 if the joint limit has been reached or exceeded. Zero 
    /// value indicates no bounce at all, and a value of one indicates 
    /// that it should bounce with velocity equal and opposite to the 
    /// collision velocity of the contact.
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 0 \n
    /// 
    /// \dotfile UniversalJoint_stop1Bounce.dot
    auto_ptr< SFFloat  > stop1Bounce;

    /// The amount of error correction to be performed in a time step 
    /// when the joint reaches the limit on axis1. Zero value means no 
    /// error correction, and a value of one means all errors should be 
    /// corrected in a single step.
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 0.8 \n
    /// 
    /// \dotfile UniversalJoint_stop1ErrorCorrection.dot
    auto_ptr< SFFloat  > stop1ErrorCorrection;

    /// The amount at which the joint should bounce the body2 back on 
    /// the axis2 if the joint limit has been reached or exceeded. Zero 
    /// value indicates no bounce at all, and a value of one indicates 
    /// that it should bounce with velocity equal and opposite to the 
    /// collision velocity of the contact.
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 0 \n
    /// 
    /// \dotfile UniversalJoint_stop2Bounce.dot
    auto_ptr< SFFloat  > stop2Bounce;

    /// The amount of error correction to be performed in a time step 
    /// when the joint reaches the limit on axis2. Zero value means no 
    /// error correction, and a value of one means all errors should be 
    /// corrected in a single step.
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 0.8 \n
    /// 
    /// \dotfile UniversalJoint_stop2ErrorCorrection.dot
    auto_ptr< SFFloat  > stop2ErrorCorrection;

    /// The current location of anchorPoint relative to body1.
    /// 
    /// <b>Access type:</b> outputOnly
    /// 
    /// \dotfile UniversalJoint_body1AnchorPoint.dot
    auto_ptr< SFVec3f  > body1AnchorPoint;

    /// The axis of body1.
    /// 
    /// <b>Access type:</b> outputOnly
    /// 
    /// \dotfile UniversalJoint_body1Axis.dot
    auto_ptr< SFVec3f  > body1Axis;

    /// The current location of anchorPoint relative to body1.
    /// 
    /// <b>Access type:</b> outputOnly
    /// 
    /// \dotfile UniversalJoint_body2AnchorPoint.dot
    auto_ptr< SFVec3f  > body2AnchorPoint     ;

    /// The axis of body2.
    /// 
    /// <b>Access type:</b> outputOnly
    /// 
    /// \dotfile UniversalJoint_body2Axis.dot
    auto_ptr< SFVec3f  > body2Axis;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:
    /// Returns a new instance of 
    /// PhysicsEngineParameters::UniversalJointParameters with the values updated with
    /// corresponding field values that have changed in the last frame. 
    /// All other values will be ignored.
    /// \param all_params if true, function returns all field values regardless of whether
    /// the values have changed.
    virtual PhysicsEngineParameters::ConstraintParameters * getConstraintParameters( bool all_params = false );

    /// Returns a new concrete instance of ConstraintParameters appropriate for this subtype of H3DBodyConstraintNode
    virtual PhysicsEngineParameters::ConstraintParameters* createConstraintParameters ();

    /// Updates the body1Axis, body2Axis, body1AnchorPoint and 
    /// body2AnchorPoint fields if they are enabled by the forceOutput field.
    void updateOutputFields();

    /// Apply the specifed transform to all joint parameters
    virtual void applyTransform ( const Matrix4f& _transform );

  };
}
#endif
