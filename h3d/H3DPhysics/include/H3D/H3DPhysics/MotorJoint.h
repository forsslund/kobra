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
/// \file MotorJoint.h
/// \brief Header file for MotorJoint, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __MOTORJOINT__
#define __MOTORJOINT__

#include <H3D/X3DNode.h>
#include <H3D/SFFloat.h>
#include <H3D/SFInt32.h>
#include <H3D/SFBool.h>
#include <H3D/H3DPhysics/H3DPhysics.h>
#include <H3D/H3DPhysics/H3DRigidBodyJointNode.h>

namespace H3D{

  /// \ingroup X3DNodes
  /// \class MotorJoint
  /// \brief The MotorJoint node allows control of the relative angular 
  /// velocities between the two bodies (specified by the body1 and body2 
  /// fields) associated with a joint. This can be especially useful with a 
  /// BallJoint where there is no restriction on the angular degrees of 
  /// freedom.
  ///
  /// The autoCalc field is used to control whether the user shall manually
  /// provide the individual angle rotations each frame or if they are to be 
  /// automatically calculated from the motor's implementation.
  ///
  /// The motorAxis fields define the axis vector of the corresponding axis. 
  /// If the value is (0, 0, 0) the corresponding axis is disabled and the 
  /// motor does not apply a force or torque along that axis. The motorAxis1
  /// field is anchored to the global frame. The motorAxis2 field is anchored
  /// to body1's frame of reference, and the motorAxis3 field is anchored to 
  /// body2's frame of reference.
  ///
  /// The three axis angle fields provide angles (in radians) for this frame
  /// for the corresponding motor axis when in user-calculated mode.
  ///
  /// When the autoCalc field is set to FALSE, the enabledAxes field indicates
  /// how many axes can currently be controlled and modified. If the value is 
  /// zero, the motor is effectively disabled. If the value is 1, only axis1 
  /// is enabled, a value of 2 has axis 1 and axis 2 enabled and a value of 3 
  /// has all axes enabled.
  ///
  /// The motor angle output fields provide the calculated angle for this 
  /// motor joint from the last frame. The motor angle rate output fields 
  /// describe the rate, in radians, that the motor is turning.
  ///
  /// The stop bounce fields describe how much the joint should bounce the body
  /// back on the corresponding axis if the joint limit has been reached or 
  /// exceeded. A value of zero indicates no bounce at all, and a value of one
  /// says that it should bounce with velocity equal and opposite to the 
  /// collision velocity of the contact.
  ///
  /// The stop error correction fields describe the amount of error 
  /// correction to be performed in a time step when the joint reaches the 
  /// limit on the corresponding axis. A value of zero means no error 
  /// correction is to be performed and a value of one means all error should 
  /// be corrected in a single step.
  ///
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../examples/RigidBody/MotorJoint.x3d">MotorJoint.x3d</a>
  ///     ( <a href="examples/MotorJoint.x3d.html">Source</a> )
  ///
  /// \par Internal routes:
  /// \dotfile MotorJoint.dot
  class H3DPHYS_API MotorJoint :  public H3DRigidBodyJointNode {
  public:
    /// Constructor.
    MotorJoint(
      Inst< SFNode > _metadata = 0,
      Inst< ValueUpdater > _valueUpdater = 0,
      Inst< SFRigidBody > _body1 = 0,
      Inst< SFRigidBody > _body2 = 0,
      Inst< MFString     > _forceOutput = 0,
      Inst< MFEngineOptions > _engineOptions = 0,
      Inst< SFTransformNode > _transform     = 0,
      Inst< SFFloat >  _axis1Angle           = 0,
      Inst< SFFloat >  _axis1Torque          = 0,
      Inst< SFFloat >  _axis2Angle           = 0,
      Inst< SFFloat >  _axis2Torque          = 0,
      Inst< SFFloat >  _axis3Angle           = 0,
      Inst< SFFloat >  _axis3Torque          = 0,
      Inst< SFInt32 >  _enabledAxes          = 0,
      Inst< SFVec3f >  _motor1Axis           = 0,
      Inst< SFVec3f >  _motor2Axis           = 0,
      Inst< SFVec3f >  _motor3Axis           = 0,
      Inst< SFFloat >  _stop1Bounce          = 0,
      Inst< SFFloat >  _stop1ErrorCorrection = 0,
      Inst< SFFloat >  _stop2Bounce          = 0,
      Inst< SFFloat >  _stop2ErrorCorrection = 0,
      Inst< SFFloat >  _stop3Bounce          = 0,
      Inst< SFFloat >  _stop3ErrorCorrection = 0,
      Inst< SFFloat >  _motor1Angle          = 0,
      Inst< SFFloat >  _motor1AngleRate      = 0,
      Inst< SFFloat >  _motor2Angle           = 0,
      Inst< SFFloat >  _motor2AngleRate       = 0,
      Inst< SFFloat >  _motor3Angle           = 0,
      Inst< SFFloat >  _motor3AngleRate       = 0,
      Inst< SFBool  >  _autoCalc             = 0 );

    /// The angle in radians for motor1Axis, for the current frame.
    /// This value is used when autoCalc is false.
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 0 \n
    /// 
    /// \dotfile MotorJoint_axis1Angle.dot
    auto_ptr< SFFloat  > axis1Angle;

    /// The torque in Newtons(?) for motor1Axis for the current frame. This
    /// value should probably be the torque required to achieve the value in
    /// axis1Angle.
    /// NOTE: This field is so badly documented in the X3D specification that
    /// I have no idea how it should work.
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 0 \n
    /// 
    /// \dotfile MotorJoint_axis1Torque.dot
    auto_ptr< SFFloat  > axis1Torque;

    /// The angle in radians for motor2Axis, for the current frame.
    /// This value is used when autoCalc is false.
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 0 \n
    /// 
    /// \dotfile MotorJoint_axis2Angle.dot
    auto_ptr< SFFloat  > axis2Angle;

    /// The torque in Newtons(?) for motor2Axis for the current frame. This
    /// value should probably be the torque required to achieve the value in
    /// axis2Angle.
    /// NOTE: This field is so badly documented in the X3D specification that
    /// I have no idea how it should work.
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 0 \n
    /// 
    /// \dotfile MotorJoint_axis2Torque.dot
    auto_ptr< SFFloat  > axis2Torque;

    /// The angle in radians for motor3Axis, for the current frame.
    /// This value is used when autoCalc is false.
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 0 \n
    /// 
    /// \dotfile MotorJoint_axis3Angle.dot
    auto_ptr< SFFloat  > axis3Angle;

    /// The torque in Newtons(?) for motor3Axis for the current frame. This
    /// value should probably be the torque required to achieve the value in
    /// axis3Angle.
    /// NOTE: This field is so badly documented in the X3D specification that
    /// I have no idea how it should work.
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 0 \n
    /// 
    /// \dotfile MotorJoint_axis3Torque.dot
    auto_ptr< SFFloat  > axis3Torque;

    /// The number of axes that can currently be controlled and 
    /// modified. If the value of enableAxes is:
    /// - 0, motor is effectively disabled.  
    /// - 1, only axis 1 is enabled
    /// - 2, axis1 and axis 2 are enabled
    /// - 3, all axes are enabled
    /// enableAxes is used when autoCalc is false.
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 1 \n
    /// 
    /// \dotfile MotorJoint_enabledAxes.dot
    auto_ptr< SFInt32  > enabledAxes;

    /// The axis vector of axis 1. Value of (0, 0, 0) disables axis 1
    /// and the motor does not apply a force or torque along axis 1. 
    /// motor1Axis1 field is anchored to the global frame. 
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> Vec3f(0, 0, 0) \n
    /// 
    /// \dotfile MotorJoint_motor1Axis.dot
    auto_ptr< SFVec3f  > motor1Axis;

    /// The axis vector of axis 2. Value of (0, 0, 0) disables axis 2
    /// and the motor does not apply a force or torque along axis 2.
    /// motor2Axis field is anchored to body1's frame of reference.
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> Vec3f(0, 0, 0) \n
    /// 
    /// \dotfile MotorJoint_motor2Axis.dot
    auto_ptr< SFVec3f  > motor2Axis;

    /// The axis vector of axis 3. Value of (0, 0, 0) disables axis 3
    /// and the motor does not apply a force or torque along axis 3.
    /// motor3Axis field is anchored to body2's frame of reference. 
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> Vec3f(0, 0, 0) \n
    /// 
    /// \dotfile MotorJoint_motor3Axis.dot
    auto_ptr< SFVec3f  > motor3Axis;

    /// The amount that the joint should bounce the body back on axis 1
    /// if the joint limit has been reached or exceeded. Zero value 
    /// indicates no bounce at all, 1 indicates bounce with velocity equal
    /// and opposite to the collision velocity of the contact.
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 0 \n
    /// 
    /// \dotfile MotorJoint_stop1Bounce.dot
    auto_ptr< SFFloat  > stop1Bounce;

    /// The amount of error correction to be performed in a time step 
    /// when the joint reaches the limit on axis 1. Zero value means no 
    /// error correction, and a value of one means all errors should be 
    /// corrected in a single step.
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 0.8 \n
    /// 
    /// \dotfile MotorJoint_stop1ErrorCorrection.dot
    auto_ptr< SFFloat  > stop1ErrorCorrection;

    /// The amount that the joint should bounce the body back on axis 2
    /// if the joint limit has been reached or exceeded. Zero value 
    /// indicates no bounce at all, 1 indicates bounce with velocity equal
    /// and opposite to the collision velocity of the contact.
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 0 \n
    /// 
    /// \dotfile MotorJoint_stop2Bounce.dot
    auto_ptr< SFFloat  > stop2Bounce;

    /// The amount of error correction to be performed in a time step 
    /// when the joint reaches the limit on axis 2. Zero value means no 
    /// error correction, and a value of one means all errors should be 
    /// corrected in a single step.
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 0.8 \n
    /// 
    /// \dotfile MotorJoint_stop1ErrorCorrection.dot
    auto_ptr< SFFloat  > stop2ErrorCorrection;

    /// The amount that the joint should bounce the body back on axis 3
    /// if the joint limit has been reached or exceeded. Zero value 
    /// indicates no bounce at all, 1 indicates bounce with velocity equal
    /// and opposite to the collision velocity of the contact.
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 0 \n
    /// 
    /// \dotfile MotorJoint_stop3Bounce.dot
    auto_ptr< SFFloat  > stop3Bounce;

    /// The amount of error correction to be performed in a time step 
    /// when the joint reaches the limit on axis 3. Zero value means no 
    /// error correction, and a value of one means all errors should be 
    /// corrected in a single step.
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 0.8 \n
    /// 
    /// \dotfile MotorJoint_stop1ErrorCorrection.dot
    auto_ptr< SFFloat  > stop3ErrorCorrection;

    /// The calculated angle in radians for this motor joint from the last frame. 
    /// 
    /// <b>Access type:</b> outputOnly
    /// 
    /// \dotfile MotorJoint_motor1Angle.dot
    auto_ptr< SFFloat  > motor1Angle;

    /// The calculated angle rate in radians at which the motor is turning. 
    /// 
    /// <b>Access type:</b> outputOnly
    /// 
    /// \dotfile MotorJoint_motor1AngleRate.dot
    auto_ptr< SFFloat  > motor1AngleRate;

    /// The calculated angle in radians for this motor joint from the last frame. 
    /// 
    /// <b>Access type:</b> outputOnly
    /// 
    /// \dotfile MotorJoint_motor2Angle.dot
    auto_ptr< SFFloat  > motor2Angle;

    /// The calculated angle rate in radians at which the motor is turning. 
    /// 
    /// <b>Access type:</b> outputOnly
    /// 
    /// \dotfile MotorJoint_motor2AngleRate.dot
    auto_ptr< SFFloat  > motor2AngleRate;

    /// The calculated angle in radians for this motor joint from the last frame. 
    /// 
    /// <b>Access type:</b> outputOnly
    /// 
    /// \dotfile MotorJoint_motor3Angle.dot
    auto_ptr< SFFloat  > motor3Angle;

    /// The calculated angle rate in radians at which the motor is turning. 
    /// 
    /// <b>Access type:</b> outputOnly
    /// 
    /// \dotfile MotorJoint_motor3AngleRate.dot
    auto_ptr< SFFloat  > motor3AngleRate;

    /// Specifies whether the user shall manually provide the individual
    /// angle rotations each frame or if they are to be automatically 
    /// calculated from the motor's implementation.
    ///
    /// <b>Access type:</b> initializeOnly
    /// <b>Default value:</b> FALSE \n
    /// 
    /// \dotfile MotorJoint_autoCalc.dot
    auto_ptr< SFBool   > autoCalc;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:
    /// Returns a new instance of 
    /// PhysicsEngineParameters::MotorJointParameters with the values updated with
    /// corresponding field values that have changed in the last frame. 
    /// All other values will be ignored.
    /// \param all_params if true, function returns all field values regardless of whether
    /// the values have changed.
    virtual PhysicsEngineParameters::ConstraintParameters * getConstraintParameters( bool all_params = false );

    /// Returns a new concrete instance of ConstraintParameters appropriate for this subtype of H3DBodyConstraintNode
    virtual PhysicsEngineParameters::ConstraintParameters* createConstraintParameters ();

    /// Updates the motor1Angle, motor1AngleRate,  motor2Angle, motor2AngleRate,
    /// motor3Angle and motor3AngleRate fields if they are enabled by the forceOutput field.
    void updateOutputFields();

    /// Apply the specifed transform to all joint parameters
    virtual void applyTransform ( const Matrix4f& _transform );

  };
}
#endif
