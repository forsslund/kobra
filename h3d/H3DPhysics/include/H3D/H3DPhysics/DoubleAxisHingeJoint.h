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
/// \file DoubleAxisHingeJoint.h
/// \brief Header file for DoubleAxisHingeJoint, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __DOUBLEAXISHINGEJOINT__
#define __DOUBLEAXISHINGEJOINT__

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
  /// \class DoubleAxisHingeJoint
  /// \brief The DoubleAxisHingeJoint node represents a joint that has two 
  /// independent axes that are located around a common anchor point. Axis 1 
  /// is specified relative to the first body (specified by the body1 field) 
  /// and axis 2 is specified relative to the second body (specified by the 
  /// body2 field). Axis 1 can have limits and a motor, axis 2 can only have 
  /// a motor.
  ///
  /// The minAngle1 and maxAngle1 fields are used to control the maximum angles 
  /// through which the hinge is allowed to travel. A hinge may not travel more 
  /// than p radians in either direction from it's initial position.
  ///
  /// The stopBounce1 field is used to set how bouncy the minimum and maximum 
  /// angle stops are for axis 1. A value of zero means they are not bouncy 
  /// while a value of 1 means maximum bounciness (full reflection of force 
  /// arriving at the stop).
  /// 
  /// The stopErrorCorrection1 and suspensionErrorCorrection fields describe 
  /// how quickly the system should resolve intersection errors due to floating 
  /// point inaccuracies. This value ranges between 0 and 1. A value of 0 means 
  /// no correction at all while a value of 1 indicates that all errors should 
  /// be corrected in a single step.
  ///
  /// The stopConstantForceMix1 and suspensionForce fields can be used to apply 
  /// damping to the calculations by violating the normal constraints by applying 
  /// a small, constant force to those calculations. This allows joints and bodies 
  /// to be a fraction springy, as well as helping to eliminate numerical instability. 
  /// The larger the value, the more soft each of the constraints being evaluated. 
  /// A value of zero indicates hard constraints so that everything is exactly 
  /// honoured. By combining the stopErrorCorrection1 and stopConstantForceMix1 
  /// fields and/or the suspensionErrorCorrection and suspensionForce fields, various 
  /// effects, such as spring-driven or spongy connections, can be emulated.
  ///
  /// The maxTorque1 field defines the maximum amount of torque that the motor 
  /// can apply on axis 1 in order to achieve the desired desiredAngularVelocity1 
  /// value. Similarly, maxTorque2 controls the maximum amount of torque to achieve 
  /// desiredAngularVelocity2 on axis 2.
  ///
  /// The hingeXAngle output fields report the current relative angle between the 
  /// two bodies in radians and the hingeXAngleRate field describes the rate at 
  /// which that angle is currently changing in radians per second.
  ///
  /// The body anchor point and body axis output fields report the current location 
  /// of the anchor point relative to the corresponding body. This can be used to 
  /// determine if the joint has broken.
  ///
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../examples/RigidBody/DoubleAxisHingeJoint.x3d">DoubleAxisHingeJoint.x3d</a>
  ///     ( <a href="examples/DoubleAxisHingeJoint.x3d.html">Source</a> )
  ///
  /// \par Internal routes:
  /// \dotfile DoubleAxisHingeJoint.dot
  class H3DPHYS_API DoubleAxisHingeJoint :  public H3DRigidBodyJointNode {
  public:
    /// Constructor.
    DoubleAxisHingeJoint( Inst< SFNode > _metadata = 0,
      Inst< ValueUpdater > _valueUpdater = 0,
      Inst< SFRigidBody > _body1 = 0,
      Inst< SFRigidBody > _body2 = 0,
      Inst< MFString     > _forceOutput = 0,
      Inst< MFEngineOptions > _engineOptions = 0,
      Inst< SFTransformNode > _transform = 0,
      Inst< SFVec3f     > _anchorPoint = 0,
      Inst< SFVec3f     > _axis1 = 0,
      Inst< SFVec3f     > _axis2 = 0,
      Inst< SFFloat     > _desiredAngularVelocity1 = 0,
      Inst< SFFloat     > _desiredAngularVelocity2 = 0,
      Inst< SFFloat     > _maxAngle1 = 0,
      Inst< SFFloat     > _maxTorque1 = 0,
      Inst< SFFloat     > _maxTorque2 = 0,
      Inst< SFFloat     > _minAngle1 = 0,
      Inst< SFFloat     > _stopBounce1 = 0,
      Inst< SFFloat     > _stopConstantForceMix1 = 0,
      Inst< SFFloat     > _stopErrorCorrection1 = 0,
      Inst< SFFloat     > _suspensionErrorCorrection = 0,
      Inst< SFFloat     > _suspensionForce = 0,
      Inst< SFVec3f     > _body1AnchorPoint = 0,
      Inst< SFVec3f     > _body2AnchorPoint = 0,
      Inst< SFVec3f     > _body1Axis = 0,
      Inst< SFVec3f     > _body2Axis = 0,
      Inst< SFFloat     > _hinge1Angle = 0,
      Inst< SFFloat     > _hinge1AngleRate = 0,
      Inst< SFFloat     > _hinge2Angle = 0,
      Inst< SFFloat     > _hinge2AngleRate = 0 );

    /// The point where the joint is centered in world coordinates.
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> Vec3f( 0, 0, 0 ) \n
    /// 
    /// \dotfile DoubleAxisHingeJoint_anchorPoint.dot
    auto_ptr< SFVec3f > anchorPoint;

    /// The first axis of the body, specified relative to body1.
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> Vec3f( 0, 0, 0 ) \n
    /// 
    /// \dotfile DoubleAxisHingeJoint_axis1.dot
    auto_ptr< SFVec3f > axis1;

    /// The second axis of the body, specified relative to body2.
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> Vec3f( 0, 0, 0 ) \n
    /// 
    /// \dotfile DoubleAxisHingeJoint_axis2.dot
    auto_ptr< SFVec3f > axis2;

    /// The angular velocity to achieve on body1.
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 0 \n
    /// 
    /// \dotfile DoubleAxisHingeJoint_desiredAngularVelocity1.dot
    auto_ptr< SFFloat > desiredAngularVelocity1;

    /// The angular velocity to achieve on body2.
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 0 \n
    /// 
    /// \dotfile DoubleAxisHingeJoint_desiredAngularVelocity2.dot
    auto_ptr< SFFloat > desiredAngularVelocity2;

    /// The maximum angle through which the hinge is allowed to travel. 
    /// (may not be more than p radians in either direction from initial position)
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value:</b> pi \n
    /// 
    /// \dotfile DoubleAxisHingeJoint_maxAngle1.dot
    auto_ptr< SFFloat > maxAngle1;

    /// The maximum amount of torque that the motor can apply on axis1 in order to 
    /// achieve the desired desiredAngularVelocity1 value. 
    /// 
    /// <b>Access type: </b> inputOutput
    /// 
    /// \dotfile DoubleAxisHingeJoint_maxTorque1.dot
    auto_ptr< SFFloat > maxTorque1;

    /// The maximum amount of torque that the motor can apply on axis2 in order to 
    /// achieve the desired desiredAngularVelocity2 value. 
    /// 
    /// <b>Access type: </b> inputOutput
    /// 
    /// \dotfile DoubleAxisHingeJoint_maxTorque2.dot
    auto_ptr< SFFloat > maxTorque2;

    /// The minimum angle through which the hinge is allowed to travel. 
    ///
    /// <b>Access type: </b> inputOutput
    /// <b>Default value:</b> -pi \n
    /// 
    /// \dotfile DoubleAxisHingeJoint_minAngle1.dot
    auto_ptr< SFFloat > minAngle1;

    /// The bounciness of the minimum and maximum angle stops for axis1. Zero indicates 
    /// no bounce while 1 indicates maximum bounce (full reflection of force arriving at
    /// the stop).
    ///
    /// <b>Access type: </b> inputOutput
    /// <b>Default value:</b> 0 \n
    /// 
    /// \dotfile DoubleAxisHingeJoint_stopBounce1.dot
    auto_ptr< SFFloat > stopBounce1;

    /// The value used to resolve intersection errors due to floating point inaccuracies. 
    /// Zero value means no correction at all while a value of 1 indicates that all errors 
    /// should be corrected in a single step.
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value:</b> 0.001 \n
    /// 
    /// \dotfile DoubleAxisHingeJoint_stopConstantForceMix1.dot
    auto_ptr< SFFloat > stopConstantForceMix1;

    /// Another value used to resolve intersection errors due to floating point inaccuracies. 
    /// Zero value means no correction at all while a value of 1 indicates that all errors 
    /// should be corrected in a single step.
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value:</b> 0.8 \n
    /// 
    /// \dotfile DoubleAxisHingeJoint_stopErrorCorrection1.dot
    auto_ptr< SFFloat > stopErrorCorrection1;

    /// The damping value that allows joints and bodies to be a fraction springy, 
    /// Also used to eliminate numerical instability. Zero indicates hard constraints.
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value:</b> 0.8 \n
    /// 
    /// \dotfile DoubleAxisHingeJoint_suspensionErrorCorrection.dot
    auto_ptr< SFFloat > suspensionErrorCorrection;

    /// Another damping value that allows joints and bodies to be a fraction springy, 
    /// Also used to eliminate numerical instability. Zero indicates hard constraints.
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value:</b> 0 \n
    /// 
    /// \dotfile DoubleAxisHingeJoint_suspensionForce.dot
    auto_ptr< SFFloat > suspensionForce;

    /// The body1AnchorPoint output field reports the current location of the 
    /// anchor point relative to body1.
    ///
    /// <b>Access type:</b> outputOnly
    /// 
    /// \dotfile DoubleAxisHingeJoint_body1AnchorPoint.dot
    auto_ptr< SFVec3f > body1AnchorPoint;

    /// The axis of body1.
    /// 
    /// <b>Access type:</b> outputOnly
    /// 
    /// \dotfile DoubleAxisHingeJoint_body2Axis.dot
    auto_ptr< SFVec3f > body1Axis;

    /// The body1AnchorPoint output field reports the current location of the 
    /// anchor point relative to body2.
    ///
    /// <b>Access type:</b> outputOnly
    /// 
    /// \dotfile DoubleAxisHingeJoint_body2AnchorPoint.dot
    auto_ptr< SFVec3f > body2AnchorPoint;

    /// The axis of body2.
    /// 
    /// <b>Access type:</b> outputOnly
    /// 
    /// \dotfile DoubleAxisHingeJoint_body2Axis.dot
    auto_ptr< SFVec3f > body2Axis;

    /// The angle of body1.
    /// 
    /// <b>Access type:</b> outputOnly
    /// 
    /// \dotfile DoubleAxisHingeJoint_hinge1Angle.dot
    auto_ptr< SFFloat > hinge1Angle;

    /// The angle rate of body1.
    /// 
    /// <b>Access type:</b> outputOnly
    /// 
    /// \dotfile DoubleAxisHingeJoint_hinge1AngleRate.dot
    auto_ptr< SFFloat > hinge1AngleRate;

    /// The angle of body1.
    /// 
    /// <b>Access type:</b> outputOnly
    /// 
    /// \dotfile DoubleAxisHingeJoint_hinge2Angle.dot
    auto_ptr< SFFloat > hinge2Angle;

    /// The angle rate of body2.
    /// 
    /// <b>Access type:</b> outputOnly
    /// 
    /// \dotfile DoubleAxisHingeJoint_hinge2AngleRate.dot
    auto_ptr< SFFloat > hinge2AngleRate;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:
    /// Returns a new instance of 
    /// PhysicsEngineParameters::DoubleAxisJointParameters with the values updated with
    /// corresponding field values that have changed in the last frame. 
    /// All other values will be ignored.
    /// \param all_params if true, function returns all field values regardless of whether
    /// the values have changed.
    virtual PhysicsEngineParameters::ConstraintParameters * getConstraintParameters( bool all_params = false );

    /// Returns a new concrete instance of ConstraintParameters appropriate for this subtype of H3DBodyConstraintNode
    virtual PhysicsEngineParameters::ConstraintParameters* createConstraintParameters ();

    /// Updates the body1Axis, body2Axis, body1AnchorPoint, body2AnchorPoint,
    /// hinge1Angle, hinge2Angle, hinge1AngleRate, hinge2AngleRate fields if 
    /// they are enabled by the forceOutput field.
    void updateOutputFields();

    /// Apply the specifed transform to all joint parameters
    virtual void applyTransform ( const Matrix4f& _transform );
  };
}
#endif
