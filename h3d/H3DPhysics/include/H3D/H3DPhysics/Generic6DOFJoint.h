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
/// \file Generic6DOFJoint.h
/// \brief Header file for Generic6DOFJoint, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __GENERIC6DOFJOINT__
#define __GENERIC6DOFJOINT__

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
  /// \class Generic6DOFJoint
  /// \brief The Generic6DOFJoint node represents a joint with 3 degrees of translational freedom and 
  /// 3 degrees of rotational freedom. Each degree of freedom, translation and rotational may have 
  /// minimum and maximum limits and a motor. The motors are controlled by specifying a desired velocity
  /// and a maximum force used to achieve that velocity.
  ///
  /// The axis1, axis2 and axis3 fields should define the x, y and z axes of a local coordinate system
  /// used to describe the degrees of freedom of the joint.
  ///
  /// If a minimum limit is greater than a maximum limit, then no limits will be enforced for that degree
  /// of freedom.
  ///
  /// The default behavior of this joint is similar to a BallJoint
  /// By default all translational degrees of freedom have the maximum and minimum limits set to zero and
  /// so are locked. All rotational limits have max < min and therefore have no limits.
  ///
  /// Engine support: This joint type is currently only supported using Bullet
  ///
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../examples/RigidBody/Generic6DOFJoint.x3d">Generic6DOFJoint.x3d</a>
  ///     ( <a href="examples/Generic6DOFJoint.x3d.html">Source</a> )
  ///
  /// \par Internal routes:
  /// \dotfile Generic6DOFJoint.dot
  class H3DPHYS_API Generic6DOFJoint :  public H3DRigidBodyJointNode {
  public:
    /// Constructor.
    Generic6DOFJoint  (
      Inst< SFNode      > _metadata = 0,
      Inst< ValueUpdater > _value_updater = 0,
      Inst< SFRigidBody > _body1 = 0,
      Inst< SFRigidBody > _body2 = 0,
      Inst< MFString    > _forceOutput = 0,
      Inst< MFEngineOptions > _engineOptions = 0,
      Inst< SFTransformNode > _transform = 0, 
      Inst< SFVec3f     > _anchorPoint = 0,
      Inst< SFVec3f     > _axis1 = 0,
      Inst< SFVec3f     > _axis2 = 0,
      Inst< SFVec3f     > _axis3 = 0,
      Inst< SFFloat     > _desiredAngularVelocity1 = 0,
      Inst< SFFloat     > _desiredAngularVelocity2 = 0,
      Inst< SFFloat     > _desiredAngularVelocity3 = 0,
      Inst< SFFloat     > _minAngle1 = 0,
      Inst< SFFloat     > _minAngle2 = 0,
      Inst< SFFloat     > _minAngle3 = 0,
      Inst< SFFloat     > _maxAngle1 = 0,
      Inst< SFFloat     > _maxAngle2 = 0,
      Inst< SFFloat     > _maxAngle3 = 0,
      Inst< SFFloat     > _maxTorque1 = 0,
      Inst< SFFloat     > _maxTorque2 = 0,
      Inst< SFFloat     > _maxTorque3 = 0,
      Inst< SFFloat     > _desiredLinearVelocity1 = 0,
      Inst< SFFloat     > _desiredLinearVelocity2 = 0,
      Inst< SFFloat     > _desiredLinearVelocity3 = 0,
      Inst< SFFloat     > _minLimit1 = 0,
      Inst< SFFloat     > _minLimit2 = 0,
      Inst< SFFloat     > _minLimit3 = 0,
      Inst< SFFloat     > _maxLimit1 = 0,
      Inst< SFFloat     > _maxLimit2 = 0,
      Inst< SFFloat     > _maxLimit3 = 0,
      Inst< SFFloat     > _maxForce1 = 0,
      Inst< SFFloat     > _maxForce2 = 0,
      Inst< SFFloat     > _maxForce3 = 0 );

    /// The point where the joint is centered in world coordinates.
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> Vec3f( 0, 0, 0 ) \n
    /// 
    /// \dotfile Generic6DOFJoint_anchorPoint.dot
    auto_ptr< SFVec3f > anchorPoint;

    // The axes of the joint: axis1, axis2 and axis3

    /// The first axis of the constraint
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> Vec3f( 1, 0, 0 ) \n
    /// 
    /// \dotfile Generic6DOFJoint_axis1.dot
    auto_ptr< SFVec3f > axis1;

    /// The second axis of the constraint
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> Vec3f( 0, 1, 0 ) \n
    /// 
    /// \dotfile Generic6DOFJoint_axis2.dot
    auto_ptr< SFVec3f > axis2;

    /// The third axis of the constraint
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> Vec3f( 0, 0, 1 ) \n
    /// 
    /// \dotfile Generic6DOFJoint_axis3.dot
    auto_ptr< SFVec3f > axis3;

    // The desired angular velocities along each axis of the joint
    // desiredAngularVelocity1, desiredAngularVelocity2 and desiredAngularVelocity3

    /// The angular velocity to achieve on axis1.
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 0 \n
    /// 
    /// \dotfile Generic6DOFJoint_desiredAngularVelocity1.dot
    auto_ptr< SFFloat > desiredAngularVelocity1;

    /// The angular velocity to achieve on axis2.
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 0 \n
    /// 
    /// \dotfile Generic6DOFJoint_desiredAngularVelocity2.dot
    auto_ptr< SFFloat > desiredAngularVelocity2;

    /// The angular velocity to achieve on axis3.
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 0 \n
    /// 
    /// \dotfile Generic6DOFJoint_desiredAngularVelocity3.dot
    auto_ptr< SFFloat > desiredAngularVelocity3;

    /// The maximum angle to which axis1 is allowed to travel.
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value:</b> 1 \n
    /// 
    /// \dotfile Generic6DOFJoint_maxAngle1.dot
    auto_ptr< SFFloat > maxAngle1;

    /// The maximum angle to which axis2 is allowed to travel.
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value:</b> 1 \n
    /// 
    /// \dotfile Generic6DOFJoint_maxAngle2.dot
    auto_ptr< SFFloat > maxAngle2;

    /// The maximum angle to which axis3 is allowed to travel.
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value:</b> 1 \n
    /// 
    /// \dotfile Generic6DOFJoint_maxAngle3.dot
    auto_ptr< SFFloat > maxAngle3;

    /// The minimum angle to which axis1 is allowed to travel.
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value:</b> -1 \n
    /// 
    /// \dotfile Generic6DOFJoint_minAngle1.dot
    auto_ptr< SFFloat > minAngle1;

    /// The minimum angle to which axis2 is allowed to travel.
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value:</b> -1 \n
    /// 
    /// \dotfile Generic6DOFJoint_minAngle2.dot
    auto_ptr< SFFloat > minAngle2;

    /// The minimum angle to which axis3 is allowed to travel.
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value:</b> -1 \n
    /// 
    /// \dotfile Generic6DOFJoint_minAngle3.dot
    auto_ptr< SFFloat > minAngle3;

    /// The maximum amount of torque that the motor can apply on axis1 in order to 
    /// achieve the desired desiredAngularVelocity1 value. 
    /// 
    /// <b>Access type: </b> inputOutput
    /// 
    /// \dotfile Generic6DOFJoint_maxTorque1.dot
    auto_ptr< SFFloat > maxTorque1;

    /// The maximum amount of torque that the motor can apply on axis2 in order to 
    /// achieve the desired desiredAngularVelocity2 value. 
    /// 
    /// <b>Access type: </b> inputOutput
    /// 
    /// \dotfile Generic6DOFJoint_maxTorque2.dot
    auto_ptr< SFFloat > maxTorque2;

    /// The maximum amount of torque that the motor can apply on axis3 in order to 
    /// achieve the desired desiredAngularVelocity3 value. 
    /// 
    /// <b>Access type: </b> inputOutput
    /// 
    /// \dotfile Generic6DOFJoint_maxTorque3.dot
    auto_ptr< SFFloat > maxTorque3;

    // The desired linear velocities along each axis of the joint
    // desiredLinearVelocity1, desiredLinearVelocity2 and desiredLinearVelocity3

    /// The linear velocity to achieve on axis1.
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 0 \n
    /// 
    /// \dotfile Generic6DOFJoint_desiredLinearVelocity1.dot
    auto_ptr< SFFloat > desiredLinearVelocity1;

    /// The linear velocity to achieve on axis2.
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 0 \n
    /// 
    /// \dotfile Generic6DOFJoint_desiredLinearVelocity2.dot
    auto_ptr< SFFloat > desiredLinearVelocity2;

    /// The linear velocity to achieve on axis3.
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 0 \n
    /// 
    /// \dotfile Generic6DOFJoint_desiredLinearVelocity3.dot
    auto_ptr< SFFloat > desiredLinearVelocity3;

    /// The minimum linear travel along axis1. 
    ///
    /// <b>Access type: </b> inputOutput
    /// <b>Default value:</b> 0 \n
    /// 
    /// \dotfile Generic6DOFJoint_minLimit1.dot
    auto_ptr< SFFloat > minLimit1;

    /// The minimum linear travel along axis2. 
    ///
    /// <b>Access type: </b> inputOutput
    /// <b>Default value:</b> 0 \n
    /// 
    /// \dotfile Generic6DOFJoint_minLimit2.dot
    auto_ptr< SFFloat > minLimit2;

    /// The minimum linear travel along axis3.
    ///
    /// <b>Access type: </b> inputOutput
    /// <b>Default value:</b> 0 \n
    /// 
    /// \dotfile Generic6DOFJoint_minLimit3.dot
    auto_ptr< SFFloat > minLimit3;

    /// The maximum linear travel along axis1. 
    ///
    /// <b>Access type: </b> inputOutput
    /// <b>Default value:</b> 0 \n
    /// 
    /// \dotfile Generic6DOFJoint_maxLimit1.dot
    auto_ptr< SFFloat > maxLimit1;

    /// The maximum linear travel along axis2. 
    ///
    /// <b>Access type: </b> inputOutput
    /// <b>Default value:</b> 0 \n
    /// 
    /// \dotfile Generic6DOFJoint_maxLimit2.dot
    auto_ptr< SFFloat > maxLimit2;

    /// The maximum linear travel along axis3.
    ///
    /// <b>Access type: </b> inputOutput
    /// <b>Default value:</b> 0 \n
    /// 
    /// \dotfile Generic6DOFJoint_maxLimit3.dot
    auto_ptr< SFFloat > maxLimit3;

    /// The maximum amount of force that the motor can apply on axis1 in order to 
    /// achieve the desired desiredLinearVelocity1 value. 
    /// 
    /// <b>Access type: </b> inputOutput
    /// 
    /// \dotfile Generic6DOFJoint_maxForce1.dot
    auto_ptr< SFFloat > maxForce1;

    /// The maximum amount of force that the motor can apply on axis2 in order to 
    /// achieve the desired desiredLinearVelocity2 value. 
    /// 
    /// <b>Access type: </b> inputOutput
    /// 
    /// \dotfile Generic6DOFJoint_maxForce2.dot
    auto_ptr< SFFloat > maxForce2;

    /// The maximum amount of force that the motor can apply on axis3 in order to 
    /// achieve the desired desiredLinearVelocity3 value. 
    /// 
    /// <b>Access type: </b> inputOutput
    /// 
    /// \dotfile Generic6DOFJoint_maxForce3.dot
    auto_ptr< SFFloat > maxForce3;

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
    //void updateOutputFields();

    /// Apply the specifed transform to all joint parameters
    virtual void applyTransform ( const Matrix4f& _transform );
  };
}
#endif
