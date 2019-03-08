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
/// \file JointPID.h
/// \brief Header file for JointPID, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __JOINTPID__
#define __JOINTPID__

#include <H3D/H3DPhysics/H3DPIDNode.h>
#include <H3D/H3DPhysics/RigidBody.h>
#include <H3D/H3DPhysics/H3DRigidBodyJointNode.h>
#include <H3D/H3DPhysics/PIDController.h>

namespace H3D {

  /// \class JointPID
  /// \brief This class can be used to smoothly drive a physics joint to a certain
  /// state using a PID control loop.
  ///
  /// Assumptions: 
  /// <ul>
  ///   <li> For a joint fixed to space, the single body must be in the body1 field </li>
  ///   <li> Forces are applied to the body in the joint's body1 field. Joint chains should
  ///        be arranged such that multiple PID controllers do not apply forces to the same body </li>
  /// </ul>
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../examples/RigidBody/JointPID.x3d">JointPID.x3d</a>
  ///     ( <a href="examples/JointPID.x3d.html">Source</a> )
  ///
  /// \par Internal routes:
  /// \dotfile JointPID.dot
  class H3DPHYS_API JointPID : public H3DPIDNode {
  public:
    typedef TypedSFNode < H3DRigidBodyJointNode > SFJointNode;
    typedef TypedSFNode < PIDController > SFPIDController;

    // Constructor.
    JointPID(
      Inst< SFNode          >  _metadata = 0,
      Inst< SFPIDController >  _linearControl = 0,
      Inst< SFPIDController >  _angularControl1 = 0,
      Inst< SFPIDController >  _angularControl2 = 0,
      Inst< SFJointNode     >  _joint = 0,
      Inst< SFFloat         >  _errorSleepThreshold = 0,
      Inst< SFBool          >  _useJointMotor = 0,
      Inst< SFBool          >  _switchForcesToBody2 = 0,
      Inst< SFBool          >  _applyTorqueAsForce = 0 );

    /// Traverse the scene graph.
    virtual void traverseSG( TraverseInfo &ti );

    /// Calculates the new PID output to update the actuation: Force/Torque
    virtual void updateActuation();

    /// PID controller for linear axis
    ///
    /// <b>Access type:</b> inputOutput
    /// 
    /// \dotfile JointPID_linearControl.dot
    auto_ptr< SFPIDController >  linearControl;

    /// PID controller for angular axis 1
    ///
    /// <b>Access type:</b> inputOutput
    /// 
    /// \dotfile JointPID_angularControl1.dot
    auto_ptr< SFPIDController >  angularControl1;

    /// PID controller for angular axis 2
    ///
    /// <b>Access type:</b> inputOutput
    /// 
    /// \dotfile JointPID_angularControl2.dot
    auto_ptr< SFPIDController >  angularControl2;

    /// The joint to control
    ///
    /// <b>Access type:</b> inputOutput
    /// 
    /// \dotfile JointPID_joint.dot
    auto_ptr< SFJointNode >  joint;

    /// Threshold for when to send forces and torques to body if body is 
    /// disabled (sleeping). If body is sleeping the PIDController error of
    /// the controller needs to be higher than this threshold for the force  
    /// to be applied to the body. This is to avoid making the JointPID 
    /// wake up the body unnecessarily.
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value: </b> 0
    /// 
    /// \dotfile JointPID_errorSleepThreshold.dot
    auto_ptr< SFFloat > errorSleepThreshold;

    /// If true, then set the velocity target for the joint motor, instead of applying
    /// external forces and torques to the bodies.
    ///
    /// This can be more stable, but is not supported for all engines
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value: </b> false
    /// 
    /// \dotfile JointPID_useJointMotor.dot
    auto_ptr < SFBool > useJointMotor;

    /// By Default forces are applied to body1, This will switch it from body1 to body2
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value: </b> false
    /// 
    /// \dotfile JointPID_switchForcesToBody2.dot
    auto_ptr < SFBool > switchForcesToBody2;

    /// By Default torque is applied to the body( except the SliderJoint ). If set to True,
    /// instead of applying a torque, an equivalent force will be applied to the body.
    /// This field has no effect for sliding joints.
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value: </b> false
    /// 
    /// \dotfile JointPID_applyTorqueAsForce.dot
    auto_ptr < SFBool > applyTorqueAsForce;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:

    /// Snapshot of errorSleepThreshold field value to be used in other threads than main.
    H3DFloat rt_error_sleep_threshold;

    /// Snapshot of useJointMotor field value to be used in other threads than main.
    bool rt_use_joint_motor;

    struct JointType {
      enum e { SingleAxisHinge, DoubleAxisHinge, Slider, Unsupported };
    };

    struct ControlType {
      enum e { Linear, Angular };
    };

    struct AxisType {
      enum e { Axis1, Axis2 };
    };

    /// Initializes for the given PhysicsEngineThread.
    virtual void initialize( PhysicsEngineThread& pt );

    PhysicsEngineParameters::JointParameters* createJointParameters();

    H3DFloat getValue( PhysicsEngineParameters::JointParameters& _joint, ControlType::e control_type, AxisType::e axis_type );

    H3DFloat getVelocity( PhysicsEngineParameters::JointParameters& _joint, ControlType::e control_type, AxisType::e axis_type );

    H3DFloat getMinTarget( PhysicsEngineParameters::JointParameters& _joint, ControlType::e control_type, AxisType::e axis_type );

    H3DFloat getMaxTarget( PhysicsEngineParameters::JointParameters& _joint, ControlType::e control_type, AxisType::e axis_type );

    Vec3f getAxis( PhysicsEngineParameters::JointParameters& _joint, ControlType::e control_type, AxisType::e axis_type );

    Vec3f getAnchorPoint( PhysicsEngineParameters::JointParameters& _joint, ControlType::e control_type, AxisType::e axis_type, bool apply_to_body1 = true );

    Vec3f doPIDControl(
      PIDController& pid,
      PhysicsEngineParameters::RigidBodyParameters& rigid_body1,
      PhysicsEngineParameters::RigidBodyParameters& rigid_body2,
      PhysicsEngineParameters::JointParameters& _joint,
      ControlType::e control_type,
      AxisType::e axis_type,
      bool use_joint_motor,
      bool apply_to_body1 = true,
      bool convert_torque_to_force = false );

    JointType::e joint_type;

    H3DBodyId body_id1;
    H3DBodyId body_id2;
    H3DConstraintId joint_id;

    PIDController* linear_PID;

    PIDController* angular_PID1;
    PIDController* angular_PID2;

    /// Is the joint fixed to space?
    /// I.e. Contains only 1 body
    bool fixed;

    /// Initial orientation of body 1
    Rotation initial_orientation1;

    /// Initial orientation of body 2
    Rotation initial_orientation2;

    /// Snapshot of switchForcesToBody2 field value to be used in other threads than main.
    bool rt_switch_force_to_body2;

    /// Snapshot of applyTorqueAsForce field value to be used in other threads than main.
    bool rt_apply_torque_as_force;

  };
}

#endif
