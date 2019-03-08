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
/// \file PIDController.h
/// \brief Header file for PIDController, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __PIDCONTROLLER__
#define __PIDCONTROLLER__

#include <H3D/H3DPhysics/RigidBody.h>
#include <H3D/H3DPhysics/H3DRigidBodyJointNode.h>
#include <H3D/H3DPhysics/ThreadedField.h>
#include <fstream>

namespace H3D {

  /// PIDController iteratively drives the values of subclasses of a H3DPIDNode so
  /// that the error between the desired and current positions and/or orientations
  /// is minimized. That way, the H3DPIDNode node reaches its target smoothly while
  /// interacting with the rest of the physics world and not interfering with the
  /// collision response.
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../examples/RigidBody/BodyPID.x3d">BodyPID.x3d</a>
  ///     ( <a href="examples/BodyPID.x3d.html">Source</a> )
  ///   - <a href="../../examples/RigidBody/JointPID.x3d">JointPID.x3d</a>
  ///     ( <a href="examples/JointPID.x3d.html">Source</a> )
  ///
  /// \par Internal routes:
  /// \dotfile PIDController.dot
  class H3DPHYS_API PIDController : public X3DNode {
  public:
    typedef TypedSFNode < H3DRigidBodyJointNode > SFJointNode;

    /// Allow PID target to be set and accessed from separate threads
    typedef ThreadedField < SFFloat > TSSFFloat;

    // Constructor.
    PIDController(
      Inst< SFNode    > _metadata = 0,
      Inst< SFBool    > _enabled = 0,
      Inst< SFBool    > _useFeedForwardVelocity = 0,
      Inst< SFVec4f   > _pidParams = 0,
      Inst< SFVec3f   > _axis = 0,
      Inst< TSSFFloat > _target = 0,
      Inst< TSSFFloat > _targetVelocity = 0,
      Inst< SFFloat   > _maxRateOfChange = 0,
      Inst< SFFloat   > _maxActuation = 0,
      Inst< SFFloat   > _maxAccumulatedError = 0,
      Inst< SFFloat   > _currentError = 0,
      Inst< SFFloat   > _currentActuation = 0,
      Inst< SFFloat   > _scale = 0,
      Inst< SFFloat   > _offset = 0,
      Inst< SFBool    > _continuousJoint = 0,
      Inst< SFFloat   > _fixedTimeStep = 0,
      Inst< SFString  > _writeToLog = 0 );

    /// Traverse the scene graph.
    virtual void traverseSG( TraverseInfo &ti );

    /// Calculates the new PID output to update the actuation: Force/Torque
    virtual H3DFloat doControl( H3DFloat _current_value, H3DFloat current_velocity, bool wrap_angles = true, H3DFloat min_target = 1, H3DFloat max_target = -1 );

    /// Thread safe access to the axis currently being controlled
    Vec3f getAxis();

    /// Thread safe access to the current error, updated at physics thread rate
    H3DFloat getCurrentError();

    /// Thread safe access to the current actuation force, updated at physics thread rate
    H3DFloat getCurrentActuation();

    /// Reset PIDController values such as errors and other values that
    /// could affect the next control loop.
    void resetPID();

    /// Enable PID control
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> true
    /// 
    /// \dotfile PIDController_enabled.dot
    auto_ptr< SFBool > enabled;

    /// Enable feedforward velocity control
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> true
    /// 
    /// \dotfile PIDController_useFeedForwardVelocity.dot
    auto_ptr< SFBool > useFeedForwardVelocity;

    /// PID parameters Kp kd Ki and kf (feedforward term)
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> Vec4f( 0, 0, 0, 0 )
    /// 
    /// \dotfile PIDController_pidParams.dot
    auto_ptr< SFVec4f > pidParams;

    /// axis of the joint, if no value is provided it uses body1's orientation as the axis
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> Vec3f( 0, 0, 0 )
    /// 
    /// \dotfile PIDController_axis.dot
    auto_ptr< SFVec3f > axis;

    /// Target angle or seperation
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 0
    /// 
    /// \dotfile PIDController_target.dot
    auto_ptr< TSSFFloat > target;

    /// Target angle or seperation velocity
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 0
    /// 
    /// \dotfile PIDController_targetVelocity.dot
    auto_ptr< TSSFFloat > targetVelocity;

    /// Limit on joint velocity
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 100
    /// 
    /// \dotfile PIDController_maxRateOfChange.dot
    auto_ptr< SFFloat > maxRateOfChange;

    /// Limit on joint actuation
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 1000
    /// 
    /// \dotfile PIDController_maxActuation.dot
    auto_ptr< SFFloat > maxActuation;

    /// Cap for accumulation error
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 5
    /// 
    /// \dotfile PIDController_maxAccumulatedError.dot
    auto_ptr< SFFloat > maxAccumulatedError;

    /// Current error
    ///
    /// <b>Access type:</b> outputOnly
    ///
    /// \dotfile PIDController_currentError.dot
    auto_ptr< SFFloat > currentError;

    /// Current actuation
    ///
    /// <b>Access type:</b> outputOnly
    ///
    /// \dotfile PIDController_currentActuation.dot
    auto_ptr< SFFloat > currentActuation;

    /// Scale the target
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 1
    /// 
    /// \dotfile PIDController_scale.dot
    auto_ptr< SFFloat > scale;

    /// Offset the scaled target
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 0
    /// 
    /// \dotfile PIDController_offset.dot
    auto_ptr< SFFloat > offset;

    /// Enable continuous joint control
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> false
    /// 
    /// \dotfile PIDController_continuousJoint.dot
    auto_ptr< SFBool > continuousJoint;

    /// The time step to use for integration
    /// 
    /// If <= 0 then the real/measured time elapsed is used
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 0
    /// 
    /// \dotfile PIDController_fixedTimeStep.dot
    auto_ptr< SFFloat > fixedTimeStep;

    /// If not empty then some log data is written to this file.
    /// Used for debugging purposes.
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> ""
    /// 
    /// \dotfile PIDController_writeToLog.dot
    auto_ptr < SFString > writeToLog;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:

    class H3DPHYS_API ValueUpdater : public EventCollectingField < PeriodicUpdate < Field > > {
      virtual void update();
    };

    /// Internal class that contains the actual values in the PID
    /// control loop. Used for thread safety.
    class H3DPHYS_API PIDControl {
    public:
      PIDControl();

      H3DFloat doControl( H3DFloat _current_value, H3DFloat current_velocity, bool wrap_angles, H3DFloat min_target, H3DFloat max_target );
      void reset();
      // Inputs
      Vec4f pid_params;
      // processed target value scale* rawTarget + offset + accumulated feedforward velocity
      H3DFloat target;
      // scaled target velocity
      H3DFloat target_velocity;

      TSSFFloat* targetField;
      TSSFFloat* targetVelocityField;

      H3DFloat scale;
      H3DFloat offset;
      H3DFloat max_rate_of_change;
      H3DFloat max_actuation;
      H3DFloat max_accumulated_error;
      Vec3f axis;
      bool enabled;
      bool use_feed_forward_velocity;
      bool continuous_joint;
      H3DFloat fixed_time_step;

      // Outputs
      H3DFloat current_error;
      H3DFloat current_actuation;

      string write_to_log;
      std::ofstream* log_file;

    protected:
      /// Holds accumulated error between target and actual value
      float accumulated_error;

      /// Holds the latest two error values, used for calculating rate of error 
      float error[2];

      /// Time stamp of previous pid loop call. 
      H3DTime prev_time;

      H3DFloat last_value;
      H3DFloat wrapped_value;

      H3DTime start_log_time;
    };

    /// The valueUpdater field is used to update values in the
    /// SoftBodyPhysicsEngine according to changes of fields in the
    /// PIDController node.
    /// C++ only field.
    ///
    /// \dotfile PIDController_valueUpdater.dot
    auto_ptr < ValueUpdater > valueUpdater;

    PIDControl pid;
    MutexLock pid_lock;

    std::ofstream log_file;
  };
}

#endif
