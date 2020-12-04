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
/// \file BodyPID.h
/// \brief Header file for BodyPID, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __BODYPID__
#define __BODYPID__

#include <H3D/H3DPhysics/H3DPIDNode.h>
#include <H3D/H3DPhysics/RigidBody.h>
#include <H3D/H3DPhysics/PIDController.h>

namespace H3D {

  /// \class BodyPID
  /// This class can be used to smoothly drive a physics object to a certain
  /// position and/or orientation using a PID control loop.
  ///
  /// By setting the physics object target position and/or orientation, the
  /// PIDControllers iteratively drives these values so that the error between
  /// the desired and current positions and/or orientations is minimized.
  /// That way, the body reaches its target smoothly while interacting with the
  /// rest of the physics world and not interfering with the collision response.
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../examples/RigidBody/BodyPID.x3d">BodyPID.x3d</a>
  ///     ( <a href="examples/BodyPID.x3d.html">Source</a> )
  ///
  /// \par Internal routes:
  /// \dotfile BodyPID.dot
  class H3DPHYS_API BodyPID : public H3DPIDNode {
  public:
    typedef TypedSFNode < RigidBody > SFBody;
    typedef TypedSFNode < PIDController > SFPIDController;

    typedef ThreadedField < SFVec3f > TSSFVec3f;
    typedef ThreadedField < SFRotation > TSSFRotation;

    /// Constructor.
    BodyPID(
      Inst< SFNode       > _metadata = 0,
      Inst< TSSFVec3f    > _targetPosition = 0,
      Inst< TSSFRotation > _targetOrientation = 0,

      Inst< SFPIDController > _linearControl1 = 0,
      Inst< SFPIDController > _linearControl2 = 0,
      Inst< SFPIDController > _linearControl3 = 0,

      Inst< SFPIDController > _angularControl1 = 0,
      Inst< SFPIDController > _angularControl2 = 0,
      Inst< SFPIDController > _angularControl3 = 0,

      Inst< SFBody          > _body = 0,
      Inst< SFString    >  _writeToLog = 0 );

    /// Traverse the scene graph.
    virtual void traverseSG( TraverseInfo &ti );

    /// Calculates the new PID output to update the actuation: Force/Torque
    virtual void updateActuation();

    /// Controls the target position which is the position the
    /// body should try to reach.
    ///
    /// <b>Access type:</b> inputOutput
    /// 
    /// \dotfile BodyPID_targetPosition.dot
    auto_ptr < TSSFVec3f > targetPosition;

    /// Controls the target orientation which is the orientation the
    /// body should try to reach.
    ///
    /// <b>Access type:</b> inputOutput
    /// 
    /// \dotfile BodyPID_targetOrientation.dot
    auto_ptr < TSSFRotation > targetOrientation;

    /// The PIDController for the x axis for position.
    ///
    /// <b>Access type:</b> inputOutput
    /// 
    /// \dotfile BodyPID_linearControl1.dot
    auto_ptr < SFPIDController > linearControl1;

    /// The PIDController for the y axis for position.
    ///
    /// <b>Access type:</b> inputOutput
    /// 
    /// \dotfile BodyPID_linearControl2.dot
    auto_ptr < SFPIDController > linearControl2;

    /// The PIDController for the z axis for position.
    ///
    /// <b>Access type:</b> inputOutput
    /// 
    /// \dotfile BodyPID_linearControl3.dot
    auto_ptr < SFPIDController > linearControl3;

    /// The PIDController for yaw angles of the orientation.
    /// See function Quaternion::toEulerAngles
    ///
    /// <b>Access type:</b> inputOutput
    /// 
    /// \dotfile BodyPID_angularControl1.dot
    auto_ptr < SFPIDController > angularControl1;

    /// The PIDController for pitch angles of the orientation.
    /// See function Quaternion::toEulerAngles
    ///
    /// <b>Access type:</b> inputOutput
    /// 
    /// \dotfile BodyPID_angularControl2.dot
    auto_ptr < SFPIDController > angularControl2;

    /// The PIDController for roll angles of the orientation.
    /// See function Quaternion::toEulerAngles
    ///
    /// <b>Access type:</b> inputOutput
    /// 
    /// \dotfile BodyPID_angularControl3.dot
    auto_ptr < SFPIDController > angularControl3;

    /// The body controlled by this node.
    ///
    /// <b>Access type:</b> inputOutput
    /// 
    /// \dotfile BodyPID_body.dot
    auto_ptr < SFBody > body;

    /// If not empty then some log data is written to this file.
    /// Used for debugging purposes.
    ///
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> ""
    /// 
    /// \dotfile BodyPID_writeToLog.dot
    auto_ptr < SFString > writeToLog;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:

    /// Initializes for the given PhysicsEngineThread.
    virtual void initialize( PhysicsEngineThread& pt );

    H3DBodyId body_id;

    // Internal pointers to the nodes in the controller fields.
    PIDController* linear_PID1;
    PIDController* linear_PID2;
    PIDController* linear_PID3;

    PIDController* angular_PID1;
    PIDController* angular_PID2;
    PIDController* angular_PID3;

    // Log variables for thread safe logging.
    H3DTime start_log_time;
    string write_to_log;
    MutexLock pid_lock;
    std::ofstream *log_file;

  };
}

#endif
