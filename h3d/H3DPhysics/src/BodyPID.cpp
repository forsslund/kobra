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
/// \file BodyPID.cpp
/// \brief Source file for BodyPID, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////
#include <H3D/H3DPhysics/BodyPID.h>

#include <fstream>

using namespace H3D;
using namespace PhysicsEngineParameters;

H3DNodeDatabase BodyPID::database( "BodyPID",
                                   &newInstance<BodyPID>,
                                   typeid(BodyPID),
                                   &H3DPIDNode::database );

namespace BodyPIDInternals {
  FIELDDB_ELEMENT( BodyPID, targetPosition, INPUT_OUTPUT )
  FIELDDB_ELEMENT( BodyPID, targetOrientation, INPUT_OUTPUT )

  FIELDDB_ELEMENT( BodyPID, linearControl1, INPUT_OUTPUT )
  FIELDDB_ELEMENT( BodyPID, linearControl2, INPUT_OUTPUT )
  FIELDDB_ELEMENT( BodyPID, linearControl3, INPUT_OUTPUT )

  FIELDDB_ELEMENT( BodyPID, angularControl1, INPUT_OUTPUT )
  FIELDDB_ELEMENT( BodyPID, angularControl2, INPUT_OUTPUT )
  FIELDDB_ELEMENT( BodyPID, angularControl3, INPUT_OUTPUT )

  FIELDDB_ELEMENT( BodyPID, body, INPUT_OUTPUT )
  FIELDDB_ELEMENT( BodyPID, writeToLog, INPUT_OUTPUT )
}

/// Constructor
BodyPID::BodyPID(
  Inst< SFNode          > _metadata,
  Inst< TSSFVec3f       > _targetPosition,
  Inst< TSSFRotation    > _targetOrientation,

  Inst< SFPIDController > _linearControl1,
  Inst< SFPIDController > _linearControl2,
  Inst< SFPIDController > _linearControl3,

  Inst< SFPIDController > _angularControl1,
  Inst< SFPIDController > _angularControl2,
  Inst< SFPIDController > _angularControl3,

  Inst< SFBody          > _body,
  Inst< SFString    >  _writeToLog ) :
  H3DPIDNode( _metadata ),
  targetPosition( _targetPosition ),
  targetOrientation( _targetOrientation ),
  linearControl1( _linearControl1 ),
  linearControl2( _linearControl2 ),
  linearControl3( _linearControl3 ),
  angularControl1( _angularControl1 ),
  angularControl2( _angularControl2 ),
  angularControl3( _angularControl3 ),
  body( _body ),
  writeToLog( _writeToLog ),
  start_log_time( -1 ) {

  type_name = "BodyPID";
  database.initFields( this );
  linear_PID1 = NULL;
  linear_PID2 = NULL;
  linear_PID3 = NULL;

  angular_PID1 = NULL;
  angular_PID2 = NULL;
  angular_PID3 = NULL;
}

void BodyPID::traverseSG( TraverseInfo &ti ) {
  H3DPIDNode::traverseSG( ti );

  string tmp_write_to_log = writeToLog->getValue();
  pid_lock.lock();
  write_to_log = tmp_write_to_log;
  pid_lock.unlock();

  PhysicsEngineThread *pt;
  // obtain the physics thread
  ti.getUserData( "PhysicsEngine", (void * *)&pt );

  if( pt && pt == engine_thread ) {
    if( linear_PID1 ) {
      linear_PID1->traverseSG( ti );
    }
    if( linear_PID2 ) {
      linear_PID2->traverseSG( ti );
    }
    if( linear_PID3 ) {
      linear_PID3->traverseSG( ti );
    }

    if( angular_PID1 ) {
      angular_PID1->traverseSG( ti );
    }
    if( angular_PID2 ) {
      angular_PID2->traverseSG( ti );
    }
    if( angular_PID3 ) {
      angular_PID3->traverseSG( ti );
    }
  }
}

void BodyPID::updateActuation() {
  if( engine_thread ) {
    // Get current body state
    RigidBodyParameters* body_params = new RigidBodyParameters;

    // Get the params for body from physics engine
    engine_thread->getRigidBodyParameters( body_id, *body_params );

    // Accumulate control force and torque
    Vec3f force, torque;

    // Linear PIDs
    Vec3f position_target = targetPosition->getValueRT();
    if( linear_PID1 ) {
      linear_PID1->target->setValue( position_target.x );
      force.x = linear_PID1->doControl(
        body_params->getPosition().x, body_params->getLinearVelocity().x, false /*don't wrap angles*/ );
    }
    if( linear_PID2 ) {
      linear_PID2->target->setValue( position_target.y );
      force.y = linear_PID2->doControl(
        body_params->getPosition().y, body_params->getLinearVelocity().y, false /*don't wrap angles*/ );
    }
    if( linear_PID3 ) {
      linear_PID3->target->setValue( position_target.z );
      force.z = linear_PID3->doControl(
        body_params->getPosition().z, body_params->getLinearVelocity().z, false /*don't wrap angles*/ );
    }

    // Angular PIDs
    Vec3f vel = body_params->getAngularVelocity();

    // Control using the error rather than the target to avoid issues
    // with euler angles continuity
    H3DUtil::Quaternion orientation_target( targetOrientation->getValueRT() );
    H3DUtil::Quaternion cur( body_params->getOrientation() );
    H3DUtil::Quaternion error = (cur*orientation_target.inverse()).inverse();
    Vec3f error_angles = error.toEulerAngles();

    if( angular_PID1 ) {
      angular_PID1->target->setValue( error_angles.x );
      torque.x = angular_PID1->doControl( 0, vel.x, false /*don't wrap angles*/ );
    }
    if( angular_PID2 ) {
      angular_PID2->target->setValue( error_angles.y );
      torque.y = angular_PID2->doControl( 0, vel.y, false /*don't wrap angles*/ );
    }
    if( angular_PID3 ) {
      angular_PID3->target->setValue( error_angles.z );
      torque.z = angular_PID3->doControl( 0, vel.z, false /*don't wrap angles*/ );
    }

    string tmp_write_to_log;
    pid_lock.lock();
    tmp_write_to_log = write_to_log;
    pid_lock.unlock();

    if( tmp_write_to_log != "" ) {
      H3DTime curTime = TimeStamp();
      ofstream f( tmp_write_to_log.c_str(), std::ofstream::app );
      if( start_log_time < 0 ) {
        start_log_time = curTime;
      }

      Vec3f cur_angles = cur.toEulerAngles();
      Vec3f tar_angles = orientation_target.toEulerAngles();

      // Scale and offset in the target are not considered
      f << (curTime - start_log_time) << "," << linear_PID1->target->getValueRT() << "," << linear_PID2->target->getValueRT() << "," << linear_PID3->target->getValueRT() << "," <<
        body_params->getPosition().x << "," << body_params->getPosition().y << "," << body_params->getPosition().z << "," <<
        tar_angles.x << "," << tar_angles.y << "," << tar_angles.z << "," <<
        cur_angles.x << "," << cur_angles.y << "," << cur_angles.z << "," << endl;

    }

    // Apply force and torque to body
    body_params->setForce( body_params->getForce() + force );
    body_params->setTorque( body_params->getTorque() + torque );
    engine_thread->setRigidBodyParameters( body_id, *body_params );
  }
}

void BodyPID::initialize( PhysicsEngineThread& pt ) {
  if( !engine_thread ) {

    RigidBody* b = body->getValue();
    if( b && b->isInitialized() ) {
      body_id = b->getBodyId();

      // handle the case where the targetPosition is set before the init.
      Vec3f tp = targetPosition->getValue();
      linearControl1->getValue()->target->setValue( tp.x );
      linearControl2->getValue()->target->setValue( tp.y );
      linearControl3->getValue()->target->setValue( tp.z );

      linear_PID1 = linearControl1->getValue();
      linear_PID2 = linearControl2->getValue();
      linear_PID3 = linearControl3->getValue();

      angular_PID1 = angularControl1->getValue();
      angular_PID2 = angularControl2->getValue();
      angular_PID3 = angularControl3->getValue();

      engine_thread = &pt;
    }

  }
}

