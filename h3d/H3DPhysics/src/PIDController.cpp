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
/// \file PIDController.cpp
/// \brief Source file for PIDController, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////
#include <H3D/H3DPhysics/PIDController.h>

using namespace H3D;
using namespace PhysicsEngineParameters;

H3DNodeDatabase PIDController::database( "PIDController",
                                         &newInstance<PIDController>,
                                         typeid(PIDController),
                                         &X3DNode::database );

namespace PIDControllerInternals {
  FIELDDB_ELEMENT( PIDController, pidParams, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PIDController, axis, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PIDController, target, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PIDController, targetVelocity, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PIDController, maxRateOfChange, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PIDController, maxActuation, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PIDController, maxAccumulatedError, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PIDController, enabled, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PIDController, useFeedForwardVelocity, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PIDController, currentError, OUTPUT_ONLY )
  FIELDDB_ELEMENT( PIDController, currentActuation, OUTPUT_ONLY )
  FIELDDB_ELEMENT( PIDController, scale, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PIDController, offset, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PIDController, continuousJoint, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PIDController, fixedTimeStep, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PIDController, writeToLog, INPUT_OUTPUT )
}

/// Constructor
PIDController::PIDController(
  Inst< SFNode      > _metadata,
  Inst< SFBool      > _enabled,
  Inst< SFBool      > _useFeedForwardVelocity,
  Inst< SFVec4f     > _pidParams,
  Inst< SFVec3f     > _axis,
  Inst< TSSFFloat   > _target,
  Inst< TSSFFloat   > _targetVelocity,
  Inst< SFFloat     > _maxRateOfChange,
  Inst< SFFloat     > _maxActuation,
  Inst< SFFloat     > _maxAccumulatedError,
  Inst< SFFloat     > _currentError,
  Inst< SFFloat     > _currentActuation,
  Inst< SFFloat     > _scale,
  Inst< SFFloat     > _offset,
  Inst< SFBool      > _continuousJoint,
  Inst< SFFloat     > _fixedTimeStep,
  Inst< SFString    > _writeToLog ) :
  X3DNode( _metadata ),
  enabled( _enabled ),
  useFeedForwardVelocity( _useFeedForwardVelocity ),
  pidParams( _pidParams ),
  axis( _axis ),
  target( _target ),
  targetVelocity( _targetVelocity ),
  maxRateOfChange( _maxRateOfChange ),
  maxActuation( _maxActuation ),
  maxAccumulatedError( _maxAccumulatedError ),
  currentError( _currentError ),
  currentActuation( _currentActuation ),
  scale( _scale ),
  offset( _offset ),
  continuousJoint( _continuousJoint ),
  fixedTimeStep( _fixedTimeStep ),
  writeToLog( _writeToLog ),
  valueUpdater( new ValueUpdater ) {

  type_name = "PIDController";
  database.initFields( this );

  pid.targetField = target.get();
  pid.targetVelocityField = targetVelocity.get();

  valueUpdater->setName( "valueUpdater" );
  valueUpdater->setOwner( this );

  // set default values
  enabled->setValue( true );
  pidParams->setValue( Vec4f( 0, 0, 0, 0 ) );
  axis->setValue( Vec3f( 0, 0, 0 ) );
  target->setValue( (H3DFloat)0 );
  targetVelocity->setValue( (H3DFloat)0 );
  maxRateOfChange->setValue( (H3DFloat)100 );
  maxActuation->setValue( (H3DFloat)1000 );
  maxAccumulatedError->setValue( (H3DFloat)5 );
  currentError->setValue( (H3DFloat)0, id );
  currentActuation->setValue( (H3DFloat)0, id );
  scale->setValue( (H3DFloat)1 );
  offset->setValue( (H3DFloat)0 );
  useFeedForwardVelocity->setValue( true );
  continuousJoint->setValue( false );
  fixedTimeStep->setValue( (H3DFloat)0 );


  enabled->route( valueUpdater );
  pidParams->route( valueUpdater );
  axis->route( valueUpdater );
  target->route( valueUpdater );
  maxRateOfChange->route( valueUpdater );
  maxActuation->route( valueUpdater );
  maxAccumulatedError->route( valueUpdater );
  currentError->route( valueUpdater );
  currentActuation->route( valueUpdater );
  useFeedForwardVelocity->route( valueUpdater );
  targetVelocity->route( valueUpdater );
  scale->route( valueUpdater );
  offset->route( valueUpdater );
  continuousJoint->route( valueUpdater );
  fixedTimeStep->route( valueUpdater );
}

void PIDController::traverseSG( TraverseInfo &ti ) {
  X3DNode::traverseSG( ti );

  string tmp_write_to_log = writeToLog->getValue();

  pid_lock.lock();
  PIDControl tmp = pid;
  pid.write_to_log = tmp_write_to_log;
  pid.log_file = &log_file;
  pid_lock.unlock();

  currentError->setValue( tmp.current_error, id );
  currentActuation->setValue( tmp.current_actuation, id );
}

H3DFloat PIDController::doControl( H3DFloat _current_value, H3DFloat current_velocity, bool wrap_angles, H3DFloat min_target, H3DFloat max_target ) {
  pid_lock.lock();
  H3DFloat actuation = pid.doControl( _current_value, current_velocity, wrap_angles, min_target, max_target );
  pid_lock.unlock();

  return actuation;
}

void PIDController::resetPID() {
  pid_lock.lock();
  pid.reset();
  pid_lock.unlock();

}

Vec3f PIDController::getAxis() {
  pid_lock.lock();
  Vec3f v = pid.axis;
  pid_lock.unlock();

  return v;
}

H3DFloat PIDController::getCurrentError() {
  pid_lock.lock();
  H3DFloat error = pid.current_error;
  pid_lock.unlock();

  return error;
}

H3DFloat PIDController::getCurrentActuation() {
  pid_lock.lock();
  H3DFloat actuation = pid.current_actuation;
  pid_lock.unlock();

  return actuation;
}

void PIDController::ValueUpdater::update() {
  PIDController* node = static_cast<PIDController*>(getOwner());

  node->pid_lock.lock();

  if( hasCausedEvent( node->pidParams ) ) {
    node->pid.pid_params = node->pidParams->getValue();
  }

  if( hasCausedEvent( node->maxRateOfChange ) ) {
    node->pid.max_rate_of_change = node->maxRateOfChange->getValue();
  }

  if( hasCausedEvent( node->maxActuation ) ) {
    node->pid.max_actuation = node->maxActuation->getValue();
  }

  if( hasCausedEvent( node->maxAccumulatedError ) ) {
    node->pid.max_accumulated_error = node->maxAccumulatedError->getValue();
  }

  if( hasCausedEvent( node->axis ) ) {
    node->pid.axis = node->axis->getValue();
    node->pid.axis.normalizeSafe();
  }

  if( hasCausedEvent( node->enabled ) ) {
    node->pid.enabled = node->enabled->getValue();
  }

  if( hasCausedEvent( node->useFeedForwardVelocity ) ) {
    node->pid.use_feed_forward_velocity = node->useFeedForwardVelocity->getValue();
  }

  if( hasCausedEvent( node->scale ) ) {
    node->pid.scale = node->scale->getValue();
  }

  if( hasCausedEvent( node->offset ) ) {
    node->pid.offset = node->offset->getValue();
  }

  if( hasCausedEvent( node->continuousJoint ) ) {
    node->pid.continuous_joint = node->continuousJoint->getValue();
  }

  if( hasCausedEvent( node->fixedTimeStep ) ) {
    node->pid.fixed_time_step = node->fixedTimeStep->getValue();
  }
  node->pid_lock.unlock();

  EventCollectingField < PeriodicUpdate < Field > >::update();
}

PIDController::PIDControl::PIDControl() :
  target( 0 ),
  target_velocity( 0 ),
  scale( 1 ),
  offset( 0 ),
  max_rate_of_change( 0 ),
  max_actuation( 0 ),
  max_accumulated_error( 0 ),
  current_error( 0 ),
  current_actuation( 0 ),
  accumulated_error( 0 ),
  prev_time( 0 ),
  last_value( 0 ),
  wrapped_value( 0 ),
  enabled( true ),
  use_feed_forward_velocity( true ),
  continuous_joint( true ),
  fixed_time_step( 0 ),
  targetField( NULL ),
  targetVelocityField( NULL ),
  start_log_time( -1 ), 
  log_file( NULL ) {

  error[0] = 0;
  error[1] = 0;
}

void PIDController::PIDControl::reset() {
  prev_time = 0;
  accumulated_error = 0;
  error[0] = 0;
  error[1] = 0;
}

H3DFloat PIDController::PIDControl::doControl( H3DFloat _current_value, H3DFloat current_velocity, bool wrap_angles, H3DFloat min_target, H3DFloat max_target ) {
  if( enabled ) {
    // time stamp the entry
    H3DTime cur_time = TimeStamp();

    if( write_to_log != "" && log_file ) {
      if( !log_file->is_open() ) {
        log_file->open( write_to_log.c_str(), std::ofstream::app );
      }
      if( start_log_time < 0 ) {
        start_log_time = cur_time;
      }
      *log_file << (cur_time - start_log_time) << "," << target << "," << _current_value << endl;
    }

    // Get new target values if they are available
    if( targetField && targetVelocityField ) {
      target = scale * targetField->getValueRT() + offset;
      target_velocity = scale * targetVelocityField->getValueRT();
    }

    H3DFloat current_value = _current_value;
    if( wrap_angles ) {

      H3DFloat delta_value = _current_value - last_value;

      if( delta_value > Constants::pi ) {
        // if value is going from -3.14 to 3.14, subtract from wrapped_value
        wrapped_value -= (H3DFloat)Constants::pi + last_value + (H3DFloat)Constants::pi - _current_value;
      } else if( delta_value < -Constants::pi ) {
        // if value is going from 3.14 to -3.14, add to wrapped_value
        wrapped_value += (H3DFloat)Constants::pi + _current_value + (H3DFloat)Constants::pi - last_value;
      } else {
        wrapped_value += delta_value;
      }
      last_value = _current_value;
      current_value = wrapped_value;
    }

    if( !continuous_joint ) {
      if( min_target < max_target ) {
        // check for joint limits
        if( target > max_target )
          target = max_target;
        if( target < min_target )
          target = min_target;
      }
    }

    // update error values
    error[0] = error[1];
    error[1] = target - current_value;
    current_error = error[1];
    // calculate error rate
    float errorRate = 0;
    float dt = 0;
    if( prev_time != 0 ) {
      if( fixed_time_step <= 0 ) {
        // use measured time step
        dt = (float)(cur_time - prev_time);
        if( dt < Constants::f_epsilon ) {
          dt = Constants::f_epsilon;
        }
      } else {
        dt = fixed_time_step;
      }
      errorRate = (float)(1.0*(error[1] - error[0]) / (1.0*dt));
    }
    prev_time = cur_time;

    // calculate error accumulation
    float accError = accumulated_error;
    accError += (float)(0.5*(error[1] + error[0])*dt);

    // cap the error accumulation
    if( accError > max_accumulated_error ) {
      accError = max_accumulated_error;
    }
    if( accError < -max_accumulated_error ) {
      accError = -max_accumulated_error;
    }
    accumulated_error = accError;

    // calculate the pid output
    //                P Term---------------| I Term---------------| D Term---------------|
    float magnitude = pid_params.x*error[1] + pid_params.y*accError + pid_params.z*errorRate;

    // cap the current_velocity
    if( current_velocity > max_rate_of_change && magnitude > 0 ) {
      magnitude = 0;
    }
    if( current_velocity < -max_rate_of_change && magnitude < 0 ) {
      magnitude = 0;
    }


    // cap the torque limit 
    if( magnitude > max_actuation ) {
      magnitude = max_actuation;
    }
    if( magnitude < -max_actuation ) {
      magnitude = -max_actuation;
    }

    current_actuation = magnitude;

    // use feedforward velocity if enabled
    if( use_feed_forward_velocity ) {
      target = target + pid_params.w * target_velocity*dt;
    }

    return current_actuation;
  } else {
    return 0;
  }
}
