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
/// \file SoftBodyPhysicsEngineThread.cpp
/// \brief cpp file for SoftBodyPhysicsEngineThread.
///
//
//////////////////////////////////////////////////////////////////////////////


#include <H3D/H3DPhysics/SoftBodyPhysicsEngineThread.h>
#include <H3D/H3DPhysics/H3DBodyModifierNode.h>
#include <H3D/H3DPhysics/H3DDeviceSoftBodyModifierNode.h>
#include <H3D/DeviceInfo.h>
#include <HAPI/HAPIHapticsDevice.h>

using namespace H3D;

SoftBodyPhysicsEngineThread::SoftBodyPhysicsEngineThread( 
  const string & _engine,
  Priority thread_priority,
  int thread_frequency,
  bool _useMainThread,
  bool _use_synchronization ):
  PhysicsEngineThread( _engine, thread_priority, thread_frequency, _useMainThread, _use_synchronization ) {
}

H3DBodyId SoftBodyPhysicsEngineThread::addSoftBody( H3DSoftBodyNodeParameters& p ) {
  p.setEngine ( *this );

  synchronousCallback( callbacks()->addSoftBody, &p );
  if( p.getBodyId() != 0 ) {
    soft_body_lock.lock();
    soft_bodies[ p.getBodyId() ] = &p;
    soft_body_lock.unlock();
  }

  return p.getBodyId();
}

bool SoftBodyPhysicsEngineThread::removeSoftBody( H3DBodyId body ) {

  soft_body_lock.lock();
  SoftBodyMap::iterator i = soft_bodies.find( body );
  bool have_body = i != soft_bodies.end();
  soft_body_lock.unlock();

  if( have_body ) {
    soft_body_lock.lock();
    H3DSoftBodyNodeParameters* params= (*i).second;
    soft_bodies.erase( i );
    soft_body_lock.unlock();
    synchronousCallback( callbacks()->removeSoftBody, params );
    delete params;
    return true;
  } else {
    return false;
  }
}

bool SoftBodyPhysicsEngineThread::setSoftBodyParameters( H3DBodyId body, 
                                                        H3DSoftBodyNodeParameters& p)
{
  p.setBodyId ( body );
  p.setEngine ( *this );
  soft_body_lock.lock();
  soft_bodies[body]->copyInputParameters( p );
  soft_body_lock.unlock();

  asynchronousCallback( callbacks()->setSoftBodyParameters, &p );

  return true;
}

bool SoftBodyPhysicsEngineThread::getSoftBodyParameters( H3DBodyId body,
                                                        H3DSoftBodyNodeParameters& params )
{
  soft_body_lock.lock();
  H3DSoftBodyNodeParameters& engine_params= *soft_bodies[body];
  params.copyOutputParameters( engine_params );
  params.copyInputParameters( engine_params );
  engine_params.onGetParametersGraphicsThread ();
  soft_body_lock.unlock();
  return true;
}
H3DModifierId SoftBodyPhysicsEngineThread::addModifier( ModifierParameters& params ) {
  params.setEngine( *this );

  modifier_lock.lock();
  modifiers[ params.getModifierId() ] = &params;
  modifier_lock.unlock();

  return params.getModifierId();
}

bool SoftBodyPhysicsEngineThread::removeModifier( H3DModifierId modifier ) {
  modifier_lock.lock();
  ModifierMap::iterator i = modifiers.find( modifier );
  bool have_modifier = i != modifiers.end();
  modifier_lock.unlock();

  if( have_modifier ) {

    modifier_lock.lock();
    H3DBodyId sbId = (*i).second->getBody1();
    soft_body_lock.lock();
    SoftBodyMap::iterator j = soft_bodies.find( sbId );
    bool have_body = j != soft_bodies.end();
    soft_body_lock.unlock();

    if( have_body ) {
      soft_body_lock.lock();
      H3DSoftBodyNodeParameters* params= (*j).second;
      params->clearVertexForces( (*i).first );
      soft_body_lock.unlock();
    }

    ModifierParameters* params= (*i).second;
    modifiers.erase( i );
    modifier_lock.unlock();
    delete params;
    return true;
  } else {
    // modifier id does not exist, error
    return false;
  }
}
bool SoftBodyPhysicsEngineThread::setModifierParameters( H3DModifierId modifier,
                                                        ModifierParameters& params )
{
  params.setModifierId ( modifier );
  params.setEngine ( *this );
  modifier_lock.lock();
  modifiers[modifier]->copyInputParameters( params );
  modifier_lock.unlock();

  return true;
}

bool SoftBodyPhysicsEngineThread::getModifierParameters( H3DModifierId modifier,
                                                        ModifierParameters& params )
{
  modifier_lock.lock();
  //params.copyOutputParameters( *modifiers[modifier] );
  params.copyInputParameters( *modifiers[modifier] );
  modifier_lock.unlock();
  return true;
}

bool SoftBodyPhysicsEngineThread::supportsPhysicsEngine( const string &engine ) {
  return registered_softbody_physics_engines().find( engine ) != 
    registered_softbody_physics_engines().end();
}

void SoftBodyPhysicsEngineThread::updateSimulation () {
  applyManipulationForces();
  applyExternalForces();
  updateSoftBodyParameters();
  clearAllForces();

  PhysicsEngineThread::updateSimulation();
}

void SoftBodyPhysicsEngineThread::updateSoftBodyParameters() {
  soft_body_lock.lock();
  for( SoftBodyMap::iterator i = soft_bodies.begin();
       i != soft_bodies.end(); ++i ) {
    H3DSoftBodyNodeParameters* params= (*i).second;
    callbacks()->getSoftBodyParameters( params );
    params->onGetParametersPhysicsThread ();
  }
  soft_body_lock.unlock();
}


// Apply forces from haptic interaction with soft bodies
void SoftBodyPhysicsEngineThread::applyManipulationForces () {

  // WARNINGUMUT: The implementation of applyManipulationForces()
  // is a little different than the previous version. Not sure if
  // this way would have an effect on the performance. Instead of
  // locking and unlocking the soft_body_lock in the beginning and
  // end of the function there are several lcokings in this version.
  // modifier_lock is also used in the same function and in
  // H3DBodyModifierNode::calculateForces() function there is a 
  // traverseLock being used. Havent observed a performance difference
  // but worth to be aware that this function deals with several locks.

  std::map<H3DBodyId,int> deviceModifierPerBody;
  modifier_lock.lock();
  for( ModifierMap::iterator i = modifiers.begin(); i != modifiers.end(); ++i ) {

    H3DBodyId sbId = (*i).second->getBody1();

    // Add the force applied by each device for each soft body
    H3DInt32 index= (*i).second->getDeviceIndex();
    if ( index == -1 ) {
      H3DBodyModifierNode* dm = (H3DBodyModifierNode*)((*i).first);
      if( dynamic_cast< H3DDeviceSoftBodyModifierNode* > (dm) )
      {
        index = deviceModifierPerBody[sbId];
        ++(deviceModifierPerBody[sbId]);
      }
      else
        index = 0;

    }
    if ( index >= 0 && index < (H3DInt32)modifiers.size() ) {

      soft_body_lock.lock();
      SoftBodyMap::iterator j = soft_bodies.find( sbId );
      bool have_body = j != soft_bodies.end();
      soft_body_lock.unlock();

      if( have_body ) {

        soft_body_lock.lock();
        H3DSoftBodyNodeParameters* params= (*j).second;
        DeviceInfo* di= DeviceInfo::getActive();
        if ( di ) {
          const NodeVector& d= di->device->getValue();
          if( index < (H3DInt32)d.size() ) {
            HAPI::HAPIHapticsDevice* hapi_device =
              ( static_cast<H3DHapticsDevice*>(d[index]) )->getHAPIDevice();
            H3DBodyModifierNode* m = (H3DBodyModifierNode*)((*i).first);
            if( m ) m->calculateForces ( *params, *hapi_device );
          }
        } 
        soft_body_lock.unlock();
      }      
    }
  }
  modifier_lock.unlock();
}

// Apply external forces to soft bodies
void SoftBodyPhysicsEngineThread::applyExternalForces () {
  soft_body_lock.lock();
  for( SoftBodyMap::iterator i = soft_bodies.begin();
    i != soft_bodies.end(); ++i ) {
      callbacks()->applyExternalForces( (*i).second );   
  }
  soft_body_lock.unlock();
}

void SoftBodyPhysicsEngineThread::clearAllForces () {
  soft_body_lock.lock();
  for( SoftBodyMap::iterator i = soft_bodies.begin();
    i != soft_bodies.end(); ++i ) {
      (*i).second->clearManipulationForces ();      
  }
  soft_body_lock.unlock();
}

vector< string > SoftBodyPhysicsEngineThread::getSupportedPhysicsEngineNames() {
  return registered_softbody_physics_engines().getAllKeys();
}
