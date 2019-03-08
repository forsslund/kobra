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
/// \file PIDCollection.cpp
/// \brief cpp file for PIDCollection, X3D scene-graph node
//
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/H3DPhysics/PIDCollection.h>

using namespace H3D;

H3DNodeDatabase PIDCollection::database( "PIDCollection",
                                         &(newInstance< PIDCollection >),
                                         typeid(PIDCollection),
                                         &X3DChildNode::database );

namespace PIDCollectionInternals {
  FIELDDB_ELEMENT( PIDCollection, enabled, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PIDCollection, pids, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PIDCollection, rbc, INPUT_OUTPUT )
}

PIDCollection::PIDCollection(
  Inst< SFBool    > _enabled,
  Inst< SFNode    > _metadata,
  Inst< MFPIDNode    > _pids,
  Inst< SFRigidBodyCollection > _rbc ) :
  enabled( _enabled ),
  X3DChildNode( _metadata ),
  pids( _pids ),
  rbc( _rbc ),
  engine_thread( NULL ),
  physics_callback_id( -1 ),
  rt_enabled( true ) {

  type_name = "PIDCollection";
  database.initFields( this );

  // set default values
  enabled->setValue( true );

  update_pid_properties.reset( new UpdatePIDProperties );
  update_pid_properties->setName( "update_pid_properties" );
  update_pid_properties->setOwner( this );
  pids->route( update_pid_properties );
  enabled->route( update_pid_properties );
}

PIDCollection::~PIDCollection() {
  if( engine_thread ) {
    // remove callback for the updatePhysics function 
    engine_thread->removeAsynchronousCallback( physics_callback_id );
    engine_thread = NULL;
  }
}

// Initialize node with physics thread 
void PIDCollection::initialize( PhysicsEngineThread& pt ) {
  // We need to have a reference to the RBC to ensure correct destruction order
  if( rbc->getValue() ) {
    engine_thread = &pt;
    // register callback for the updatPhysics function 
    physics_callback_id = engine_thread->asynchronousCallback( PIDCollection::updatePhysics, this );
  }
}

// Implementation of traverseSG 
void PIDCollection::traverseSG( TraverseInfo &ti ) {
  X3DChildNode::traverseSG( ti );
  PhysicsEngineThread *pt;
  // obtain the physics thread
  ti.getUserData( "PhysicsEngine", (void * *)&pt );

  if( pt ) {
    if( !engine_thread ) {
      initialize( *pt );
    }
    if( pt == engine_thread ) {
      // Loop through all the jointPID nodes
      for( MFPIDNode::const_iterator i = pids->begin(); i != pids->end(); i++ ) {
        H3DPIDNode *ptr = static_cast<H3DPIDNode *>(*i);
        if( ptr ) {
          // call the traverseSG function for the child nodes
          ptr->traverseSG( ti );
        }
      }
    }
  }
}

// Callback function to execute all PID loops at physics thread rate
H3DUtil::PeriodicThread::CallbackCode PIDCollection::updatePhysics( void* data ) {
  PIDCollection* pid = static_cast<PIDCollection*>(data);
  if( pid->engine_thread->isSimulationRunning() && pid->rt_enabled ) {
    // Loop through all the jointPID nodes
    pid->rt_pids_lock.lock();
    for( AutoRefVector<H3DPIDNode>::const_iterator i = pid->rt_pids.begin(); i != pid->rt_pids.end(); i++ ) {
      H3DPIDNode* ptr = *i;
      if( ptr ) {
        // call the update function for new actuation value
        ptr->updateActuation();
      }
    }
    pid->rt_pids_lock.unlock();
  }

  return H3DUtil::PeriodicThread::CALLBACK_CONTINUE;
}

void PIDCollection::UpdatePIDProperties::update() {
  PIDCollection* o = static_cast <PIDCollection*> (getOwner());

  o->rt_pids_lock.lock();
  o->rt_pids.resize( o->pids->size() );
  const NodeVector& _pids = o->pids->getValue();
  for( size_t i = 0; i < _pids.size(); i++ ) {
    o->rt_pids.set( i, static_cast <H3DPIDNode*> (_pids[i]) );
  }
  o->rt_enabled = o->enabled->getValue();
  o->rt_pids_lock.unlock();
}
