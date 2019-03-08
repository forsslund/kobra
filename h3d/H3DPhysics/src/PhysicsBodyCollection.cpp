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
/// \file PhysicsBodyCollection.cpp
/// \brief Source file for PhysicsBodyCollection, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/H3DPhysics/PhysicsBodyCollection.h>
#include <H3D/H3DPhysics/SoftBodyPhysicsEngineThread.h>
#include <H3D/DeviceInfo.h>

using namespace H3D;

H3DNodeDatabase PhysicsBodyCollection::database( "PhysicsBodyCollection", 
                                                &(newInstance< PhysicsBodyCollection >), 
                                                typeid( PhysicsBodyCollection ),
                                                &RigidBodyCollection::database);

namespace PhysicsBodyCollectionInternals {
  // Make an alias rigidBodies for bodies
  FieldDBInsert string_rigidBodies( INPUT_OUTPUT( &PhysicsBodyCollection::database, "rigidBodies", &RigidBodyCollection::bodies ) );
  FIELDDB_ELEMENT( PhysicsBodyCollection, softBodies, INPUT_OUTPUT )
  // Make an alias constraints for joints
  FieldDBInsert string_constraints( INPUT_OUTPUT( &PhysicsBodyCollection::database, "constraints", &RigidBodyCollection::joints ) );
  FIELDDB_ELEMENT( PhysicsBodyCollection, modifiers, INPUT_OUTPUT )
}

PhysicsBodyCollection::PhysicsBodyCollection (
  Inst< MFNode  > _set_contacts,
  Inst< SFBool  > _autoDisable,
  Inst< SFFloat > _constantForceMix,
  Inst< SFFloat > _contactSurfaceThickness ,
  Inst< SFFloat > _disableAngularSpeed,
  Inst< SFFloat > _disableLinearSpeed,
  Inst< SFFloat > _disableTime,
  Inst< SFBool  > _enabled,
  Inst< SFFloat > _errorCorrection,
  Inst< SFVec3f > _gravity,
  Inst< SFInt32 > _iterations,
  Inst< SFFloat > _maxCorrectionSpeed,
  Inst< SFNode  > _metadata,
  Inst< SFBool  > _preferAccuracy,
  Inst< SFCollisionCollection  > _collider,
  Inst< ValueUpdater > _valueUpdater,
  Inst< EnableDisable > _enableDisable,
  Inst< SFString > _physicsEngine,
  Inst< SFInt32 > _desiredUpdateRate,
  Inst< SFInt32 > _updateRate,
  Inst< SFTime  > _stepUpdateTime,
  Inst< EnableUseMainThread  > _useMainThread,
  Inst< MFEngineOptions > _engineOptions,
  Inst< MFRigidBody > _bodies,
  Inst< MFPhysicsBodyConstraint > _joints,
  Inst< MFSoftBody > _softBodies,
  Inst< MFBodyModifier  > _modifiers ) :
RigidBodyCollection( _metadata, _set_contacts, _autoDisable, _bodies, _constantForceMix,
                         _contactSurfaceThickness,_disableAngularSpeed, _disableLinearSpeed, _disableTime,
                         _enabled, _errorCorrection, _gravity, _iterations, _joints, _maxCorrectionSpeed,
                         _preferAccuracy, _collider, _valueUpdater, _enableDisable, _physicsEngine,
                         _desiredUpdateRate, _updateRate, _stepUpdateTime, _useMainThread, _engineOptions ),
                         rigidBodies( _bodies ),
                         softBodies ( _softBodies ),
                         constraints( _joints ),
                         modifiers( _modifiers )
{
  // init fields
  type_name = "PhysicsBodyCollection";
  database.initFields( this );

  vector< string > physics_engine_names =
    SoftBodyPhysicsEngineThread::getSupportedPhysicsEngineNames();
  physicsEngine->clearValidValues();
  physicsEngine->addValidValues( physics_engine_names.begin(),
                                 physics_engine_names.end() );
  if( physicsEngine->isValidValue( "Bullet" ) )
    physicsEngine->setValue ( "Bullet" );
  else if( !physics_engine_names.empty() )
    physicsEngine->setValue ( physics_engine_names.front() );
}

PhysicsBodyCollection::~PhysicsBodyCollection() {
  if( simulationThread.get() ) {
    simulationThread->stopSimulation();
    constraints->clear();
    modifiers->clear();
    rigidBodies->clear();
    softBodies->clear();    
    collider.reset(NULL); 
  }
}

void PhysicsBodyCollection::initialize() {
  // Don't call RigidBodyCollection::initialize(); since it does the same thing
  // as the code below but for rigid bodies.
  X3DChildNode::initialize();

  const string &engine = physicsEngine->getValue();

  if( SoftBodyPhysicsEngineThread::supportsPhysicsEngine( engine ) ) {
    bool use_synchronization =
      syncGraphicsFrames->getValue() > 0 &&
      syncPhysicsFrames->getValue() > 0;

    simulationThread.reset( 
      new SoftBodyPhysicsEngineThread( engine, 
      PeriodicThreadBase::NORMAL_PRIORITY, 
      desiredUpdateRate->getValue(),
      useMainThread->getValue(),
      use_synchronization ) );
      simulationThread->setThreadName( "PhysicsBodyCollection " + engine + " engine thread" );
  } else {
    Console( 4 ) << "Warning: Unsupported physics engine "
                 << "specified in PhysicsBodyCollection node: "
                 << engine;
    set< string > physics_engine_names = physicsEngine->getValidValues();
    string physics_engine_to_use = "";
    if( physics_engine_names.size() > 0 ) {
      Console(4) << ". This build of H3DPhysics supports the following "
                 << "physics engines for PhysicsBodyCollection:" << endl;
      unsigned int j = 0;
      for( set< string >::iterator i = physics_engine_names.begin();
           i != physics_engine_names.end(); ++i, ++j ) {
        Console(4) << "\"" << *i << "\"";
        if( j < physics_engine_names.size() - 1 )
           Console(4) << ", ";
      }
      Console(4) << "." << endl;
      if( physicsEngine->isValidValue( "Bullet" ) ) {
        physics_engine_to_use = "Bullet";
      } else {
        physics_engine_to_use = *physics_engine_names.begin();
      }
    }

    if( physics_engine_to_use != "" ) {
      Console( 4 ) << "Using \"" << physics_engine_to_use << "\" instead.";
      simulationThread.reset(
        new SoftBodyPhysicsEngineThread( physics_engine_to_use,
        PeriodicThreadBase::NORMAL_PRIORITY,
        desiredUpdateRate->getValue(),
        useMainThread->getValue() ) );
      simulationThread->setThreadName( "PhysicsBodyCollection " + physics_engine_to_use + " engine thread" );
    } else {
      Console ( 4 ) << "Error: No supported physics engine!" << endl;
    }
    Console( 4 ) << endl;
  }
}

void PhysicsBodyCollection::traverseSimulation( TraverseInfo & ti ) {
  // update list of current devices to use to apply interaction force to bodies
  if( DeviceInfo* di = DeviceInfo::getActive() ) {
    simulationThread->setHapticsDevices( di->device->getValue() );
  }

  valueUpdater->upToDate();

  // traverse collider
  CollisionCollection *cc = collider->getValue();
  if( cc ) cc->traverseSG( ti );

  // Soft bodies must be traversed before joints which
  // might reference them
  const NodeVector &sb = softBodies->getValue();
  for( unsigned int i = 0; i < sb.size(); ++i ) {
    if( sb[i] ) sb[i]->traverseSG( ti );
  }

  // traverse rigid bodies
  const NodeVector &rb = rigidBodies->getValue();
  for( unsigned int i = 0; i < rb.size(); ++i ) {
    if( rb[i] ) rb[i]->traverseSG( ti );
  }

  // traverse all constraints
  // constraints use both rigid and soft bodies, so
  // must be traversed after both of those to ensure
  // correct initialization order
  const NodeVector &c = constraints->getValue();
  for( unsigned int i = 0; i < c.size(); ++i ) {
    if( c[i] ) c[i]->traverseSG( ti );
  }

  // traverse all modifiers
  // constraints use both rigid and soft bodies, so
  // must be traversed after both of those to ensure
  // correct initialization order
  const NodeVector &m = modifiers->getValue();
  for( unsigned int i = 0; i < m.size(); ++i ) {
    if( m[i] ) m[i]->traverseSG( ti );
  }
}

void PhysicsBodyCollection::MFSoftBody::onRemove( Node * n ) {
  PhysicsBodyCollection *srbc = 
    static_cast< PhysicsBodyCollection * >( getOwner() );
  if( srbc->simulationThread.get() ) {
    H3DSoftBodyNode* sb = static_cast< H3DSoftBodyNode * >( n );
    sb->deleteBody();
  }
  TypedMFNode< H3DSoftBodyNode >::onRemove( n );
}

void PhysicsBodyCollection::MFBodyModifier::onRemove( Node * n ) {
  PhysicsBodyCollection *srbc = 
    static_cast< PhysicsBodyCollection * >( getOwner() );
  if( srbc->simulationThread.get() ) {
    H3DBodyModifierNode* a = static_cast< H3DBodyModifierNode * >( n );
    a->deleteModifier();
  }
  TypedMFNode< H3DBodyModifierNode >::onRemove( n );
}
