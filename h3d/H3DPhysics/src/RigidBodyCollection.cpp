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
/// \file RigidBodyCollection.cpp
/// \brief Source file for RigidBodyCollection, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/H3DPhysics/RigidBodyCollection.h>
#include <H3D/DeviceInfo.h>

using namespace H3D;

H3DNodeDatabase RigidBodyCollection::database( "RigidBodyCollection", 
                                              &(newInstance< RigidBodyCollection >), 
                                              typeid( RigidBodyCollection ),
                                              &X3DChildNode::database);

#ifdef USE_PROFILER
H3DTime RigidBodyCollection::lastProfileTime= 0.0;
H3DTime RigidBodyCollection::profileInterval= 5.0;
#endif

namespace RigidBodyCollectionInternals {
  FIELDDB_ELEMENT( RigidBodyCollection, set_contacts           , INPUT_ONLY )
  FIELDDB_ELEMENT( RigidBodyCollection, autoDisable            , INPUT_OUTPUT )
  FIELDDB_ELEMENT( RigidBodyCollection, constantForceMix       , INPUT_OUTPUT )
  FIELDDB_ELEMENT( RigidBodyCollection, contactSurfaceThickness, INPUT_OUTPUT )
  FIELDDB_ELEMENT( RigidBodyCollection, disableAngularSpeed    , INPUT_OUTPUT )
  FIELDDB_ELEMENT( RigidBodyCollection, disableLinearSpeed     , INPUT_OUTPUT )
  FIELDDB_ELEMENT( RigidBodyCollection, disableTime            , INPUT_OUTPUT )
  FIELDDB_ELEMENT( RigidBodyCollection, enabled                , INPUT_OUTPUT )
  FIELDDB_ELEMENT( RigidBodyCollection, errorCorrection        , INPUT_OUTPUT )
  FIELDDB_ELEMENT( RigidBodyCollection, gravity                , INPUT_OUTPUT )
  FIELDDB_ELEMENT( RigidBodyCollection, iterations             , INPUT_OUTPUT )
  FIELDDB_ELEMENT( RigidBodyCollection, maxCorrectionSpeed     , INPUT_OUTPUT )
  FIELDDB_ELEMENT( RigidBodyCollection, preferAccuracy         , INPUT_OUTPUT )
  FIELDDB_ELEMENT( RigidBodyCollection, collider               , INITIALIZE_ONLY )
  FIELDDB_ELEMENT( RigidBodyCollection, physicsEngine          , INITIALIZE_ONLY )
  FIELDDB_ELEMENT( RigidBodyCollection, desiredUpdateRate      , INITIALIZE_ONLY )
  FIELDDB_ELEMENT( RigidBodyCollection, updateRate             , OUTPUT_ONLY )
  FIELDDB_ELEMENT( RigidBodyCollection, stepUpdateTime         , OUTPUT_ONLY )
  FIELDDB_ELEMENT( RigidBodyCollection, useMainThread          , INPUT_OUTPUT )
  FIELDDB_ELEMENT( RigidBodyCollection, engineOptions          , INPUT_OUTPUT )
  FIELDDB_ELEMENT( RigidBodyCollection, bodies, INPUT_OUTPUT )
  FIELDDB_ELEMENT( RigidBodyCollection, joints, INPUT_OUTPUT )
  FIELDDB_ELEMENT( RigidBodyCollection, renderCollidables, INPUT_OUTPUT )
  FIELDDB_ELEMENT( RigidBodyCollection, renderOnlyEnabledCollidables, INPUT_OUTPUT )
  FIELDDB_ELEMENT( RigidBodyCollection, useStaticTimeStep, INPUT_OUTPUT )
  FIELDDB_ELEMENT( RigidBodyCollection, syncGraphicsFrames, INITIALIZE_ONLY )
  FIELDDB_ELEMENT( RigidBodyCollection, syncPhysicsFrames, INITIALIZE_ONLY )
}

RigidBodyCollection::RigidBodyCollection( Inst< SFNode  > _metadata,
                                          Inst< MFNode  > _set_contacts,
                                          Inst< SFBool  > _autoDisable,
                                          Inst< MFRigidBody > _bodies,
                                          Inst< SFFloat > _constantForceMix,
                                          Inst< SFFloat > _contactSurfaceThickness ,
                                          Inst< SFFloat > _disableAngularSpeed,
                                          Inst< SFFloat > _disableLinearSpeed,
                                          Inst< SFFloat > _disableTime,
                                          Inst< SFBool  > _enabled,
                                          Inst< SFFloat > _errorCorrection,
                                          Inst< SFVec3f > _gravity,
                                          Inst< SFInt32 > _iterations,
                                          Inst< MFJoint > _joints,
                                          Inst< SFFloat > _maxCorrectionSpeed,
                                          Inst< SFBool  > _preferAccuracy,
                                          Inst< SFCollisionCollection  > _collider,
                                          Inst< ValueUpdater > _valueUpdater,
                                          Inst< EnableDisable > _enableDisable,
                                          Inst< SFString > _physicsEngine,
                                          Inst< SFInt32 > _desiredUpdateRate,
                                          Inst< SFInt32 > _updateRate,
                                          Inst< SFTime  > _stepUpdateTime,
                                          Inst< EnableUseMainThread > _useMainThread,
                                          Inst< MFEngineOptions > _engineOptions,
                                          Inst< DisplayList > _displayList,
                                          Inst< SFBool > _renderCollidables,
                                          Inst< SFBool > _renderOnlyEnabledCollidables,
                                          Inst< SFBool  > _useStaticTimeStep,
                                          Inst< SFInt32 > _syncGraphicsFrames,
                                          Inst< SFInt32 > _syncPhysicsFrames ) :
  X3DChildNode( _metadata ),
  H3DDisplayListObject( _displayList ),
  set_contacts( _set_contacts ),
  autoDisable( _autoDisable ),
  bodies( _bodies ),
  constantForceMix( _constantForceMix ),
  contactSurfaceThickness( _contactSurfaceThickness ),
  disableAngularSpeed( _disableAngularSpeed ),
  disableLinearSpeed( _disableLinearSpeed ),
  disableTime( _disableTime ),
  enabled( _enabled ),
  errorCorrection( _errorCorrection ),
  gravity( _gravity ),
  iterations( _iterations ),
  joints(_joints ),
  maxCorrectionSpeed( _maxCorrectionSpeed ),
  preferAccuracy( _preferAccuracy ),
  collider( _collider ),
  valueUpdater( _valueUpdater ),
  enableDisable ( _enableDisable ),
  physicsEngine( _physicsEngine ),
  desiredUpdateRate( _desiredUpdateRate ),
  updateRate( _updateRate ),
  stepUpdateTime( _stepUpdateTime ),
  useMainThread ( _useMainThread ),
  engineOptions ( _engineOptions ),
  renderCollidables( _renderCollidables ),
  renderOnlyEnabledCollidables( _renderOnlyEnabledCollidables ), 
  useStaticTimeStep( _useStaticTimeStep ),
  syncGraphicsFrames( _syncGraphicsFrames ),
  syncPhysicsFrames( _syncPhysicsFrames ),
  nr_graphics_steps( 0 ) {

  // initialize fields
  type_name = "RigidBodyCollection";
  database.initFields( this );

  displayList->setOwner( this );
  displayList->setCacheMode( H3DDisplayListObject::DisplayList::OFF );

  // Initialize the H3DPhysics python module
  H3DPhysicsPythonInterface::getInstance();

  valueUpdater->setName( "valueUpdater" );
  valueUpdater->setOwner( this );

  enableDisable->setName( "enableDisable" );
  enableDisable->setOwner( this );

  collider->setOwner( this );

  // set default values
  renderCollidables->setValue( false );
  autoDisable->setValue( false );
  constantForceMix->setValue( (H3DFloat) 0.00001 );
  contactSurfaceThickness->setValue( 0 );
  disableAngularSpeed->setValue( 0 );
  disableLinearSpeed->setValue( 0 );
  disableTime->setValue( 0 );
  enabled->setValue( true );
  errorCorrection->setValue( 0.2f );
  gravity->setValue( Vec3f(0, -9.8f, 0) );
  iterations->setValue( 10 );
  maxCorrectionSpeed->setValue( -1 );
  preferAccuracy->setValue( false );
  vector< string > physics_engine_names =
    PhysicsEngineThread::getSupportedPhysicsEngineNames();
  physicsEngine->addValidValues( physics_engine_names.begin(),
                                 physics_engine_names.end() );
  if( physicsEngine->isValidValue( "ODE" ) )
    physicsEngine->setValue ( "ODE" );
  else if( !physics_engine_names.empty() )
    physicsEngine->setValue ( physics_engine_names.front() );
  desiredUpdateRate->setValue( 100, id );
  updateRate->setValue( 0, id );
  stepUpdateTime->setValue( 0, id );
  useMainThread->setValue ( false );
  renderOnlyEnabledCollidables->setValue( false );
  useStaticTimeStep->setValue( false );
  syncGraphicsFrames->setValue( 0 );
  syncPhysicsFrames->setValue( 0 );

  // set up routes
  autoDisable->route( valueUpdater );
  constantForceMix->route( valueUpdater );
  contactSurfaceThickness->route( valueUpdater );
  disableAngularSpeed->route( valueUpdater );
  disableLinearSpeed->route( valueUpdater );
  disableTime->route( valueUpdater );
  errorCorrection->route( valueUpdater );
  gravity->route( valueUpdater );
  iterations->route( valueUpdater );
  maxCorrectionSpeed->route( valueUpdater );
  preferAccuracy->route( valueUpdater );
  useStaticTimeStep->route( valueUpdater );
  engineOptions->route ( valueUpdater );
}

RigidBodyCollection::~RigidBodyCollection() {
  if( simulationThread.get() ) {
    simulationThread->stopSimulation();
    joints->clear();
    bodies->clear();
    collider.reset(NULL); 
  }
}

void RigidBodyCollection::render() {
  if( renderCollidables->getValue() ) {
    const bool &render_only_enabled_collidables = renderOnlyEnabledCollidables->getValue();
    for( MFRigidBody::const_iterator i = bodies->begin(); i != bodies->end(); i++ ) {
      RigidBody *body = static_cast< RigidBody * >( *i );
      body->renderCollidable( render_only_enabled_collidables );
    }
  }
}

void RigidBodyCollection::initialize() {
  X3DChildNode::initialize();

  const string &engine = physicsEngine->getValue();

  if( PhysicsEngineThread::supportsPhysicsEngine( engine ) ) {
    bool use_synchronization = 
      syncGraphicsFrames->getValue() > 0 && 
      syncPhysicsFrames->getValue() > 0;

    simulationThread.reset( 
             new PhysicsEngineThread( engine, 
                                      PeriodicThreadBase::NORMAL_PRIORITY, 
                                      desiredUpdateRate->getValue(),
                                      useMainThread->getValue(),
                                      use_synchronization ) );
    simulationThread->setThreadName( "RigidBodyCollection " + engine + " engine thread" );
  } else {
    Console( 4 ) << "Warning: Unsupported physics engine "
                 << "specified in RigidBodyCollection node: "
                 << engine;
    set< string > physics_engine_names = physicsEngine->getValidValues();
    string physics_engine_to_use = "";
    if( physics_engine_names.size() > 0 ) {
      Console(4) << ". This build of H3DPhysics supports the following "
                 << "physics engines for RigidBodyCollection:" << endl;
      size_t j = 0;
      for( set< string >::iterator i = physics_engine_names.begin();
           i != physics_engine_names.end(); ++i, ++j ) {
        Console(4) << "\"" << *i << "\"";
        if( j < physics_engine_names.size() - 1 )
           Console(4) << ", ";
      }
      Console(4) << "." << endl;
      if( physicsEngine->isValidValue( "ODE" ) ) {
        physics_engine_to_use = "ODE";
      } else {
        physics_engine_to_use = *physics_engine_names.begin();
      }
    }

    if( physics_engine_to_use != "" ) {
      Console( 4 ) << "Using \"" << physics_engine_to_use << "\" instead.";
      simulationThread.reset(
             new PhysicsEngineThread( physics_engine_to_use,
                                      PeriodicThreadBase::NORMAL_PRIORITY, 
                                      desiredUpdateRate->getValue(),
                                      useMainThread->getValue() ) );
      simulationThread->setThreadName( "RigidBodyCollection " + physics_engine_to_use + " engine thread" );
    } else {
      Console ( 4 ) << "Error: No supported rigid body physics engine!" << endl;
    }
    Console( 4 ) << endl;
  }
}

void RigidBodyCollection::traverseSG(TraverseInfo &ti) {
  ti.setUserData( "PhysicsEngine", simulationThread.get() );

#ifdef USE_PROFILER
  // Report profiling results
  H3DTime t= Scene::time->getValue();
  if ( t - lastProfileTime > profileInterval ) {
    PROFILER_UPDATE ();
    lastProfileTime= t;
    Console(4) << PROFILER_OUTPUT_TREE_STRING () << endl << endl;
  }
#endif

  if( simulationThread.get() ) {
    if( waitForSimulationSteps() ) {
      beginSimulationSteps();
      H3DTIMER_BEGIN( "traverseSimulation" )
      traverseSimulation( ti );
      H3DTIMER_END( "traverseSimulation" )
      beginSynchronizeSteps();
    }

    // update output field values
    updateRate->setValue( simulationThread->getUpdateRate(), id );
    stepUpdateTime->setValue( simulationThread->getLastLoopTime(), id );

    // Step the simulation thread, if graphics and physics thread are
    // combined, async callbacks are executed here
    simulationThread->mainThreadStep ();

    // Set up route to enable/disable the simulation only after first 
    // iteration. Then simulation begins after all elements have been added.
    // Otherwise gravity etc will cause bodies to move before joints have
    // been added, for example.
    enabled->route( enableDisable );

    // Synchronizes with physics engine thread, i.e deletes pending objects.
    simulationThread->synchroniseWithSceneGraph();
  }
}

void H3D::RigidBodyCollection::traverseSimulation( TraverseInfo & ti ) {
  // update list of current devices to use to apply interaction force to bodies
  if( DeviceInfo* di = DeviceInfo::getActive() ) {
    simulationThread->setHapticsDevices( di->device->getValue() );
  }

  valueUpdater->upToDate();

  // traverse collider
  CollisionCollection *cc = collider->getValue();
  if( cc ) cc->traverseSG( ti );

  // traverse all bodies
  const NodeVector &c = bodies->getValue();
  for( unsigned int i = 0; i < c.size(); ++i ) {
    if( c[i] ) c[i]->traverseSG( ti );
  }

  // traverse all joints
  const NodeVector &j = joints->getValue();
  for( unsigned int i = 0; i < j.size(); ++i ) {
    if( j[i] ) j[i]->traverseSG( ti );
  }
}

bool RigidBodyCollection::waitForSimulationSteps() {
  if( simulationThread->useSynchronization() ) {
    if( (int)nr_graphics_steps >= syncGraphicsFrames->getValue() ) {
      simulationThread->waitForSimulationSteps();
      nr_graphics_steps = 0;
    }
    ++nr_graphics_steps;
    return nr_graphics_steps == 1;
  } else {
    return true;
  }
}

void RigidBodyCollection::beginSimulationSteps() {
  if( simulationThread->useSynchronization() ) {
    simulationThread->beginSimulationSteps( syncPhysicsFrames->getValue() );
  }
}

void RigidBodyCollection::beginSynchronizeSteps() {
  if( simulationThread->useSynchronization() ) {
    simulationThread->beginSynchronizeSteps();
  }
}

PhysicsEngineParameters::WorldParameters* RigidBodyCollection::getWorldParameters(bool all_params ) {

  PhysicsEngineParameters::WorldParameters *params = createWorldParameters();

  if( all_params || valueUpdater->hasCausedEvent( autoDisable ) ) {
    params->setAutoDisable( autoDisable->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( constantForceMix ) ) {
    params->setConstantForceMix( constantForceMix->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( contactSurfaceThickness ) ) {
    params->setContactSurfaceThickness( contactSurfaceThickness->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( disableAngularSpeed ) ) {
    params->setDisableAngularSpeed( disableAngularSpeed->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( disableLinearSpeed ) ) {
    params->setDisableLinearSpeed( disableLinearSpeed->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( disableTime ) ) {
    params->setDisableTime( disableTime->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( errorCorrection ) ) {
    params->setErrorCorrection( errorCorrection->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( gravity ) ) {
    params->setGravity( gravity->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( iterations ) ) {
    params->setIterations( iterations->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( maxCorrectionSpeed ) ) {
    params->setMaxCorrectionSpeed( maxCorrectionSpeed->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( preferAccuracy ) ) {
    params->setPreferAccuracy( preferAccuracy->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( useStaticTimeStep ) ) {
    params->setUseStaticTimeStep( useStaticTimeStep->getValue() );
  }
  if ( H3DEngineOptions* options= engineOptions->getOptions( simulationThread->getEngine() ) ) {
    if ( all_params || valueUpdater->hasCausedEvent ( options->valueUpdater ) ) {
      params->setEngineOptions ( options->valueUpdater->getParameters( all_params ) );
    }
  }

  return params;
}

void RigidBodyCollection::ValueUpdater::update() {
  RigidBodyCollection *pbc = static_cast< RigidBodyCollection * >( getOwner());
  // dont do anything unless we have an initialized thread
  if( pbc->simulationThread.get() ) {
    WorldParameters *params = pbc->getWorldParameters();
    pbc->simulationThread->setWorldParameters( *params );
  }
  EventCollectingField< PeriodicUpdate< Field > >::update();
}

void RigidBodyCollection::EnableDisable::update() {
  AutoUpdate< SFBool >::update();
  RigidBodyCollection *pbc = static_cast< RigidBodyCollection * >( getOwner());

  if ( pbc->simulationThread.get() ) {
    if ( value ) {
      pbc->nr_graphics_steps = pbc->syncGraphicsFrames->getValue();
      pbc->simulationThread->startSimulation();
    } else {
      pbc->simulationThread->stopSimulation();
    }
  }
}

void RigidBodyCollection::EnableUseMainThread::onValueChange ( const bool& b ) {
  RigidBodyCollection *rbc = static_cast< RigidBodyCollection * >( getOwner());

  if ( rbc->simulationThread.get() ) {
    rbc->simulationThread->setUseMainThread ( b );
  }
}

void RigidBodyCollection::MFRigidBody::onRemove( Node * n ) {
  RigidBody *rb = static_cast< RigidBody * >( n );
  RigidBodyCollection *rbc = 
    static_cast< RigidBodyCollection * >( getOwner() );
  if( rbc->simulationThread.get() ) {
    rb->deleteBody();
  }
  TypedMFNode< RigidBody >::onRemove( n );
}

void RigidBodyCollection::MFBodyConstraint::onRemove( Node * n ) {
  H3DBodyConstraintNode *constraint = 
    static_cast< H3DBodyConstraintNode * >( n );
  RigidBodyCollection *rbc = 
    static_cast< RigidBodyCollection * >( getOwner() );
  if( rbc->simulationThread.get() ) {
    constraint->deleteConstraint();
  }
  TypedMFNode< H3DBodyConstraintNode >::onRemove( n );
}
