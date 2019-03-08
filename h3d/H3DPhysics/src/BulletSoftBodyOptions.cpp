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
/// \file BulletSoftBodyOptions.cpp
/// \brief Source file for BulletSoftBodyOptions, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////


#include <H3D/H3DPhysics/BulletSoftBodyOptions.h>

using namespace H3D;
using namespace PhysicsEngineParameters;

H3DNodeDatabase BulletSoftBodyOptions::database( "BulletSoftBodyOptions", 
                                      &newInstance<BulletSoftBodyOptions>, 
                                      typeid( BulletSoftBodyOptions ),
                                      &H3DEngineOptions::database);

namespace BulletSoftBodyOptionsInternals {
  FIELDDB_ELEMENT( BulletSoftBodyOptions, collisionMargin, INPUT_OUTPUT )
  FIELDDB_ELEMENT( BulletSoftBodyOptions, pIterations, INPUT_OUTPUT )
  FIELDDB_ELEMENT( BulletSoftBodyOptions, dIterations, INPUT_OUTPUT )
  FIELDDB_ELEMENT( BulletSoftBodyOptions, cIterations, INPUT_OUTPUT )
  FIELDDB_ELEMENT( BulletSoftBodyOptions, collisionOptions, INPUT_OUTPUT )
  FIELDDB_ELEMENT( BulletSoftBodyOptions, nrClusters, INPUT_OUTPUT )
  FIELDDB_ELEMENT( BulletSoftBodyOptions, bendingContraintDistance, INPUT_OUTPUT )
  FIELDDB_ELEMENT( BulletSoftBodyOptions, enablePerEdgeStiffness, INPUT_OUTPUT )
  FIELDDB_ELEMENT( BulletSoftBodyOptions, enableFastEdgeBuilding, INPUT_OUTPUT )
  FIELDDB_ELEMENT( BulletSoftBodyOptions, softRigidClusterHardness, INPUT_OUTPUT )
  FIELDDB_ELEMENT( BulletSoftBodyOptions, softRigidClusterImpulseSplit, INPUT_OUTPUT )
  FIELDDB_ELEMENT( BulletSoftBodyOptions, softKineticClusterHardness, INPUT_OUTPUT )
  FIELDDB_ELEMENT( BulletSoftBodyOptions, softKineticClusterImpulseSplit, INPUT_OUTPUT )
  FIELDDB_ELEMENT( BulletSoftBodyOptions, softRigidHardness, INPUT_OUTPUT )
  FIELDDB_ELEMENT( BulletSoftBodyOptions, softKineticHardness, INPUT_OUTPUT )
  FIELDDB_ELEMENT( BulletSoftBodyOptions, collisionGroup, INPUT_OUTPUT )
  FIELDDB_ELEMENT( BulletSoftBodyOptions, collidesWith, INPUT_OUTPUT )
  FIELDDB_ELEMENT( BulletSoftBodyOptions, poseMatchingCoefficient, INPUT_OUTPUT )
  FIELDDB_ELEMENT( BulletSoftBodyOptions, poseMatchingVolume, INPUT_OUTPUT )
  FIELDDB_ELEMENT( BulletSoftBodyOptions, poseMatchingFrame, INPUT_OUTPUT )
}

BulletSoftBodyOptions::BulletSoftBodyOptions(
  Inst< SFNode > _metadata,
  Inst< ValueUpdater  > _valueUpdater,
  Inst< SFFloat > _collisionMargin,
  Inst< SFInt32 > _pIterations,
  Inst< SFInt32 > _dIterations,
  Inst< SFInt32 > _cIterations,
  Inst< MFCollisionOptions > _collisionOptions,
  Inst< SFInt32 > _nrClusters,
  Inst< SFInt32 > _bendingContraintDistance,
  Inst< SFBool > _enablePerEdgeStiffness,
  Inst< SFBool > _enableFastEdgeBuilding,
  Inst< SFFloat > _softRigidClusterHardness,
  Inst< SFFloat > _softRigidClusterImpulseSplit,
  Inst< SFFloat > _softKineticClusterHardness,
  Inst< SFFloat > _softKineticClusterImpulseSplit,
  Inst< SFFloat > _softRigidHardness,
  Inst< SFFloat > _softKineticHardness,
  Inst< SFInt32 > _collisionGroup,
  Inst< SFInt32 > _collidesWith,
  Inst< SFFloat > _poseMatchingCoefficient,
  Inst< SFBool > _poseMatchingVolume,
  Inst< SFBool > _poseMatchingFrame ):
  H3DEngineOptions ( _metadata, _valueUpdater ),
  collisionMargin ( _collisionMargin ),
  pIterations ( _pIterations ),
  dIterations ( _dIterations ),
  cIterations ( _cIterations ),
  collisionOptions ( _collisionOptions ),
  nrClusters ( _nrClusters ),
  bendingContraintDistance ( _bendingContraintDistance ),
  enablePerEdgeStiffness ( _enablePerEdgeStiffness ),
  enableFastEdgeBuilding ( _enableFastEdgeBuilding ),
  softRigidClusterHardness ( _softRigidClusterHardness ),
  softRigidClusterImpulseSplit ( _softRigidClusterImpulseSplit ),
  softKineticClusterHardness ( _softKineticClusterHardness ),
  softKineticClusterImpulseSplit ( _softKineticClusterImpulseSplit ),
  softRigidHardness ( _softRigidHardness ),
  softKineticHardness( _softKineticHardness ),
  collisionGroup( _collisionGroup ),
  collidesWith( _collidesWith ),
  poseMatchingCoefficient ( _poseMatchingCoefficient ),
  poseMatchingVolume( _poseMatchingVolume),
  poseMatchingFrame( _poseMatchingFrame )
{
  type_name = "BulletSoftBodyOptions";
  database.initFields( this );

  collisionMargin->setValue ( 0.01f );
  pIterations->setValue ( 1 );
  dIterations->setValue ( 0 );
  cIterations->setValue ( 4 );

  collisionOptions->addValidValue ( "SDF_RIGIDSOFT" );
  collisionOptions->addValidValue ( "CLUSTER_RIGIDSOFT" );
  collisionOptions->addValidValue ( "VERTEXFACE_SOFTSOFT" );
  collisionOptions->addValidValue ( "CLUSTER_SOFTSOFT" );
  collisionOptions->addValidValue ( "CLUSTER_SELF" );
  collisionOptions->addValidValue ( "" );

  collisionOptions->push_back ( "SDF_RIGIDSOFT" );

  nrClusters->setValue ( 64 );
  bendingContraintDistance->setValue ( 2 );

  enablePerEdgeStiffness->setValue ( false );
  enableFastEdgeBuilding->setValue ( true );

  softRigidClusterHardness->setValue ( (H3DFloat)0.1 );
  softRigidClusterImpulseSplit->setValue ( (H3DFloat)0.5 );
  softKineticClusterHardness->setValue ( (H3DFloat)1.0 );
  softKineticClusterImpulseSplit->setValue ( (H3DFloat)0.5 );

  softRigidHardness->setValue( H3DFloat ( 1.0 ) );
  softKineticHardness->setValue( H3DFloat ( 0.1 ) );

  collisionGroup->setValue( 0 );
  collidesWith->setValue( 0 );

  poseMatchingCoefficient->setValue( 0 );
  poseMatchingVolume->setValue( false );
  poseMatchingFrame->setValue( false );

  collisionMargin->route ( valueUpdater );
  pIterations->route ( valueUpdater );
  dIterations->route ( valueUpdater );
  cIterations->route ( valueUpdater );
  collisionOptions->route ( valueUpdater );
  nrClusters->route ( valueUpdater );
  bendingContraintDistance->route ( valueUpdater );
  enablePerEdgeStiffness->route ( valueUpdater );
  enableFastEdgeBuilding->route ( valueUpdater );
  softRigidClusterHardness->route ( valueUpdater );
  softRigidClusterImpulseSplit->route ( valueUpdater );
  softKineticClusterHardness->route ( valueUpdater );
  softKineticClusterImpulseSplit->route ( valueUpdater );
  softRigidHardness->route( valueUpdater );
  softKineticHardness->route( valueUpdater );
  collisionGroup->route( valueUpdater );
  collidesWith->route( valueUpdater );
  poseMatchingCoefficient->route( valueUpdater );
  poseMatchingVolume->route( valueUpdater );
  poseMatchingFrame->route( valueUpdater );
}

PhysicsEngineParameters::EngineOptionParameters* BulletSoftBodyOptions::getParameters( bool all_params ) {
  BulletSoftBodyParameters* params= new BulletSoftBodyParameters;

  if ( all_params || valueUpdater->hasCausedEvent ( collisionMargin ) ) {
    params->setCollisionMargin ( collisionMargin->getValue() );
  }

  if ( all_params || valueUpdater->hasCausedEvent ( pIterations ) ) {
    params->setPIterations ( pIterations->getValue() );
  }

  if ( all_params || valueUpdater->hasCausedEvent ( dIterations ) ) {
    params->setDIterations ( dIterations->getValue() );
  }

  if ( all_params || valueUpdater->hasCausedEvent ( cIterations ) ) {
    params->setCIterations ( cIterations->getValue() );
  }

  if ( all_params || valueUpdater->hasCausedEvent ( collisionOptions ) ) {
    collisionOptions->validate ();
    const vector<string>& options= collisionOptions->getValue();
    unsigned int bitMask= 0;

    if ( find ( options.begin(), options.end(), "CLUSTER_RIGIDSOFT" ) != options.end() ) {
      bitMask|= BulletSoftBodyParameters::CLUSTER_RIGIDSOFT;
    }
    if ( find ( options.begin(), options.end(), "SDF_RIGIDSOFT" ) != options.end() ) {
      bitMask|= BulletSoftBodyParameters::SDF_RIGIDSOFT;
    }
    if ( find ( options.begin(), options.end(), "VERTEXFACE_SOFTSOFT" ) != options.end() ) {
      bitMask|= BulletSoftBodyParameters::VERTEXFACE_SOFTSOFT;
    }
    if ( find ( options.begin(), options.end(), "CLUSTER_SOFTSOFT" ) != options.end() ) {
      bitMask|= BulletSoftBodyParameters::CLUSTER_SOFTSOFT;
    }
    if( find( options.begin(), options.end(), "CLUSTER_SELF" ) != options.end() ) {
      bitMask |= BulletSoftBodyParameters::CLUSTER_SELF;
    }

    params->setCollisionOptions ( bitMask );
  }

  if ( all_params || valueUpdater->hasCausedEvent ( nrClusters ) ) {
    params->setNrClusters ( nrClusters->getValue() );
  }

  if ( all_params || valueUpdater->hasCausedEvent ( bendingContraintDistance ) ) {
    params->setBendingContraintDistance ( bendingContraintDistance->getValue() );
  }

  if ( all_params || valueUpdater->hasCausedEvent ( enablePerEdgeStiffness ) ) {
    params->setEnablePerEdgeStiffness ( enablePerEdgeStiffness->getValue() );
  }

  if ( all_params || valueUpdater->hasCausedEvent ( enableFastEdgeBuilding ) ) {
    params->setEnableFastEdgeBuilding ( enableFastEdgeBuilding->getValue() );
  }

  if ( all_params || valueUpdater->hasCausedEvent ( softRigidClusterHardness ) ) {
    params->setSoftRigidClusterHardness ( softRigidClusterHardness->getValue() );
  }

  if ( all_params || valueUpdater->hasCausedEvent ( softRigidClusterImpulseSplit ) ) {
    params->setSoftRigidClusterImpulseSplit ( softRigidClusterImpulseSplit->getValue() );
  }

  if ( all_params || valueUpdater->hasCausedEvent ( softKineticClusterHardness ) ) {
    params->setSoftKineticClusterHardness ( softKineticClusterHardness->getValue() );
  }

  if ( all_params || valueUpdater->hasCausedEvent ( softKineticClusterImpulseSplit ) ) {
    params->setSoftKineticClusterImpulseSplit ( softKineticClusterImpulseSplit->getValue() );
  }

  if( all_params || valueUpdater->hasCausedEvent( softRigidHardness ) ) {
    params->setSoftRigidHardness ( softRigidHardness->getValue() );
  }

  if( all_params || valueUpdater->hasCausedEvent( softKineticHardness ) ) {
    params->setSoftKineticHardness( softKineticHardness->getValue() );
  }

  if( all_params || valueUpdater->hasCausedEvent( collisionGroup ) ) {
    params->setCollisionGroup( collisionGroup->getValue() );
  }

  if( all_params || valueUpdater->hasCausedEvent( collidesWith ) ) {
    params->setCollidesWith( collidesWith->getValue() );
  }

  if( all_params || valueUpdater->hasCausedEvent( poseMatchingCoefficient ) ) {
    params->setPoseMatchingCoefficient( poseMatchingCoefficient->getValue() );
  }

  if( all_params || valueUpdater->hasCausedEvent( poseMatchingVolume ) ) {
    params->setPoseMatchingVolume( poseMatchingVolume->getValue() );
  }

  if( all_params || valueUpdater->hasCausedEvent( poseMatchingFrame ) ) {
    params->setPoseMatchingFrame( poseMatchingFrame->getValue() );
  }

  return params;
}