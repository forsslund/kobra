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
/// \file PhysXRigidBodyOptions.cpp
/// \brief Source file for PhysXRigidBodyOptions, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////


#include <H3D/H3DPhysics/PhysXRigidBodyOptions.h>

using namespace H3D;
using namespace PhysicsEngineParameters;

H3DNodeDatabase PhysXRigidBodyOptions::database( "PhysXRigidBodyOptions", 
                                                 "PhysX3RigidBodyOptions",
                                                 &newInstance<PhysXRigidBodyOptions>, 
                                                 typeid( PhysXRigidBodyOptions ),
                                                 &H3DEngineOptions::database);

namespace PhysXSoftBodyOptionsInternals {
  FIELDDB_ELEMENT( PhysXRigidBodyOptions, contactReportThreshold, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PhysXRigidBodyOptions, solverPositionIterations, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PhysXRigidBodyOptions, solverVelocityIterations, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PhysXRigidBodyOptions, createAsStatic, INITIALIZE_ONLY )
  FIELDDB_ELEMENT( PhysXRigidBodyOptions, maxDepenetrationVelocity, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PhysXRigidBodyOptions, linearVelocityDamping, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PhysXRigidBodyOptions, angularVelocityDamping, INPUT_OUTPUT )
    
}

PhysXRigidBodyOptions::PhysXRigidBodyOptions(
  Inst< SFNode > _metadata,
  Inst< ValueUpdater  > _valueUpdater,
  Inst< SFFloat > _contactReportThreshold,
  Inst< SFInt32 > _solverPositionIterations,
  Inst< SFInt32 > _solverVelocityIterations,
  Inst< SFBool > _createAsStatic,
  Inst< SFFloat > _maxDepenetrationVelocity,
  Inst< SFFloat > _linearVelocityDamping,
  Inst< SFFloat > _angularVelocityDamping):
  H3DEngineOptions ( _metadata, _valueUpdater ),
  contactReportThreshold ( _contactReportThreshold ),
  solverPositionIterations ( _solverPositionIterations ),
  solverVelocityIterations ( _solverVelocityIterations ),
  createAsStatic ( _createAsStatic ),
  maxDepenetrationVelocity( _maxDepenetrationVelocity ),
  linearVelocityDamping( _linearVelocityDamping ),
  angularVelocityDamping( _angularVelocityDamping ){
  type_name = "PhysXRigidBodyOptions";
  database.initFields( this );

  contactReportThreshold->setValue ( 1.0f );
  solverPositionIterations->setValue ( 4 );
  solverVelocityIterations->setValue ( 1 );
  createAsStatic->setValue( false );
  maxDepenetrationVelocity->setValue( 10.0f );
  linearVelocityDamping->setValue( 0.0f );
  angularVelocityDamping->setValue( 0.05f );

  contactReportThreshold->route ( valueUpdater );
  solverPositionIterations->route ( valueUpdater );
  solverVelocityIterations->route ( valueUpdater );
  maxDepenetrationVelocity->route( valueUpdater );
  linearVelocityDamping->route( valueUpdater );
  angularVelocityDamping->route( valueUpdater );
}

PhysicsEngineParameters::EngineOptionParameters* PhysXRigidBodyOptions::getParameters( bool all_params ) {
  PhysXRigidBodyParameters* params= new PhysXRigidBodyParameters;

  if ( all_params || valueUpdater->hasCausedEvent ( contactReportThreshold ) ) {
    params->setContactReportThreshold ( contactReportThreshold->getValue() );
  }

  if ( all_params || valueUpdater->hasCausedEvent ( solverPositionIterations ) ) {
    params->setSolverPositionIterations ( solverPositionIterations->getValue() );
  }

  if ( all_params || valueUpdater->hasCausedEvent ( solverVelocityIterations ) ) {
    params->setSolverVelocityIterations ( solverVelocityIterations->getValue() );
  }

  if( all_params || valueUpdater->hasCausedEvent( maxDepenetrationVelocity ) ) {
    params->setMaxDepenetrationVelocity( maxDepenetrationVelocity->getValue() );
  }

  if( all_params || valueUpdater->hasCausedEvent( linearVelocityDamping ) ) {
    params->setLinearVelocityDamping( linearVelocityDamping->getValue() );
  }

  if( all_params || valueUpdater->hasCausedEvent( angularVelocityDamping ) ) {
    params->setAngularVelocityDamping( angularVelocityDamping->getValue() );
  }

  if( all_params ) {
    params->setCreateAsStatic( createAsStatic->getValue() );
  }

  return params;
}