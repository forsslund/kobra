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
/// \file PhysX3RigidBodyOptions.cpp
/// \brief Source file for PhysX3RigidBodyOptions, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////


#include <H3D/H3DPhysics/PhysX3RigidBodyOptions.h>

using namespace H3D;
using namespace PhysicsEngineParameters;

H3DNodeDatabase PhysX3RigidBodyOptions::database( "PhysX3RigidBodyOptions", 
                                                 &newInstance<PhysX3RigidBodyOptions>, 
                                                 typeid( PhysX3RigidBodyOptions ),
                                                 &H3DEngineOptions::database);

namespace PhysX3SoftBodyOptionsInternals {
  FIELDDB_ELEMENT( PhysX3RigidBodyOptions, contactReportThreshold, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PhysX3RigidBodyOptions, solverPositionIterations, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PhysX3RigidBodyOptions, solverVelocityIterations, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PhysX3RigidBodyOptions, createAsStatic, INITIALIZE_ONLY )
}

PhysX3RigidBodyOptions::PhysX3RigidBodyOptions(
  Inst< SFNode > _metadata,
  Inst< ValueUpdater  > _valueUpdater,
  Inst< SFFloat > _contactReportThreshold,
  Inst< SFInt32 > _solverPositionIterations,
  Inst< SFInt32 > _solverVelocityIterations,
  Inst< SFBool > _createAsStatic):
  H3DEngineOptions ( _metadata, _valueUpdater ),
  contactReportThreshold ( _contactReportThreshold ),
  solverPositionIterations ( _solverPositionIterations ),
  solverVelocityIterations ( _solverVelocityIterations ),
  createAsStatic ( _createAsStatic ) {
  type_name = "PhysX3RigidBodyOptions";
  database.initFields( this );

  contactReportThreshold->setValue ( 1.0f );
  solverPositionIterations->setValue ( 4 );
  solverVelocityIterations->setValue ( 1 );
  createAsStatic->setValue( false );

  contactReportThreshold->route ( valueUpdater );
  solverPositionIterations->route ( valueUpdater );
  solverVelocityIterations->route ( valueUpdater );
}

PhysicsEngineParameters::EngineOptionParameters* PhysX3RigidBodyOptions::getParameters( bool all_params ) {
  PhysX3RigidBodyParameters* params= new PhysX3RigidBodyParameters;

  if ( all_params || valueUpdater->hasCausedEvent ( contactReportThreshold ) ) {
    params->setContactReportThreshold ( contactReportThreshold->getValue() );
  }

  if ( all_params || valueUpdater->hasCausedEvent ( solverPositionIterations ) ) {
    params->setSolverPositionIterations ( solverPositionIterations->getValue() );
  }

  if ( all_params || valueUpdater->hasCausedEvent ( solverVelocityIterations ) ) {
    params->setSolverVelocityIterations ( solverVelocityIterations->getValue() );
  }

  if( all_params ) {
    params->setCreateAsStatic( createAsStatic->getValue() );
  }

  return params;
}