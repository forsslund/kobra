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
/// \file BulletWorldOptions.cpp
/// \brief Source file for BulletWorldOptions, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////


#include <H3D/H3DPhysics/BulletWorldOptions.h>

using namespace H3D;
using namespace PhysicsEngineParameters;

H3DNodeDatabase BulletWorldOptions::database( "BulletWorldOptions", 
                                             &newInstance<BulletWorldOptions>, 
                                             typeid( BulletWorldOptions ),
                                             &H3DEngineOptions::database);

namespace BulletWorldOptionsInternals {
  FIELDDB_ELEMENT( BulletWorldOptions, worldScale, INPUT_OUTPUT )
  FIELDDB_ELEMENT( BulletWorldOptions, fixedTimeStep, INPUT_OUTPUT )
  FIELDDB_ELEMENT( BulletWorldOptions, collisionMargin, INPUT_OUTPUT )
  FIELDDB_ELEMENT( BulletWorldOptions, maxVelocityLinear, INPUT_OUTPUT )
  FIELDDB_ELEMENT( BulletWorldOptions, maxVelocityAngular, INPUT_OUTPUT )
}

BulletWorldOptions::BulletWorldOptions(Inst< SFNode > _metadata,
                                       Inst< ValueUpdater  > _valueUpdater,
                                       Inst< SFFloat > _worldScale,
                                       Inst< SFFloat > _fixedTimeStep,
                                       Inst< SFFloat > _collisionMargin,
                                       Inst< SFFloat > _maxVelocityLinear,
                                       Inst< SFFloat > _maxVelocityAngular ):
H3DEngineOptions ( _metadata, _valueUpdater ),
worldScale ( _worldScale ),
fixedTimeStep ( _fixedTimeStep ),
collisionMargin ( _collisionMargin ),
maxVelocityLinear ( _maxVelocityLinear ),
maxVelocityAngular ( _maxVelocityAngular )
{
  type_name = "BulletWorldOptions";
  database.initFields( this );

  worldScale->setValue ( 1.0f );
  fixedTimeStep->setValue ( 1 / 1000.0f );
  collisionMargin->setValue ( 0.01f );
  maxVelocityLinear->setValue( -1.f );
  maxVelocityAngular->setValue( -1.f );

  worldScale->route ( valueUpdater );
  fixedTimeStep->route ( valueUpdater );
  collisionMargin->route ( valueUpdater );
  maxVelocityLinear->route ( valueUpdater );
  maxVelocityAngular->route ( valueUpdater );
}

PhysicsEngineParameters::EngineOptionParameters* BulletWorldOptions::getParameters( bool all_params ) {
  BulletWorldParameters* params= new BulletWorldParameters;

  if ( all_params || valueUpdater->hasCausedEvent ( worldScale ) ) {
    params->setWorldScale ( worldScale->getValue() );
  }
  if ( all_params || valueUpdater->hasCausedEvent ( fixedTimeStep ) ) {
    params->setFixedTimeStep ( fixedTimeStep->getValue() );
  }
  if ( all_params || valueUpdater->hasCausedEvent ( collisionMargin ) ) {
    params->setCollisionMargin ( collisionMargin->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( maxVelocityLinear ) ) {
    params->setMaxVelocityLinear( maxVelocityLinear->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( maxVelocityAngular ) ) {
    params->setMaxVelocityAngular( maxVelocityAngular->getValue() );
  }

  return params;
}
