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


#include <H3D/H3DPhysics/BulletRigidBodyOptions.h>

using namespace H3D;
using namespace PhysicsEngineParameters;

H3DNodeDatabase BulletRigidBodyOptions::database( "BulletRigidBodyOptions", 
                                                 &newInstance<BulletRigidBodyOptions>, 
                                                 typeid( BulletRigidBodyOptions ),
                                                 &H3DEngineOptions::database);

namespace BulletRigidBodyOptionsInternals {
  FIELDDB_ELEMENT( BulletRigidBodyOptions, collisionGroup, INPUT_OUTPUT )
  FIELDDB_ELEMENT( BulletRigidBodyOptions, collidesWith, INPUT_OUTPUT )
  FIELDDB_ELEMENT( BulletRigidBodyOptions, softBodyCollisionOptions, INPUT_OUTPUT )
}

BulletRigidBodyOptions::BulletRigidBodyOptions(
  Inst< SFNode > _metadata,
  Inst< ValueUpdater  > _valueUpdater,
  Inst< SFInt32 > _collisionGroup,
  Inst< SFInt32 > _collidesWith,
  Inst< SFString > _softBodyCollisionOptions ):
H3DEngineOptions ( _metadata, _valueUpdater ),
collisionGroup ( _collisionGroup ),
collidesWith ( _collidesWith ),
softBodyCollisionOptions ( _softBodyCollisionOptions ) {
  type_name = "BulletRigidBodyOptions";
  database.initFields( this );

  collisionGroup->setValue ( 0 );
  collidesWith->setValue ( 0 );

  softBodyCollisionOptions->addValidValue( "DEFAULT" );
  softBodyCollisionOptions->addValidValue( "SDF_RIGIDSOFT" );
  softBodyCollisionOptions->addValidValue( "CLUSTER_RIGIDSOFT" );
  softBodyCollisionOptions->setValue( "DEFAULT" );

  collisionGroup->route ( valueUpdater );
  collidesWith->route ( valueUpdater );
  softBodyCollisionOptions->route( valueUpdater );
}

PhysicsEngineParameters::EngineOptionParameters* BulletRigidBodyOptions::getParameters( bool all_params ) {
  BulletRigidBodyParameters* params= new BulletRigidBodyParameters;

  if ( all_params || valueUpdater->hasCausedEvent ( collisionGroup ) ) {
    params->setCollisionGroup ( collisionGroup->getValue() );
  }

  if ( all_params || valueUpdater->hasCausedEvent ( collidesWith ) ) {
    params->setCollidesWith ( collidesWith->getValue() );
  }

  if( all_params || valueUpdater->hasCausedEvent( softBodyCollisionOptions ) ) {
    const std::string& option = softBodyCollisionOptions->getValue();
    if( option == "SDF_RIGIDSOFT" ) {
      params->setSoftBodyCollisionOptions( BulletRigidBodyParameters::SoftBodyCollisionOptions::SDF_RIGIDSOFT );
    } else if( option == "CLUSTER_RIGIDSOFT" ) {
      params->setSoftBodyCollisionOptions( BulletRigidBodyParameters::SoftBodyCollisionOptions::CLUSTER_RIGIDSOFT );
    } else {
      params->setSoftBodyCollisionOptions( BulletRigidBodyParameters::SoftBodyCollisionOptions::DEFAULT );
    }
  }

  return params;
}