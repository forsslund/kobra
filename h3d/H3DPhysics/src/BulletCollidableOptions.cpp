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
/// \file BulletCollidableOptions.cpp
/// \brief Source file for BulletCollidableOptions, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////


#include <H3D/H3DPhysics/BulletCollidableOptions.h>

using namespace H3D;
using namespace PhysicsEngineParameters;

H3DNodeDatabase BulletCollidableOptions::database( "BulletCollidableOptions", 
                                                  &newInstance<BulletCollidableOptions>, 
                                                  typeid( BulletCollidableOptions ),
                                                  &H3DEngineOptions::database);

namespace BulletCollidableOptionsInternals {
  FIELDDB_ELEMENT( BulletCollidableOptions, collisionMargin, INPUT_OUTPUT )
  FIELDDB_ELEMENT( BulletCollidableOptions, convex, INPUT_OUTPUT )
}

BulletCollidableOptions::BulletCollidableOptions( Inst< SFNode > _metadata,
                                                 Inst< ValueUpdater  > _valueUpdater,
                                                 Inst< SFFloat > _collisionMargin,
                                                 Inst< SFBool > _convex ):
H3DEngineOptions ( _metadata, _valueUpdater ),
collisionMargin ( _collisionMargin ),
convex ( _convex )
{
  type_name = "BulletCollidableOptions";
  database.initFields( this );

  collisionMargin->setValue ( 0.01f );
  convex->setValue ( false );

  collisionMargin->route ( valueUpdater );
  convex->route ( valueUpdater );
}

PhysicsEngineParameters::EngineOptionParameters* BulletCollidableOptions::getParameters( bool all_params ) {
  BulletCollidableParameters* params= new BulletCollidableParameters;

  if ( all_params || valueUpdater->hasCausedEvent ( collisionMargin ) ) {
    params->setCollisionMargin ( collisionMargin->getValue() );
  }
  if ( all_params || valueUpdater->hasCausedEvent ( convex ) ) {
    params->setConvex ( convex->getValue() );
  }

  return params;
}