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
/// \file BulletAttachmentOptions.cpp
/// \brief Source file for BulletAttachmentOptions, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////


#include <H3D/H3DPhysics/BulletAttachmentOptions.h>

using namespace H3D;
using namespace PhysicsEngineParameters;

H3DNodeDatabase BulletAttachmentOptions::database( "BulletAttachmentOptions", 
                                                  &newInstance<BulletAttachmentOptions>, 
                                                  typeid( BulletAttachmentOptions ),
                                                  &H3DEngineOptions::database);

namespace BulletAttachmentOptionsInternals {
  FIELDDB_ELEMENT( BulletAttachmentOptions, collide, INPUT_OUTPUT )
}

BulletAttachmentOptions::BulletAttachmentOptions(Inst< SFNode > _metadata,
                                                 Inst< ValueUpdater  > _valueUpdater,
                                                 Inst< SFBool > _collide ):
H3DEngineOptions ( _metadata, _valueUpdater ),
collide ( _collide )
{
  type_name = "BulletAttachmentOptions";
  database.initFields( this );

  collide->setValue ( true );
  collide->route ( valueUpdater );
}

PhysicsEngineParameters::EngineOptionParameters* BulletAttachmentOptions::getParameters( bool all_params ) {
  BulletAttachmentParameters* params= new BulletAttachmentParameters;

  if ( all_params || valueUpdater->hasCausedEvent ( collide ) ) {
    params->setCollide ( collide->getValue() );
  }

  return params;
}
