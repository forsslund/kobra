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
/// \file H3DEngineOptions.cpp
/// \brief Source file for H3DEngineOptions, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////


#include <H3D/H3DPhysics/H3DEngineOptions.h>

using namespace H3D;

H3DNodeDatabase H3DEngineOptions::database( "H3DEngineOptions", 
                                           NULL, 
                                           typeid( H3DEngineOptions ),
                                           &X3DNode::database);

H3DEngineOptions::H3DEngineOptions(Inst< SFNode > _metadata,
                                   Inst< ValueUpdater  > _valueUpdater):
X3DNode ( _metadata ),
valueUpdater ( _valueUpdater )
{
  type_name = "H3DEngineOptions";
  database.initFields( this );

  valueUpdater->setOwner( this );
  valueUpdater->setName( "valueUpdater" );
}

PhysicsEngineParameters::EngineOptionParameters* 
H3DEngineOptions::ValueUpdater::getParameters( bool all_params ) {
  allParams= all_params;
  upToDate();
  return params.get();
}

void H3DEngineOptions::ValueUpdater::update() {
  H3DEngineOptions* node= static_cast < H3DEngineOptions* > ( getOwner() );
  params.reset ( node->getParameters ( allParams ) );

  EventCollectingField < Field >::update();
}
