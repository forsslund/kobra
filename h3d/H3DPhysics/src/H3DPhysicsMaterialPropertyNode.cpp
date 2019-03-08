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
/// \file H3DPhysicsMaterialPropertyNode.cpp
/// \brief Source file for H3DPhysicsMaterialPropertyNode, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////
#include <H3D/H3DPhysics/H3DPhysicsMaterialPropertyNode.h>

using namespace H3D;

H3DNodeDatabase H3DPhysicsMaterialPropertyNode::database( "H3DPhysicsMaterialPropertyNode", 
                                                         NULL, 
                                                         typeid( H3DPhysicsMaterialPropertyNode ),
                                                         &X3DNode::database);

namespace H3DPhysicsMaterialPropertyNodeInternals {
  FIELDDB_ELEMENT( H3DPhysicsMaterialPropertyNode, unitType, INITIALIZE_ONLY )
}

H3DPhysicsMaterialPropertyNode::H3DPhysicsMaterialPropertyNode(
  Inst< SFNode > _metadata,
  Inst< ValueUpdater  > _valueUpdater,
  Inst< SFString > _unitType ):
X3DNode( _metadata ),
valueUpdater( _valueUpdater ),
unitType( _unitType ),
materialPropertyChanged( new Field ){

  type_name = "H3DPhysicsMaterialPropertyNode";
  database.initFields( this );

  materialPropertyChanged->setOwner( this );
  materialPropertyChanged->setName( "materialPropertyChanged" );

  unitType->setValue("UNIT_UNIFORM");

  valueUpdater->setOwner( this );
  valueUpdater->setName( "valueUpdater" );

  unitType->route ( valueUpdater );
  unitType->route ( materialPropertyChanged );
}

PhysicsEngineParameters::MaterialPropertyParameters* H3DPhysicsMaterialPropertyNode::getMaterialPropertyParameters(bool all_params ) {

  PhysicsEngineParameters::MaterialPropertyParameters *params = createMaterialPropertyParameters ();

  if ( all_params || valueUpdater->hasCausedEvent ( unitType ) ) {
    params->setUnitType ( unitTypeFromString ( unitType->getValue() ) );
  }

  return params;
}

PhysicsEngineParameters::MaterialPropertyParameters::UnitType H3DPhysicsMaterialPropertyNode::unitTypeFromString ( const string& str ) {
  if ( str == "UNIT_NODE" ) return PhysicsEngineParameters::MaterialPropertyParameters::UNIT_NODE;
  else if ( str == "UNIT_EDGE" ) return PhysicsEngineParameters::MaterialPropertyParameters::UNIT_EDGE;
  else if ( str == "UNIT_ELEMENT" ) return PhysicsEngineParameters::MaterialPropertyParameters::UNIT_ELEMENT;
  else return PhysicsEngineParameters::MaterialPropertyParameters::UNIT_UNIFORM;
}

PhysicsEngineParameters::MaterialPropertyParameters* 
H3DPhysicsMaterialPropertyNode::ValueUpdater::getMaterialPropertyParameters( bool all_params ) {
  allParams= all_params;
  upToDate();
  return params.get();
}

void H3DPhysicsMaterialPropertyNode::ValueUpdater::update() {
  H3DPhysicsMaterialPropertyNode* node= static_cast < H3DPhysicsMaterialPropertyNode* > ( getOwner() );
  params.reset ( node->getMaterialPropertyParameters ( allParams ) );

  EventCollectingField < PeriodicUpdate < Field > >::update();
}
