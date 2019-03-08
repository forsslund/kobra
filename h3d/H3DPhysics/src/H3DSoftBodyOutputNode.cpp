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
/// \file H3DSoftBodyOutputNode.cpp
/// \brief Source file for H3DSoftBodyOutputNode, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////
#include <H3D/H3DPhysics/H3DSoftBodyOutputNode.h>

using namespace H3D;

H3DNodeDatabase H3DSoftBodyOutputNode::database( "H3DSoftBodyOutputNode", 
                                                NULL, 
                                                typeid( H3DSoftBodyOutputNode ),
                                                &X3DNode::database);

namespace H3DSoftBodyOutputNodeInternals {
  FIELDDB_ELEMENT( H3DSoftBodyOutputNode, index, INPUT_OUTPUT)
  FIELDDB_ELEMENT( H3DSoftBodyOutputNode, name, INPUT_OUTPUT )
  FIELDDB_ELEMENT( H3DSoftBodyOutputNode, unitType, INPUT_OUTPUT )
}

H3DSoftBodyOutputNode::H3DSoftBodyOutputNode(
  Inst< SFNode > _metadata,
  Inst< ValueUpdater > _valueUpdater,
  Inst< SFString > _name,
  Inst< MFInt32 > _index,
  Inst< SFString > _unitType ):
X3DNode( _metadata ),
valueUpdater ( _valueUpdater ),
name ( _name ),
index( _index ),
unitType( _unitType ) {

  type_name = "H3DSoftBodyOutputNode";
  database.initFields( this );

  valueUpdater->setName ( "valueUpdater" );
  valueUpdater->setOwner ( this );

  unitType->addValidValue ( "UNIT_NODE" );
  unitType->addValidValue ( "UNIT_EDGE" );
  unitType->addValidValue ( "UNIT_ELEMENT" );

  unitType->setValue ( "UNIT_NODE" );

  name->route ( valueUpdater );
  index->route ( valueUpdater );
  unitType->route ( valueUpdater );
}

PhysicsEngineParameters::H3DSoftBodyOutputParameters*
H3DSoftBodyOutputNode::getSoftBodyOutputParameters ( bool all_params ) {
  PhysicsEngineParameters::H3DSoftBodyOutputParameters* params= createSoftBodyOutputParameters();

  // Set back pointer to node to allow fields to be populated later
  params->setNode ( *this );

  if ( all_params || valueUpdater->hasCausedEvent ( index ) ) {
    params->setIndex ( index->getValue() );
  }
  if ( all_params || valueUpdater->hasCausedEvent ( unitType ) ) {
    params->setUnitType ( getUnitTypeFromString ( unitType->getValue() ) );
  }

  return params;
}

PhysicsEngineParameters::H3DSoftBodyOutputParameters::UnitType H3DSoftBodyOutputNode::getUnitTypeFromString ( const std::string& str ) {
  if ( str == "UNIT_NODE" ) {
    return PhysicsEngineParameters::H3DSoftBodyOutputParameters::UNIT_NODE;
  } else if ( str == "UNIT_EDGE" ) {
    return PhysicsEngineParameters::H3DSoftBodyOutputParameters::UNIT_EDGE;
  } else if ( str == "UNIT_ELEMENT" ) {
    return PhysicsEngineParameters::H3DSoftBodyOutputParameters::UNIT_ELEMENT;
  } else {
    Console(4) << "Warning: Invalid H3DSoftBodyOutputNode unitType " << str << ", using UNIT_NODE instead!" << endl;
    return PhysicsEngineParameters::H3DSoftBodyOutputParameters::UNIT_NODE;
  }
}

PhysicsEngineParameters::H3DSoftBodyOutputParameters* 
H3DSoftBodyOutputNode::ValueUpdater::getParameters( bool all_params ) {
  allParams= all_params;
  upToDate();
  return params.get();
}

void H3DSoftBodyOutputNode::ValueUpdater::update() {
  H3DSoftBodyOutputNode* node= static_cast < H3DSoftBodyOutputNode* > ( getOwner() );
  params.reset ( node->getSoftBodyOutputParameters ( allParams ) );

  EventCollectingField < Field >::update();
}