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
/// \file SoftBodyVec3fAttribute.cpp
/// \brief Source file for SoftBodyVec3fAttribute, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////
#include <H3D/H3DPhysics/SoftBodyVec3fAttribute.h>

using namespace H3D;

H3DNodeDatabase SoftBodyVec3fAttribute::database( "SoftBodyVec3fAttribute", 
                                                 &(newInstance< SoftBodyVec3fAttribute >), 
                                                 typeid( SoftBodyVec3fAttribute ),
                                                 &H3DSoftBodyOutputNode::database);

namespace SoftBodyVec3fAttributeInternals {
  FIELDDB_ELEMENT( SoftBodyVec3fAttribute, value, OUTPUT_ONLY )
}

SoftBodyVec3fAttribute::SoftBodyVec3fAttribute(
  Inst< SFNode > _metadata,
  Inst< ValueUpdater > _valueUpdater,
  Inst< SFString > _name,
  Inst< MFInt32 > _index,
  Inst< SFString > _unitType,
  Inst< MFVec3f > _value ):
H3DSoftBodyOutputNode( _metadata, _valueUpdater, _name, _index, _unitType ),
value( _value ) {
  type_name = "SoftBodyVec3fAttribute";
  database.initFields( this );

  name->addValidValue ( "OUTPUT_FORCE" );
  name->addValidValue ( "OUTPUT_INTERACTION_FORCE" );
  name->addValidValue ( "OUTPUT_EXTERNAL_FORCE" );
  name->addValidValue ( "OUTPUT_VELOCITY" );
}

PhysicsEngineParameters::SoftBodyVec3fAttributeParameters* SoftBodyVec3fAttribute::createSoftBodyOutputParameters () {
  return new PhysicsEngineParameters::SoftBodyVec3fAttributeParameters;
}

PhysicsEngineParameters::SoftBodyVec3fAttributeParameters* SoftBodyVec3fAttribute::getSoftBodyOutputParameters( bool all_params ) {
  PhysicsEngineParameters::SoftBodyVec3fAttributeParameters* params= static_cast<PhysicsEngineParameters::SoftBodyVec3fAttributeParameters*>
    (H3DSoftBodyOutputNode::getSoftBodyOutputParameters ( all_params ));
  
  if ( all_params || valueUpdater->hasCausedEvent ( name ) ) {
    params->setOutputType ( getOutputTypeFromString ( name->getValue() ) );
  }

  return params;
}

void SoftBodyVec3fAttribute::setOutputParameters ( PhysicsEngineParameters::H3DSoftBodyOutputParameters& params ) {
  PhysicsEngineParameters::SoftBodyVec3fAttributeParameters* p= dynamic_cast<PhysicsEngineParameters::SoftBodyVec3fAttributeParameters*>(&params);
  if ( p ) {
    value->setValue ( p->getValues(), id );
  }
}

PhysicsEngineParameters::SoftBodyVec3fAttributeParameters::OutputType SoftBodyVec3fAttribute::getOutputTypeFromString ( const std::string& str ) {
  if ( str == "OUTPUT_FORCE" ) {
    return PhysicsEngineParameters::SoftBodyVec3fAttributeParameters::OUTPUT_FORCE;
  } else if ( str == "OUTPUT_INTERACTION_FORCE" ) {
    return PhysicsEngineParameters::SoftBodyVec3fAttributeParameters::OUTPUT_INTERACTION_FORCE;
  } else if ( str == "OUTPUT_EXTERNAL_FORCE" ) {
    return PhysicsEngineParameters::SoftBodyVec3fAttributeParameters::OUTPUT_EXTERNAL_FORCE;
  } else if ( str == "OUTPUT_VELOCITY" ) {
    return PhysicsEngineParameters::SoftBodyVec3fAttributeParameters::OUTPUT_VELOCITY;
  } else {
    Console(4) << "Warning: Invalid SoftBodyVec3fAttribute name " << str << ", using OUTPUT_FORCE instead!" << endl;
    return PhysicsEngineParameters::SoftBodyVec3fAttributeParameters::OUTPUT_FORCE;
  }
}