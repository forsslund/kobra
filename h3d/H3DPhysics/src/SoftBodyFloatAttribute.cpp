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
/// \file SoftBodyFloatAttribute.cpp
/// \brief Source file for SoftBodyFloatAttribute, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////
#include <H3D/H3DPhysics/SoftBodyFloatAttribute.h>

using namespace H3D;

H3DNodeDatabase SoftBodyFloatAttribute::database( "SoftBodyFloatAttribute", 
                                                 &(newInstance< SoftBodyFloatAttribute >), 
                                                 typeid( SoftBodyFloatAttribute ),
                                                 &H3DSoftBodyOutputNode::database);

namespace SoftBodyFloatAttributeInternals {
  FIELDDB_ELEMENT( SoftBodyFloatAttribute, value, OUTPUT_ONLY )
}

SoftBodyFloatAttribute::SoftBodyFloatAttribute(
  Inst< SFNode > _metadata,
  Inst< ValueUpdater > _valueUpdater,
  Inst< SFString > _name,
  Inst< MFInt32 > _index,
  Inst< SFString > _unitType,
  Inst< MFFloat > _value ):
H3DSoftBodyOutputNode( _metadata, _valueUpdater, _name, _index, _unitType ),
value( _value ) {

  type_name = "SoftBodyFloatAttribute";
  database.initFields( this );

  name->addValidValue ( "OUTPUT_FORCE_MAGNITUDE" );
  name->addValidValue ( "OUTPUT_INTERACTION_FORCE_MAGNITUDE" );
  name->addValidValue ( "OUTPUT_EXTERNAL_FORCE_MAGNITUDE" );
  name->addValidValue ( "OUTPUT_SPEED" );
}

PhysicsEngineParameters::SoftBodyFloatAttributeParameters* SoftBodyFloatAttribute::createSoftBodyOutputParameters () {
  return new PhysicsEngineParameters::SoftBodyFloatAttributeParameters;
}

PhysicsEngineParameters::SoftBodyFloatAttributeParameters* SoftBodyFloatAttribute::getSoftBodyOutputParameters( bool all_params ) {
  PhysicsEngineParameters::SoftBodyFloatAttributeParameters* params= static_cast<PhysicsEngineParameters::SoftBodyFloatAttributeParameters*>
    (H3DSoftBodyOutputNode::getSoftBodyOutputParameters ( all_params ));

  if ( all_params || valueUpdater->hasCausedEvent ( name ) ) {
    params->setOutputType ( getOutputTypeFromString ( name->getValue() ) );
  }

  return params;
}

void SoftBodyFloatAttribute::setOutputParameters ( PhysicsEngineParameters::H3DSoftBodyOutputParameters& params ) {
  PhysicsEngineParameters::SoftBodyFloatAttributeParameters* p= dynamic_cast<PhysicsEngineParameters::SoftBodyFloatAttributeParameters*>(&params);
  if ( p ) {
    value->setValue ( p->getValues(), id );
  }
}

PhysicsEngineParameters::SoftBodyFloatAttributeParameters::OutputType SoftBodyFloatAttribute::getOutputTypeFromString ( const std::string& str ) {
  if ( str == "OUTPUT_FORCE_MAGNITUDE" ) {
    return PhysicsEngineParameters::SoftBodyFloatAttributeParameters::OUTPUT_FORCE_MAGNITUDE;
  } else if ( str == "OUTPUT_INTERACTION_FORCE_MAGNITUDE" ) {
    return PhysicsEngineParameters::SoftBodyFloatAttributeParameters::OUTPUT_INTERACTION_FORCE_MAGNITUDE;
  } else if ( str == "OUTPUT_EXTERNAL_FORCE_MAGNITUDE" ) {
    return PhysicsEngineParameters::SoftBodyFloatAttributeParameters::OUTPUT_EXTERNAL_FORCE_MAGNITUDE;
  } else if ( str == "OUTPUT_SPEED" ) {
    return PhysicsEngineParameters::SoftBodyFloatAttributeParameters::OUTPUT_SPEED;
  } else {
    Console(4) << "Warning: Invalid SoftBodyFloatAttribute name " << str << ", using OUTPUT_FORCE_MAGNITUDE instead!" << endl;
    return PhysicsEngineParameters::SoftBodyFloatAttributeParameters::OUTPUT_FORCE_MAGNITUDE;
  }
}