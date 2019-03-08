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
/// \file UniformMass.cpp
/// \brief Source file for UniformMass, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////
#include <H3D/H3DPhysics/UniformMass.h>

using namespace H3D;

H3DNodeDatabase UniformMass::database( "UniformMass", 
                                      &(newInstance< UniformMass >), 
                                      typeid( UniformMass ),
                                      &H3DPhysicsMassNode::database);

UniformMass::UniformMass(
                         Inst< SFNode > _metadata,
                         Inst< ValueUpdater  > _valueUpdater,
                         Inst< SFString > _unitType,
                         Inst< SFMass > _mass ):
H3DPhysicsMassNode( _metadata, _valueUpdater, _unitType, _mass ){

  type_name = "UniformMass";
  database.initFields( this );
}

PhysicsEngineParameters::MaterialPropertyParameters* UniformMass::createMaterialPropertyParameters() {
  return new PhysicsEngineParameters::MassParameters();
}

PhysicsEngineParameters::MaterialPropertyParameters* UniformMass::getMaterialPropertyParameters( bool all_params ){

  PhysicsEngineParameters::MassParameters *params= 
    static_cast<PhysicsEngineParameters::MassParameters*>(H3DPhysicsMassNode::getMaterialPropertyParameters ( all_params ));

  if ( all_params || valueUpdater->hasCausedEvent ( mass ) ) {
    params->setValue( mass->getValue() );
  }

  return params;
}
