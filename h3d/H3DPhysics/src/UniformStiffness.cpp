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
/// \file UniformStiffness.cpp
/// \brief Source file for UniformStiffness, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////
#include <H3D/H3DPhysics/UniformStiffness.h>

using namespace H3D;

H3DNodeDatabase UniformStiffness::database( "UniformStiffness", 
                                           &(newInstance< UniformStiffness >), 
                                           typeid( UniformStiffness ),
                                           &H3DPhysicsStiffnessNode::database);

UniformStiffness::UniformStiffness(
                                   Inst< SFNode > _metadata,
                                   Inst< ValueUpdater  > _valueUpdater,
                                   Inst< SFString > _unitType,
                                   Inst< SFStiffness > _stiffness ):
H3DPhysicsStiffnessNode( _metadata, _valueUpdater, _unitType, _stiffness ){

  type_name = "UniformStiffness";
  database.initFields( this );
}

PhysicsEngineParameters::MaterialPropertyParameters* UniformStiffness::createMaterialPropertyParameters() {
  return new PhysicsEngineParameters::StiffnessParameters();
}

PhysicsEngineParameters::MaterialPropertyParameters* UniformStiffness::getMaterialPropertyParameters( bool all_params ){

  PhysicsEngineParameters::StiffnessParameters *params= 
    static_cast<PhysicsEngineParameters::StiffnessParameters*>(H3DPhysicsStiffnessNode::getMaterialPropertyParameters ( all_params ));

  if ( all_params || valueUpdater->hasCausedEvent ( stiffness ) ) {
    params->setValue( stiffness->getValue() );
  }

  return params;
}
