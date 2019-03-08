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
/// \file UniformPoissonRatio.cpp
/// \brief Source file for UniformPoissonRatio, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////
#include <H3D/H3DPhysics/UniformPoissonRatio.h>

using namespace H3D;

H3DNodeDatabase UniformPoissonRatio::database( "UniformPoissonRatio", 
                                              &(newInstance< UniformPoissonRatio >), 
                                              typeid( UniformPoissonRatio ),
                                              &H3DPhysicsPoissonRatioNode::database);

UniformPoissonRatio::UniformPoissonRatio(
  Inst< SFNode > _metadata,
  Inst< ValueUpdater  > _valueUpdater,
  Inst< SFString > _unitType,
  Inst< SFPoissonRatio > _poissonRatio ):
H3DPhysicsPoissonRatioNode( _metadata, _valueUpdater, _unitType, _poissonRatio ){

  type_name = "UniformPoissonRatio";
  database.initFields( this );
}
PhysicsEngineParameters::MaterialPropertyParameters* UniformPoissonRatio::createMaterialPropertyParameters() {
  return new PhysicsEngineParameters::PoissonRatioParameters();
}
PhysicsEngineParameters::MaterialPropertyParameters* UniformPoissonRatio::getMaterialPropertyParameters( bool all_params ){

  PhysicsEngineParameters::PoissonRatioParameters *params= 
    static_cast<PhysicsEngineParameters::PoissonRatioParameters*>(H3DPhysicsPoissonRatioNode::getMaterialPropertyParameters ( all_params ));

  if ( all_params || valueUpdater->hasCausedEvent ( poissonRatio ) ) {
    params->setValue( poissonRatio->getValue() );
  }

  return params;
}
