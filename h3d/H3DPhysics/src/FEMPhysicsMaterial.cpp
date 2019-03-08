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
/// \file FEMPhysicsMaterial.cpp
/// \brief Source file for FEMPhysicsMaterial, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////
#include <H3D/H3DPhysics/FEMPhysicsMaterial.h>
#include <H3D/H3DPhysics/SoftBodyParameters.h>

using namespace H3D;
using namespace PhysicsEngineParameters;

H3DNodeDatabase FEMPhysicsMaterial::database( "FEMPhysicsMaterial", 
                                             &(newInstance< FEMPhysicsMaterial >), 
                                             typeid( FEMPhysicsMaterial ),
                                             &PhysicsMaterial::database);

namespace FEMPhysicsMaterialInternals {
  FIELDDB_ELEMENT( FEMPhysicsMaterial, poissonRatio, INPUT_OUTPUT )  
}

FEMPhysicsMaterial::FEMPhysicsMaterial(Inst< SFNode > _metadata,
                                       Inst< ValueUpdater  > _valueUpdater,
                                       Inst< SFH3DPhysicsMassNode > _mass,
                                       Inst< SFH3DPhysicsDampingNode > _damping,
                                       Inst< SFH3DPhysicsFrictionNode > _friction,
                                       Inst< SFH3DPhysicsElasticityNode > _elasticity,
                                       Inst< SFH3DPhysicsPoissonRatioNode > _poissonRatio ):
PhysicsMaterial( _metadata, _valueUpdater, _mass, _damping, _friction, _elasticity ),
poissonRatio( _poissonRatio ) {

  type_name = "FEMPhysicsMaterial";
  database.initFields( this );

  poissonRatio->route( valueUpdater );
  previousPoissonRatio = poissonRatio->getValue();
}

PhysicsEngineParameters::H3DPhysicsMaterialParameters* FEMPhysicsMaterial::createH3DPhysicsMaterialParameters() {
  return new PhysicsEngineParameters::FEMPhysicsMaterialParameters();
}

PhysicsEngineParameters::H3DPhysicsMaterialParameters* FEMPhysicsMaterial::getH3DPhysicsMaterialParameters(bool all_params ) {

  FEMPhysicsMaterialParameters* params= 
    static_cast<FEMPhysicsMaterialParameters*>(PhysicsMaterial::getH3DPhysicsMaterialParameters(all_params));

  if ( all_params || valueUpdater->hasCausedEvent ( poissonRatio ) ) {
    params->setPoissonRatio ( poissonRatio->getValue() );

    // If the poissonRatio field is set to a different node
    // create all the poissonRatio parameters in order to
    // initialize them in the physics_engine layer.
    bool material_params = all_params;
    if( previousPoissonRatio != poissonRatio->getValue() )
    {
      material_params = true;
      previousPoissonRatio = poissonRatio->getValue();
    }
    if ( poissonRatio->getValue() ) {
      params->setPoissonRatioParameters( dynamic_cast< PoissonRatioParameters* >
        (poissonRatio->getValue()->valueUpdater->getMaterialPropertyParameters( material_params ) ) );
    }
  }

  return params;
}
