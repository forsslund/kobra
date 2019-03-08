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
/// \file PhysicsMaterial.cpp
/// \brief Source file for PhysicsMaterial, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////
#include <H3D/H3DPhysics/PhysicsMaterial.h>
#include <H3D/H3DPhysics/SoftBodyParameters.h>

using namespace H3D;
using namespace PhysicsEngineParameters;

H3DNodeDatabase PhysicsMaterial::database( "PhysicsMaterial", 
                                          &(newInstance< PhysicsMaterial >), 
                                          typeid( PhysicsMaterial ),
                                          &H3DPhysicsMaterialNode::database);

namespace PhysicsMaterialInternals {
  FIELDDB_ELEMENT( PhysicsMaterial, elasticity, INPUT_OUTPUT )
}

PhysicsMaterial::PhysicsMaterial( Inst< SFNode > _metadata,
                                 Inst< ValueUpdater > _valueUpdater,
                                 Inst< SFH3DPhysicsMassNode > _mass,
                                 Inst< SFH3DPhysicsDampingNode > _damping,
                                 Inst< SFH3DPhysicsFrictionNode > _friction,
                                 Inst< SFH3DPhysicsElasticityNode > _elasticity) :
H3DPhysicsMaterialNode( _metadata, _valueUpdater, _mass, _damping, _friction ),
elasticity( _elasticity ) {

  type_name = "PhysicsMaterial";
  database.initFields( this );

  elasticity->route( valueUpdater );
  previousElasticity = elasticity->getValue();
}

PhysicsEngineParameters::H3DPhysicsMaterialParameters* PhysicsMaterial::createH3DPhysicsMaterialParameters() {
  return new PhysicsEngineParameters::PhysicsMaterialParameters();
}

PhysicsEngineParameters::H3DPhysicsMaterialParameters* PhysicsMaterial::getH3DPhysicsMaterialParameters( bool all_params ) {

  PhysicsMaterialParameters* params= 
    static_cast<PhysicsMaterialParameters*>(H3DPhysicsMaterialNode::getH3DPhysicsMaterialParameters(all_params));

  if ( all_params || valueUpdater->hasCausedEvent ( elasticity ) ) {
    params->setElasticity ( elasticity->getValue() );

    // If the elasticity field is set to a different node
    // create all the elasticity parameters in order to
    // initialize them in the physics_engine layer.
    bool material_params = all_params;
    if( previousElasticity != elasticity->getValue() )
    {
      material_params = true;
      previousElasticity = elasticity->getValue();
    }
    params->setElasticityParameters( dynamic_cast<PhysicsEngineParameters::ElasticityParameters*>
      (elasticity->getValue()->valueUpdater->getMaterialPropertyParameters( material_params )) );
  }

  return params;
}
