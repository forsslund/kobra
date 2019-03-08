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
/// \file MassSpringPhysicsMaterial.cpp
/// \brief Source file for MassSpringPhysicsMaterial, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////
#include <H3D/H3DPhysics/MassSpringPhysicsMaterial.h>
#include <H3D/H3DPhysics/SoftBodyParameters.h>

using namespace H3D;
using namespace PhysicsEngineParameters;

H3DNodeDatabase MassSpringPhysicsMaterial::database( "MassSpringPhysicsMaterial", 
                                                    &(newInstance< MassSpringPhysicsMaterial >), 
                                                    typeid( MassSpringPhysicsMaterial ),
                                                    &H3DPhysicsMaterialNode::database);

namespace MassSpringPhysicsMaterialInternals {
  FIELDDB_ELEMENT( MassSpringPhysicsMaterial, stiffness, INPUT_OUTPUT )
  FIELDDB_ELEMENT( MassSpringPhysicsMaterial, stiffnessAngular, INPUT_OUTPUT )
  FIELDDB_ELEMENT( MassSpringPhysicsMaterial, stiffnessVolume, INPUT_OUTPUT )
}

MassSpringPhysicsMaterial::MassSpringPhysicsMaterial(
  Inst< SFNode > _metadata,
  Inst< ValueUpdater > _valueUpdater,
  Inst< SFH3DPhysicsMassNode > _mass,
  Inst< SFH3DPhysicsDampingNode > _damping,
  Inst< SFH3DPhysicsFrictionNode > _friction,
  Inst< SFH3DPhysicsStiffnessNode > _stiffness,
  Inst< SFH3DPhysicsStiffnessNode > _stiffnessAngular,
  Inst< SFH3DPhysicsStiffnessNode > _stiffnessVolume ) :
H3DPhysicsMaterialNode( _metadata, _valueUpdater, _mass, _damping, _friction ),
stiffness( _stiffness ),
stiffnessAngular( _stiffnessAngular ),
stiffnessVolume( _stiffnessVolume ) {

  type_name = "MassSpringPhysicsMaterial";
  database.initFields( this );

  stiffness->route( valueUpdater );
  stiffness->route( materialChanged );

  stiffnessAngular->route( valueUpdater );
  stiffnessAngular->route( materialChanged );

  stiffnessVolume->route( valueUpdater );
  stiffnessVolume->route( materialChanged );

  previousStiffness = stiffness->getValue();
  previousStiffnessAngular = stiffnessAngular->getValue();
  previousStiffnessVolume = stiffnessVolume->getValue();
}

PhysicsEngineParameters::H3DPhysicsMaterialParameters* MassSpringPhysicsMaterial::createH3DPhysicsMaterialParameters() {
  return new PhysicsEngineParameters::MassSpringPhysicsMaterialParameters();
}

PhysicsEngineParameters::H3DPhysicsMaterialParameters* MassSpringPhysicsMaterial::getH3DPhysicsMaterialParameters(bool all_params ) {

  MassSpringPhysicsMaterialParameters* params= 
    static_cast<MassSpringPhysicsMaterialParameters*>(H3DPhysicsMaterialNode::getH3DPhysicsMaterialParameters(all_params));

  // Flags to force setting the angular or volume stiffness based on the main stiffness
  bool update_angular_stiffness = false;
  bool update_volume_stiffness = false;

  if ( all_params || valueUpdater->hasCausedEvent ( stiffness ) ) {

    params->setStiffness ( stiffness->getValue() );
    // If the stiffness field is set to a different node
    // create all the stiffness parameters in order to
    // initialize them in the physics_engine layer.
    bool material_params = all_params;
    if( previousStiffness != stiffness->getValue() )
    {
      material_params = true;
      previousStiffness = stiffness->getValue();
    }
    if ( stiffness->getValue() ) {
      params->setStiffnessParameters( dynamic_cast<PhysicsEngineParameters::StiffnessParameters*>
        (stiffness->getValue()->valueUpdater->getMaterialPropertyParameters( material_params ) ) );

      // If angular or volume stiffness is missing, then use the main stiffness
      update_angular_stiffness = !stiffnessAngular->getValue();
      update_volume_stiffness = !stiffnessVolume->getValue();
    }
  }

  if( all_params || update_angular_stiffness || valueUpdater->hasCausedEvent( stiffnessAngular ) ) {

    // Default to main stiffness if not specified
    H3DPhysicsStiffnessNode* stiffness_angular = stiffnessAngular->getValue();
    stiffness_angular  = stiffness_angular ? stiffness_angular : stiffness->getValue();

    params->setStiffnessAngular( stiffness_angular );
    // If the stiffness field is set to a different node
    // create all the stiffness parameters in order to
    // initialize them in the physics_engine layer.
    bool material_params = all_params;
    if( previousStiffnessAngular != stiffness_angular ) {
      material_params = true;
      previousStiffnessAngular = stiffness_angular;
    }
    if( stiffness_angular ) {
      params->setStiffnessAngularParameters( dynamic_cast<PhysicsEngineParameters::StiffnessParameters*>
        (stiffness_angular->valueUpdater->getMaterialPropertyParameters( material_params )) );
    }
  }

  if( all_params || update_volume_stiffness || valueUpdater->hasCausedEvent( stiffnessVolume ) ) {

    // Default to main stiffness if not specified
    H3DPhysicsStiffnessNode* stiffness_volume = stiffnessVolume->getValue();
    stiffness_volume = stiffness_volume ? stiffness_volume : stiffness->getValue();

    params->setStiffnessVolume( stiffness_volume );
    // If the stiffness field is set to a different node
    // create all the stiffness parameters in order to
    // initialize them in the physics_engine layer.
    bool material_params = all_params;
    if( previousStiffnessVolume != stiffness_volume ) {
      material_params = true;
      previousStiffnessVolume = stiffness_volume;
    }
    if( stiffness_volume ) {
      params->setStiffnessVolumeParameters( dynamic_cast<PhysicsEngineParameters::StiffnessParameters*>
        (stiffness_volume->valueUpdater->getMaterialPropertyParameters( material_params )) );
    }
  }

  return params;
}
