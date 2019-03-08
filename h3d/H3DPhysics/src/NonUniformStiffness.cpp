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
/// \file NonUniformStiffness.cpp
/// \brief Source file for NonUniformStiffness, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////
#include <H3D/H3DPhysics/NonUniformStiffness.h>

using namespace H3D;

H3DNodeDatabase NonUniformStiffness::database( "NonUniformStiffness", 
                                              &(newInstance< NonUniformStiffness >), 
                                              typeid( NonUniformStiffness ),
                                              &H3DPhysicsStiffnessNode::database);

namespace NonUniformStiffnessInternals {
  FIELDDB_ELEMENT( NonUniformStiffness, stiffnessPerUnit, INPUT_OUTPUT )
}

NonUniformStiffness::NonUniformStiffness(
  Inst< SFNode > _metadata,
  Inst< ValueUpdater  > _valueUpdater,
  Inst< SFString > _unitType,
  Inst< SFStiffness > _stiffness,
  Inst< TrackedMFFloat > _stiffnessPerUnit ):
H3DPhysicsStiffnessNode ( _metadata, _valueUpdater, _unitType, _stiffness ),
stiffnessPerUnit( _stiffnessPerUnit ) {

  type_name = "NonUniformStiffness";
  database.initFields( this );

  unitType->setValue ( "UNIT_EDGE" );

  stiffnessPerUnit->route( materialPropertyChanged );
  stiffnessPerUnit->route( valueUpdater );
}

PhysicsEngineParameters::MaterialPropertyParameters* NonUniformStiffness::createMaterialPropertyParameters() {
  return new PhysicsEngineParameters::StiffnessParameters();
}

PhysicsEngineParameters::MaterialPropertyParameters* NonUniformStiffness::getMaterialPropertyParameters( bool all_params ){

  PhysicsEngineParameters::StiffnessParameters *params= 
    static_cast<PhysicsEngineParameters::StiffnessParameters*>(H3DPhysicsStiffnessNode::getMaterialPropertyParameters ( all_params ));

  if ( all_params || valueUpdater->hasCausedEvent ( stiffnessPerUnit ) ) {
    // Update all values
    params->setValue( stiffnessPerUnit->getValue() );
    if ( stiffnessPerUnit->haveTrackedChanges() ) {
      // Indicate which values have changed
      params->setChanges ( stiffnessPerUnit->getEdits () );
      stiffnessPerUnit->resetTracking();
    }
  }

  return params;
}
