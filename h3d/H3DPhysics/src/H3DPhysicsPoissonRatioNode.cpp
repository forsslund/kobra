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
/// \file H3DPhysicsPoissonRatioNode.cpp
/// \brief Source file for H3DPhysicsPoissonRatioNode, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////
#include <H3D/H3DPhysics/H3DPhysicsPoissonRatioNode.h>

using namespace H3D;

H3DNodeDatabase H3DPhysicsPoissonRatioNode::database( "H3DPhysicsPoissonRatioNode", 
                                                     NULL, 
                                                     typeid( H3DPhysicsPoissonRatioNode ),
                                                     &H3DPhysicsMaterialPropertyNode::database);

namespace H3DPhysicsPoissonRatioNodeInternals {
  FIELDDB_ELEMENT( H3DPhysicsPoissonRatioNode, poissonRatio, INPUT_OUTPUT )
}

H3DPhysicsPoissonRatioNode::H3DPhysicsPoissonRatioNode(
  Inst< SFNode > _metadata,
  Inst< ValueUpdater  > _valueUpdater,
  Inst< SFString > _unitType,
  Inst< SFPoissonRatio > _poissonRatio ):
H3DPhysicsMaterialPropertyNode( _metadata, _valueUpdater, _unitType ),
poissonRatio( _poissonRatio ){

  type_name = "H3DPhysicsPoissonRatioNode";
  database.initFields( this );

  poissonRatio->setOwner( this );
  poissonRatio->setValue( 0.0 );
  poissonRatio->route( materialPropertyChanged );

  poissonRatio->route( valueUpdater );
}
PhysicsEngineParameters::MaterialPropertyParameters* H3DPhysicsPoissonRatioNode::getMaterialPropertyParameters(bool all_params ) {
  return H3DPhysicsMaterialPropertyNode::getMaterialPropertyParameters( all_params );  
}
