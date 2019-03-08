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
/// \file H3DPhysicsElasticityNode.cpp
/// \brief Source file for H3DPhysicsElasticityNode, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////
#include <H3D/H3DPhysics/H3DPhysicsElasticityNode.h>

using namespace H3D;

H3DNodeDatabase H3DPhysicsElasticityNode::database( "H3DPhysicsElasticityNode", 
                                                   NULL, 
                                                   typeid( H3DPhysicsElasticityNode ),
                                                   &H3DPhysicsMaterialPropertyNode::database);

namespace H3DPhysicsElasticityNodeInternals {
  FIELDDB_ELEMENT( H3DPhysicsElasticityNode, elasticity, INPUT_OUTPUT )
}

H3DPhysicsElasticityNode::H3DPhysicsElasticityNode(
  Inst< SFNode > _metadata,
  Inst< ValueUpdater  > _valueUpdater,
  Inst< SFString > _unitType,
  Inst< SFElasticity > _elasticity ):
H3DPhysicsMaterialPropertyNode( _metadata, _valueUpdater, _unitType ),
elasticity( _elasticity ){

  type_name = "H3DPhysicsElasticityNode";
  database.initFields( this );

  elasticity->setOwner( this );
  elasticity->setValue( 0.0 );
  elasticity->route( materialPropertyChanged );

  elasticity->route( valueUpdater );
}
PhysicsEngineParameters::MaterialPropertyParameters* H3DPhysicsElasticityNode::getMaterialPropertyParameters(bool all_params ) {
  return H3DPhysicsMaterialPropertyNode::getMaterialPropertyParameters( all_params );  
}
