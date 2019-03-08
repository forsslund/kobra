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
/// \file H3DDeformationStrategyNode.cpp
/// \brief Source file for H3DDeformationStrategyNode, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////
#include <H3D/H3DPhysics/H3DDeformationStrategyNode.h>

using namespace H3D;

H3DNodeDatabase H3DDeformationStrategyNode::database( "H3DDeformationStrategyNode", 
                                                     NULL, 
                                                     typeid( H3DDeformationStrategyNode ),
                                                     &Node::database);

namespace H3DDeformationStrategyNodeInternals {
  FIELDDB_ELEMENT( H3DDeformationStrategyNode, h3dSolver, INITIALIZE_ONLY )
  FIELDDB_ELEMENT( H3DDeformationStrategyNode, timeStep, INPUT_OUTPUT )
}

H3DDeformationStrategyNode::H3DDeformationStrategyNode(
  Inst< SFFloat > _timeStep,
  Inst< SFH3DSolverNode > _h3dSolver,
  Inst< ValueUpdater > _valueUpdater):
Node(),
timeStep( _timeStep ),
h3dSolver( _h3dSolver ),
valueUpdater( _valueUpdater ),
strategyChanged( new Field ){

  type_name = "H3DDeformationStrategyNode";
  database.initFields( this );

  timeStep->setValue( 0.01f );

  strategyChanged->setOwner( this );
  strategyChanged->setName( "strategyChanged" );
  h3dSolver->route( strategyChanged, id );

  valueUpdater->setOwner( this );
  valueUpdater->setName( "valueUpdater" );
  h3dSolver->route( valueUpdater, id );

}
PhysicsEngineParameters::DeformationStrategyParameters * H3DDeformationStrategyNode::getDeformationStrategyParameters( bool all_params ) {
  PhysicsEngineParameters::DeformationStrategyParameters *params = 
    createDeformationStrategyParameters();

  if( all_params || valueUpdater->hasCausedEvent( h3dSolver ) ) {
    params->setSolverType( h3dSolver->getValue() );
  }

  return params;
}

PhysicsEngineParameters::DeformationStrategyParameters* 
H3DDeformationStrategyNode::ValueUpdater::getDeformationStrategyParameters( bool all_params ) {
  allParams= all_params;
  upToDate();
  return params.get();
}

void H3DDeformationStrategyNode::ValueUpdater::update() {
  H3DDeformationStrategyNode* node= static_cast < H3DDeformationStrategyNode* > ( getOwner() );
  params.reset ( node->getDeformationStrategyParameters ( allParams ) );
  EventCollectingField < Field >::update();
}

