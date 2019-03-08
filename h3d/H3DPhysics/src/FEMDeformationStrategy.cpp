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
/// \file FEMDeformationStrategy.cpp
/// \brief Source file for FEMDeformationStrategy, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////
#include <H3D/H3DPhysics/FEMDeformationStrategy.h>

using namespace H3D;

H3DNodeDatabase FEMDeformationStrategy::database( "FEMDeformationStrategy", 
                                                 &(newInstance< FEMDeformationStrategy >),
                                                 typeid( FEMDeformationStrategy ),
                                                 &H3DDeformationStrategyNode::database);


FEMDeformationStrategy::FEMDeformationStrategy(
  Inst< SFFloat > _timeStep,
  Inst< SFH3DSolverNode > _h3dSolver,
  Inst< ValueUpdater > _valueUpdater):
H3DDeformationStrategyNode( _timeStep, _h3dSolver, _valueUpdater ){

  type_name = "FEMDeformationStrategy";
  database.initFields( this );

}
PhysicsEngineParameters::DeformationStrategyParameters * FEMDeformationStrategy::createDeformationStrategyParameters( ) {
  return new PhysicsEngineParameters::FEMDeformationStrategyParameters();
}
