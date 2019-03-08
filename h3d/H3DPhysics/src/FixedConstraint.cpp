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
/// \file FixedConstraint.cpp
/// \brief Source file for FixedConstraint, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////


#include <H3D/H3DPhysics/FixedConstraint.h>

using namespace H3D;

H3DNodeDatabase FixedConstraint::database( "FixedConstraint",
                                          &(newInstance< FixedConstraint >), 
                                          typeid( FixedConstraint ),
                                          &H3DVertexBodyConstraintNode::database);

FixedConstraint::FixedConstraint(Inst< SFNode       > _metadata,
                                 Inst< ValueUpdater > _value_updater,
                                 Inst< SFH3DBodyNode  > _body1,
                                 Inst< MFString     > _forceOutput,
                                 Inst< MFEngineOptions > _engineOptions,
                                 Inst< MFInt32 > _index ):
H3DVertexBodyConstraintNode( _metadata, _value_updater, _body1, _forceOutput, _engineOptions, _index )
{
  type_name = "FixedConstraint";
  database.initFields( this );  
}

PhysicsEngineParameters::ConstraintParameters* FixedConstraint::createConstraintParameters () {
  return new PhysicsEngineParameters::FixedConstraintParameters();
}

PhysicsEngineParameters::ConstraintParameters * FixedConstraint::getConstraintParameters(bool all_params ) {

  PhysicsEngineParameters::FixedConstraintParameters* params= 
    static_cast<PhysicsEngineParameters::FixedConstraintParameters*>(H3DVertexBodyConstraintNode::getConstraintParameters( all_params ));

  return params;
}
