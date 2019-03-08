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
/// \file H3DSolverNode.cpp
/// \brief Source file for H3DSolverNode, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////
#include <H3D/H3DPhysics/H3DSolverNode.h>

using namespace H3D;

H3DNodeDatabase H3DSolverNode::database( "H3DSolverNode", 
                                        NULL, 
                                        typeid( H3DSolverNode ),
                                        &Node::database);

namespace H3DSolverNodeInternals {
  FIELDDB_ELEMENT( H3DSolverNode, solverType, OUTPUT_ONLY )
}

H3DSolverNode::H3DSolverNode(Inst< SFString > _solverType ):
Node(),
solverType( _solverType ),
solverChanged( new Field ){

  type_name = "H3DSolverNode";
  database.initFields( this );

  solverType->setValue("DEFAULT_SOLVER" , id);

  solverChanged->setOwner( this );
  solverChanged->setName( "solverChanged" );
  solverType->route( solverChanged );

}
