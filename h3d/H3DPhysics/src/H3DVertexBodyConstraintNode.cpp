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
/// \file H3DVertexBodyConstraintNode.cpp
/// \brief Source file for H3DVertexBodyConstraintNode, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////


#include <H3D/H3DPhysics/H3DVertexBodyConstraintNode.h>
#include <H3D/H3DPhysics/SoftBodyPhysicsEngineThread.h>

using namespace H3D;

H3DNodeDatabase H3DVertexBodyConstraintNode::database( "H3DVertexBodyConstraintNode", 
                                                      NULL, 
                                                      typeid( H3DVertexBodyConstraintNode ),
                                                      &H3DBodyConstraintNode::database);

namespace H3DVertexBodyConstraintNodeInternals {
  FIELDDB_ELEMENT( H3DVertexBodyConstraintNode, index, INPUT_OUTPUT )
}

H3DVertexBodyConstraintNode::H3DVertexBodyConstraintNode(
  Inst< SFNode       > _metadata,
  Inst< ValueUpdater > _value_updater,
  Inst< SFH3DBodyNode  > _body1,
  Inst< MFString     > _forceOutput,
  Inst< MFEngineOptions > _engineOptions,
  Inst< MFInt32 > _index ):
H3DBodyConstraintNode( _metadata, _value_updater, _body1, _forceOutput, _engineOptions ),
index( _index )  
{
  type_name = "H3DVertexBodyConstraintNode";
  database.initFields( this );

  index->route ( valueUpdater );
}
bool H3DVertexBodyConstraintNode::initializeConstraint( H3D::PhysicsEngineThread& pt ) {
  if( isInitialized() ) return false;

  if ( SoftBodyPhysicsEngineThread* sbpt= dynamic_cast<SoftBodyPhysicsEngineThread*>(&pt) ) {

    valueUpdater->upToDate();

    engine_thread= &pt;
    ConstraintParameters* p= getConstraintParameters( true );
    if( !p )
      return false;
    constraint_id= sbpt->addConstraint( *p );
    return true;
  }
  else {
    return false;
  }
}

PhysicsEngineParameters::ConstraintParameters * H3DVertexBodyConstraintNode::getConstraintParameters(bool all_params ) {

  PhysicsEngineParameters::VertexBodyConstraintParameters* params= 
    static_cast<PhysicsEngineParameters::VertexBodyConstraintParameters*>(H3DBodyConstraintNode::getConstraintParameters( all_params ));

  if( params && ( all_params || valueUpdater->hasCausedEvent( index ) ) ) {
    params->setIndex( index->getValue() );
  }

  return params;
}
