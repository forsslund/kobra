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
/// \file H3DJointNode.cpp
/// \brief Source file for H3DJointNode, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////


#include <H3D/H3DPhysics/H3DJointNode.h>

using namespace H3D;

H3DNodeDatabase H3DJointNode::database( "H3DJointNode", 
                                       NULL, 
                                       typeid( H3DJointNode ),
                                       &H3DBodyConstraintNode::database);

namespace H3DJointNodeInternals {
  FIELDDB_ELEMENT( H3DJointNode, body2, INPUT_OUTPUT )
  FIELDDB_ELEMENT( H3DJointNode, transform, INPUT_OUTPUT )
}

H3DJointNode::H3DJointNode(Inst< SFNode       > _metadata,
                           Inst< ValueUpdater > _value_updater,
                           Inst< SFH3DBodyNode  > _body1,
                           Inst< SFH3DBodyNode  > _body2, 
                           Inst< MFString     > _forceOutput,
                           Inst< MFEngineOptions > _engineOptions,
                           Inst< SFTransformNode > _transform ):
H3DBodyConstraintNode( _metadata, _value_updater, _body1, _forceOutput, _engineOptions ),
body2( _body2 ),
transform ( _transform )
{
  type_name = "H3DJointNode";
  database.initFields( this );

  body2->route ( valueUpdater );
}

bool H3DJointNode::initializeConstraint( H3D::PhysicsEngineThread& pt ) {
  if ( MatrixTransform* t= transform->getValue() ) {
    applyTransform ( t->matrix->getValue() );
  }
  return H3DBodyConstraintNode::initializeConstraint ( pt );
}

PhysicsEngineParameters::ConstraintParameters * H3DJointNode::getConstraintParameters(bool all_params ) {

  PhysicsEngineParameters::JointParameters* params= 
    static_cast<PhysicsEngineParameters::JointParameters*>(H3DBodyConstraintNode::getConstraintParameters( all_params ));

  if( !params ) return NULL;

  if ( all_params || valueUpdater->hasCausedEvent ( body2 ) ) {
    if ( body2->getValue() ) {
      if ( H3DBodyId body2_id= body2->getValue()->getBodyId() ) {
        params->setBody2 ( body2_id );
      } else {
        uninitializedBodyWarning ( *body2->getValue() );
        return NULL;
      }
    }
  }    

  return params;
}
