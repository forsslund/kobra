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
/// \file H3DAttachmentNode.cpp
/// \brief Source file for H3DAttachmentNode, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/H3DPhysics/H3DAttachmentNode.h>
#include <H3D/H3DPhysics/SoftBodyPhysicsEngineThread.h>

using namespace H3D;  

H3DNodeDatabase H3DAttachmentNode::database( "H3DAttachmentNode", 
                                            NULL, 
                                            typeid( H3DAttachmentNode ),
                                            &H3DBodyConstraintNode::database);

namespace H3DAttachmentNodeInternals {
  FIELDDB_ELEMENT( H3DAttachmentNode, body2, INPUT_OUTPUT )
  FIELDDB_ELEMENT( H3DAttachmentNode, index, INPUT_OUTPUT )
}

H3DAttachmentNode::H3DAttachmentNode(
                                     Inst< SFNode > _metadata,
                                     Inst< ValueUpdater > _valueUpdater,
                                     Inst< SFH3DSoftBodyNode > _body1,
                                     Inst< MFString     > _forceOutput,
                                     Inst< MFEngineOptions > _engineOptions,
                                     Inst< SFH3DBodyNode > _body2,
                                     Inst< TrackedMFInt32 > _index ) :
H3DBodyConstraintNode ( _metadata, _valueUpdater, _body1, _forceOutput, _engineOptions ),
body2( _body2 ),
index ( _index ) {
  // init fields
  type_name = "H3DAttachmentNode";
  database.initFields( this );

  body2->route ( valueUpdater );
  index->route ( valueUpdater );
}

bool H3DAttachmentNode::initializeConstraint( H3D::PhysicsEngineThread& pt ) {
  if( isInitialized() ) return false;

  if ( SoftBodyPhysicsEngineThread* sbpt= dynamic_cast<SoftBodyPhysicsEngineThread*>(&pt) ) {

    valueUpdater->upToDate();

    engine_thread= &pt;
    ConstraintParameters* p= getConstraintParameters( true );
    if( p ) {
      constraint_id= sbpt->addConstraint( *p );
      return true;
    } else
      return false;
  }
  else {
    return false;
  }
}

PhysicsEngineParameters::ConstraintParameters* H3DAttachmentNode::getConstraintParameters(bool all_params ) {

  PhysicsEngineParameters::H3DAttachmentParameters* params= 
    dynamic_cast<PhysicsEngineParameters::H3DAttachmentParameters*>(H3DBodyConstraintNode::getConstraintParameters( all_params ));
  if( !params )
    return NULL;

  if ( all_params || valueUpdater->hasCausedEvent ( body2 ) ) {
    if ( body2->getValue() ) {
      if ( H3DBodyId body2_id= body2->getValue()->getBodyId() ) {
        params->setBody2 ( body2_id );
      } else {
        uninitializedBodyWarning ( *body2->getValue() );
        delete params;
        return NULL;
      }
    }
  }    

  if ( all_params || valueUpdater->hasCausedEvent ( index ) ) {
    // Update all values
    params->setIndex( index->getValue() );
    if ( index->haveTrackedChanges() ) {
      // Indicate which values have changed
      params->setIndexChanges ( index->getEdits () );
      index->resetTracking();
    }
  }

  return params;
}
