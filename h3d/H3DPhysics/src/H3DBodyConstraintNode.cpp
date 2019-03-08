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
/// \file H3DBodyConstraintNode.cpp
/// \brief Source file for H3DBodyConstraintNode, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/H3DPhysics/H3DBodyConstraintNode.h>

using namespace H3D;  

H3DNodeDatabase H3DBodyConstraintNode::database( "H3DBodyConstraintNode", 
                                                NULL, 
                                                typeid( H3DBodyConstraintNode ),
                                                &H3DBodyInteractorNode::database);

namespace H3DBodyConstraintNodeInternals {
  FIELDDB_ELEMENT( H3DBodyConstraintNode, forceOutput, INPUT_OUTPUT )
  FIELDDB_ELEMENT( H3DBodyConstraintNode, engineOptions, INPUT_OUTPUT )
}


H3DBodyConstraintNode::H3DBodyConstraintNode(
  Inst< SFNode > _metadata,
  Inst< ValueUpdater > _valueUpdater,
  Inst< SFH3DBodyNode > _body1,
  Inst< MFString     > _forceOutput,
  Inst< MFEngineOptions > _engineOptions ):
H3DBodyInteractorNode ( _metadata, _body1 ),
engineOptions ( _engineOptions ),
valueUpdater ( _valueUpdater ),
forceOutput ( _forceOutput ),
constraint_id ( 0 )
{
  // init fields
  type_name = "H3DBodyConstraintNode";
  database.initFields( this );

  valueUpdater->setName ( "valueUpdater" );
  valueUpdater->setOwner ( this );

  forceOutput->push_back( "NONE" );
  forceOutput->route ( valueUpdater );
  body1->route ( valueUpdater );

}

H3DBodyConstraintNode::~H3DBodyConstraintNode () {
  if ( isInitialized() ) {
    deleteConstraint();
  }
}

bool H3DBodyConstraintNode::initializeConstraint( H3D::PhysicsEngineThread& pt ) {
  if( isInitialized() ) return false;

  valueUpdater->upToDate();
  engine_thread= &pt;
  ConstraintParameters* p= getConstraintParameters( true );
  if( !p ){
    engine_thread = NULL;
    return false;
  }
  constraint_id= engine_thread->addConstraint( *p );
  return true;
}

bool H3DBodyConstraintNode::deleteConstraint() {
  if ( isInitialized() && constraint_id ) {
    engine_thread->removeConstraint( constraint_id );
    engine_thread = NULL;
    return true;
  }
  return false;
}

void H3DBodyConstraintNode::traverseSG( TraverseInfo &ti ) {
  X3DNode::traverseSG(ti);
  PhysicsEngineThread *pt = NULL;
  ti.getUserData( "PhysicsEngine", (void * *)&pt );

  if( pt ) {
    if( !isInitialized() ) {
      // initialize the constraint for the given physics engine if it has
      // not been initialized yet.
      initializeConstraint( *pt );      
    }
    if( pt == engine_thread ) {

      // update the output fields of the constraint node.
      updateOutputFields();
    }
  }
}

ConstraintParameters* H3DBodyConstraintNode::getConstraintParameters( bool all_params ) {
  ConstraintParameters* params= createConstraintParameters ();
  if( !params ) return NULL;
  if ( all_params || valueUpdater->hasCausedEvent ( body1 ) ) {
    if ( body1->getValue() ) {
      if ( H3DBodyId body1_id= body1->getValue()->getBodyId() ) {
        params->setBody1 ( body1_id );
      } else {
        uninitializedBodyWarning ( *body1->getValue() );
        delete params;
        return NULL;
      }
    } else {
      Console(4) << "Warning: The body1 field of " << getName() << " must contain an H3DBodyNode." << endl;
    }
  }

  if ( H3DEngineOptions* options= engineOptions->getOptions ( engine_thread->getEngine() ) ) {
    if ( all_params || valueUpdater->hasCausedEvent ( options->valueUpdater ) ) {
      params->setEngineOptions ( options->valueUpdater->getParameters( all_params ) );
    }
  }

  return params;
}

void H3DBodyConstraintNode::ValueUpdater::update() {
  H3DBodyConstraintNode* constraint = static_cast< H3DBodyConstraintNode * >( getOwner());
  if ( constraint->isInitialized() && constraint->constraint_id ) {
    ConstraintParameters* params = constraint->getConstraintParameters();
    constraint->engine_thread->setConstraintParameters( constraint->constraint_id, *params );
  }
  
  EventCollectingField< PeriodicUpdate< Field > >::update();  
  
}