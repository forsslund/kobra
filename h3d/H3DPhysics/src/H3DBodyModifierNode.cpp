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
/// \file H3DBodyModifierNode.cpp
/// \brief Source file for H3DBodyModifierNode, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/H3DPhysics/H3DBodyModifierNode.h>
#include <H3D/H3DPhysics/SoftBodyPhysicsEngineThread.h>

using namespace H3D;  

H3DNodeDatabase H3DBodyModifierNode::database( "H3DBodyModifierNode", 
                                              NULL, 
                                              typeid( H3DBodyModifierNode ),
                                              &H3DBodyInteractorNode::database);

namespace H3DBodyModifierNodeInternals {
  FIELDDB_ELEMENT( H3DBodyModifierNode, engineOptions, INPUT_OUTPUT )
}

H3DBodyModifierNode::H3DBodyModifierNode( Inst< SFNode > _metadata,
                                         Inst< ValueUpdater > _valueUpdater,
                                         Inst< SFH3DBodyNode > _body1,
                                         Inst< MFEngineOptions > _engineOptions ) :
H3DBodyInteractorNode ( _metadata, _body1 ), 
engineOptions ( _engineOptions ),
valueUpdater ( _valueUpdater ),
modifier_id ( -1 )
{
  // init fields
  type_name = "H3DBodyModifierNode";
  database.initFields( this );

  valueUpdater->setName ( "valueUpdater" );
  valueUpdater->setOwner ( this );

  body1->route ( valueUpdater );
}

H3DBodyModifierNode::~H3DBodyModifierNode () {
  if ( isInitialized() ) {
    deleteModifier();
  }
}

bool H3DBodyModifierNode::initializeModifier( H3D::PhysicsEngineThread *pt ) {
  if( isInitialized() ) return false;

  if ( SoftBodyPhysicsEngineThread* sbpt= dynamic_cast<SoftBodyPhysicsEngineThread*>(pt) ) {

    valueUpdater->upToDate();

    engine_thread= pt;
    ModifierParameters* p= getModifierParameters( true );
    if( !p ){
      engine_thread = NULL;
      return false;
    }
    modifier_id = (H3DModifierId)(this);
    p->setModifierId( modifier_id );
    sbpt->addModifier( *p );
    return true;
  }
  else {
    return false;
  }
}

bool H3DBodyModifierNode::deleteModifier() {
  if ( isInitialized() ) {
    if ( SoftBodyPhysicsEngineThread* sbpt =
      dynamic_cast<SoftBodyPhysicsEngineThread*>(engine_thread) )
    {
      sbpt->removeModifier( modifier_id );
      engine_thread = NULL;
      return true;
    }
  }
  return false;
}

void H3DBodyModifierNode::traverseSG( TraverseInfo &ti ) {
  X3DNode::traverseSG(ti);
  PhysicsEngineThread *pt = NULL;
  ti.getUserData( "PhysicsEngine", (void * *)&pt );

  if( pt ) {
    if( !isInitialized() ) {
      // initialize the modifier for the given physics engine if it has
      // not been initialized yet.
      initializeModifier( pt );
    }
    if( pt == engine_thread ) {
      updateOutputFields();
    }
  }
}

PhysicsEngineParameters::ModifierParameters* H3DBodyModifierNode::createModifierParameters(){
  return new PhysicsEngineParameters::ModifierParameters();
}
ModifierParameters* H3DBodyModifierNode::getModifierParameters( bool all_params ) {
  ModifierParameters* params= createModifierParameters ();
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
      Console(4) << "Warning: The body1 field of " << getName() <<
        " must contain an H3DBodyNode." << endl;
      delete params;
      return NULL;

    }
  }

  if ( H3DEngineOptions* options= engineOptions->getOptions ( engine_thread->getEngine() ) ) {
    if ( all_params || valueUpdater->hasCausedEvent ( options->valueUpdater ) ) {
      params->setEngineOptions ( options->valueUpdater->getParameters( all_params ) );
    }
  }

  return params;
}

void H3DBodyModifierNode::ValueUpdater::update() {
  H3DBodyModifierNode* modifier = static_cast< H3DBodyModifierNode * >( getOwner());
  if ( modifier->isInitialized() ) {
    ModifierParameters* params = modifier->getModifierParameters();
    if( params ) {
      if ( SoftBodyPhysicsEngineThread* sbpt =
        dynamic_cast<SoftBodyPhysicsEngineThread*>(modifier->engine_thread))
        sbpt->setModifierParameters( modifier->modifier_id, *params );
    }
  }

  EventCollectingField< PeriodicUpdate< Field > >::update();
}