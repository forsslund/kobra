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
/// \file CollisionCollection.cpp
/// \brief cpp file for CollisionCollection, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/H3DPhysics/CollisionCollection.h>
#include <H3D/H3DPhysics/RigidBodyCollection.h>
#include <H3D/H3DPhysics/X3DNBodyCollidableNode.h>
#include <H3D/H3DPhysics/X3DNBodyCollisionSpaceNode.h>
#include <H3D/H3DPhysics/CollidableShape.h>
#include <H3D/H3DPhysics/CollisionSpace.h>

using namespace H3D;

H3DNodeDatabase CollisionCollection::database( "CollisionCollection",
                                               &(newInstance< CollisionCollection >),
                                               typeid(CollisionCollection),
                                               &X3DChildNode::database );

namespace CollisionCollectionInternals {
  FIELDDB_ELEMENT( CollisionCollection, appliedParameters, INPUT_OUTPUT )
  FIELDDB_ELEMENT( CollisionCollection, bounce, INPUT_OUTPUT )
  FIELDDB_ELEMENT( CollisionCollection, collidables, INPUT_OUTPUT )
  FIELDDB_ELEMENT( CollisionCollection, collidableGroups, INPUT_OUTPUT )
  FIELDDB_ELEMENT( CollisionCollection, collidableExceptionGroups, INPUT_OUTPUT )
  FIELDDB_ELEMENT( CollisionCollection, enabled, INPUT_OUTPUT )
  FIELDDB_ELEMENT( CollisionCollection, frictionCoefficients, INPUT_OUTPUT )
  FIELDDB_ELEMENT( CollisionCollection, minBounceSpeed, INPUT_OUTPUT )
  FIELDDB_ELEMENT( CollisionCollection, slipFactors, INPUT_OUTPUT )
  FIELDDB_ELEMENT( CollisionCollection, softnessConstantForceMix, INPUT_OUTPUT )
  FIELDDB_ELEMENT( CollisionCollection, softnessErrorCorrection, INPUT_OUTPUT )
  FIELDDB_ELEMENT( CollisionCollection, surfaceSpeed, INPUT_OUTPUT )
  FIELDDB_ELEMENT( CollisionCollection, collidableSelectionGroups, INPUT_OUTPUT )
  FIELDDB_ELEMENT( CollisionCollection, contactReportMode, INITIALIZE_ONLY )

}

CollisionCollection::CollisionCollection( Inst< MFString  > _appliedParameters,
                                          Inst< SFFloat   > _bounce,
                                          Inst< MFCollidable > _collidables,
                                          Inst< MFCollidableGroup > _collidableGroups,
                                          Inst< MFCollidableExceptionGroup > _collidableExceptionGroups,
                                          Inst< SFBool    > _enabled,
                                          Inst< SFVec2f   > _frictionCoefficients,
                                          Inst< SFNode    > _metadata,
                                          Inst< SFFloat   > _minBounceSpeed,
                                          Inst< SFVec2f   > _slipFactors,
                                          Inst< SFFloat   > _softnessConstantForceMix,
                                          Inst< SFFloat   > _softnessErrorCorrection,
                                          Inst< SFVec2f   > _surfaceSpeed,
                                          Inst< ValueUpdater > _valueUpdater,
                                          Inst< MFCollidableSelectionGroup > _collidableSelectionGroups,
                                          Inst< SFString > _contactReportMode ) :
  X3DChildNode( _metadata ),
  appliedParameters( _appliedParameters ),
  bounce( _bounce ),
  collidables( _collidables ),
  collidableGroups( _collidableGroups ),
  collidableExceptionGroups( _collidableExceptionGroups ),
  collidableSelectionGroups( _collidableSelectionGroups ),
  enabled( _enabled ),
  frictionCoefficients( _frictionCoefficients ),
  minBounceSpeed( _minBounceSpeed ),
  slipFactors( _slipFactors ),
  softnessConstantForceMix( _softnessConstantForceMix ),
  softnessErrorCorrection( _softnessErrorCorrection ),
  surfaceSpeed( _surfaceSpeed ),
  engine_thread( NULL ),
  valueUpdater( _valueUpdater ),
  collisionCollectionInitialized( false ),
  rbc( NULL ),
  contactReportMode( _contactReportMode ) {

  type_name = "CollisionCollection";
  database.initFields( this );

  valueUpdater->setOwner( this );
  valueUpdater->setName( "valueUpdater" );

  collidables->setOwner( this );

  collidableExceptionGroups->setOwner( this );
  collidableSelectionGroups->setOwner( this );

  appliedParameters->push_back( "BOUNCE" );
  bounce->setValue( 0.0 );
  enabled->setValue( true );
  frictionCoefficients->setValue( Vec2f( 0, 0 ) );
  minBounceSpeed->setValue( (H3DFloat)0.1 );
  slipFactors->setValue( Vec2f( 0, 0 ) );
  softnessConstantForceMix->setValue( (H3DFloat)0.00001 );
  softnessErrorCorrection->setValue( (H3DFloat)0.8 );
  surfaceSpeed->setValue( Vec2f( 0, 0 ) );

  contactReportMode->addValidValue( "DEFAULT" );
  contactReportMode->addValidValue( "ALL" );
  contactReportMode->setValue( "DEFAULT" );

  appliedParameters->route( valueUpdater );
  bounce->route( valueUpdater );
  enabled->route( valueUpdater );
  frictionCoefficients->route( valueUpdater );
  minBounceSpeed->route( valueUpdater );
  slipFactors->route( valueUpdater );
  softnessConstantForceMix->route( valueUpdater );
  softnessErrorCorrection->route( valueUpdater );
  surfaceSpeed->route( valueUpdater );
}

CollisionCollection::~CollisionCollection() {
  deleteCollidableGroups();
  deleteCollidableExceptionGroups();
  deleteCollidableSelectionGroups();
  deleteCollidables();
}

void CollisionCollection::MFCollidable::onAdd( Node *n ) {
  MFNode::onAdd( n );
  // Check that the type is correct
  if( !dynamic_cast<X3DNBodyCollidableNode *>(n) &&
      !dynamic_cast<X3DNBodyCollisionSpaceNode *>(n) ) {
    Node *pi = getPrototypeNode( n );
    if( !dynamic_cast<X3DNBodyCollidableNode *>(pi) &&
        !dynamic_cast<X3DNBodyCollisionSpaceNode *>(pi) ) {
      stringstream s;
      s << "Expecting " << typeid(X3DNBodyCollidableNode).name();
      s << " or " << typeid(X3DNBodyCollisionSpaceNode).name();
      throw InvalidNodeType( n->getTypeName(),
                             s.str(),
                             H3D_FULL_LOCATION );
    }
  }
}

void CollisionCollection::MFCollidable::onRemove( Node *n ) {
  CollisionCollection *cc = static_cast<CollisionCollection*>(getOwner());

  if( n )
    cc->deleteCollidable( n );

  MFNode::onRemove( n );
}

void CollisionCollection::traverseSG( H3D::TraverseInfo &ti ) {
  X3DChildNode::traverseSG( ti );
  H3D::PhysicsEngineThread *pt = NULL;
  ti.getUserData( "PhysicsEngine", (void * *)&pt );
  if( pt ) {
    if( engine_thread == NULL ) {
      engine_thread = pt;
    }
    if( engine_thread == pt ) {
      initializeCollisionCollection();
      const NodeVector &c = collidables->getValue();
      for( unsigned int i = 0; i < c.size(); ++i ) {
        if( c[i] ) {
          initializeCollidable( c[i] );
          c[i]->traverseSG( ti );
        }
      }

      const NodeVector &cg = collidableGroups->getValue();
      for( unsigned int i = 0; i < cg.size(); ++i ) {
        if( cg[i] ) {
          cg[i]->traverseSG( ti );
        }
      }

      const NodeVector &ceg = collidableExceptionGroups->getValue();
      for( unsigned int i = 0; i < ceg.size(); i++ ) {
        if( ceg[i] ) {
          ceg[i]->traverseSG( ti );
        }
      }

      const NodeVector &csg = collidableSelectionGroups->getValue();
      for( unsigned int i = 0; i < csg.size(); i++ ) {
        if( csg[i] ) {
          csg[i]->traverseSG( ti );
        }
      }
    }
  }
}

void CollisionCollection::initializeCollisionCollection() {
  if( !collisionCollectionInitialized && engine_thread ) {
    PhysicsEngineParameters::GlobalContactParameters p = getGlobalContactParameters();
    engine_thread->setGlobalContactParameters( p );
    collisionCollectionInitialized = true;
  }
}

void CollisionCollection::initializeCollidable( Node * n ) {
  if( X3DNBodyCollidableNode *c = dynamic_cast<X3DNBodyCollidableNode *>(n) ) {
    c->initializeCollidable( engine_thread, this );
    c->setRigidBodyCollection( rbc );
  } else if( CollisionSpace *s = dynamic_cast<CollisionSpace *>(n) ) {
    s->initializeSpace( engine_thread, this );
  }
}

void CollisionCollection::deleteCollidable( Node * n ) {
  if( X3DNBodyCollidableNode *c = dynamic_cast<X3DNBodyCollidableNode *>(n) ) {
    c->deleteCollidable();
    c->setRigidBodyCollection();
  } else if( CollisionSpace *s = dynamic_cast<CollisionSpace *>(n) ) {
    s->deleteSpace();
  }
}

void CollisionCollection::deleteCollidables() {
  collidables->clear();
}

void CollisionCollection::deleteCollidableGroups() {
  collidableGroups->clear();
}

void CollisionCollection::deleteCollidableExceptionGroups() {
  collidableExceptionGroups->clear();
}

void CollisionCollection::deleteCollidableSelectionGroups() {
  collidableSelectionGroups->clear();
}

PhysicsEngineParameters::GlobalContactParameters CollisionCollection::getGlobalContactParameters() {
  PhysicsEngineParameters::GlobalContactParameters p;
  p.collision_enabled = enabled->getValue();
  p.applied_parameters = appliedParameters->getValue();
  p.bounce = bounce->getValue();
  p.friction_coefficients = frictionCoefficients->getValue();
  p.min_bounce_speed = minBounceSpeed->getValue();
  p.slip_factors = slipFactors->getValue();
  p.softness_constant_force_mix = softnessConstantForceMix->getValue();
  p.softness_error_correction = softnessErrorCorrection->getValue();
  p.surface_speed = surfaceSpeed->getValue();
  p.contact_report_mode = contactReportMode->getValue();
  p.engine_thread = engine_thread;

  return p;
}

void CollisionCollection::ValueUpdater::update() {
  CollisionCollection * cc = static_cast<CollisionCollection *>(getOwner());
  if( cc->engine_thread ) {
    PhysicsEngineParameters::GlobalContactParameters p = cc->getGlobalContactParameters();
    cc->engine_thread->setGlobalContactParameters( p );
  }

  EventCollectingField< PeriodicUpdate< Field > >::update();
}
void CollisionCollection::setRigidBodyCollection( RigidBodyCollection* _rbc ) {
  rbc = _rbc;
  const NodeVector &c = collidables->getValue();
  for( unsigned int i = 0; i < c.size(); ++i ) {
    if( X3DNBodyCollidableNode* cn = dynamic_cast<X3DNBodyCollidableNode*>(c[i]) ) {
      cn->setRigidBodyCollection( rbc );
    }
  }
}

