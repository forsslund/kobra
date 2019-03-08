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
/// \file CollidableOffset.cpp
/// \brief cpp file for CollidableOffset, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////


#include <H3D/H3DPhysics/CollidableOffset.h>
#include <H3D/H3DPhysics/CollidableShape.h>
#include <H3D/H3DPhysics/CollisionSpace.h>

using namespace H3D;

H3DNodeDatabase CollidableOffset::database( "CollidableOffset", 
                                           &(newInstance< CollidableOffset >), 
                                           typeid( CollidableOffset ),
                                           &X3DNBodyCollidableNode::database);

namespace CollidableOffsetInternals {
  FIELDDB_ELEMENT( CollidableOffset, collidable, INITIALIZE_ONLY )
}

CollidableOffset::CollidableOffset(Inst< SFBool > _enabled,
                                   Inst< SFNode  >  _metadata, 
                                   Inst< SFRotation >  _rotation,
                                   Inst< SFVec3f >  _translation,
                                   Inst< SFVec3f >  _scale,
                                   Inst< SFBound     > _bound,
                                   Inst< SFVec3f >  _bboxCenter,
                                   Inst< SFVec3f  >  _bboxSize,
                                   Inst< ValueUpdater > _valueUpdater,
                                   Inst< MFEngineOptions > _engineOptions,
                                   Inst< SFCollidable > _collidable ):
X3DNBodyCollidableNode( _enabled, _metadata, _rotation, 
                       _translation, _scale, _bound, _bboxCenter, 
                       _bboxSize, _valueUpdater, _engineOptions ),
                       collidable( _collidable )
{

  type_name = "CollidableOffset";
  database.initFields( this );

}

CollidableOffset::~CollidableOffset() {
  if ( collidableInitialized() )
    deleteCollidable();
}

int CollidableOffset::initializeCollidable( H3D::PhysicsEngineThread *pt, X3DNode *parent ) {
  if( collidableInitialized() ) return 1;

  // If collidable field value is NULL, do not create a collidable in
  // the physics engine at all, since it is meaningless.
  if ( collidable->getValue() == NULL )
    return 0;

  X3DNBodyCollidableNode *c = 
    dynamic_cast< H3D::X3DNBodyCollidableNode * >( collidable->getValue() );

  // Initialize the collidable
  c->initializeCollidable( pt, NULL );

  // Purge all pending updates to parameters since we will initialise all
  // parameters when the body is created. Otherwise the parameters will be
  // set again next frame
  valueUpdater->upToDate();
  engine_thread= pt;
  PhysicsEngineParameters::OffsetParameters* p= getCollidableParameters ( true );

  // Get the space id of parent space 
  if ( CollisionSpace *cs = dynamic_cast< CollisionSpace * >( parent ) ) {
    p->setParentSpaceId( cs->getSpaceId() );
  } else if ( parent == NULL ) {
    p->setParentSpaceId( CollidableParameters::NO_SPACE );
  } else {
    p->setParentSpaceId( CollidableParameters::WORLD_SPACE );
  }

  collidable_id = engine_thread->addCollidable( *p );
  collidable_id_map[collidable_id] = this;
  return 0;
}

int CollidableOffset::deleteCollidable() {
  if ( collidableInitialized() ) {
    if ( collidable->getValue() != NULL ) {
      static_cast< X3DNBodyCollidableNode * >(collidable->getValue())->deleteCollidable();
    }
    std::map< H3DCollidableId, X3DNBodyCollidableNode * >::iterator j = 
      collidable_id_map.find( collidable_id );
    if( j != collidable_id_map.end() ) collidable_id_map.erase( j );
    engine_thread->removeCollidable( collidable_id );
    engine_thread = NULL;
    return 0;
  }
  return -1;
}

Shape * CollidableOffset::getShape() {
  if ( CollidableShape *c = dynamic_cast< CollidableShape* >(collidable->getValue()) ) {
    return static_cast< Shape* >( c->shape->getValue() );
  } else if ( CollidableOffset *c = dynamic_cast< CollidableOffset* >(collidable->getValue()) ) {
    return c->getShape();
  }
  return NULL;
}

PhysicsEngineParameters::OffsetParameters* CollidableOffset::getCollidableParameters( bool all_params ) {
  OffsetParameters* p= static_cast<OffsetParameters*> ( X3DNBodyCollidableNode::getCollidableParameters ( all_params ) );
  X3DNBodyCollidableNode *c = 
    dynamic_cast< H3D::X3DNBodyCollidableNode * >( collidable->getValue() );

  p->collidable = c->getCollidableId();

  return p;
}
void CollidableOffset::addToCollidableGroup( H3DCollidableGroupId groupId ) {
  X3DNBodyCollidableNode::addToCollidableGroup( groupId );
  X3DNBodyCollidableNode *c = 
    dynamic_cast< H3D::X3DNBodyCollidableNode * >( collidable->getValue() );
  c->addToCollidableGroup( groupId );
}

void CollidableOffset::removeFromCollidableGroup( H3DCollidableGroupId groupId ) {
  X3DNBodyCollidableNode *c = 
    dynamic_cast< H3D::X3DNBodyCollidableNode * >( collidable->getValue() );
  c->removeFromCollidableGroup( groupId );
  X3DNBodyCollidableNode::removeFromCollidableGroup( groupId );
}
