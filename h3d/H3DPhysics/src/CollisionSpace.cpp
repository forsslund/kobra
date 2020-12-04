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
/// \file CollisionSpace.cpp
/// \brief cpp file for CollisionSpace, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/H3DPhysics/CollisionSpace.h>
#include <H3D/H3DPhysics/CollidableShape.h>
#include <H3D/H3DPhysics/X3DNBodyCollidableNode.h>

using namespace H3D;



H3DNodeDatabase CollisionSpace::database( "CollisionSpace", 
                                         &(newInstance< CollisionSpace >), 
                                         typeid( CollisionSpace ),
                                         &X3DNBodyCollisionSpaceNode::database);

namespace CollisionSpaceInternals {
  FIELDDB_ELEMENT( CollisionSpace, collidables, INPUT_OUTPUT )
  FIELDDB_ELEMENT( CollisionSpace, useGeometry, INITIALIZE_ONLY )
}

CollisionSpace::CollisionSpace( Inst< MFCollidable  > _collidables,
                               Inst< SFBool  > _enabled,
                               Inst< SFNode  > _metadata,
                               Inst< SFBool  > _useGeometry,
                               Inst< SFBound > _bound,
                               Inst< SFVec3f > _bboxCenter,
                               Inst< SFVec3f > _bboxSize ) :
X3DNBodyCollisionSpaceNode( _enabled, _metadata, _bound, 
                           _bboxCenter, _bboxSize ),
                           collidables( _collidables ),
                           useGeometry( _useGeometry ),
  use_union_bound( false )
{

  type_name = "CollisionSpace";
  database.initFields( this );

  useGeometry->setValue( false );
}

CollisionSpace::~CollisionSpace() {
  if ( isInitialized() ) {
    deleteSpace();
  }
}

void CollisionSpace::initialize() {
  const Vec3f &size = bboxSize->getValue();
  if( size.x == -1 && size.y == -1 && size.z == -1 ) {
    use_union_bound = true;
  } else {
    BoxBound *bb = new BoxBound();
    bb->center->setValue( bboxCenter->getValue() );
    bb->size->setValue( bboxSize->getValue() );
    bound->setValue( bb );
  }
  X3DNBodyCollisionSpaceNode::initialize();
} 

void CollisionSpace::traverseSG( H3D::TraverseInfo &ti ) {
  X3DNBodyCollisionSpaceNode::traverseSG( ti );
  updateNodeChanges();
  const NodeVector &c = collidables->getValue();
  for( unsigned int i = 0; i < c.size(); ++i ) {
    if( c[i] )
      c[i]->traverseSG( ti );
  }
}

int CollisionSpace::initializeSpace( H3D::PhysicsEngineThread *pt, X3DNode *n ) {
  if ( isInitialized() ) return 1;
  PhysicsEngineParameters::SpaceParameters *p = new  PhysicsEngineParameters::SpaceParameters;
  p->setEngine(*pt) ;

  p->setEnabled( enabled->getValue() );
  p->setUseGeometry( useGeometry->getValue() );

  // Get the space id of parent space 
  if ( CollisionSpace *cs = dynamic_cast< CollisionSpace * >( n ) ) {
    p->setParentSpaceId( cs->getSpaceId() ) ;
  } else if ( n == NULL ) {
    p->setParentSpaceId( CollidableParameters::NO_SPACE );
  } else {
    p->setParentSpaceId( CollidableParameters::WORLD_SPACE );
  }

  engine_thread = pt;
  space_id = engine_thread->addSpace( *p );
  space_id_map[space_id] = this;
  return 0;
}

bool CollisionSpace::isInitialized() {
  return engine_thread != NULL;
}

int CollisionSpace::deleteSpace() {
  if ( isInitialized() ) {
    collidables->clear();
    updateNodeChanges();
    std::map< H3DSpaceId, X3DNBodyCollisionSpaceNode * >::iterator j = space_id_map.find( space_id );
    if( j != space_id_map.end() ) space_id_map.erase( j );
    engine_thread->removeSpace( space_id );
    engine_thread = NULL;
    return 0;
  }
  return -1;
}

void CollisionSpace::updateNodeChanges() {
  for( NodesChangedList::iterator i = nodes_changed.begin();
    i != nodes_changed.end(); ++i ) {
      Node *n = (*i).second.get();

      if( (*i).first ) {
        // add
        if( X3DNBodyCollidableNode *cs = dynamic_cast< X3DNBodyCollidableNode * >(n) ) {
          cs->initializeCollidable( engine_thread, this );
        } else if ( CollisionSpace *cs = dynamic_cast< CollisionSpace * >(n) ) {
          cs->initializeSpace( engine_thread, this );
        }
      } else {
        // remove
        if( X3DNBodyCollidableNode *cs = dynamic_cast< X3DNBodyCollidableNode * >(n) ) {
          cs->deleteCollidable();
        } else if ( CollisionSpace *cs = dynamic_cast< CollisionSpace * >(n) ) {
          cs->deleteSpace();
        }
      }
  }
  nodes_changed.clear();
}

void CollisionSpace::addToCollidableGroup( H3DCollidableGroupId groupId ) {
  X3DNBodyCollisionSpaceNode::addToCollidableGroup( groupId );
  const NodeVector &c = collidables->getValue();
  for( unsigned int i = 0; i < c.size(); ++i ) {
    if( X3DNBodyCollidableNode *cn = dynamic_cast< X3DNBodyCollidableNode * >(c[i]) ) {
      cn->addToCollidableGroup( groupId );
    } else if ( X3DNBodyCollisionSpaceNode *cs = dynamic_cast< X3DNBodyCollisionSpaceNode * >(c[i]) ) {
      cs->addToCollidableGroup( groupId );
    }
  }
}

void CollisionSpace::removeFromCollidableGroup( H3DCollidableGroupId groupId ) {
  const NodeVector &c = collidables->getValue();
  for( unsigned int i = 0; i < c.size(); ++i ) {
    if( X3DNBodyCollidableNode *cn = dynamic_cast< X3DNBodyCollidableNode * >(c[i]) ) {
      cn->removeFromCollidableGroup( groupId );
    } else if ( X3DNBodyCollisionSpaceNode *cs = dynamic_cast< X3DNBodyCollisionSpaceNode * >(c[i]) ) {
      cs->removeFromCollidableGroup( groupId );
    }
  }
  X3DNBodyCollisionSpaceNode::removeFromCollidableGroup( groupId );
}

void CollisionSpace::MFCollidable::onAdd( Node *n ) {
  MFNode::onAdd( n );
  // Check that the type is correct
  if( !dynamic_cast< X3DNBodyCollidableNode * >( n ) && 
    !dynamic_cast< X3DNBodyCollisionSpaceNode * >( n ) ) {
      Node *pi = getPrototypeNode( n );
      if( !dynamic_cast< X3DNBodyCollidableNode * >( pi ) &&
        !dynamic_cast< X3DNBodyCollisionSpaceNode * >( pi ) ) {
          stringstream s;
          s << "Expecting " << typeid( X3DNBodyCollidableNode ).name();
          s << " or " << typeid( X3DNBodyCollisionSpaceNode ).name();
          throw InvalidNodeType( n->getTypeName(),
            s.str(),
            H3D_FULL_LOCATION );
      }
  }
  // Add to nodes_changed
  H3DBoundedObject *c = dynamic_cast< H3DBoundedObject* >( n );
  CollisionSpace *cs = static_cast< CollisionSpace* >( getOwner() );
  if( n ) {
    cs->nodes_changed.push_back( make_pair(true, AutoRef< Node >(n) ) );
    c->bound->route( cs->bound );
  }
}

void CollisionSpace::MFCollidable::onRemove( Node *n ) {
  H3DBoundedObject *c = dynamic_cast< H3DBoundedObject* >( n );
  CollisionSpace *cs = static_cast< CollisionSpace* >( getOwner() );
  if(n) {
    cs->nodes_changed.push_back( make_pair(false, AutoRef< Node >(n) ) );
    c->bound->unroute( cs->bound );
  }
  MFNode::onRemove( n );
}

void CollisionSpace::SFBound::update() {
  value = Bound::SFBoundUnion( routes_in.begin(),
    routes_in.end() );
}
