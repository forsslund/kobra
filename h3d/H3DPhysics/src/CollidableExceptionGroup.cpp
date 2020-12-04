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
/// \file CollidableExceptionGroup.cpp
/// \brief cpp file for CollidableExceptionGroup, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////


#include <H3D/H3DPhysics/CollidableExceptionGroup.h>

using namespace H3D;

H3DNodeDatabase CollidableExceptionGroup::database( "CollidableExceptionGroup", 
                                          &(newInstance< CollidableExceptionGroup >), 
                                          typeid( CollidableExceptionGroup ),
                                          &X3DChildNode::database);

namespace CollidableExceptionGroupInternals {
  FIELDDB_ELEMENT( CollidableExceptionGroup, collidables, INPUT_OUTPUT )
}

CollidableExceptionGroup::CollidableExceptionGroup(Inst< SFNode  >  _metadata , 
                                                    Inst< MFCollidable > _collidables ) :
  X3DChildNode( _metadata ),
  collidables( _collidables ) {

  type_name = "CollidableExceptionGroup";
  database.initFields( this );

  collidables->setOwner( this );

  collidableExceptionGroup_id = reserveCollidableExceptionGroupId();
}

CollidableExceptionGroup::~CollidableExceptionGroup() {
  deleteCollidableExceptionGroup();  
  freeCollidableExceptionGroupId( collidableExceptionGroup_id );
}

void CollidableExceptionGroup::traverseSG(H3D::TraverseInfo &ti) {
  X3DChildNode::traverseSG( ti );
  H3D::PhysicsEngineThread *pt = NULL;
  ti.getUserData("PhysicsEngine", (void * *) &pt );
  if ( pt ) {
    //if ( engine_thread == NULL ) {
    //  engine_thread = pt;
    //}
    //if ( engine_thread == pt ) {

      for( unsigned int i = 0; i < newX3DNBodyCollidableNodes.size(); i++ ) {
        H3DCollidableId collidable_id = newX3DNBodyCollidableNodes[i]->getCollidableId();
        if( !collidable_id ){
          Console (4) << "Warning: X3DNBodyCollidableNode " <<
            newX3DNBodyCollidableNodes[i]->getName() << " in collidableExceptionGroup " <<
            getName() << " is not initialized."<<
            " All collidables must be added to collisionCollection separately"<<
            " before being added to a collidableGroup."<< endl;
        }
      }
      newX3DNBodyCollidableNodes.clear();

      for( unsigned int i = 0; i < newX3DNBodyCollisionSpaceNodes.size(); i++ ) {
        H3DSpaceId space_id= newX3DNBodyCollisionSpaceNodes[i]->getSpaceId();
        if( !space_id ){
          Console (4) << "Warning: X3DNBodyCollisionSpaceNode " <<
            newX3DNBodyCollisionSpaceNodes[i]->getName() << " in collidableExceptionGroup " <<
            getName() << " is not initialized."<<
            " All collidables must be added to collisionCollection separately"<<
            " before being added to a collidableGroup."<< endl;
        }
      }
      newX3DNBodyCollisionSpaceNodes.clear();

    //}
  }
}

void CollidableExceptionGroup::deleteCollidableExceptionGroup() {
  collidables->clear();
}

H3DCollidableExceptionGroupId CollidableExceptionGroup::reserveCollidableExceptionGroupId() {
  static const int MAXIMUMNUMBEROFEXCEPTIONGROUPS = 32;
  static H3DCollidableExceptionGroupId group_id;

  H3DCollidableExceptionGroupId id;
  CollidableParameters::CollidableExceptionGroupList &fList = getFreeGroupIds();
  if( !fList.empty() ) {
    id = fList.back();
    fList.pop_back();
  } else {
    id = group_id;
    if( group_id < MAXIMUMNUMBEROFEXCEPTIONGROUPS - 1 )
      group_id++;
    else {
      Console( 4 ) << "WARNING: Maximum number of CollidableExceptionGroups are reached. Undefined Behaviour!!!" << endl;
    }
  }
  return id;
}

void CollidableExceptionGroup::freeCollidableExceptionGroupId( H3DCollidableExceptionGroupId id ) {
  CollidableParameters::CollidableExceptionGroupList &fList = getFreeGroupIds();
  fList.push_back( id );
}

CollidableParameters::CollidableExceptionGroupList &CollidableExceptionGroup::getFreeGroupIds() {
  static CollidableParameters::CollidableExceptionGroupList freeGroupIds;
  return freeGroupIds;
}

void CollidableExceptionGroup::MFCollidable::onAdd( Node *n ) {
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
  else if ( X3DNBodyCollidableNode *cn = dynamic_cast< X3DNBodyCollidableNode * >( n ) )
  {
    CollidableExceptionGroup *cg = static_cast< CollidableExceptionGroup * >( getOwner() );
    cn->addToCollidableExceptionGroup( cg->collidableExceptionGroup_id );
    cg->newX3DNBodyCollidableNodes.push_back( cn );
  }
  else if ( X3DNBodyCollisionSpaceNode *cs = dynamic_cast< X3DNBodyCollisionSpaceNode * >( n ) )
  {
    CollidableExceptionGroup *cg = static_cast< CollidableExceptionGroup * >( getOwner() );
    cs->addToCollidableExceptionGroup( cg->collidableExceptionGroup_id );
    cg->newX3DNBodyCollisionSpaceNodes.push_back( cs );
  }  
}
void CollidableExceptionGroup::MFCollidable::onRemove( Node *n ) {

  CollidableExceptionGroup *cg = static_cast< CollidableExceptionGroup * >( getOwner() ); 
  if ( X3DNBodyCollidableNode *cn = dynamic_cast< X3DNBodyCollidableNode * >( n ) )
    cn->removeFromCollidableExceptionGroup( cg->collidableExceptionGroup_id );
  else if ( X3DNBodyCollisionSpaceNode *cs = dynamic_cast< X3DNBodyCollisionSpaceNode * >( n ) )
    cs->removeFromCollidableExceptionGroup( cg->collidableExceptionGroup_id );      

  MFNode::onRemove( n );
}
