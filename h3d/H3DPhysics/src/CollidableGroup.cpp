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
/// \file CollidableGroup.cpp
/// \brief cpp file for CollidableGroup, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////


#include <H3D/H3DPhysics/CollidableGroup.h>
#include <H3D/H3DPhysics/X3DNBodyCollisionSpaceNode.h>

using namespace H3D;

H3DNodeDatabase CollidableGroup::database( "CollidableGroup", 
                                          &(newInstance< CollidableGroup >), 
                                          typeid( CollidableGroup ),
                                          &X3DChildNode::database);

namespace CollidableGroupInternals {
  FIELDDB_ELEMENT( CollidableGroup, collidables, INPUT_OUTPUT )
}

CollidableGroup::CollidableGroup(Inst< SFNode  >  _metadata , 
                                 Inst< MFCollidable > _collidables ):
X3DChildNode( _metadata ),
collidables( _collidables ) {

  type_name = "CollidableGroup";
  database.initFields( this );

  collidables->setOwner( this );

  collidableGroup_id = (H3DCollidableGroupId)this;
}

CollidableGroup::~CollidableGroup() {
  deleteCollidableGroup();  
}

void CollidableGroup::traverseSG(H3D::TraverseInfo &ti) {
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
        if( !collidable_id){
          Console (4) << "Warning: X3DNBodyCollidableNode " <<
            newX3DNBodyCollidableNodes[i]->getName() << " in collidableGroup " <<
            getName() << " is not initialized."<<
            " All collidables must be added to collisionCollection separately"<<
            " before being added to a collidableGroup."<< endl;
        }
      }
      newX3DNBodyCollidableNodes.clear();

     for( unsigned int i = 0; i < newX3DNBodyCollisionSpaceNodes.size(); i++ ) {
        H3DSpaceId space_id= newX3DNBodyCollisionSpaceNodes[i]->getSpaceId();
        if( !space_id){
          Console (4) << "Warning: X3DNBodyCollisionSpaceNode " <<
            newX3DNBodyCollisionSpaceNodes[i]->getName() << " in collidableGroup " <<
            getName() << " is not initialized."<<
            " All collidables must be added to collisionCollection separately"<<
            " before being added to a collidableGroup."<< endl;
        }
      }
      newX3DNBodyCollisionSpaceNodes.clear();

    //}
  }
}

void CollidableGroup::deleteCollidableGroup() {
  collidables->clear();
}

void CollidableGroup::MFCollidable::onAdd( Node *n ) {
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
    CollidableGroup *cg = static_cast< CollidableGroup * >( getOwner() ); 
    cn->addToCollidableGroup( cg->collidableGroup_id );
    cg->newX3DNBodyCollidableNodes.push_back( cn );
  }
  else if ( dynamic_cast< X3DNBodyCollisionSpaceNode * >( n ) )
  {
    CollidableGroup *cg = static_cast< CollidableGroup * >( getOwner() ); 
    cn->addToCollidableGroup( cg->collidableGroup_id );
  }  
}
void CollidableGroup::MFCollidable::onRemove( Node *n ) {

  CollidableGroup *cg = static_cast< CollidableGroup * >( getOwner() ); 
  if ( X3DNBodyCollidableNode *cn = dynamic_cast< X3DNBodyCollidableNode * >( n ) )
    cn->removeFromCollidableGroup( cg->collidableGroup_id );
  else if ( X3DNBodyCollisionSpaceNode *cs = dynamic_cast< X3DNBodyCollisionSpaceNode * >( n ) )
    cs->removeFromCollidableGroup( cg->collidableGroup_id );      

  MFNode::onRemove( n );
}
