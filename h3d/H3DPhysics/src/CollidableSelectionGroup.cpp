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
/// \file CollidableSelectionGroup.cpp
/// \brief cpp file for CollidableSelectionGroup, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////


#include <H3D/H3DPhysics/CollidableSelectionGroup.h>

using namespace H3D;

H3DNodeDatabase CollidableSelectionGroup::database( "CollidableSelectionGroup",
                                                    &(newInstance< CollidableSelectionGroup >),
                                                    typeid(CollidableSelectionGroup),
                                                    &X3DChildNode::database );

namespace CollidableSelectionGroupInternals {
  FIELDDB_ELEMENT( CollidableSelectionGroup, collidables, INPUT_OUTPUT )
  FIELDDB_ELEMENT( CollidableSelectionGroup, selectedCollidables, INPUT_OUTPUT )
}

CollidableSelectionGroup::CollidableSelectionGroup( Inst< SFNode  >  _metadata,
                                                    Inst< MFCollidable > _collidables,
                                                    Inst< MFSelectedCollidable > _selectedCollidables ) :
  X3DChildNode( _metadata ),
  collidables( _collidables ),
  selectedCollidables( _selectedCollidables ) {

  type_name = "CollidableSelectionGroup";
  database.initFields( this );

  collidables->setOwner( this );
  selectedCollidables->setOwner( this );

  collidableSelectionGroup_id = reserveCollidableSelectionGroupId();
}

CollidableSelectionGroup::~CollidableSelectionGroup() {
  deleteCollidableSelectionGroup();
  freeCollidableSelectionGroupId( collidableSelectionGroup_id );
}

void CollidableSelectionGroup::traverseSG( H3D::TraverseInfo &ti ) {
  X3DChildNode::traverseSG( ti );
  H3D::PhysicsEngineThread *pt;
  ti.getUserData( "PhysicsEngine", (void * *)&pt );
  if( pt ) {

    for( unsigned int i = 0; i < new_X3DNBodyCollidableNodes.size(); i++ ) {
      H3DCollidableId collidable_id = new_X3DNBodyCollidableNodes[i]->getCollidableId();
      if( !collidable_id ) {
        Console( 4 ) << "Warning: X3DNBodyCollidableNode " <<
          new_X3DNBodyCollidableNodes[i]->getName() << " in collidableSelectionGroup " <<
          getName() << " is not initialized." <<
          " All collidables must be added to collisionCollection separately" <<
          " before being added to a collidableGroup." << endl;
      }
    }
    new_X3DNBodyCollidableNodes.clear();

    for( unsigned int i = 0; i < new_X3DNBodyCollisionSpaceNodes.size(); i++ ) {
      H3DSpaceId space_id = new_X3DNBodyCollisionSpaceNodes[i]->getSpaceId();
      if( !space_id ) {
        Console( 4 ) << "Warning: X3DNBodyCollisionSpaceNode " <<
          new_X3DNBodyCollisionSpaceNodes[i]->getName() << " in collidableSelectionGroup " <<
          getName() << " is not initialized." <<
          " All collidables must be added to collisionCollection separately" <<
          " before being added to a collidableGroup." << endl;
      }
    }
    new_X3DNBodyCollisionSpaceNodes.clear();

  }
}

void CollidableSelectionGroup::deleteCollidableSelectionGroup() {
  collidables->clear();
  selectedCollidables->clear();
}

bool CollidableSelectionGroup::isNodeInGroup( Node *n ) {

  if( find( previous_selected_collidables.begin(), previous_selected_collidables.end(), n )
      != previous_selected_collidables.end() )
    return true;

  if( find( previous_collidables.begin(), previous_collidables.end(), n )
      != previous_collidables.end() )
    return true;

  return false;
}

H3DCollidableSelectionGroupId CollidableSelectionGroup::reserveCollidableSelectionGroupId() {
  static const int MAXIMUMNUMBEROFEXCEPTIONGROUPS = 16;
  static H3DCollidableSelectionGroupId group_id;

  H3DCollidableSelectionGroupId id;
  CollidableParameters::CollidableSelectionGroupList &fList = getFreeGroupIds();
  if( !fList.empty() ) {
    id = static_cast<H3DCollidableSelectionGroupId>(fList.back());
    fList.pop_back();
  } else {
    id = group_id;
    if( group_id < MAXIMUMNUMBEROFEXCEPTIONGROUPS - 1 )
      group_id++;
    else {
      Console( 4 ) << "WARNING: Maximum number of CollidableSelectionGroups are reached. Undefined Behaviour!!!" << endl;
    }
  }
  return id;
}

void CollidableSelectionGroup::freeCollidableSelectionGroupId( H3DCollidableSelectionGroupId id ) {
  CollidableParameters::CollidableSelectionGroupList &fList = getFreeGroupIds();
  fList.push_back( static_cast<H3DUtil::H3DFloat>(id) );
}

CollidableParameters::CollidableSelectionGroupList &CollidableSelectionGroup::getFreeGroupIds() {
  static CollidableParameters::CollidableSelectionGroupList freeGroupIds;
  return freeGroupIds;
}

void CollidableSelectionGroup::MFCollidable::onAdd( Node *n ) {
  MFNode::onAdd( n );
  CollidableSelectionGroup *cg = static_cast<CollidableSelectionGroup *>(getOwner());

  if( cg->isNodeInGroup( n ) ) {
    Console(4) << "ERROR: Collidable which is already in the group is trying to be added to collidables " <<
      " in CollidableSelectionGroup. " << endl;
    cg->collidables->erase( n );
    return;
  }

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
  } else if( X3DNBodyCollidableNode *cn = dynamic_cast<X3DNBodyCollidableNode *>(n) ) {
    // Check if the collidable is already in another selection group.
    if( cn->inCollidableSelectionGroup->getValue().size() > 0 ) {
      Console(4) << "ERROR: Collidable which is already in another CollidableSelectionGroup is trying to be added to collidables " <<
        " in CollidableSelectionGroup. " << endl;
      cg->collidables->erase( n );
      return;
    }
    cn->addToCollidableSelectionGroup( cg->collidableSelectionGroup_id );
    cg->new_X3DNBodyCollidableNodes.push_back( cn );
    cg->previous_collidables.push_back( cn );
  } else if( X3DNBodyCollisionSpaceNode *cs = dynamic_cast<X3DNBodyCollisionSpaceNode *>(n) ) {
    // Check if the collidable is already in another selection group.
    if( cs->inCollidableSelectionGroup->getValue().size() > 0 ) {
      Console(4) << "ERROR: Collidable which is already in another CollidableSelectionGroup is trying to be added to collidables " <<
        " in CollidableSelectionGroup. " << endl;
      cg->collidables->erase( n );
      return;
    }
    cs->addToCollidableSelectionGroup( cg->collidableSelectionGroup_id );
    cg->new_X3DNBodyCollisionSpaceNodes.push_back( cs );
    cg->previous_collidables.push_back( cs );
  }

}
void CollidableSelectionGroup::MFCollidable::onRemove( Node *n ) {
  CollidableSelectionGroup *cg = static_cast<CollidableSelectionGroup *>(getOwner());
  if( X3DNBodyCollidableNode *cn = dynamic_cast<X3DNBodyCollidableNode *>(n) )
    cn->removeFromCollidableSelectionGroup( cg->collidableSelectionGroup_id );
  else if( X3DNBodyCollisionSpaceNode *cs = dynamic_cast<X3DNBodyCollisionSpaceNode *>(n) )
    cs->removeFromCollidableSelectionGroup( cg->collidableSelectionGroup_id );

  std::vector<Node*>::iterator rn = find( cg->previous_collidables.begin(), cg->previous_collidables.end(), n );
  if( rn != cg->previous_collidables.end() )
    cg->previous_collidables.erase( rn );

  MFNode::onRemove( n );
}

void CollidableSelectionGroup::MFSelectedCollidable::onAdd( Node *n ) {
  MFNode::onAdd( n );
  CollidableSelectionGroup *cg = static_cast<CollidableSelectionGroup *>(getOwner());

  if( cg->isNodeInGroup( n ) ) {
    Console(4) << "ERROR: Collidable which is already in the group is trying to be added to selectedCollidables " <<
      " in CollidableSelectionGroup. " << endl;
    cg->selectedCollidables->erase( n );
    return;
  }

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
  } else if( X3DNBodyCollidableNode *cn = dynamic_cast<X3DNBodyCollidableNode *>(n) ) {
    // Check if the collidable is already in another selection group's collidable field.
    if( cn->inCollidableSelectionGroup->getValue().size() > 0 ) {
      std::vector< H3DFloat > currentList = cn->inCollidableSelectionGroup->getValue();
      for( std::vector< H3DFloat >::iterator j = currentList.begin(); j < currentList.end(); ++j ) {
        if( H3DAbs( (*j) - (int)(*j) ) < H3DUtil::Constants::f_epsilon ) {
          Console(4) << "ERROR: Collidable which is already in another CollidableSelectionGroup's collidables is trying to be added to selectedCollidables " <<
            " in CollidableSelectionGroup. " << endl;
          cg->selectedCollidables->erase( n );
          return;
        }
      }
    }
    cn->addToCollidableSelectionGroup( cg->collidableSelectionGroup_id, true );
    cg->new_X3DNBodyCollidableNodes.push_back( cn );
    cg->previous_selected_collidables.push_back( cn );
  } else if( X3DNBodyCollisionSpaceNode *cs = dynamic_cast<X3DNBodyCollisionSpaceNode *>(n) ) {
    // Check if the collidable is already in another selection group's collidable field.
    if( cs->inCollidableSelectionGroup->getValue().size() > 0 ) {
      std::vector< H3DFloat > currentList = cn->inCollidableSelectionGroup->getValue();
      for( std::vector< H3DFloat >::iterator j = currentList.begin(); j < currentList.end(); ++j ) {
        if( H3DAbs( (*j) - (int)(*j) ) < H3DUtil::Constants::f_epsilon ) {
          Console(4) << "ERROR: Collidable which is already in another CollidableSelectionGroup's collidables is trying to be added to selectedCollidables " <<
            " in CollidableSelectionGroup. " << endl;
          cg->selectedCollidables->erase( n );
          return;
        }
      }
    }
    cs->addToCollidableSelectionGroup( cg->collidableSelectionGroup_id, true );
    cg->new_X3DNBodyCollisionSpaceNodes.push_back( cs );
    cg->previous_selected_collidables.push_back( cs );
  }

}
void CollidableSelectionGroup::MFSelectedCollidable::onRemove( Node *n ) {

  CollidableSelectionGroup *cg = static_cast<CollidableSelectionGroup *>(getOwner());
  H3DCollidableSelectionGroupId shifted_id = cg->collidableSelectionGroup_id << 16;
  if( X3DNBodyCollidableNode *cn = dynamic_cast<X3DNBodyCollidableNode *>(n) )
    cn->removeFromCollidableSelectionGroup( shifted_id );
  else if( X3DNBodyCollisionSpaceNode *cs = dynamic_cast<X3DNBodyCollisionSpaceNode *>(n) )
    cs->removeFromCollidableSelectionGroup( shifted_id );

  std::vector<Node*>::iterator rn = find( cg->previous_selected_collidables.begin(), cg->previous_selected_collidables.end(), n );
  if( rn != cg->previous_selected_collidables.end() )
    cg->previous_selected_collidables.erase( rn );

  MFNode::onRemove( n );
}
