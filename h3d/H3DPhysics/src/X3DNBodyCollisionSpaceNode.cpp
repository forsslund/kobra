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
/// \file X3DNBodyCollisionSpaceNode.cpp
/// \brief Source file for X3DNBodyCollisionSpaceNode, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////
#include <H3D/H3DPhysics/X3DNBodyCollisionSpaceNode.h>

using namespace H3D;

std::map< H3DSpaceId, X3DNBodyCollisionSpaceNode * > X3DNBodyCollisionSpaceNode::space_id_map;

H3DNodeDatabase X3DNBodyCollisionSpaceNode::database( "X3DNBodyCollisionSpaceNode",
                                                     NULL,
                                                     typeid( X3DNBodyCollisionSpaceNode ),
                                                     &X3DNode::database);

namespace X3DNBodyCollisionSpaceNodeInternals {
  FIELDDB_ELEMENT( X3DNBodyCollisionSpaceNode, enabled, INPUT_OUTPUT )
  FIELDDB_ELEMENT( X3DNBodyCollisionSpaceNode, bboxCenter, INITIALIZE_ONLY )
  FIELDDB_ELEMENT( X3DNBodyCollisionSpaceNode, bboxSize, INITIALIZE_ONLY )
}

X3DNBodyCollisionSpaceNode::X3DNBodyCollisionSpaceNode(
  Inst< SFBool  >  _enabled,
  Inst< SFNode  >  _metadata,
  Inst< SFBound >  _bound,
  Inst< SFVec3f >  _bboxCenter,
  Inst< SFVec3f >  _bboxSize,
  Inst< MFH3DCollidableGroupId > _inCollidableGroup,
  Inst< MFH3DCollidableExceptionGroupId > _inCollidableExceptionGroup,
  Inst< MFH3DCollidableSelectionGroupId > _inCollidableSelectionGroup ):
X3DNode( _metadata ),
X3DBoundedObject( _bound, _bboxCenter, _bboxSize ),
enabled( _enabled ),
engine_thread( NULL ),
inCollidableGroup( _inCollidableGroup ),
inCollidableExceptionGroup( _inCollidableExceptionGroup ),
inCollidableSelectionGroup( _inCollidableSelectionGroup ),
space_id( 0 ) {

  type_name = "X3DNBodyCollisionSpaceNode";
  database.initFields( this );

  enabled->setValue( true );

  H3DCollidableGroupIdList defaultList;
  defaultList.push_back( -1 );
  inCollidableGroup->setValue( defaultList );

}

void X3DNBodyCollisionSpaceNode::addToCollidableGroup( H3DCollidableGroupId groupId ) {
  H3DCollidableGroupIdList currentList = inCollidableGroup->getValue();
  currentList.push_back( groupId );

  std::vector< H3DCollidableGroupId >::iterator j =
    std::find( currentList.begin(), currentList.end(), -1 );
  if( j != currentList.end() ) currentList.erase( j );
  inCollidableGroup->setValue( currentList );
}
void X3DNBodyCollisionSpaceNode::removeFromCollidableGroup( H3DCollidableGroupId groupId ) {
  H3DCollidableGroupIdList currentList = inCollidableGroup->getValue();

  std::vector< H3DCollidableGroupId >::iterator j =
    std::find( currentList.begin(), currentList.end(), groupId );
  if( j != currentList.end() ) currentList.erase( j );
  if( currentList.size() == 0 )
    currentList.push_back( -1 );
  inCollidableGroup->setValue( currentList );
}

void X3DNBodyCollisionSpaceNode::addToCollidableExceptionGroup( H3DCollidableExceptionGroupId groupId ) {

  H3DCollidableExceptionGroupIdList currentList = inCollidableExceptionGroup->getValue();
  std::vector< H3DCollidableExceptionGroupId >::iterator j = std::find( currentList.begin(), currentList.end(), groupId );

  if( j == currentList.end() ){
    currentList.push_back( groupId );
    inCollidableExceptionGroup->setValue( currentList );
    //Console (4) << "Adding the collidable " << this->getName() << " to the CollidableExceptionGroup with id : " << groupId <<endl;
  }

}
void X3DNBodyCollisionSpaceNode::removeFromCollidableExceptionGroup( H3DCollidableExceptionGroupId groupId ) {

  H3DCollidableExceptionGroupIdList currentList = inCollidableExceptionGroup->getValue();
  std::vector< H3DCollidableExceptionGroupId >::iterator j = std::find( currentList.begin(), currentList.end(), groupId );

  if( j != currentList.end() ){
    currentList.erase( j );
    inCollidableExceptionGroup->setValue( currentList );
    //Console (4) << "Removing the collidable " << this->getName() << " from the CollidableExceptionGroup with id : " << groupId <<endl;
  }

}

void X3DNBodyCollisionSpaceNode::addToCollidableSelectionGroup( H3DCollidableSelectionGroupId groupId, bool selected ) {

  //H3DCollidableSelectionGroupIdList currentList = inCollidableSelectionGroup->getValue();
  std::vector< H3DFloat > currentList = inCollidableSelectionGroup->getValue();

  float group_id = (float)groupId + (selected ? 0.4f : 0.0f);
  std::vector< H3DFloat >::iterator j = std::find_if( currentList.begin(), currentList.end(), EpsilonCompareFunctor( group_id ) );
  if( j == currentList.end() ) {

    float other_id = (float)groupId + (selected ? 0.0f : 0.4f);
    std::vector< H3DFloat >::iterator jj = std::find_if( currentList.begin(), currentList.end(), EpsilonCompareFunctor( other_id ) );
    if( jj != currentList.end() ) {
      Console (4) << "The collidable " << this << " can not be added to both collidables and selectedCollidables of the CollidableSelectionGroup with id: " << groupId <<endl;
      return;
    }
    currentList.push_back( group_id );
    inCollidableSelectionGroup->setValue( currentList );
    //Console (4) << "Adding the collidable " << this->getName() << " to the " <<  (selected ? "selectedCollidable" : "collidble") << " field of the CollidableSelectionGroup with id : " << groupId <<endl;
  }

}

void X3DNBodyCollisionSpaceNode::removeFromCollidableSelectionGroup( H3DCollidableSelectionGroupId groupId ) {

  std::vector< H3DFloat > currentList = inCollidableSelectionGroup->getValue();
  float collidableId = (float)groupId;
  float selectedCollidableId = float( groupId >> 16 ) + 0.4f;

  std::vector< H3DFloat >::iterator j = std::find_if( currentList.begin(), currentList.end(), EpsilonCompareFunctor( collidableId ) );
  if( j != currentList.end() ){
    currentList.erase( j );
    //Console (4) << "Removing the collidable " << this->getName() << " from the collidable field of the CollidableSelectionGroup with id : " << groupId <<endl;

  }

  j = std::find_if( currentList.begin(), currentList.end(), EpsilonCompareFunctor( selectedCollidableId ) );
  if( j != currentList.end() ) {
    currentList.erase( j );
    //Console (4) << "Removing the collidable " << this->getName() << " from the selectedCollidables field of the CollidableSelectionGroup with id : " << groupId <<endl;
  }

  inCollidableSelectionGroup->setValue( currentList );

}
