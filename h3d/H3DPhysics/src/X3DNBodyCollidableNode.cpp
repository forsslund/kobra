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
/// \file X3DNBodyCollidableNode.cpp
/// \brief cpp file for X3DNBodyCollidableNode, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/H3DPhysics/X3DNBodyCollidableNode.h>

using namespace H3D;

std::map< H3DCollidableId, X3DNBodyCollidableNode * > X3DNBodyCollidableNode::collidable_id_map;

H3DNodeDatabase X3DNBodyCollidableNode::database( "X3DNBodyCollidableNode",
                                                 NULL,
                                                 typeid( X3DNBodyCollidableNode ),
                                                 &X3DChildNode::database);

namespace X3DNBodyCollidableNodeInternals {
  FIELDDB_ELEMENT( X3DNBodyCollidableNode, enabled, INPUT_OUTPUT )
  FIELDDB_ELEMENT( X3DNBodyCollidableNode, rotation, INPUT_OUTPUT )
  FIELDDB_ELEMENT( X3DNBodyCollidableNode, translation, INPUT_OUTPUT )
  FIELDDB_ELEMENT( X3DNBodyCollidableNode, scale, INPUT_OUTPUT )
  FIELDDB_ELEMENT( X3DNBodyCollidableNode, bboxCenter, INITIALIZE_ONLY )
  FIELDDB_ELEMENT( X3DNBodyCollidableNode, bboxSize, INITIALIZE_ONLY )
  FIELDDB_ELEMENT( X3DNBodyCollidableNode, engineOptions, INPUT_OUTPUT )

}

X3DNBodyCollidableNode::X3DNBodyCollidableNode(
  Inst< SFBool > _enabled,
  Inst< SFNode  >  _metadata,
  Inst< SFRotation > _rotation,
  Inst< SFVec3f > _translation,
  Inst< SFVec3f > _scale,
  Inst< SFBound > _bound,
  Inst< SFVec3f >  _bboxCenter,
  Inst< SFVec3f > _bboxSize,
  Inst< ValueUpdater > _valueUpdater,
  Inst< MFEngineOptions > _engineOptions,
  Inst< MFH3DCollidableGroupId > _inCollidableGroup,
  Inst< MFH3DCollidableExceptionGroupId > _inCollidableExceptionGroup,
  Inst< MFH3DCollidableSelectionGroupId > _inCollidableSelectionGroup,
  Inst< DisplayList > _displayList ) :
  X3DChildNode( _metadata ),
  X3DBoundedObject( _bound, _bboxCenter, _bboxSize ),
  H3DDisplayListObject( _displayList ),
  enabled( _enabled ),
  rotation( _rotation ),
  translation( _translation ),
  scale( _scale ),
  valueUpdater( _valueUpdater ),
  engineOptions( _engineOptions ),
  engine_thread( NULL ),
  collidable_id( 0 ),
  rbc( NULL ),
  inCollidableGroup( _inCollidableGroup ),
  inCollidableExceptionGroup( _inCollidableExceptionGroup ),
  inCollidableSelectionGroup( _inCollidableSelectionGroup ) {

  type_name = "X3DNBodyCollidableNode";
  database.initFields( this );

  displayList->setOwner( this );

  valueUpdater->setName( "valueUpdater" );
  valueUpdater->setOwner( this );

  enabled->setValue( true );
  rotation->setValue( Rotation( 0, 0, 1, 0 ) );
  translation->setValue( Vec3f( 0, 0, 0 ) );
  scale->setValue( Vec3f( 1, 1, 1 ) );
  bboxCenter->setValue( Vec3f( 0, 0, 0 ) );
  bboxSize->setValue( Vec3f( -1, -1, -1 ), id );

  rotation->route( displayList );
  translation->route( displayList );
  scale->route( displayList );

  H3DCollidableGroupIdList defaultList;
  defaultList.push_back( -1 );
  inCollidableGroup->setValue( defaultList );

  enabled->route( valueUpdater );
  rotation->route( valueUpdater );
  translation->route( valueUpdater );
  scale->route( valueUpdater );
  engineOptions->route( valueUpdater );
  inCollidableGroup->route( valueUpdater );
  inCollidableExceptionGroup->route( valueUpdater );
  inCollidableSelectionGroup->route( valueUpdater );
}


bool X3DNBodyCollidableNode::collidableInitialized() {
  return engine_thread != NULL;
}

X3DNBodyCollidableNode *X3DNBodyCollidableNode::getCollidableFromId( H3DCollidableId id ) {
  std::map< H3DCollidableId, X3DNBodyCollidableNode * >::iterator j = collidable_id_map.find( id );
  if( j != collidable_id_map.end() )
    return (*j).second;
  else
    return NULL;
}

void X3DNBodyCollidableNode::addToCollidableGroup( H3DCollidableGroupId groupId ) {

  H3DCollidableGroupIdList currentList = inCollidableGroup->getValue();
  currentList.push_back( groupId );

  std::vector< H3DCollidableGroupId >::iterator j =
    std::find( currentList.begin(), currentList.end(), -1 );
  if( j != currentList.end() ) currentList.erase( j );
  inCollidableGroup->setValue( currentList );
}
void X3DNBodyCollidableNode::removeFromCollidableGroup( H3DCollidableGroupId groupId ) {

  H3DCollidableGroupIdList currentList = inCollidableGroup->getValue();

  std::vector< H3DCollidableGroupId >::iterator j =
    std::find( currentList.begin(), currentList.end(), groupId );
  if( j != currentList.end() ) currentList.erase( j );
  if( currentList.size() == 0 )
    currentList.push_back( -1 );
  inCollidableGroup->setValue( currentList );

}

void X3DNBodyCollidableNode::addToCollidableExceptionGroup( H3DCollidableExceptionGroupId groupId ) {

  H3DCollidableExceptionGroupIdList currentList = inCollidableExceptionGroup->getValue();
  std::vector< H3DCollidableExceptionGroupId >::iterator j = std::find( currentList.begin(), currentList.end(), groupId );

  if( j == currentList.end() ){
    currentList.push_back( groupId );
    inCollidableExceptionGroup->setValue( currentList );
  }

}
void X3DNBodyCollidableNode::removeFromCollidableExceptionGroup( H3DCollidableExceptionGroupId groupId ) {

  H3DCollidableExceptionGroupIdList currentList = inCollidableExceptionGroup->getValue();
  std::vector< H3DCollidableExceptionGroupId >::iterator j = std::find( currentList.begin(), currentList.end(), groupId );

  if( j != currentList.end() ){
    currentList.erase( j );
    inCollidableExceptionGroup->setValue( currentList );
  }

}

void X3DNBodyCollidableNode::addToCollidableSelectionGroup( H3DCollidableSelectionGroupId groupId, bool selected ) {

  std::vector< H3DFloat > currentList = inCollidableSelectionGroup->getValue();

  float group_id = (float)groupId + (selected ? 0.4f : 0.0f);
  std::vector< H3DFloat >::iterator j = std::find_if( currentList.begin(), currentList.end(), EpsilonCompareFunctor( group_id ) );
  if( j == currentList.end() ) {

    float other_id = (float)groupId + (selected ? 0.0f : 0.4f);
    std::vector< H3DFloat >::iterator jj = std::find_if( currentList.begin(), currentList.end(), EpsilonCompareFunctor( other_id ) );
    if( jj != currentList.end() ) {
      Console (4) << "The collidable " << this->getName() << " can not be added to both collidables and selectedCollidables of the CollidableSelectionGroup with id: " << groupId <<endl;
      return;
    }
    currentList.push_back( group_id );
    inCollidableSelectionGroup->setValue( currentList );
  }

}

void X3DNBodyCollidableNode::removeFromCollidableSelectionGroup( H3DCollidableSelectionGroupId groupId ) {

  std::vector< H3DFloat > currentList = inCollidableSelectionGroup->getValue();
  float collidableId = (float)groupId;
  float selectedCollidableId = float( groupId >> 16 ) + 0.4f;

  std::vector< H3DFloat >::iterator j = std::find_if( currentList.begin(), currentList.end(), EpsilonCompareFunctor( collidableId ) );
  if( j != currentList.end() ){
    currentList.erase( j );

  }

  j = std::find_if( currentList.begin(), currentList.end(), EpsilonCompareFunctor( selectedCollidableId ) );
  if( j != currentList.end() ) {
    currentList.erase( j );
  }

  inCollidableSelectionGroup->setValue( currentList );
}

PhysicsEngineParameters::CollidableParameters
*X3DNBodyCollidableNode::getCollidableParameters( bool all_params ) {
  PhysicsEngineParameters::CollidableParameters *params = createCollidableParameters();

#ifdef H3DPHYSICS_ENGINE_PARAMETERS_INCLUDE_NAME
  params->setName( this->getName() );
#endif

  if( all_params || valueUpdater->hasCausedEvent( enabled ) ) {
    params->setEnabled( enabled->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( rotation ) ) {
    params->setRotation( rotation->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( translation ) ) {
    params->setTranslation( translation->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( scale ) ) {
    params->setScale( scale->getValue() );
  }
  if( H3DEngineOptions* options = engineOptions->getOptions( engine_thread->getEngine() ) ) {
    if( all_params || valueUpdater->hasCausedEvent( options->valueUpdater ) ) {
      params->setEngineOptions( options->valueUpdater->getParameters( all_params ) );
    }
  }
  if( all_params || valueUpdater->hasCausedEvent( inCollidableGroup ) ) {
    params->setCollidableGroupList( inCollidableGroup->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( inCollidableExceptionGroup ) ) {
    params->setCollidableExceptionGroupList( inCollidableExceptionGroup->getValue() );
  }
  if( all_params || valueUpdater->hasCausedEvent( inCollidableSelectionGroup ) ) {
    params->setCollidableSelectionGroupList( inCollidableSelectionGroup->getValue() );
  }

  return params;
}

void X3DNBodyCollidableNode::ValueUpdater::update() {
  X3DNBodyCollidableNode *c = static_cast<X3DNBodyCollidableNode *>(getOwner());
  if( c->collidableInitialized() ) {
    CollidableParameters *params = c->getCollidableParameters();
    c->engine_thread->setCollidableParameters( c->collidable_id, *params );
  }
  EventCollectingField< PeriodicUpdate< Field > >::update();
}
void X3DNBodyCollidableNode::setRigidBodyCollection( RigidBodyCollection* _rbc ) {
  rbc = _rbc;
}


