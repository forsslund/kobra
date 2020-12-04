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
/// \file PopupMenu.cpp
/// \brief .cpp file for PopupMenu.
///
//
//
//////////////////////////////////////////////////////////////////////////////

// UI includes
#include <H3D/UI/PopupMenu.h>

using namespace H3D;

H3DNodeDatabase PopupMenu::database( 
        "PopupMenu", 
        &newInstance<PopupMenu>,
        typeid( PopupMenu ),
        &H3DWidgetNode::database 
        );

namespace PopupMenuInternals {
  FIELDDB_ELEMENT( PopupMenu, popupOffset, INPUT_OUTPUT )
  FIELDDB_ELEMENT( PopupMenu, popupFrame, INITIALIZE_ONLY )
  FIELDDB_ELEMENT( PopupMenu, button, INITIALIZE_ONLY )
  FIELDDB_ELEMENT( PopupMenu, sticky, INPUT_OUTPUT )
}

PopupMenu::PopupMenu( Inst< SFNode          > _metadata,
                      Inst< SFBound         > _bound,
                      Inst< DisplayList     > _displayList,
                      Inst< SFTag           > _tag,
                      Inst< SFEnabled       > _enabled,
                      Inst< SFVec3f         > _desiredSize,
                      Inst< SFVec3f         > _actualSize,
                      Inst< SFLayoutInfoNode > _layoutInfo,
                      Inst< SFVec3f         > _popupOffset,
                      Inst< SFFrame         > _popupFrame,
                      Inst< SFButtonNode    > _button,
                      Inst< SFBool          > _sticky ) :
  H3DWidgetNode( _metadata, _bound, _displayList, _tag, 
                 _enabled, _desiredSize, _actualSize, _layoutInfo ),
  popupOffset( _popupOffset ),
  popupFrame( _popupFrame ),
  button( _button ),
  sticky( _sticky ),
  popup_transform( new Transform ),
  popupTransformWidgets( new PopupTransformWidgets ) {

  type_name = "PopupMenu";
  database.initFields( this );
  sticky->setValue( true );

  popupTransformWidgets->setName( "popupTransformWidgets" );
  popupTransformWidgets->setOwner( this );

  transform->displayList->route( displayList );
  popup_transform->setName( "popup_transform" );
  transform->children->push_back( popup_transform.get() );

  popupOffset->route( popup_transform->translation );
  popupTransformWidgets->route( popup_transform->children );
  
}

void PopupMenu::initialize() {
  H3DWidgetNode::initialize();
  H3DButtonNode *b = button->getValue();
  if( b ) {
    b->state->route( popupTransformWidgets );
    popupFrame->route( popupTransformWidgets );
  }
}

void PopupMenu::render() {
  H3DWidgetNode::render();
  transform->displayList->callList();
}

void PopupMenu::traverseSG( TraverseInfo &ti ) {
  H3DWidgetNode::traverseSG( ti );
  transform->traverseSG( ti );
}

void PopupMenu::setParent( H3DWidgetNode * _parent ) {
  H3DWidgetNode::setParent( _parent );

  // Since our parent has changed, notify our children so they know the full hierarchy
  if( H3DButtonNode* b = button->getValue() ) {
    b->setParent( this );
  }
  //if( Frame* f = popupFrame->getValue() ) {
  //  f->setParent( this );
  //}
}

void PopupMenu::SFButtonNode::onAdd( Node *n ) {
  TypedSFNode< H3DButtonNode >::onAdd( n );
  H3DButtonNode *_button = static_cast< H3DButtonNode * >( n );
  PopupMenu *popup = static_cast< PopupMenu * >( getOwner() );
  if( _button ) {
    _button->desiredSize->route( popup->desiredSize );
    _button->actualSize->route( popup->actualSize, popup->id );
    popup->transform->children->push_back( _button );
    if( !_button->getParent() )
      _button->setParent( popup );
  }
}

void PopupMenu::SFFrame::onAdd( Node *n ) {
  TypedSFNode< Frame >::onAdd( n );
  Frame *frame = static_cast< Frame * >( n );
  PopupMenu *popup = static_cast< PopupMenu * >( getOwner() );
  if( frame ) {
    frame->tag->route( popup->tag );
    //if( !frame->getParent() ) {
    //  frame->setParent( popup );
    //}
  }
}

void PopupMenu::SFFrame::onRemove( Node * ) {
  
}

void PopupMenu::PopupTransformWidgets::update() {
  value.clear();
  if( routes_in.size() > 0 ) {
    bool state = static_cast< SFBool * >( routes_in[0] )->getValue();
    if( state ) {
      value.push_back( static_cast< SFNode * >( routes_in[1] )->getValue() );
    }
  }
}

void PopupMenu::SFTag::update() {
  SFString::update();
  PopupMenu *popup = static_cast< PopupMenu * >( getOwner() );
  H3DButtonNode *b = popup->button->getValue();
  if( b && 
      !popup->sticky->getValue() &&
      b->state->getValue() ) {
    b->state->setValue( false );
  }
}

void PopupMenu::SFEnabled::onValueChange( const bool &new_value ) {
  PopupMenu *popup = dynamic_cast< PopupMenu * >( getOwner() );
  if( popup ) {
    H3DButtonNode *b = popup->button->getValue();
    if( b ) {
      if( !new_value ) b->state->setValue( false );
      b->enabled->setValue( new_value );
    }
  }
}

