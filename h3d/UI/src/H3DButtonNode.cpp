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
/// \file H3DButtonNode.cpp
/// \brief CPP file for H3DButtonNode.
///
//
//
//////////////////////////////////////////////////////////////////////////////

// UI includes
#include <H3D/UI/H3DButtonNode.h>
#include <H3D/UI/Frame.h>

// H3D includes
#include <H3D/Shape.h>

// STL includes
#include <algorithm>

using namespace H3D;

H3DNodeDatabase H3DButtonNode::database( 
        "H3DButtonNode", 
        NULL,
        typeid( H3DButtonNode ),
        &H3DLabeledWidget::database 
        );

H3DButtonNode::ButtonGroupMap H3DButtonNode::button_group_map;

namespace H3DButtonNodeInternals {
  FIELDDB_ELEMENT( H3DButtonNode, isPressed, OUTPUT_ONLY )
  FIELDDB_ELEMENT( H3DButtonNode, state, INPUT_OUTPUT )
  FIELDDB_ELEMENT( H3DButtonNode, buttonMode, INPUT_OUTPUT )
  FIELDDB_ELEMENT( H3DButtonNode, buttonGroup, INPUT_OUTPUT )
}

H3DButtonNode::H3DButtonNode( Inst< SFNode          > _metadata,
                              Inst< SFBound         > _bound,
                              Inst< DisplayList     > _displayList,
                              Inst< SFString        > _tag,
                              Inst< SFBool          > _enabled,
                              Inst< SFVec3f         > _desiredSize,
                              Inst< SFVec3f         > _actualSize,
                              Inst< SFAppearanceNode > _appearance,
                              Inst< SFAppearanceNode > _textAppearance,
                              Inst< SFLayoutInfoNode > _layoutInfo,
                              Inst< MFString        > _text,
                              Inst< SFFontStyleNode > _fontStyle,
                              Inst< SFBool          > _isPressed,
                              Inst< ButtonState     > _state,
                              Inst< SFString        > _buttonMode,
                              Inst< SFInt32         > _buttonGroup ) :
  H3DLabeledWidget( _metadata, _bound, _displayList, _tag, 
                    _enabled, _desiredSize, _actualSize, _appearance, _textAppearance, _layoutInfo,
                    _text, _fontStyle ),
  isPressed( _isPressed ),
  state( _state ),
  buttonMode( _buttonMode ),
  buttonGroup( _buttonGroup ) {
  type_name = "H3DButtonNode";
  database.initFields( this );
  isPressed->setValue( false, id );
  state->setValue( false, id );
  buttonGroup->setValue( -1, id );
  buttonMode->setValue( "NORMAL" );
  
  isPressed->routeNoEvent( state );
  buttonMode->routeNoEvent( state );
}

void H3DButtonNode::ButtonState::setValue( const bool &v, int _id ) {
  if( !static_cast< H3DButtonNode* >(getOwner())->enabled->getValue() ) {
    return;
  }

  bool previous_value = value;
  H3DButtonNode *button = static_cast< H3DButtonNode * >( getOwner() );
  const string &mode = button->buttonMode->getValue();
  if( mode == "RADIO_PRESS" ||
      mode == "RADIO_RELEASE" ||
      mode == "RADIO_TOGGLE_PRESS" ||
      mode == "RADIO_TOGGLE_RELEASE" ) {
    if( !previous_value && v ) { 
      // this radio button gets set to active so t
      H3DInt64 group = button->getButtonGroup();

      // deactive the previously active button
      for( list< H3DButtonNode * >::iterator i = 
             button_group_map[group].begin();
           i != button_group_map[group].end();
           ++i ) {
        if( *i != getOwner() && (*i)->state->getValue() ) {
          (*i)->state->setValue( false );
        }
      }
    }
  }
  
  SFBool::setValue( v, _id );
  
  if( value && !previous_value ) {
    button->tag->touch();
  }
}

void H3DButtonNode::ButtonState::update() {
  if( !static_cast< H3DButtonNode* >(getOwner())->enabled->getValue() ) {
    return;
  }

  bool is_pressed = static_cast< SFBool * >( routes_in[0] )->getValue();
  const string &mode = static_cast< SFString * >( routes_in[1] )->getValue();
  H3DButtonNode *button = static_cast< H3DButtonNode * >( getOwner() );
  bool previous_value = value;
  if( event.ptr == routes_in[0] ) {
    if( mode == "NORMAL" ) {
      value = is_pressed;
    } else if( mode == "TOGGLE_PRESS" ) {
      if( is_pressed ) value = !value;
    } else if( mode == "TOGGLE_RELEASE" ) {
      if( !is_pressed && !neverBeenTouched ) value = !value;
      neverBeenTouched = false;
    } else if( mode == "RADIO_PRESS" ||
               mode == "RADIO_TOGGLE_PRESS" ) {
      if( is_pressed ) {
        H3DInt64 group = button->getButtonGroup();
        // deactive the previously active button
        for( list< H3DButtonNode * >::iterator i = 
               button_group_map[group].begin();
             i != button_group_map[group].end();
             ++i ) {
          if( *i != getOwner() && (*i)->state->getValue() ) {
            (*i)->state->setValue( false );
          }
        }
        if( mode == "RADIO_PRESS" )
          value = true;
        else
          value = !value;
      }
    } else if( mode == "RADIO_RELEASE" ||
               mode == "RADIO_TOGGLE_RELEASE" ) {
      if( !is_pressed  && !neverBeenTouched ) {
        H3DInt64 group = button->getButtonGroup();
        // deactive the previously active button
        for( list< H3DButtonNode * >::iterator i = 
               button_group_map[group].begin();
             i != button_group_map[group].end();
             ++i ) {
          if( *i != getOwner() && (*i)->state->getValue() ) {
            (*i)->state->setValue( false );
          }
        }
        if( mode == "RADIO_PRESS" )
          value = true;
        else
          value = !value;
      }
      neverBeenTouched = false;
    }
    if( value && !previous_value ) button->tag->touch();
  }
}

void H3DButtonNode::ButtonState::propagateEvent( Event e ) {
  if( static_cast< H3DButtonNode* >(getOwner())->enabled->getValue() ) {
    TypedField< AutoUpdate< SFBool >,
      Types< SFBool, SFString > >::propagateEvent( e );
  }
}

void H3DButtonNode::setParent( H3DWidgetNode * _parent ) {
  H3DLabeledWidget::setParent( _parent );
  const string &mode = buttonMode->getValue();
  if( mode == "RADIO_PRESS" ||
      mode == "RADIO_RELEASE" ||
      mode == "RADIO_TOGGLE_PRESS" ||
      mode == "RADIO_TOGGLE_RELEASE" ) {
    H3DInt64 group = getButtonGroup();
    if( _parent ) {
      std::list<H3DButtonNode *>::iterator it =
        std::find( button_group_map[group].begin(),
          button_group_map[group].end(),
          this );
      if( it == button_group_map[group].end() ) {

        button_group_map[group].push_back( this );
        if( button_group_map[group].size() == 1 &&
          (mode == "RADIO_PRESS" ||
            mode == "RADIO_RELEASE") ) {
          state->setValue( true );
        } else {
          if( state->getValue() ) {
            for( list< H3DButtonNode * >::iterator i =
              button_group_map[group].begin();
              i != button_group_map[group].end();
              ++i ) {
              if( *i != this && (*i)->state->getValue() ) {
                (*i)->state->setValue( false );
              }
            }
            tag->touch();
          }
        }
      }

    } else {
      list<H3DButtonNode *>::iterator i = 
        std::find( button_group_map[group].begin(),
                    button_group_map[group].end(),
                    this );
      if( i != button_group_map[group].end() ) {
        button_group_map[ group ].erase( i ); 
        if( state->getValue() &&
            !button_group_map[group].empty() )
          button_group_map[group].front()->state->setValue( true );
      }
    }
  }
}


H3DInt64 H3DButtonNode::getButtonGroup() {
  H3DInt64 group = static_cast<H3DInt64>(buttonGroup->getValue());
  if( group == -1 ) {

    // Find the first Frame above the button in the 
    // hierarchy to use as the button group id
    H3DWidgetNode* p = getParent();
    Frame *frame = dynamic_cast< Frame * >( p );
    while( p && !frame ) {
      p = p->getParent();
      if( p ) {
        frame = dynamic_cast< Frame * >(p);
      }
    }

    if( frame ) group = (H3DInt64) frame;
  }
  return group;
}

void H3DButtonNode::initialize() {
  const string &mode = buttonMode->getValue();
  if( mode == "RADIO_PRESS" ||
      mode == "RADIO_RELEASE" ||
      mode == "RADIO_TOGGLE_PRESS" ||
      mode == "RADIO_TOGGLE_RELEASE" ) {
    if( buttonGroup->getValue() != -1 ) {
      H3DInt64 group = getButtonGroup();
      button_group_map[ group ].push_back( this ); 
      if( button_group_map[ group ].size() == 1 &&
          ( mode == "RADIO_PRESS" ||
            mode == "RADIO_RELEASE" ) ) {
        state->setValue( true );
      } else {
        if( state->getValue() ) {
          for( list< H3DButtonNode * >::iterator i = 
                 button_group_map[group].begin();
               i != button_group_map[group].end();
               ++i ) {
            if( *i != this && (*i)->state->getValue() ) {
              (*i)->state->setValue( false );
            }
          }
          tag->touch();
        }
      }
    }
  }
  H3DLabeledWidget::initialize();
}
