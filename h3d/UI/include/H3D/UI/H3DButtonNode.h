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
/// \file H3DButtonNode.h
/// \brief Header file for H3DButtonNode.
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __H3DBUTTONNODE_H__
#define __H3DBUTTONNODE_H__

// UI includes
#include <H3D/UI/H3DLabeledWidget.h>

// H3D includes
#include <H3D/SFInt32.h>

// STL includes
#include <map>
#include <list>

namespace H3D {

  /// \ingroup AbstractNodes
  /// \class H3DButtonNode
  /// This is the base node type for all user interface button nodes.
  ///
  /// \par Internal routes:
  /// \dotfile H3DButtonNode.dot
  class UI_API H3DButtonNode : 
    public H3DLabeledWidget {
  public:

    /// Class for handling the state field of the H3DButtonNode class.
    /// routes_in[0] is the isPressed field.
    /// routes_in[1] is the buttonMode field. 
    class UI_API ButtonState: public TypedField< AutoUpdate< SFBool >,
    Types< SFBool, SFString > > {
    public:
      ButtonState(){ neverBeenTouched = true; }
      virtual void setValue( const bool &v, int _id = 0 );
    protected:
      virtual void update();
      bool neverBeenTouched;

      /// Propagates an event to all Fields we are routed to.
      virtual void propagateEvent( Event e );
    };

    /// Constructor.
    H3DButtonNode( Inst< SFNode          > _metadata        = 0,
                   Inst< SFBound         > _bound           = 0,
                   Inst< DisplayList     > _displayList     = 0,
                   Inst< SFString        > _tag             = 0,
                   Inst< SFBool          > _enabled         = 0,
                   Inst< SFVec3f         > _desiredSize     = 0,
                   Inst< SFVec3f         > _actualSize      = 0,
                   Inst< SFAppearanceNode > _appearance     = 0,
                   Inst< SFAppearanceNode > _textAppearance = 0,
                   Inst< SFLayoutInfoNode > _layoutInfo     = 0,
                   Inst< MFString        > _text            = 0,
                   Inst< SFFontStyleNode > _fontStyle       = 0,
                   Inst< SFBool          > _isPressed       = 0,
                   Inst< ButtonState     > _state           = 0,
                   Inst< SFString        > _buttonMode      = 0,
                   Inst< SFInt32         > _buttonGroup     = 0 );

    /// Destructor
    ~H3DButtonNode();

    /// Initialize puts the button in the specified buttonGroup.
    virtual void initialize();

    /// Get the group id of the group the button belongs to. 
    virtual H3DInt64 getButtonGroup();

    /// When setting parent the button must adhere to a group in case
    /// it will be a radio button.
    virtual void setParent( H3DWidgetNode * _parent = NULL );

    /// This field is true if the button is pressed and false if it is not.
    /// 
    /// <b>Access type:</b> outputOnly
    /// 
    /// \dotfile H3DButtonNode_isPressed.dot
    auto_ptr< SFBool > isPressed;

    /// This field contains the state of the button. The interpretation
    /// of its value depends on the value of the buttonMode field. If 
    /// it is "TOGGLE" or "RADIO" then its value indicates if it is
    /// toggled on or not. If it is "NORMAL" it is the same as the isPressed
    /// field.
    /// 
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> false \n
    /// 
    /// \dotfile H3DButtonNode_state.dot
    auto_ptr< ButtonState > state;

    /// This field defines how the button should behave. Valid values
    /// are "NORMAL", "TOGGLE_PRESS", "TOGGLE_RELEASE", "RADIO_PRESS",
    /// "RADIO_RELEASE", "RADIO_TOGGLE_PRESS" and "RADIO_TOGGLE_RELEASE".
    /// If "NORMAL" mode the state 
    /// field will be true when the button is pressed and false otherwise.
    /// If "TOGGLE" the first press on the button toggle it on and the state
    /// field will be true. It will then remain true until the button is 
    /// pressed again.
    /// "RADIO" means that it is a radio button which means that if you 
    /// put several radio buttons into a frame only one can be active
    /// at any time, i.e. only one has its state field set to true. If
    /// another radio button is pressed than the currently active one
    /// the previously active button will be inactivated and the new button
    /// activated.
    /// 
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> "NORMAL" \n
    /// <b>Valid values:</b> "NORMAL", "TOGGLE_PRESS", "TOGGLE_RELEASE", 
    /// "RADIO_PRESS", "RADIO_RELEASE", "RADIO_TOGGLE_PRESS" and 
    /// "RADIO_TOGGLE_RELEASE".
    /// 
    /// \dotfile H3DButtonNode_buttonMode.dot
    auto_ptr< SFString > buttonMode;

    /// The buttonGroup field specifies an identifier for grouping buttons
    /// together. Grouping buttons can be useful when, for example, you 
    /// specify radio buttons. When a radio button's state gets set to true,
    /// all other radio buttons in the same group will be set to false. 
    /// If you specify the group to be -1, all buttons in the same Frame
    /// will be put into the same group automatically.
    ///
    /// Note: Changes to buttonGroup made after the widget is added to its
    /// parent frame may be ignored.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> -1 \n
    /// 
    /// \dotfile H3DButtonNode_buttonGroup.dot
    auto_ptr< SFInt32 > buttonGroup;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:
    typedef map< H3DInt64, list< H3DButtonNode * > > ButtonGroupMap;
    static ButtonGroupMap button_group_map;
  };
}

#endif
