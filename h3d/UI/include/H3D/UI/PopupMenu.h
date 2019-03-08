
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
/// \file PopupMenu.h
/// \brief Header file for PopupMenu.
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __POPUPMENU_H__
#define __POPUPMENU_H__

// UI includes
#include "H3D/UI/H3DButtonNode.h"
#include "H3D/UI/Frame.h"

namespace H3D {

  /// \ingroup UINodes
  /// \class PopupMenu
  /// \brief The PopupMenu node is a menu that pops up when pressing 
  /// a button.
  ///
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../x3d/PopupMenu.x3d">PopupMenu.x3d</a>
  ///     ( <a href="x3d/PopupMenu.x3d.html">Source</a> )
  ///
  /// \par Internal routes:
  /// \dotfile PopupMenu.dot
  class UI_API PopupMenu : 
    public H3DWidgetNode {
  public:
    
    class UI_API SFFrame: public TypedSFNode< Frame > {
      virtual void onAdd( Node * );
      virtual void onRemove( Node * );
    };

    class UI_API SFButtonNode: public TypedSFNode< H3DButtonNode > {
      virtual void onAdd( Node * );
      virtual void onRemove( Node * );
    };

    class UI_API PopupTransformWidgets:
      public TypedField< DependentMFNode<
        H3DWidgetNode,
        FieldRef< H3DDisplayListObject,
          H3DDisplayListObject::DisplayList,
          &H3DDisplayListObject::displayList >,
          true >, 
        Types< SFBool, SFFrame > > {

      virtual void update();
    };

    class UI_API SFTag: public AutoUpdate<SFString> {
      virtual void update();
    };

    class UI_API SFEnabled : public OnValueChangeSField< SFBool >{
     protected:
       virtual void onValueChange( const bool &new_value );
    };

    /// Constructor.
    PopupMenu( Inst< SFNode          > _metadata    = 0,
               Inst< SFBound         > _bound       = 0,
               Inst< DisplayList     > _displayList = 0,
               Inst< SFTag           > _tag         = 0,
               Inst< SFEnabled       > _enabled     = 0,
               Inst< SFVec3f         > _desiredSize = 0,
               Inst< SFVec3f         > _actualSize  = 0,
               Inst< SFLayoutInfoNode > _layoutInfo = 0,
               Inst< SFVec3f         > _popupOffset = 0,
               Inst< SFFrame         > _popupFrame  = 0,
               Inst< SFButtonNode    > _button      = 0,
               Inst< SFBool          > _sticky      = 0 );

    virtual void resize( const Vec3f &new_size ) {
      H3DButtonNode *b = button->getValue();
      if( b ) {
        b->resize( new_size );
      } 
    }

    virtual void initialize();
    virtual void render();
    virtual void traverseSG( TraverseInfo &ti);

    /// Set the parent of this widget node
    virtual void setParent( H3DWidgetNode * _parent = NULL );

    /// The offset from the center of the button that the popup
    /// frame will appear. 
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> actualSize.x/2 0 0.001 \n
    /// 
    /// \dotfile PopupMenu_popupOffset.dot
    auto_ptr< SFVec3f > popupOffset;

    /// The Frame with widgets to pop up when activating the menu.
    /// 
    /// <b>Access type:</b> inputOutput
    /// 
    /// \dotfile PopupMenu_popupFrame.dot
    auto_ptr< SFFrame > popupFrame;

    /// The button to use to activate the menu. When the 'state' field of the
    /// button is true the menu is opened.
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 0 \n
    /// 
    /// \dotfile PopupMenu_button.dot
    auto_ptr< SFButtonNode > button;

    /// If the sticky field is set to true, the only way to close the menu
    /// is for the 'state' field of the button to become false. If it is false
    /// it will close automatically after a button has been pressed in the menu.
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> true \n
    /// 
    /// \dotfile PopupMenu_sticky.dot
    auto_ptr< SFBool > sticky;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;
  protected:
    /// Internal scenegraph node.
    AutoRef< Transform > popup_transform;
    /// Internal scenegraph node.
    auto_ptr< PopupTransformWidgets > popupTransformWidgets;
  };
}

#endif
