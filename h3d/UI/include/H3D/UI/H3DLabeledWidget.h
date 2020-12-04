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
/// \file H3DLabeledWidget.h
/// \brief Header file for H3DLabeledWidget.
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __H3DLABELEDWIDGET_H__
#define __H3DLABELEDWIDGET_H__

// UI includes
#include <H3D/UI/H3DWidgetNode.h>

// H3D includes
#include <H3D/X3DFontStyleNode.h>
#include <H3D/X3DAppearanceNode.h>
#include <H3D/X3DGeometryNode.h>
#include <H3D/DependentNodeFields.h>
#include <H3D/X3D.h>
#include <H3D/MFString.h>



namespace H3D {
  class Shape;
  /// \ingroup AbstractNodes
  /// \class H3DLabeledWidget
  /// This is the base node type for all user interface widgets with text
  /// on them.
  ///
  /// \par Internal routes:
  /// \dotfile H3DLabeledWidget.dot
  class UI_API H3DLabeledWidget : 
    public H3DWidgetNode {
  public:

    typedef TypedSFNode< X3DFontStyleNode > SFFontStyleNode;
    /// SFAppearanceNode is dependent on the displayList field of its
    /// encapsulated X3DAppearanceNode node, i.e. an event from that
    /// field will trigger an event from the SFAppearanceNode as well.
    typedef DependentSFNode< X3DAppearanceNode, 
                             FieldRef<H3DDisplayListObject, 
                                      H3DDisplayListObject::DisplayList,
                                      &H3DDisplayListObject::displayList >, 
                             true > 
    SFAppearanceNode;
        
    /// SFGeometryNode is dependent on the displayList field of its
    /// encapsulated X3DGeometryNode node, i.e. an event from that
    /// field will trigger an event from the SFGeometryNode as
    /// well. 
    typedef DependentSFNode< X3DGeometryNode, 
                             FieldRef<H3DDisplayListObject, 
                                      H3DDisplayListObject::DisplayList,
                                      &H3DDisplayListObject::displayList >, 
                             true > 
    SFGeometryNode;
    
    /// Constructor.
    H3DLabeledWidget( Inst< SFNode           > _metadata       = 0,
                      Inst< SFBound          > _bound          = 0,
                      Inst< DisplayList      > _displayList    = 0,
                      Inst< SFString         > _tag            = 0,
                      Inst< SFBool           > _enabled        = 0,
                      Inst< SFVec3f          > _desiredSize    = 0,
                      Inst< SFVec3f          > _actualSize     = 0,
                      Inst< SFAppearanceNode > _appearance     = 0,
                      Inst< SFAppearanceNode > _textAppearance = 0,
                      Inst< SFLayoutInfoNode > _layoutInfo     = 0,
                      Inst< MFString         > _text           = 0,
                      Inst< SFFontStyleNode  > _fontStyle      = 0,
                      Inst< MFNode           > _customNodes    = 0 );
  
    virtual void render();
    virtual void traverseSG( TraverseInfo &ti );
    virtual void resize( const Vec3f &new_size );

    /// The text to be printed on the widget.
    /// 
    /// <b>Access type:</b> inputOutput
    /// 
    /// \dotfile H3DLabeledWidget_text.dot
    auto_ptr< MFString > text;

    /// Font to use for the text on the widget.
    /// 
    /// <b>Access type:</b> inputOutput
    /// 
    /// \dotfile H3DLabeledWidget_fontStyle.dot
    auto_ptr< SFFontStyleNode > fontStyle;

    /// field storing an X3DAppearanceNode
    auto_ptr< SFAppearanceNode > appearance;

    /// The appearance node used for text
    auto_ptr<  SFAppearanceNode > textAppearance;

    /// Any additional nodes may be added here and will be transformed with the widget
    ///
    /// Additional nodes can be used to further customize the appearance of widget
    auto_ptr < MFNode > customNodes;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:

    /// Internal field storing an X3DGeometryNode
    auto_ptr< SFGeometryNode   > widgetGeometry;

    X3D::DEFNodes widget_defs;
    X3D::DEFNodes text_defs;
  };
}

#endif
