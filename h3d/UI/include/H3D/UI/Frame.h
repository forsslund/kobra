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
/// \file Frame.h
/// \brief Header file for Frame.
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __FRAME_H__
#define __FRAME_H__

// UI includes
#include <H3D/UI/H3DWidgetNode.h>
#include <H3D/UI/H3DLayoutManagerNode.h>

// H3D includes
#include <H3D/DependentNodeFields.h>

namespace H3D {

  /// \ingroup UINodes
  /// \class Frame
  /// The Frame is a frame in which widgets can be placed.
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../x3d/buttons.x3d">buttons.x3d</a>
  ///     ( <a href="x3d/buttons.x3d.html">Source</a> )
  ///
  /// \par Internal routes:
  /// \dotfile Frame.dot
  class UI_API Frame : 
    public H3DWidgetNode {
  public:
    /// The base class of MFWidgetNode.
    typedef DependentMFNode<
      H3DWidgetNode,
      FieldRef< H3DWidgetNode,
      Field, &H3DWidgetNode::repack >,
      true > MFWidgetBase;

    /// The MFWidgetNode is specialized to set the parent field in all 
    /// the widgets that are added to this
    class UI_API MFWidgetNode: public MFWidgetBase {
      public:
      /// Extended to set the parent field in the added widget to the Frame 
      /// it is added to.
      virtual void onAdd( Node *n ) {
        MFWidgetBase::onAdd( n );
        if( n ) {
          Frame *frame = static_cast< Frame * >( getOwner() );
          H3DWidgetNode *widget = static_cast< H3DWidgetNode * >( n );
          widget->tag->routeNoEvent( frame->tag );
          frame->enabled->route( widget->enabled );
          if( !widget->getParent() )
             widget->setParent( frame );
          else {
            stringstream s;
            s << "Trying to add " << widget->getName() << " widget to "
              << frame->getName() << " when parent already exists. "<< ends;
            throw Exception::H3DAPIException( s.str(), H3D_FULL_LOCATION );
          }
        }
      }

      /// Extended to set the parent field to NULL in the widget that is 
      /// removed.
      virtual void onRemove( Node *n ) {
        if( n ) {
          Frame *frame = static_cast< Frame * >( getOwner() );
          H3DWidgetNode *widget = static_cast< H3DWidgetNode * >( n );
          widget->tag->unroute( frame->tag );
          static_cast< H3DWidgetNode * >( n )->setParent( NULL );
          frame->enabled->unroute( widget->enabled );
        }
        MFWidgetBase::onRemove( n );
      }
    };

    /// The SFLayoutManagerNode is a field that must contain a 
    /// H3DLayoutManagerNode
    typedef TypedSFNode< H3DLayoutManagerNode > SFLayoutManagerNode;

    /// The RepackField class is used to repack the widgets in the frame
    /// when an event happens. The render() and traverseSG() functions in Frame
    /// makes sure that the field is up to date and any field that one
    /// wants to cause a repack of the frame when changed should be
    /// routed to the repackField 
    class UI_API RepackField: 
      public Field {
    protected:
      /// Repack the frame using the current layoutmanager.
      virtual void update() {
        Field::update();
        Frame *frame = static_cast< Frame * >( getOwner() );
        H3DLayoutManagerNode *manager = frame->layoutManager->getValue();
        if( manager ) {
          manager->pack( frame );
        } else {
          stringstream s;
          s << "No H3DLayoutManagerNode specified for " << frame->getName() 
            << " when trying to pack Frame. " << ends;
          throw Exception::H3DAPIException( s.str(), H3D_FULL_LOCATION );
        }
      }
    };

    /// Constructor.
    Frame( Inst< SFNode              > _metadata      = 0,
           Inst< SFBound             > _bound         = 0,
           Inst< DisplayList         > _displayList   = 0,
           Inst< SFString            > _tag           = 0,
           Inst< SFBool              > _enabled       = 0,
           Inst< SFVec3f             > _desiredSize   = 0,
           Inst< SFVec3f             > _actualSize    = 0,
           Inst< SFLayoutInfoNode    > _layoutInfo    = 0,
           Inst< MFWidgetNode        > _children      = 0,
           Inst< SFLayoutManagerNode > _layoutManager = 0 );
    
    /// Render the Frame with openGL.
    virtual void render();

    /// Traverse the scenegraph.
    virtual void traverseSG( TraverseInfo &tt );

    /// Set the parent of this widget node
    virtual void setParent( H3DWidgetNode * _parent = NULL );

    /// The children field contains the widgets that should be contained in the
    /// Frame.
    /// 
    /// <b>Access type: </b> inputOutput
    /// 
    /// \dotfile Frame_children.dot
    auto_ptr< MFWidgetNode > children;

    /// The layoutManager field contains a H3DLayoutManagerNode that determines
    /// how the widgets in the children field should be packed inside the frame.
    /// 
    /// <b>Access type: </b> inputOutput
    /// <b>Default value: </b> GridLayoutManager
    /// 
    /// \dotfile Frame_layoutManager.dot
    auto_ptr< SFLayoutManagerNode > layoutManager;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;
  protected:

    /// The RepackField field is used to repack the widgets in the frame
    /// when an event happens. The render() and traverseSG() functions in Frame
    /// makes sure that the field is up to date and any field that one
    /// wants to cause a repack of the frame when changed should be
    /// routed to the repackField. This field is only accessible from C++.
    /// 
    /// \dotfile Frame_repackField.dot
    auto_ptr< RepackField > repackField;
  };
}

#endif
