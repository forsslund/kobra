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
/// \file H3DWidgetNode.h
/// \brief Header file for H3DWidgetNode.
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __H3DWIDGETNODE_H__
#define __H3DWIDGETNODE_H__

// UI includes
#include <H3D/UI/UI.h>
#include <H3D/UI/H3DLayoutInfoNode.h>

// H3D includes
#include <H3D/X3DChildNode.h>
#include <H3D/X3DBoundedObject.h>
#include <H3D/H3DDisplayListObject.h>
#include <H3D/Transform.h>
#include <H3D/SFString.h>
#include <H3D/SFBool.h>
#include <H3D/SFVec3f.h>

namespace H3D {

  /// \ingroup AbstractNodes
  /// \class H3DWidgetNode
  /// This is the base node type for all user interface widget nodes.
  ///
  /// \par Internal routes:
  /// \dotfile H3DWidgetNode.dot
  class UI_API H3DWidgetNode : 
    public X3DChildNode,
    public X3DBoundedObject,
    public H3DDisplayListObject {
  public:
    typedef TypedSFNode< H3DLayoutInfoNode > SFLayoutInfoNode;


    /// Constructor.
    H3DWidgetNode( Inst< SFNode           > _metadata    = 0,
                   Inst< SFBound          > _bound       = 0,
                   Inst< DisplayList      > _displayList = 0,
                   Inst< SFString         > _tag         = 0,
                   Inst< SFBool           > _enabled     = 0,
                   Inst< SFVec3f          > _desiredSize = 0,
                   Inst< SFVec3f          > _actualSize  = 0,
                   Inst< SFLayoutInfoNode > _layoutInfo  = 0 );

    virtual void traverseSG( TraverseInfo &ti );

    /// Detect intersection between a line segment and the components of this
    /// widget.
    /// \param from The start of the line segment.
    /// \param to The end of the line segment.
    /// \param result Contains info about the closest intersection for every
    /// object that intersects the line.
    /// \returns true if intersected, false otherwise.
    virtual bool lineIntersect( 
      const Vec3f &from,
      const Vec3f &to,    
      LineIntersectResult &result );

    /// Find closest point on the components of this widget to point p.
    /// \param p The point to find the closest point to.
    /// \param result A struct containing various results of closest
    /// points such as which geometries the closest points where
    /// detected on.
    virtual void closestPoint( const Vec3f &p,
                               NodeIntersectResult &result );

    /// Detect collision between a moving sphere and the components of this
    /// widget.
    /// \param radius The radius of the sphere
    /// \param from The start position of the sphere
    /// \param to The end position of the sphere.
    /// \param result A struct containing various results of intersections
    /// such as which geometries intersected the moving sphere.
    /// \returns true if intersected, false otherwise.
    virtual bool movingSphereIntersect( H3DFloat radius,
                                        const Vec3f &from, 
                                        const Vec3f &to,
                                        NodeIntersectResult &result );

    /// Resize the widget.
    /// \param new_size The new size to use.
    virtual void resize( const Vec3f &new_size ) {
      const Vec3f &current_size = actualSize->getValue();
      Vec3f diff = current_size - new_size;
      if( diff * diff > Constants::f_epsilon )
        actualSize->setValue( new_size, id );
    }

    /// Reposition the widget.
    /// \param new_pos The new position to use.
    virtual void reposition( const Vec3f &new_pos ) {
      const Vec3f &current_pos = transform->translation->getValue();
      Vec3f diff = current_pos - new_pos;
      if( diff * diff > Constants::f_epsilon )
        transform->translation->setValue( new_pos );
    }

    /// Get the parent of this widget node.
    inline H3DWidgetNode * getParent() {
      return parent;
    }

    /// Set the parent of this widget node, virtual since some widgets
    /// need additional updates when parent is set. For example
    /// H3DButtonNodes.
    virtual void setParent( H3DWidgetNode * _parent = NULL ) {
      parent = _parent;
    }

    /// The tag field contains a string that identifies the widget. E.g.
    /// if several widgets are in the same RadioFrame the tag field of
    /// the RadioFrame will have the tag value of the button that is
    /// currently active.
    /// 
    /// <b>Access type:</b> inputOutput
    /// 
    /// \dotfile H3DWidgetNode_tag.dot
    auto_ptr< SFString > tag;

    /// The value of the enabled field specifies if the widget is enabled or
    /// not. A widget that is not enabled generates no events. 
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> true
    /// 
    /// \dotfile H3DWidgetNode_enabled.dot
    auto_ptr< SFBool > enabled;

    /// The desired size of the widget. This is a hint on how big a user wants
    /// the widget to be. The system might not be able to give it that size
    /// however depending on restriction from other soures, e.g. sizes of 
    /// Frames etc. The actual size that the widget ends up with is given
    /// in the actualSize field.
    /// 
    /// <b>Access type:</b> inputOutput
    /// <b>Default value:</b> 1, 1, 0
    /// 
    /// \dotfile H3DWidgetNode_desiredSize.dot
    auto_ptr< SFVec3f > desiredSize;

    /// The actual size of the widget.
    /// 
    /// <b>Access type:</b> outputOnly
    /// 
    /// \dotfile H3DWidgetNode_actualSize.dot
    auto_ptr< SFVec3f > actualSize;

    /// If not empty the containing node ( a subclass of H3DLayoutInfo )
    /// is used for positioning the widget.
    /// 
    /// <b>Access type:</b> inputOutput
    /// 
    /// \dotfile H3DWidgetNode_layoutInfo.dot
    auto_ptr< SFLayoutInfoNode > layoutInfo;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:
    /// Internal transform node used to create the widget.
    /// Routes may occur to and from fields in this transform. Those fields
    /// will be named transform_fieldname in the route-graph in doxygen
    /// documentation.
    AutoRef< Transform > transform;

    /// A pointer to the parent node for this widget, for example to the
    /// frame it resides in.
    H3DWidgetNode * parent;

  public:

    /// Fields which require the widget to be repacked within its parent
    /// should be routed here
    /// C++ only field.
    auto_ptr < Field > repack;
  };
}

#endif
