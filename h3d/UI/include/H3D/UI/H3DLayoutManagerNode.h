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
/// \file H3DLayoutManagerNode.h
/// \brief Header file for H3DLayoutManagerNode.
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __H3DLAYOUTMANAGERNODE_H__
#define __H3DLAYOUTMANAGERNODE_H__

// UI includes
#include <H3D/UI/UI.h>

// H3D includes
#include <H3D/X3DChildNode.h>
#include <H3D/X3DBoundedObject.h>
#include <H3D/H3DDisplayListObject.h>

namespace H3D {
  class Frame;

  /// \ingroup AbstractNodes
  /// \class H3DLayoutManagerNode
  /// \brief This is the base node type for all layout manager.
  /// A layout manager devices how nodes should be packed into a Frame.
  class UI_API H3DLayoutManagerNode : 
    public X3DNode,
    public X3DBoundedObject,
    public H3DDisplayListObject {
  public:

    /// Constructor.
    H3DLayoutManagerNode( Inst< SFNode      > _metadata    = 0,
                          Inst< SFBound     > _bound       = 0,
                          Inst< DisplayList > _displayList = 0 );

    /// Pack
    virtual void pack( Frame * ) {}

    
    /// Returns the default xml containerField attribute value.
    /// For this node it is "layoutInfo".
    virtual string defaultXMLContainerField() {
      return "layoutManager";
    }
    
    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;
  };
}

#endif
