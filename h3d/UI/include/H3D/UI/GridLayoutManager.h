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
/// \file GridLayoutManager.h
/// \brief Header file for GridLayoutManager.
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __GRIDLAYOUTMANAGER_H__
#define __GRIDLAYOUTMANAGER_H__

#include "H3D/UI/H3DLayoutManagerNode.h"

namespace H3D {
  
  /// \ingroup UINodes
  /// \class GridLayoutManager
  /// \brief The GridLayoutManager puts all widgets in a grid and lets
  /// each widget decide through the GridInfo node where on the grid
  /// it should be located and how many cells it should span.
  ///
  /// There is only one node inheriting from H3DLayoutManagerNode
  /// and that is GridLayoutManager. Therefore it is used by default
  /// by every the Frame node so all x3d examples are examples of the
  /// GridLayoutManager.
  ///
  /// \par Internal routes:
  /// \dotfile GridLayoutManager.dot
  class UI_API GridLayoutManager : 
    public H3DLayoutManagerNode {
  public:

    /// Constructor.
    GridLayoutManager( Inst< SFNode      > _metadata    = 0,
                       Inst< SFBound     > _bound       = 0,
                       Inst< DisplayList > _displayList = 0 );
    
    /// Pack
    virtual void pack( Frame *frame );
    
    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;
  };
}

#endif
