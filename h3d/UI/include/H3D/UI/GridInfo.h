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
/// \file GridInfo.h
/// \brief Header file for GridInfo.
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __GRIDINFO_H__
#define __GRIDINFO_H__

// UI includes
#include <H3D/UI/H3DLayoutInfoNode.h>

// H3D includes
#include <H3D/SFInt32.h>
#include <H3D/SFVec2f.h>
#include <H3D/SFString.h>

namespace H3D {

  /// \ingroup UINodes
  /// \class GridInfo
  /// \brief The GridInfo node gives information to the GridLayoutManager
  /// about which position in the grid widget is to be positioned in.
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../x3d/buttons.x3d">buttons.x3d</a>
  ///     ( <a href="x3d/buttons.x3d.html">Source</a> )
  ///
  /// \par Internal routes:
  /// \dotfile GridInfo.dot
  class UI_API GridInfo : 
    public H3DLayoutInfoNode {
  public:

    /// Constructor.
    GridInfo( Inst< SFNode   > _metadata        = 0,
              Inst< SFInt32  > _row             = 0,
              Inst< SFInt32  > _column          = 0,
              Inst< SFInt32  > _rowSpan         = 0,
              Inst< SFInt32  > _columnSpan      = 0,
              Inst< SFVec2f  > _padding         = 0,
              Inst< SFVec2f  > _internalPadding = 0,
              Inst< SFString > _sticky          = 0
              );

    /// The row field contains the index of the row to insert this 
    /// widget in in a GridLayout. -1 means the first empty row in the grid.
    ///
    /// <b>Access type:</b> initializeOnly \n
    /// <b>Default value:</b> -1 \n
    /// 
    /// \dotfile GridInfo_row.dot
    auto_ptr< SFInt32 > row;

    /// The column field contains the index of the column to insert this 
    /// widget in in a GridLayout. 
    ///
    /// <b>Access type:</b> initializeOnly \n
    /// <b>Default value:</b> 0 \n
    /// 
    /// \dotfile GridInfo_column.dot
    auto_ptr< SFInt32 > column;

    /// The rowSpan field indicates how many rows this widget should span.
    ///
    /// <b>Access type:</b> initializeOnly \n
    /// <b>Default value:</b> 1 \n
    /// 
    /// \dotfile GridInfo_rowSpan.dot
    auto_ptr< SFInt32 > rowSpan;

    /// The columnSpan field indicates how many columns this widget should 
    /// span.
    ///
    /// <b>Access type:</b> initializeOnly \n
    /// <b>Default value:</b> 1 \n
    /// 
    /// \dotfile GridInfo_columnSpan.dot
    auto_ptr< SFInt32 > columnSpan;

    /// The padding field indicates how much padding should be placed
    /// around the widget in a grid cell.
    ///
    /// <b>Access type:</b> initializeOnly \n
    /// <b>Default value:</b> Vec2f(0,0) \n
    /// 
    /// \dotfile GridInfo_padding.dot
    auto_ptr< SFVec2f > padding;

    /// The internalpadding field indicates how much padding should be placed
    /// inside the widget borders.
    ///
    /// <b>Access type:</b> initializeOnly \n
    /// <b>Default value:</b> Vec2f(0,0) \n
    /// 
    /// \dotfile GridInfo_column.dot
    auto_ptr< SFVec2f > internalPadding;

    /// The sticky field defines how to expand the widget if the resulting 
    /// cell is larger than the widget itself. This can be any combination of
    /// the values "S", "N", "E", and "W", or "NW", "NE", "SW", and "SE". For
    /// example "W" (west) means that the widget should be aligned to the left
    /// cell border. "W+E" means that the widget should be stretched 
    /// horizontally to fill the whole cell. "W+E+N+S" means that the widget 
    /// should be expanded in both directions. Default is to center the widget
    /// in the cell.
    ///
    /// <b>Access type:</b> initializeOnly \n
    /// <b>Default value:</b> "" \n
    /// 
    /// \dotfile GridInfo_sticky.dot
    auto_ptr< SFString > sticky;

    /// The H3DNodeDatabase for this node
    static H3DNodeDatabase database;
  };
}

#endif
