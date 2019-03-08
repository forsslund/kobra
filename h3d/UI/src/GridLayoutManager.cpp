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
/// \file GridLayoutManager.cpp
/// \brief CPP file for GridLayoutManager, X3D scene-graph node
///
//
//
//////////////////////////////////////////////////////////////////////////////

// UI includes
#include <H3D/UI/GridLayoutManager.h>
#include <H3D/UI/Frame.h>
#include <H3D/UI/GridInfo.h>

// STL includes
#include <map>

using namespace H3D;

H3DNodeDatabase GridLayoutManager::database( 
        "GridLayoutManager", 
        &newInstance< GridLayoutManager >,
        typeid( GridLayoutManager ),
        &X3DNode::database 
        );

namespace GridLayoutManagerInternals {
  // The Grid class is a help class to keep information about where widgets
  // are located in the grid, the size of the grid, which cells are taken etc.
  // The size of the grid will be expanded automatially when widgets are added
  // to it.
  class Grid {
  public:
    // Add a widget to the grid.
    bool addWidget( H3DWidgetNode *widget, 
                    int row, int column, 
                    int row_span, int col_span ) {
      if( row == -1 )
        row = int( grid.size() );
      
      int current_nr_columns = 0;
      if( grid.size() > 0 ) {
        current_nr_columns = int( grid[0].size() );
      }
      
      // make sure the grid has enough rows.
      if( row + row_span > (int)grid.size() ) {
        grid.resize( row + row_span, vector< bool >( current_nr_columns, false ) );
      }
      
      // make sure the grid has enough columns.
      if( column + col_span > current_nr_columns  ) {
        for( vector< vector< bool > >::iterator i = grid.begin();
             i != grid.end(); ++i ) {
          (*i).resize( column + col_span, false );
        }
      }
      
      bool grid_is_free = true;
      
      for( int r = row; r < row + row_span; ++r ) {
        for( int c = column; c < column + col_span; ++c ) {
          if( grid[r][c] ) grid_is_free = false;
          grid[r][c] = true;
        }
      }
      widgetGridMap[ widget ] = Vec2f( (H3DFloat)row, (H3DFloat)column );    
      return grid_is_free;
    }
    
    // Gets the dimensions of the grid, i.e. the number of cells in x and y.
    Vec2f getGridDimensions() {
      if( grid.size() > 0 )
        return Vec2f( (H3DFloat)grid.size(), (H3DFloat)grid[0].size() );
      else 
        return Vec2f( 0, 0 );
    }
    
    // Gets the cell a widget is in in the grid.
    Vec2f widgetCell( H3DWidgetNode *widget ) {
      return widgetGridMap[ widget ];
    }
    
    // Clear the current grid.
    void clear() {
      widgetGridMap.clear();
      grid.clear();
    }
    
  protected:
    map< H3DWidgetNode *, Vec2f > widgetGridMap;
    vector< vector< bool > > grid;
    
  };

}

GridLayoutManager::GridLayoutManager( Inst< SFNode      > _metadata   ,
                                      Inst< SFBound     > _bound      ,
                                      Inst< DisplayList > _displayList  ) :
 H3DLayoutManagerNode( _metadata, _bound, _displayList ) {
  type_name = "GridLayoutManager";
  database.initFields( this );
}

void GridLayoutManager::pack( Frame *frame ) {

  // the maximum column index used by the children in the frame 
  int max_column = -1;
  // the maximum row index used by the children in the frame 
  int max_row = -1;

  GridLayoutManagerInternals::Grid grid;
  
  // add all widgets to a Grid structure
  for( Frame::MFWidgetNode::const_iterator i = frame->children->begin();
       i != frame->children->end();
       ++i ) {
    H3DWidgetNode *widget = 
      static_cast< H3DWidgetNode * >(*i);
    GridInfo *gi = 
      dynamic_cast< GridInfo * >( widget->layoutInfo->getValue() );
    int row = -1;
    int column = 0;
    int row_span = 1;
    int column_span = 1;
    if( gi ) {
      row = gi->row->getValue();
      column = gi->column->getValue();
      row_span = gi->rowSpan->getValue();
      column_span = gi->columnSpan->getValue();
    }
    
    bool success = grid.addWidget( widget, row, column, 
                                   row_span, column_span );
    if( !success ) {
      Console(LogLevel::Warning) << widget->getName() << " widget will be "
        << "packed into the same cell(" << row << ", " << column 
        << ") as previously packed widget. When packing " 
        << frame->getName() << "." << endl;
    }
  }

  // The number of cells in x and y directions.
  Vec2f grid_dims = grid.getGridDimensions();

  // The size of the frame.
  Vec3f frame_size;

  if( frame->getParent() ) 
    // The frame has a parent and the size has therefore been set by the
    // parent node already. So the size of the frame is the actualSize field.
    frame_size = frame->actualSize->getValue();
  else {
    // If there is no parent to the Frame to pack we have to first resize
    // it to the desiredSize.
    frame_size = frame->desiredSize->getValue();
    frame->resize( frame_size );
  }

  // calculate the size of the grid and cells
  Vec2f cell_size( frame_size.x / grid_dims.y,
                   frame_size.y / grid_dims.x );

  // resize the children to the grid size, taking into account possible
  // padding and sticky options and move each child into its cell.
  for( Frame::MFWidgetNode::const_iterator i = frame->children->begin();
       i != frame->children->end();
       ++i ) {
    H3DWidgetNode *widget = 
      static_cast< H3DWidgetNode * >(*i);
    GridInfo *gi = 
      dynamic_cast< GridInfo * >( widget->layoutInfo->getValue() );
    int row_span = 1;
    int column_span = 1;
    string sticky_string = "";
    Vec2f padding(0,0);
    if( gi ) {
      row_span = gi->rowSpan->getValue();
      column_span = gi->columnSpan->getValue();
      padding = gi->padding->getValue();
      sticky_string = gi->sticky->getValue();
    }

    // The size of each cell that the widget can fit in, e.g. the 
    // total cell size - padding.
    Vec2f cell_dims( column_span * cell_size.x - 2 * padding.x,
                     row_span * cell_size.y - 2 * padding.y );

    Vec3f desired_widget_size = widget->desiredSize->getValue();

    // widgets will only be scaled uniformly so we check what the
    // scaling factor has to be in order to make the widget fit inside
    // the cell. If the widget is smaller than the cell size the scale
    // factor remains 1.
    H3DFloat scale_factor = 1;
    if( desired_widget_size.x > cell_dims.x ) {
      H3DFloat ratio = cell_dims.x / desired_widget_size.x;
      if( ratio < scale_factor ) scale_factor = ratio;
    }
    if( desired_widget_size.y > cell_dims.y ) {
      H3DFloat ratio = cell_dims.y / desired_widget_size.y;
      if( ratio < scale_factor ) scale_factor = ratio;
    }
    
    // the new size of the widget after scaling it to fit within the cell. 
    Vec3f new_size = Vec3f( desired_widget_size.x * scale_factor,
                            desired_widget_size.y * scale_factor,
                            frame_size.z );  


    // Calculate the new position of the widget inside the frame. From the 
    // all widgets are positioned in the center of the frame.

    Vec2f cell = grid.widgetCell( widget );
    int row = (int)cell.x;
    int column = (int)cell.y;

    // The center point of the frame. 
    Vec3f frame_center_point = frame_size / 2.0;

    // The wanted center point of the widget inside the frame.
    Vec3f widget_center_point( (column + column_span /2.0f ) * cell_size.x,
                               frame_size.y - (row + row_span / 2.0f ) * cell_size.y,
                               frame_size.z/2.f );

    // The new position to move to.
    Vec3f new_pos = widget_center_point - frame_center_point;

    // Parse the sticky string to see if we have any sticky options. If so make the
    // adjustments to the new_size and new_pos parameters to conform to the sticky
    // options.
    bool w , e, n, s;
    w = e = n = s = false;

    // Find all the parts available between the '+' characters in the string.
    vector< string > parts;
    string parse_string = sticky_string;
    string::size_type pos = parse_string.find( "+" );
    while( pos != string::npos ) {
      parts.push_back( parse_string.substr( 0,pos ) );
      parse_string = parse_string.substr( pos+1, parse_string.size() );
      pos = parse_string.find( "+" );
    }
    if( parse_string.size() > 0 )
      parts.push_back( parse_string );

    // Check each part and set the appropriate bool value.
    for( unsigned int j = 0; j < parts.size(); ++j ) {
      if( parts[j] == "S" ) s = true;
      else if( parts[j] == "W" ) w = true;
      else if( parts[j] == "E" ) e = true;
      else if( parts[j] == "N" ) n = true;
      else if( parts[j] == "NW" ) { n = true; w = true; }
      else if( parts[j] == "NE" ) { n = true; e = true; }
      else if( parts[j] == "SW" ) { s = true; w = true; }
      else if( parts[j] == "SE" ) { s = true; e = true; }
      else {
        Console(LogLevel::Warning) << "Invalid sticky option \"" << parts[j] 
          << "\" in GridInfo in " << widget->getName() << " node." << endl;
      }
    }

    // Change the size and position depending on the sticky options chosen.
    if( w ) {
      if( e ) {
        new_size.x = cell_dims.x;
      } else {
        new_pos.x -= (cell_dims.x - new_size.x)/2;  
      }
    } else {
       if( e ) {
         new_pos.x += (cell_dims.x - new_size.x)/2;  
      }
    }

    if( n ) {
      if( s ) {
        new_size.y = cell_dims.y;
      } else {
        new_pos.y += (cell_dims.y - new_size.y)/2;  
      }
    } else {
      if( s ) {
        new_pos.y -= (cell_dims.y - new_size.y)/2;  
      }
    }

    // Resize and reposition the widget.
    widget->resize( new_size );
    widget->reposition( new_pos );
  }
}
