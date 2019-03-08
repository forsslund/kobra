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
/// \file GridInfo.cpp
/// \brief CPP file for GridInfo, X3D scene-graph node
///
//
//
//////////////////////////////////////////////////////////////////////////////

// UI includes
#include <H3D/UI/GridInfo.h>

using namespace H3D;

H3DNodeDatabase GridInfo::database( 
        "GridInfo", 
        &newInstance< GridInfo >,
        typeid( GridInfo ),
        &H3DLayoutInfoNode::database 
        );

namespace GridInfoInternals {
  FIELDDB_ELEMENT( GridInfo, row, INITIALIZE_ONLY )
  FIELDDB_ELEMENT( GridInfo, column, INITIALIZE_ONLY )
  FIELDDB_ELEMENT( GridInfo, rowSpan, INITIALIZE_ONLY )
  FIELDDB_ELEMENT( GridInfo, columnSpan, INITIALIZE_ONLY )
  FIELDDB_ELEMENT( GridInfo, padding, INITIALIZE_ONLY )
  FIELDDB_ELEMENT( GridInfo, internalPadding, INITIALIZE_ONLY )
  FIELDDB_ELEMENT( GridInfo, sticky, INITIALIZE_ONLY )
}

GridInfo::GridInfo( Inst< SFNode   > _metadata,
                    Inst< SFInt32  > _row,
                    Inst< SFInt32  > _column,
                    Inst< SFInt32  > _rowSpan,
                    Inst< SFInt32  > _columnSpan,
                    Inst< SFVec2f  > _padding,
                    Inst< SFVec2f  > _internalPadding,
                    Inst< SFString > _sticky ) :
  H3DLayoutInfoNode( _metadata ),
  row( _row ),
  column( _column ),
  rowSpan( _rowSpan ),
  columnSpan( _columnSpan ),
  padding( _padding ),
  internalPadding( _internalPadding ),
  sticky( _sticky ) {
  type_name = "GridInfo";
  database.initFields( this );

  row->setValue( -1 );
  column->setValue( 0 );
  rowSpan->setValue( 1 );
  columnSpan->setValue( 1 );
  padding->setValue( Vec2f( 0, 0 ) );
  internalPadding->setValue( Vec2f( 0, 0 ) );
  sticky->setValue( "" );
}


