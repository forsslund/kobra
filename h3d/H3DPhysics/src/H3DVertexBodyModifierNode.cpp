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
/// \file H3DVertexBodyModifierNode.cpp
/// \brief Source file for H3DVertexBodyModifierNode, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/H3DPhysics/H3DVertexBodyModifierNode.h>

using namespace H3D;  

H3DNodeDatabase H3DVertexBodyModifierNode::database( "H3DVertexBodyModifierNode", 
                                                    NULL, 
                                                    typeid( H3DVertexBodyModifierNode ),
                                                    &H3DBodyModifierNode::database);

namespace H3DVertexBodyModifierNodeInternals {
  FIELDDB_ELEMENT( H3DVertexBodyModifierNode, index, INPUT_OUTPUT )
}

H3DVertexBodyModifierNode::H3DVertexBodyModifierNode (
  Inst< SFNode > _metadata,
  Inst< ValueUpdater > _valueUpdater,
  Inst< SFH3DBodyNode > _body1,
  Inst< MFEngineOptions > _engineOptions,
  Inst< MFInt32 > _index ) :
H3DBodyModifierNode( _metadata, _valueUpdater, _body1, _engineOptions), 
index( _index )
{
  // init fields
  type_name = "H3DVertexBodyModifierNode";
  database.initFields( this );

}
