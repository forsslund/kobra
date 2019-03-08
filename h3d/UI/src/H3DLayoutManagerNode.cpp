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
/// \file H3DLayoutManagerNode.cpp
/// \brief CPP file for H3DLayoutManagerNode, X3D scene-graph node
///
//
//
//////////////////////////////////////////////////////////////////////////////

// UI includes
#include <H3D/UI/H3DLayoutManagerNode.h>

using namespace H3D;

H3DNodeDatabase H3DLayoutManagerNode::database( 
        "H3DLayoutManagerNode", 
        NULL,
        typeid( H3DLayoutManagerNode ),
        &X3DNode::database 
        );

namespace H3DLayoutManagerNodeInternals {
}

H3DLayoutManagerNode::H3DLayoutManagerNode( Inst< SFNode      > _metadata   ,
                                            Inst< SFBound     > _bound      ,
                                            Inst< DisplayList > _displayList  ) :
  X3DNode( _metadata ),
  X3DBoundedObject( _bound ),
  H3DDisplayListObject( _displayList ) {

  type_name = "H3DLayoutManagerNode";
  database.initFields( this );

  displayList->setOwner( this );
  bound->setOwner( this );
  

}


