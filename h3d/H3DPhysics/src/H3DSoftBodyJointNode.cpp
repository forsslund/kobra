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
/// \file H3DSoftBodyJointNode.cpp
/// \brief Source file for H3DSoftBodyJointNode, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////


#include <H3D/H3DPhysics/H3DSoftBodyJointNode.h>

using namespace H3D;

H3DNodeDatabase H3DSoftBodyJointNode::database( "H3DSoftBodyJointNode", 
                                               NULL, 
                                               typeid( H3DSoftBodyJointNode ),
                                               &H3DJointNode::database);

H3DSoftBodyJointNode::H3DSoftBodyJointNode(
  Inst< SFNode       > _metadata,
  Inst< ValueUpdater > _value_updater,
  Inst< SFH3DSoftBodyNode  > _body1,
  Inst< SFH3DBodyNode  > _body2, 
  Inst< MFString     > _forceOutput,
  Inst< MFEngineOptions > _engineOptions,
  Inst< SFTransformNode > _transform ):
H3DJointNode( _metadata, _value_updater,
             _body1, _body2, _forceOutput, _engineOptions, _transform )
{
  type_name = "H3DSoftBodyJointNode";
  database.initFields( this );  
}
