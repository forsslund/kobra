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
/// \file H3DRigidBodyJointNode.h
/// \brief Header file for H3DRigidBodyJointNode, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __H3DRIGIDBODYJOINTNODE__
#define __H3DRIGIDBODYJOINTNODE__

#include <H3D/MFString.h>
#include <H3D/H3DPhysics/H3DJointNode.h>

namespace H3D{

  /// \ingroup AbstractNodes
  /// \class H3DRigidBodyJointNode
  /// \brief  The H3DRigidBodyJointNode abstract node type is the
  /// base type for joints between rigid bodies.
  class H3DPHYS_API H3DRigidBodyJointNode : public H3DJointNode {
  public:

    /// Constructor.
    H3DRigidBodyJointNode(
      Inst< SFNode > _metadata = 0,
      Inst< ValueUpdater > _valueUpdater = 0,
      Inst< SFRigidBody > _body1 = 0,
      Inst< SFRigidBody > _body2 = 0,
      Inst< MFString     > _forceOutput = 0,
      Inst< MFEngineOptions > _engineOptions = 0,
      Inst< SFTransformNode > _transform = 0 );

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;    

  };
}
#endif
