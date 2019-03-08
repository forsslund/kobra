//////////////////////////////////////////////////////////////////////////////
//    Copyright 2004-2019, SenseGraphics AB
//
//    This file is part of MedX3D.
//
//    MedX3D is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    MedX3D is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with MedX3D; if not, write to the Free Software
//    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
//    A commercial license is also available. Please contact us at 
//    www.sensegraphics.com for more information.
//
//
/// \file X3DComposableVolumeRenderStyleNode.h
/// \brief Header file for X3DComposableVolumeRenderStyleNode node
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __X3DCOMPOSABLEVOLUMERENDERSTYLENODE_H__
#define __X3DCOMPOSABLEVOLUMERENDERSTYLENODE_H__

#include <H3D/MedX3D/X3DVolumeRenderStyleNode.h>

namespace H3D {
  
  /// \ingroup AbstractNodes
  /// \class X3DComposableVolumeRenderStyleNode
  /// This abstract node type is the base type for all node types that allow
  /// rendering styles to be sequentially composed together to form a single
  /// renderable output. The output of one style may be used as the input of
  /// the next style. Composition in this manner is performed using the
  /// ComposableVolumeStyle node.
  ///
  class MEDX3D_API X3DComposableVolumeRenderStyleNode : 
  public X3DVolumeRenderStyleNode {
  public:

    /// Constructor.
    X3DComposableVolumeRenderStyleNode( Inst< DisplayList > _displayList = 0,
                                        Inst< SFBool >      _enabled     = 0,
                                        Inst< SFNode >      _metadata    = 0 ):
      X3DVolumeRenderStyleNode( _displayList,
                                _enabled,
                                _metadata ) {}
    
  };
}

#endif
