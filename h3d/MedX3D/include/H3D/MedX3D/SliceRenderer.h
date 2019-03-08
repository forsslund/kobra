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
/// \file SliceRenderer.h
/// \brief Header file for SliceRenderer.
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __SLICERENDERER_H__
#define __SLICERENDERER_H__

#include <H3D/MedX3D/H3DVolumeRendererNode.h>

namespace H3D {

  /// \ingroup Nodes
  /// \class SliceRenderer
  /// \brief The SliceRenderer node implements 3D texture based view-aligned
  /// slices volume rendering. It is used in X3DVolumeNodes to specify 
  /// the rendering algorithm to use.
  ///
  /// The nrSlices field specifies how many slices to use for the rendering.
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../x3d/SliceRenderer.x3d">SliceRenderer.x3d</a>
  ///     ( <a href="x3d/SliceRenderer.x3d.html">Source</a> )
  class MEDX3D_API SliceRenderer : public H3DVolumeRendererNode {
  public:
    
    /// Constructor.
    SliceRenderer( Inst< SFNode  > _metadata = 0,
                   Inst< SFInt32 > _nrSlices = 0 );
    
    /// Build the shader for a single volume.
    virtual void buildShader( X3DVolumeNode * );

    /// Do volume rendering for a single volume.
    virtual void render( X3DVolumeNode * );

    /// The nrSlices field specifies how many slices to use for the
    /// rendering.
    /// 
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> 100 \n
    /// <b>Allowed values:</b> [ 1, inf )\n
    auto_ptr< SFInt32 > nrSlices;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;
    
  protected:

    /// Render function for the 3D texture slice based version
    void renderSlices(X3DVolumeNode *volume, int planes, const Vec3f &size );
  };
}

#endif
