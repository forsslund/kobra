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
/// \file ShadowTransform.h
/// \brief Header file for ShadowTransform.
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __SHADOWTRANSFORM_H__
#define __SHADOWTRANSFORM_H__

#include <H3D/H3DShadowObjectNode.h>
#include <H3D/DirectionalLight.h>
#include <H3D/PointLight.h>
#include <H3D/SFRotation.h>
#include <H3D/MatrixTransform.h>
#include <H3D/X3DGeometryNode.h>

namespace H3D {

  /// \ingroup H3DNodes
  /// \class ShadowTransform
  /// The ShadowTransform transforms another H3DShadowObjectNode with
  /// the values from the transform field.
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../../H3DAPI/examples/All/ShadowCaster.x3d">ShadowCaster.x3d</a>
  ///     ( <a href="examples/ShadowCaster.x3d.html">Source</a> )
  ///
  /// \par Internal routes:
  /// \dotfile ShadowTransform.dot  
  class ShadowTransform : public H3DShadowObjectNode {
  public:

    typedef TypedMFNode< H3DShadowObjectNode > MFShadowObjectNode;

    /// Constructor.
    ShadowTransform( Inst< SFNode          > _metadata  = 0,
                     Inst< SFTransformNode > _transform = 0,
                     Inst< MFShadowObjectNode  > _shadowVolume = 0,
                     Inst< SFBool > _enabled = 0 );

    /// Traverse into the ShadowTransform and through into any
    /// shadow volumes, preparing them for rendering. This is called by shadow caster
    virtual void update();

    void computeShadowVolumeInformationCPU( const LightDataStruct& light_data, Matrix4f accumulated_fwd, bool render_caps, std::vector< Vec4d >& coord );

    void renderShadowGPU( const LightDataStruct& light_data, Matrix4f accumulated_fwd, bool render_caps );

    /// The shadow volume we want to transform.
    auto_ptr< MFShadowObjectNode > shadowVolume;

    /// The H3DNodeDatabase object for this node.
    static H3DNodeDatabase database;

  protected:
    // copies of fields that are updated in traverse and are used in computation of shadow volumes
    // these are thread safe copies
    Matrix4f transform_matrix_ts;
    bool     is_enabled_ts;
    std::vector<H3DShadowObjectNode*>  shadow_volumes_ts;

  };
}

#endif
