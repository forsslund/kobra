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
/// \file SilhouetteEnhancementVolumeStyle.h
/// \brief Header file for SilhouetteEnhancementVolumeStyle node.
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __SILHOUETTEENHANCEMENTVOLUMESTYLE_H__
#define __SILHOUETTEENHANCEMENTVOLUMESTYLE_H__

#include <H3D/MedX3D/X3DComposableVolumeRenderStyleNode.h>

#include <H3D/X3DTextureNode.h>
#include <H3D/X3DTexture3DNode.h>
#include <H3D/DependentNodeFields.h>
#include <H3D/SFColorRGBA.h>
#include <H3D/SFFloat.h>

namespace H3D {
  
  /// \ingroup X3DNodes
  /// \class SilhouetteEnhancementVolumeStyle
  /// Provides silhouette enhancement for the volume rendering style. 
  /// Enhancement of the basic volume is provided by darkening voxels based 
  /// on their orientation relative to the view direction. Perpendicular 
  /// voxels are completely opaque while voxels parallel are completely 
  /// transparent. A threshold can be set where the proportion of how close 
  /// to perpendicular the direction needs to be before the values are made 
  /// more opaque by changing the silhouetteFactor field value.
  ///
  /// Og = Ov * (ksc + kss(1 - |n . V|) ^ kse)
  ///
  /// where
  /// - ksc controls the scaling of non-sihlouette regions 
  /// (silhouetteRetainedOpacity)
  /// - kss is the amount of the sihlouette enhancement to use 
  /// (silhouetteBoundaryOpacity) 
  /// - kse is a power function to control the sharpness of the sihlouette. 
  /// (silhouetteSharpness) 
  ///
  /// The surfaceNormals field contains a 3D texture with at least 3 component 
  /// values. Each voxel in the texture represents the surface normal 
  /// direction for the corresponding voxel in the base data source. This 
  /// texture should be identical in dimensions to the source data. If not, 
  /// the implementation may interpolate or average between adjacent voxels 
  /// to determine the average normal at the voxel required. If this field is 
  /// empty, the implementation shall automatically determine the surface 
  /// normal using algorithmic means.
  ///
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../x3d/SilhouetteEnhancementVolumeStyle.x3d">SilhouetteEnhancementVolumeStyle.x3d</a>
  ///     ( <a href="x3d/SilhouetteEnhancementVolumeStyle.x3d.html">Source</a> )
  ///
  class MEDX3D_API SilhouetteEnhancementVolumeStyle : 
    public X3DComposableVolumeRenderStyleNode {
  public:    
    typedef DependentSFNode< X3DTexture3DNode, 
                             FieldRef<H3DDisplayListObject,
                                      H3DDisplayListObject::DisplayList,
                                      &H3D::H3DDisplayListObject::displayList>,
                             true > SFTexture3DNode;
    
    /// Constructor.
    SilhouetteEnhancementVolumeStyle(
      Inst< DisplayList >  _displayList = 0,
      Inst< SFBool >       _enabled = 0,
      Inst< SFNode >       _metadata = 0,
      Inst< SFFloat > _silhouetteBoundaryOpacity = 0,
      Inst< SFFloat > _silhouetteRetainedOpacity = 0,
      Inst< SFFloat > _silhouetteSharpness = 0,
      Inst< SFTexture3DNode > _surfaceNormals = 0);
    
    // Add uniforms. For more info check X3DVolumeRenderStyleNode.
    virtual string addUniforms( ComposedShader *s );
    
    // Add shader code. For more info check X3DVolumeRenderStyleNode.
    virtual string getShaderCode();

    // Add volumes normals if needed. For more info check
    // X3DVolumeRenderStyleNode.
    virtual void updateUniformFields( X3DVolumeNode *vd );
    
    /// Returns true if the style requires the default calculated normals
    /// for the voxels being rendered. If it returns true the variable
    /// "sample_default_normal" will be available in the shader loop.
    /// Implemented to return true if nor surfaceNormals have been specified.
    virtual bool requiresDefaultNormals() {
      return( surfaceNormals->getValue() == NULL );
    }

    /// The silhouette boundary opacity
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> 0.0 \n
    /// <b>Valid range:</b> [0-infinity)
    auto_ptr<SFFloat> silhouetteBoundaryOpacity;

    /// The silhouette retained opacity
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> 1.0 \n
    /// <b>Valid range:</b> [0,1]
    auto_ptr<SFFloat> silhouetteRetainedOpacity;

    /// The silhouette sharpness
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> 0.5 \n
    /// <b>Valid range:</b> [0-infinity)
    auto_ptr<SFFloat> silhouetteSharpness;
        
    /// The field containing the surface normals
    /// 
    /// <b>Access type:</b> inputOutput \n
    auto_ptr< SFTexture3DNode > surfaceNormals;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;
  protected:
    // updated in updateUniformFields. Set to true if the surfaceNormals
    // field is not NULL.
    bool had_normals;
  };
}

#endif
