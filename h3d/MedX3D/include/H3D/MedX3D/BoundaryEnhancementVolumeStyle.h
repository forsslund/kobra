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
/// \file BoundaryEnhancementVolumeStyle.h
/// \brief Header file for BoundaryEnhancementVolumeStyle node
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __BOUNDARYENHANCEMENTVOLUMESTYLE_H__
#define __BOUNDARYENHANCEMENTVOLUMESTYLE_H__

#include <H3D/MedX3D/X3DComposableVolumeRenderStyleNode.h>

#include <H3D/X3DTextureNode.h>
#include <H3D/X3DTexture3DNode.h>
#include <H3D/DependentNodeFields.h>
#include <H3D/SFColorRGBA.h>
#include <H3D/SFFloat.h>

namespace H3D {

  /// \ingroup X3DNodes
  /// \class BoundaryEnhancementVolumeStyle
  /// Provides boundary enhancement for the volume rendering style. In this
  /// style the colour rendered is based on the gradient magnitude. Faster
  /// changing gradients (surface normals) are darker than slower changing.
  /// Areas of different density are made more visible relative to parts
  /// that are relatively constant density.
  ///
  /// The surfaceNormals field is used to provide pre-calculated surface normal
  /// information for each voxel. If provided, this shall be used for all
  /// lighting calculations. If not provided, the implementation shall
  /// automatically generate surface normals using an implementation-specific
  /// method. If a value is provided, it shall be exactly the same voxel
  /// dimensions as the base volume data that it represents. If the dimension
  /// are not identical then the browser shall generate a warning and
  /// automatically generate its own internal normals as though no value was
  /// provided for this field.
  ///
  /// The output colour for this style is obtained by combining a fraction of
  /// the volume's original opacity with an enhancement based on the local
  /// boundary strength (magnitude of the gradient between adjacent voxels).
  /// Colour components from the input are transfered unmodified to the output.
  /// The function used is
  ///
  /// Cg = Cv
  /// Og = Ov ( kgc + kgs(|delta_f|)^kge)
  ///
  /// where
  ///
  /// - kgc is the amount of initial opacity to mix into the output
  /// (retainedOpacity)
  /// - kgs is the amount of the gradient enhancement to use (boundaryOpacity)
  /// - kge is a power function to control the slope of the opacity curve to
  /// highlight the dataset. (opacityFactor)
  ///
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../x3d/BoundaryEnhancementVolumeStyle.x3d">BoundaryEnhancementVolumeStyle.x3d</a>
  ///     ( <a href="x3d/BoundaryEnhancementVolumeStyle.x3d.html">Source</a> )
  ///
  class MEDX3D_API BoundaryEnhancementVolumeStyle : 
    public X3DComposableVolumeRenderStyleNode {
  public:
    typedef DependentSFNode< X3DTexture3DNode,
                             FieldRef<H3DDisplayListObject,
                                      H3DDisplayListObject::DisplayList,
                                      &H3D::H3DDisplayListObject::displayList>,
                             true > SFTexture3DNode;

    /// Constructor.
    BoundaryEnhancementVolumeStyle( Inst< DisplayList >  _displayList = 0,
                                    Inst< SFBool >       _enabled = 0,
                                    Inst< SFNode >       _metadata = 0,
                                    Inst< SFFloat > _retainedOpacity = 0,
                                    Inst< SFFloat > _boundaryOpacity = 0,
                                    Inst< SFFloat > _opacityFactor = 0,
                                  Inst< SFTexture3DNode > _surfaceNormals = 0);
    
    // add default transfer function and volumes normals if needed
    // For more info check X3DVolumeRenderStyleNode.
    virtual void updateUniformFields( X3DVolumeNode *vd );
    
    // add uniforms
    // For more info check X3DVolumeRenderStyleNode.
    virtual string addUniforms( ComposedShader *s );
    
    // add inside loop
    // For more info check X3DVolumeRenderStyleNode.
    virtual string getShaderCode();

    /// Returns true if the style requires the default calculated normals
    /// for the voxels being rendered. If it returns true the variable
    /// "sample_default_normal" will be available in the shader loop.
    /// Implemented to return true if nor surfaceNormals have been specified.
    virtual bool requiresDefaultNormals() {
      return( surfaceNormals->getValue() == NULL );
    }
       
    /// The retained opacity.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> 1.0 \n
    auto_ptr<SFFloat> retainedOpacity;

    /// The boundary opacity.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> 0.0 \n
    auto_ptr<SFFloat> boundaryOpacity;
    
    /// The opacity factor.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> 1.0 \n
    auto_ptr<SFFloat> opacityFactor;
    
    /// The surfaceNormals field contains a 3D texture with at least 3
    /// component values. Each voxel in the texture represents the surface
    /// normal direction for the corresponding voxel in the base data source.
    /// This texture should be identical in dimensions to the source data.
    /// If not, the implementation may interpolate or average between adjacent
    /// voxels to determine the average normal at the voxel required. If this
    /// field is empty, the implementation shall automatically determine the
    /// surface normal using algorithmic means.
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
