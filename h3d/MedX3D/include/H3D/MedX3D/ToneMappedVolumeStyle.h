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
/// \file ToneMappedVolumeStyle.h
/// \brief Header file for ToneMappedVolumeStyle node
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __TONEMAPPEDVOLUMESTYLE_H__
#define __TONEMAPPEDVOLUMESTYLE_H__

#include <H3D/MedX3D/MedX3D.h>
#include <H3D/MedX3D/X3DComposableVolumeRenderStyleNode.h>

#include <H3D/X3DTextureNode.h>
#include <H3D/DependentNodeFields.h>

namespace H3D {
  
  /// \ingroup X3DNodes
  /// \class ToneMappedVolumeStyle
  /// Renders the volume using the Gooch shading model of two-toned warm/cool 
  /// colouring. Two colours are defined, a warm colour and a cool colour and 
  /// the renderer shades between them based on the orientation of the voxel 
  /// relative to the user. This is not the same as the basic ISO surface 
  /// shading and lighting. The following colour formula is used: 
  ///
  /// cc(i) = (1 + Li . n) * 0.5
  /// Cg = Sum cci * warmColor + (1 - cci) * coolColor
  ///
  /// The warmColor and coolColor fields define the two colours to be used at 
  /// the limits of the spectrum. The warmColor field is used for surfaces 
  /// facing towards the light, while the coolColor is used for surfaces 
  /// facing away from the light direction.
  ///
  /// The surfaceNormals field contains a 3D texture with at least 3 component 
  /// values. Each voxel in the texture represents the surface normal direction
  /// for the corresponding voxel in the base data source. This texture should 
  /// be identical in dimensions to the source data. If not, the implementation
  /// may interpolate or average between adjacent voxels to determine the 
  /// average normal at the voxel required. If this field is empty, the 
  /// implementation shall automatically determine the surface normal using 
  /// algorithmic means.
  ///
  /// The final output colour is determined by combining the interpolated 
  /// colour value Cg with the opacity of the corresponding voxel. Colour 
  /// components of the voxel are ignored.
  ///
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../x3d/ToneMappedVolumeStyle.x3d">ToneMappedVolumeStyle.x3d</a>
  ///     ( <a href="x3d/ToneMappedVolumeStyle.x3d.html">Source</a> )
  ///
  class MEDX3D_API ToneMappedVolumeStyle : 
  public X3DComposableVolumeRenderStyleNode {
  public:
    typedef DependentSFNode< X3DTexture3DNode, 
                             FieldRef<H3DDisplayListObject,
                                      H3DDisplayListObject::DisplayList,
                                      &H3D::H3DDisplayListObject::displayList>,
                             true > SFTexture3DNode;
    
    /// Constructor.
    ToneMappedVolumeStyle( Inst< DisplayList >   _displayList = 0,
                           Inst< SFBool >        _enabled     = 0,
                           Inst< SFNode >        _metadata    = 0,
                           Inst< SFColorRGBA >   _coolColor   = 0,
                           Inst< SFColorRGBA >   _warmColor   = 0,
                           Inst< SFTexture3DNode > _surfaceNormals = 0 );
    
    // Add uniforms. For more info check X3DVolumeRenderStyleNode.
    virtual string addUniforms( ComposedShader *c );
    
    // Add pre loop. For more info check X3DVolumeRenderStyleNode.
    virtual string getShaderCode();
    
    // add volumes normals if needed. For more info check
    // X3DVolumeRenderStyleNode.
    virtual void updateUniformFields( X3DVolumeNode *vd );

    /// Returns true if the style requires the default calculated normals
    /// for the voxels being rendered. If it returns true the variable
    /// "sample_default_normal" will be available in the shader loop.
    /// Implemented to return true if nor surfaceNormals have been specified.
    virtual bool requiresDefaultNormals() {
      return( surfaceNormals->getValue() == NULL );
    }

    /// Returns true if the styles requries the nr_enabled_lights constant
    /// and getEnabledLights macro in the shader.
    virtual bool requiresEnabledLights() { return true; }
    
    /// The warmColor and coolColor fields define the two colours to be used at
    /// the limits of the spectrum. The coolColor is used for surfaces
    /// facing away from the light direction.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> 0 0 1 0 \n
    /// <b>Valid range:</b> [0,1]
    auto_ptr< SFColorRGBA > coolColor;

    /// The warmColor and coolColor fields define the two colours to be used at
    /// the limits of the spectrum. The warmColor field is used for surfaces 
    /// facing towards the light.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> 1 1 0 0 \n
    /// <b>Valid range:</b> [0,1]
    auto_ptr< SFColorRGBA > warmColor;

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
    /// <b>Default value:</b> NULL
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
