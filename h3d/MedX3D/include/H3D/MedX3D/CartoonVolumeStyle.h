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
/// \file CartoonVolumeStyle.h
/// \brief Header file for CartoonVolumeStyle node
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __CARTOONVOLUMESTYLE_H__
#define __CARTOONVOLUMESTYLE_H__

#include <H3D/MedX3D/X3DComposableVolumeRenderStyleNode.h>

#include <H3D/X3DTextureNode.h>
#include <H3D/DependentNodeFields.h>

namespace H3D {

  /// \ingroup X3DNodes
  /// \class CartoonVolumeStyle
  /// Uses the cartoon-style nonphotorealistic rendering of the volume.
  /// Cartoon rendering uses two colours that are rendered in a series of
  /// distinct flat-shaded sections based on the local surface normal's
  /// closeness to the average normal, with no gradients in between.
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
  /// The parallelColor field specifies the colour to be used for surface
  /// normals that are orthogonal to the viewer's current location (the plane
  /// of the surface itself is parallel to the user's view direction).
  ///
  /// The orthogonalColor field specifies the colour to be used for surface
  /// normals that are parallel to the viewer's current location (the plane
  /// of the surface itself is orthogonal to the user's view direction).
  /// Surfaces that are further than orthgonal to the view direction
  /// (ie back facing) are not rendered and shall have no colour calculated
  /// for them.
  ///
  /// The colorSteps field indicates how many distinct colours should be taken
  /// from the interpolated colours and used to render the object. If the value
  /// is 1, then no colour interpolation takes place, and only the orthogonal
  /// colour is used to render the surface with. Any other value and the
  /// colours are interpolated between parallelColor and orthogonalColor in HSV
  /// colour space for the RGB components, and linearly for the alpha
  /// component. From this, determine the number of colours using a midpoint
  /// calculation.
  ///
  /// To determine the colours to be used, the angles for the surface normal
  /// relative to the view direction are used. Divide the range pi/2 by
  /// colorSteps.(The two ends of the spectrum are not interpolated in this
  /// way and shall use the specified field values). For each of the ranges,
  /// other than the two ends, find the midpoint angle and determine the
  /// interpolated colour at that point.
  /// 
  /// For example, using the default field values, the colour ranges would be:
  /// - 1,1,1 for angles [0 - pi/8)
  /// - 0.625,0.625,0.625 for angles [pi/8 - pi/4),
  /// - 0.375,0.375,0.375 for angles [pi/4 - 3pi/8),
  /// - 0,0,0 for angles [3pi/8 - pi/2]
  ///
  /// The final output colour is determined by combining this interpolated
  /// colour value with the opacity of the incoming opacity. Colour components
  /// of the incoming colour are ignored.
  ///
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../x3d/CartoonVolumeStyle.x3d">CartoonVolumeStyle.x3d</a>
  ///     ( <a href="x3d/CartoonVolumeStyle.x3d.html">Source</a> )
  ///
  class MEDX3D_API CartoonVolumeStyle : 
  public X3DComposableVolumeRenderStyleNode {
  public:    
    typedef DependentSFNode< X3DTexture3DNode, 
                             FieldRef<H3DDisplayListObject,
                                      H3DDisplayListObject::DisplayList,
                                      &H3D::H3DDisplayListObject::displayList>,
                             true > SFTexture3DNode;
    
    /// Constructor.
    CartoonVolumeStyle( Inst< DisplayList >   _displayList = 0,
                        Inst< SFBool >        _enabled     = 0,
                        Inst< SFNode >        _metadata    = 0,
                        Inst< SFColorRGBA >   _parallelColor   = 0,
                        Inst< SFColorRGBA >   _orthogonalColor   = 0,
                        Inst< SFInt32 >       _colorSteps  = 0,
                        Inst< SFTexture3DNode > _surfaceNormals = 0 );
    
    // add uniforms. For more info check X3DVolumeRenderStyleNode.
    virtual string addUniforms( ComposedShader *c );
    
    // add shader code. For more info check X3DVolumeRenderStyleNode.
    virtual string getShaderCode();
     
    // add volumes normals if needed. For more info check
    // X3DVolumeRenderStyleNode.
    virtual void updateUniformFields( X3DVolumeNode *vd );

    // get shader init code. For more info check X3DVolumeRenderStyleNode.
    virtual string getShaderInitCode();

    /// Returns true if the style requires the default calculated normals
    /// for the voxels being rendered. If it returns true the variable
    /// "sample_default_normal" will be available in the shader loop.
    /// Implemented to return true if nor surfaceNormals have been specified.
    virtual bool requiresDefaultNormals() {
      return( surfaceNormals->getValue() == NULL );
    }

    /// The parallelColor field specifies the colour to be used for surface
    /// normals that are orthogonal to the viewer's current location (the plane
    /// of the surface itself is parallel to the user's view direction).
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> 0 0 0 1\n
    /// <b>Valid range:</b> [0,1] \n
    auto_ptr< SFColorRGBA > parallelColor;

    /// The orthogonalColor field specifies the colour to be used for surface
    /// normals that are parallel to the viewer's current location (the plane
    /// of the surface itself is orthogonal to the user's view direction).
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> 1 1 1 1\n
    /// <b>Valid range:</b> [0,1] \n
    auto_ptr< SFColorRGBA > orthogonalColor;

    /// The colorSteps field indicates how many distinct colours should be taken
    /// from the interpolated colours and used to render the object.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> 4 \n
    /// <b>Valid range:</b> [1,64] \n
    auto_ptr< SFInt32 > colorSteps;

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
    /// <b>Default value:</b> NULL \n
    auto_ptr< SFTexture3DNode > surfaceNormals;
    
    // copy of field that is added to shader
    auto_ptr< SFTexture3DNode > surfaceNormals_glsl;
    
    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;
  protected:
    // updated in updateUniformFields. Set to true if the surfaceNormals
    // field is not NULL.
    bool had_normals;
  };
}

#endif
