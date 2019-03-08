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
/// \file ProjectionVolumeStyle.h
/// \brief Header file for ProjectionVolumeStyle node
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __PROJECTIONVOLUMESTYLE_H__
#define __PROJECTIONVOLUMESTYLE_H__

#include <H3D/MedX3D/X3DVolumeRenderStyleNode.h>

#include <H3D/X3DTextureNode.h>
#include <H3D/DependentNodeFields.h>

namespace H3D {

  /// \ingroup X3DNodes
  /// \class ProjectionVolumeStyle
  /// The ProjectionVolumeStyle volume style node uses the voxel data directly
  /// to generate output colour based on the values of voxel data along the
  /// viewing rays from the eye point. 
  ///
  /// If the value of type is "MAX", Maximum Intensity Projection(MIP) will
  /// be used to generate the output colour. This rendering style also
  /// includes the option to use the extended form of Local Maximum Intensity
  /// Projection. The output colour is determined by projecting rays into the
  /// voxel data from the viewer location and finding the maximum voxel value
  /// found along that ray. If the intensityThreshold value is non-zero 
  /// rendering will use the first maximum value encountered that exceeds the
  /// threshold rather than the maximum found along the entire ray. 
  /// 
  /// If the value of type is "MIN", Minimum Intensity Projection is used. 
  /// This works similar to Maximum Intensity Projection with the difference
  /// that the minimum voxel value along the ray is used. 
  ///
  /// If the value of type is "AVERAGE", Average Intensity Projection is used.
  /// In this case the average value of all voxels along the ray is used as
  /// the output colour. The intensityThreshold value is ignored. This is a 
  /// simple approximation of X-Ray.
  ///
  ///  Since the output of this node is a set of intensity values, all colour
  /// components have the same value. The intensity is derived from the
  /// average of all colour components of the voxel data (though typical
  /// usage will only use single component textures). The Alpha channel is
  /// passed through as-is from the underlying data. If there is no alpha
  /// channel, an alpha value of 1 is used.
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../x3d/ProjectionVolumeStyle.x3d">ProjectionVolumeStyle.x3d</a>
  ///     ( <a href="x3d/ProjectionVolumeStyle.x3d.html">Source</a> )
  ///
  class MEDX3D_API ProjectionVolumeStyle : public X3DVolumeRenderStyleNode {
  public:
    /// The SFTextureNode field is dependent on the displayList field
    /// of the containing X3DTextureNode node.
    typedef DependentSFNode< X3DTextureNode, 
                             FieldRef< H3DDisplayListObject,
                                       H3DDisplayListObject::DisplayList,
                                       &H3DDisplayListObject::displayList >, 
                             true >
    SFTextureNode;

    /// Field that contains the default transfer function
    struct DefaultTransferFunction : public SFTextureNode {
      virtual void update();
    };
    
    /// Constructor.
    ProjectionVolumeStyle( Inst< DisplayList >   _displayList = 0,
                           Inst< SFBool >        _enabled     = 0,
                           Inst< SFNode >        _metadata    = 0,
                           Inst< SFFloat >       _intensityThreshold = 0,
                           Inst< SFString >      _type = 0 );

    // Add uniforms
    // For more info check X3DVolumeRenderStyleNode.
    virtual string addUniforms( ComposedShader *s );

    // Add initializations
    // For more info check X3DVolumeRenderStyleNode.
    virtual string getShaderInitCode();

    // Add post shader code.
    // For more info check X3DVolumeRenderStyleNode.
    virtual string getShaderPostCode();

    // Add shader code
    // For more info check X3DVolumeRenderStyleNode.
    virtual string getShaderCode();
    
    /// If the intensityThreshold value is non-zero then rendering will
    /// use the first maximum value encountered that exceeds the threshold
    /// rather than the maximum found along the entire ray during ray-casting.
    /// Only applicaple when type is "MAX".
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> 0 \n
    /// <b>Valid range:</b> [0-infinity)
    auto_ptr<SFFloat> intensityThreshold;
    
    /// Defines the type of projection. 
    /// "MAX" - Maximum Intensity Projection
    /// "MIN" - Minimum Intensity Projection
    /// "AVERAGE" - Average Intensity Projection
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> "MAX" \n
    /// <b>Valid values:</b> "MAX", "MIN", "AVERAGE" 
    auto_ptr< SFString > type;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;
  };
}

#endif
