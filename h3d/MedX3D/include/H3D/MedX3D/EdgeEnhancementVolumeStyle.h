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
/// \file EdgeEnhancementVolumeStyle.h
/// \brief Header file for EdgeEnhancementVolumeStyle node
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __EDGEENHANCEMENTVOLUMESTYLE_H__
#define __EDGEENHANCEMENTVOLUMESTYLE_H__

#include <H3D/MedX3D/X3DComposableVolumeRenderStyleNode.h>

#include <H3D/X3DTextureNode.h>
#include <H3D/X3DTexture3DNode.h>
#include <H3D/DependentNodeFields.h>
#include <H3D/SFColor.h>

namespace H3D {

  /// \ingroup X3DNodes
  /// \class EdgeEnhancementVolumeStyle
  /// Provides edge enhancement for the volume rendering style. Enhancement of
  /// the basic volume is provided by darkening voxels based on the orientation
  /// of their surface normal relative to the view direction. Perpendicular
  /// normals colour the voxels according to the edgeColor while voxels with
  /// parallel normals are not changed at all. A threshold can be set where the
  /// proportion of how close to parallel the normal direction needs to be
  /// before no colour changes are made.
  ///
  /// The gradientThreshold field defines the minimum angle (in radians) away
  /// from the view direction vector that the surface normal needs to be before
  /// any enhancement is applied.
  ///
  /// The edgeColor field defines the colour to be used to highlight the edges.
  ///
  /// The surfaceNormals field contains a 3D texture with at least 3 component
  /// values. Each voxel in the texture represents the surface normal direction
  /// for the corresponding voxel in the base data source. This texture should
  /// be identical in dimensions to the source data. If not, the implementation
  /// may interpolate or average between adjacent voxels to determine the average
  /// normal at the voxel required. If this field is empty, the implementation
  /// shall automatically determine the surface normal using algorithmic means.
  ///
  /// The final colour is determined by:
  /// Cg = Cv if (|n . V|) >= cos(gradientThreshold) else
  ///      Cv * (|n . V|) + edgeColor * (1 - (|n . V|))
  /// Og = Ov
  ///
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../x3d/EdgeEnhancementVolumeStyle.x3d">EdgeEnhancementVolumeStyle.x3d</a>
  ///     ( <a href="x3d/EdgeEnhancementVolumeStyle.x3d.html">Source</a> )
  ///
  class MEDX3D_API EdgeEnhancementVolumeStyle : 
    public X3DComposableVolumeRenderStyleNode {
  public:
    typedef DependentSFNode< X3DTexture3DNode, 
                             FieldRef<H3DDisplayListObject,
                                      H3DDisplayListObject::DisplayList,
                                      &H3D::H3DDisplayListObject::displayList>,
                             true > SFTexture3DNode;

    /// Constructor.
    EdgeEnhancementVolumeStyle( Inst< DisplayList >     _displayList    = 0,
                                Inst< SFBool >          _enabled        = 0,
                                Inst< SFNode >          _metadata       = 0,
                                Inst< SFColor >     _edgeColor      = 0,
                                Inst< SFFloat >         _gradientThreshold = 0,
                                Inst< SFTexture3DNode > _surfaceNormals = 0 );
    
    /// Function for adding all the uniform variables to use in the shader.
    /// This is done by adding a field to the ComposedShader that is used,
    /// easiest done with the addUniformToFragmentShader help function,
    /// and returning a string of glsl uniform declarations to be used
    /// in the shader.
    virtual string addUniforms( ComposedShader *s );

    /// Returns true if the style requires the default calculated normals
    /// for the voxels being rendered. If it returns true the variable
    /// "sample_default_normal" will be available in the shader loop.
    /// Implemented to return true if nor surfaceNormals have been specified.
    virtual bool requiresDefaultNormals() {
      return( surfaceNormals->getValue() == NULL );
    }
    
    /// Virtual function for adding glsl fragment shader code to calculate
    /// color and opacity at a sample point within the raycaster loop.
    /// After the code returned by this function the glsl variable 
    /// 'sample_color' should be set for the color of the sample. See
    /// shader source traverseRay function to see what parameters are 
    /// available(INSIDE-LOOP).
    virtual string getShaderCode();
    
    /// This function will be called once per scene-graph to possibly update 
    /// the value of uniform fields. 
    virtual void updateUniformFields( X3DVolumeNode *vd );
    
    /// The edgeColor field defines the colour to be used to highlight the
    /// edges.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> 0 0 0 \n
    /// <b>Valid range:</b> [0,1] \n
    auto_ptr<SFColor> edgeColor;
    
    /// The gradientThreshold field defines the minimum angle (in radians) away
    /// from the view direction vector that the surface normal needs to be
    /// before any enhancement is applied.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> 0.4 \n
    /// <b>Valid range:</b> [0,pi/2] \n
    auto_ptr<SFFloat> gradientThreshold;
    
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
