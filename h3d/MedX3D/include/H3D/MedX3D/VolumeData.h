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
/// \file VolumeData.h
/// \brief Header file for VolumeData, X3D volume scene-graph 
/// node
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __VOLUMEDATA_H__
#define __VOLUMEDATA_H__

#include <H3D/X3DTexture2DNode.h>
#include <H3D/MedX3D/MedX3D.h>
#include <H3D/MedX3D/X3DVolumeNode.h>
#include <H3D/MedX3D/X3DVolumeRenderStyleNode.h>

namespace H3D {
  
  /// \ingroup X3DNodes
  /// \class VolumeData
  /// Defines the volume information to be used on a simple non-segmented
  /// volumetric description that uses a single rendering style node for the 
  /// complete volume.
  /// The renderStyle field allows the user to specify a specific rendering
  /// technique to be used on this volumetric object. If the value not
  /// specified by the user, the implementation shall use a
  /// OpacityMapVolumeStyle node with default values.
  ///
  /// The voxels field provides the raw voxel information to be used by the
  /// specific rendering styles. The value is any X3DTexture3DNode type and
  /// may have any number of colour components defined. The specific
  /// interpretation for the values at each voxel shall be defined by the value
  /// of the renderStyle field. If more than one node is defined for this field
  /// then each node after the first shall be treated as a mipmap level of
  /// monotonically decreasing size. Each level should be half the dimensions
  /// of the previous level
  ///
  /// Note: The following fields are H3D extensions to the X3D-specification:
  ///
  /// The surfaceNormals field contains normals from the current voxel data 
  /// using gradients and creates an X3DTexture3DNode with the normals.
  ///
  /// The renderer field contains a H3DVolumeRendererNode that defines
  /// the volume rendering technique to use and set its properties. 
  /// By default a RayCaster renderer is used.
  ///
  /// The filterType field determines what filter type to use when calculating
  /// the value at a sample point. 
  /// Valid values are:
  /// - "DEFAULT" - use the filtering type from the 3d texture node.
  /// - "NEAREST" - use nearest voxel value
  /// - "LINEAR" - use tri linear interpolation
  /// - "CUBIC_B_SPLINE" - use cubic b-spline filter(smoothing filter)
  /// - "CATMULL_ROM" - use cubic Catmull-Rom filter
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../x3d/VolumeData.x3d">VolumeData.x3d</a>
  ///     ( <a href="x3d/VolumeData.x3d.html">Source</a> )
  ///
  class MEDX3D_API VolumeData : public X3DVolumeNode {
    
  public:

    class MEDX3D_API SFVolumeRenderStyleNode: 
    public DependentSFNode< X3DVolumeRenderStyleNode, 
      FieldRef< H3DDisplayListObject,
      H3DDisplayListObject::DisplayList,
      &H3DDisplayListObject::displayList >, 
      true >  {

      typedef DependentSFNode< X3DVolumeRenderStyleNode, 
        FieldRef< H3DDisplayListObject,
        H3DDisplayListObject::DisplayList,
        &H3DDisplayListObject::displayList >, 
        true > SFVolumeRenderStyleNodeBase;
      virtual void onAdd( Node *n);      
      virtual void onRemove( Node *n);
    };
    
    /// Constructor.
    VolumeData( Inst< DisplayList >             _displayList = 0, 
                Inst< SFVec3f >                 _dimensions  = 0,
                Inst< SFNode  >                 _metadata    = 0,
                Inst< SFVolumeRenderStyleNode > _renderStyle = 0,
                Inst< SFTexture3DNode >         _voxels      = 0, // obs SF
                Inst< SurfaceNormals >          _surfaceNormals = 0,
                Inst< SFBound >                 _bound       = 0,
                Inst< SFVec3f >                 _bboxCenter  = 0,
                Inst< SFVec3f >                 _bboxSize    = 0
    );
    
    /// Render the volume using OpenGL.
    virtual void render();

    /// This function will be called once per scene-graph to possibly update 
    /// the value of uniform fields. Calls updateUniformFields in the 
    /// current render style.
    virtual void updateUniformFields();
    
    /// Function for adding all the uniform variables to use in the shader.
    /// Calls addUniforms in the current render style.
    virtual string addUniforms( ComposedShader *shader );

    /// Virtual function for adding glsl fragment shader code to calculate
    /// color and opacity at a sample point within the raycaster loop.
    /// Calls getShaderCode in the current render style.
    virtual string getShaderCode();

    /// Virtual function for adding glsl fragment shader code to calculate
    /// opacity at a sample point within the raycaster loop.
    /// Calls getShaderCode in the current render style.
    virtual string getShaderCodeOpacityOnly();

    /// Virtual function for adding function to the fragment shader that
    /// can be called in other of the shader build functions.
    /// Calls getShaderFunctions in the current render style.
    virtual string getShaderFunctions();

    /// Virtual function for adding glsl fragment shader code to the
    /// traverseRay function of the ray caster.
    /// Calls getShaderInitCode in the current render style.
    virtual string getShaderInitCode();

    /// Virtual function to specify the compositing code, i.e. how the
    /// current sample color should be composited with the previous one.
    /// Chooses front-to-back or MIP compositing depending on the 
    /// volume style.
    virtual string getShaderCompositingCode();

    /// Set the blend mode to use in the slice based renderer.
    virtual void setSliceRenderBlendMode();

    /// Virtual function for adding glsl fragment shader code to the
    /// traverseRay function of the ray caster.
    /// Calls getShaderPostCode in the current render style.
    virtual string getShaderPostCode();

    /// Returns true if the style requires the default calculated normals
    /// for the voxels being rendered. If it returns true the variable
    /// "sample_default_normal" will be available in the shader loop.
    virtual bool requiresDefaultNormals();

    /// Returns true if the styles requries the nr_enabled_lights constant
    /// and getEnabledLights macro in the shader.
    virtual bool requiresEnabledLights();

    /// Traverse the scene-graph.
    virtual void traverseSG( TraverseInfo &ti );

    /// The field containing the X3DVolumeRenderStyleNode node to be used when
    /// rendering the volume.
    /// 
    /// <b>Access type:</b> inputOutput
    auto_ptr< SFVolumeRenderStyleNode > renderStyle;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:

    /// Virtual function used by the 
    /// SFEmptySpaceClassificationTexture2D::update to create a 
    /// 2d texture with values representing if a range of values
    /// contain empt space or not. The function returns
    /// true if the value range min_v to max_v can be empty, i.e. there
    /// exists a value in the range that causes the alpha value to be 0.
    virtual bool isEmptySpace( H3DFloat min_v, H3DFloat max_v );
  };
  
}

#endif
