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
/// \file SegmentedVolumeData.h
/// \brief Header file for SegmentedVolumeData, X3D volume scene-graph 
/// node
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __SEGMENTEDVOLUMEDATA_H__
#define __SEGMENTEDVOLUMEDATA_H__

#include <H3D/MedX3D/X3DVolumeNode.h>
#include <H3D/MedX3D/X3DVolumeRenderStyleNode.h>

#include <H3D/X3DTexture3DNode.h>
#include <H3D/DependentNodeFields.h>
#include <H3D/SFMatrix4f.h>
#include <H3D/SFFloat.h>
#include <H3D/PeriodicUpdate.h>
#include <H3D/MFBool.h>

namespace H3D {

  /// \ingroup X3DNodes
  /// \class SegmentedVolumeData
  /// Defines a segmented volume data set that allows for representation of
  /// different rendering styles for each segment identifier.
  ///
  /// The renderStyle field optionally describes a particular rendering style
  /// to be used. If this field has a non-zero number of values, then the 
  /// defined rendering style is to be applied to the object. If the object 
  /// is segmented, then the index of the segment shall look up the rendering 
  /// style at the given index in this array of values and apply that style to 
  /// data described by that segment ID. If the field value is not specified 
  /// by the user, the implementation shall use a OpacityMapVolumeStyle node 
  /// with default values.
  ///
  /// The voxels field holds a 3D texture with the data for each voxel. For 
  /// each voxel there is a corresponding segment identifier supplied in the 
  /// segmentIdentifiers field, which contains a single component texture. 
  /// If the segmentIdentifiers texture is not identical in size to the main 
  /// voxels, it shall be ignored. If it contains more than one colour 
  /// component, only the red component of the colour shall be used to define 
  /// the identifier.
  ///
  /// The segmentEnabled field allows for controlling whether a segment should 
  /// be rendered or not. The indices of this array corresponds to the segment 
  /// ID. A value at index i of FALSE marks any data with the corresponding 
  /// segment ID to be not rendered. If a segment ID is used that is greater 
  /// than the length of the array, the value is assumed to be TRUE.
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
  /// Slicing restrictions:
  /// If using slice based rendering only the isosurface with the lowest
  /// isovalue will be rendered and the isosurface will always be completely 
  /// opaque. Use raycasting instead for full functionallity. 
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../x3d/SegmentedVolumeData.x3d">SegmentedVolumeData.x3d</a>
  ///     ( <a href="x3d/SegmentedVolumeData.x3d.html">Source</a> )
  ///
  class MEDX3D_API SegmentedVolumeData : public X3DVolumeNode {
    
  public:
    
    class MEDX3D_API MFVolumeRenderStyleNode: 
    public TypedMFNode< X3DVolumeRenderStyleNode > {
      virtual void onAdd( Node *n);      
      virtual void onRemove( Node *n);
    };

    /// Constructor.
    SegmentedVolumeData( Inst< DisplayList >             _displayList = 0, 
                         Inst< SFVec3f >                 _dimensions  = 0,
                         Inst< SFNode  >                 _metadata    = 0,
                         Inst< MFVolumeRenderStyleNode > _renderStyle = 0,
                         Inst< SFTexture3DNode >         _voxels      = 0,
                         Inst< SFTexture3DNode >         _segmentIdentifiers = 0,
                         Inst< MFBool  >                 _segmentEnabled = 0,
                         Inst< SurfaceNormals >          _surfaceNormals = 0,
                         Inst< SFBound >                 _bound       = 0,
                         Inst< SFVec3f >                 _bboxCenter  = 0,
                         Inst< SFVec3f >                 _bboxSize    = 0 );
    
    /// Render the volume using OpenGL.
    virtual void render();
    
    /// Traverse the scenegraph.
    virtual void traverseSG( TraverseInfo &ti );
    
    /// This function will be called once per scene-graph to possibly update 
    /// the value of uniform fields. Calls updateUniformFields in the 
    /// current render styles.
    virtual void updateUniformFields();
    
    /// Function for adding all the uniform variables to use in the shader.
    /// Calls addUniforms in the current render styles.
    virtual string addUniforms( ComposedShader *shader );

    /// Virtual function for adding glsl fragment shader code to calculate
    /// color and opacity at a sample point within the raycaster loop.
    /// Calls getShaderCode in the current render styles.
    virtual string getShaderCode();

    /// Virtual function for adding glsl fragment shader code to calculate
    /// opacity at a sample point within the raycaster loop.
    /// Calls getShaderCode in the current render style.
    virtual string getShaderCodeOpacityOnly();

    /// Virtual function for adding function to the fragment shader that
    /// can be called in other of the shader build functions.
    /// Calls getShaderFunctions in the current render styles.
    virtual string getShaderFunctions();

    /// Virtual function for adding glsl fragment shader code to the
    /// traverseRay function of the ray caster.
    /// Calls getShaderInitCode in the current render styles.
    virtual string getShaderInitCode();

    /// Virtual function for adding glsl fragment shader code to the
    /// traverseRay function of the ray caster.
    /// Calls getShaderPostCode in the current render styles.
    virtual string getShaderPostCode();

    /// Returns true if the style requires the default calculated normals
    /// for the voxels being rendered. If it returns true the variable
    /// "sample_default_normal" will be available in the shader loop.
    virtual bool requiresDefaultNormals();

    /// Returns true if the styles requries the nr_enabled_lights constant
    /// and getEnabledLights macro in the shader.
    virtual bool requiresEnabledLights();

    /// The field containing the X3DVolumeRenderStyleNode node to be used when
    /// rendering the volume.
    /// 
    /// <b>Access type:</b> inputOutput
    auto_ptr< MFVolumeRenderStyleNode > renderStyle;

    /// For each voxel there is a corresponding segment identifier supplied in
    /// the segmentIdentifiers field, which contains a single component
    /// texture. If the segmentIdentifiers texture is not identical in size to
    /// the main voxels, it shall be ignored. If it contains more than one
    /// colour component, only the red component of the colour shall be used
    /// to define the identifier.
    /// 
    /// <b>Access type:</b> inputOutput
    auto_ptr< SFTexture3DNode > segmentIdentifiers;

    /// The segmentEnabled field allows for controlling whether a segment
    /// should be rendered or not. The indices of this array corresponds to the
    /// segment ID.
    /// 
    /// <b>Access type:</b> inputOutput
    auto_ptr< MFBool > segmentEnabled;
    
    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;
  protected:
    class MEDX3D_API SegmentMaxId: 
    public TypedField< SFInt32, SFTexture3DNode > {
      virtual void update();
    };
    
    auto_ptr< SegmentMaxId > segmentMaxId;

  };
  
}

#endif
