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
/// \file ISOSurfaceVolumeData.h
/// \brief Header file for ISOSurfaceVolumeData, X3D volume scene-graph 
/// node
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __ISOSURFACEVOLUMEDATA_H__
#define __ISOSURFACEVOLUMEDATA_H__

#include <H3D/MedX3D/MedX3D.h>
#include <H3D/MedX3D/X3DVolumeNode.h>
#include <H3D/MedX3D/X3DVolumeRenderStyleNode.h>
#include <H3D/MFFloat.h>

namespace H3D {

  /// \ingroup X3DNodes
  /// \class ISOSurfaceVolumeData
  /// Defines a dataset of isosurfaces in the volume data,
  ///
  /// This data representation has one of three possible modes of operation
  /// based on the values of the two fields surfaceValues and contourStepSize.
  /// If surfaceValues has a single value defined, then render the isosurface
  /// that corresponds to that value. If contourStepSize is non-zero, then 
  /// also render all isosurfaces that are multiples of that step size from
  /// the initial surface value. For example, with a surface value of 0.25
  /// and a step size of 0.1, then any additional isosurfaces at 0.05, 0.15,
  /// 0.35, 0.45, etc shall also be rendered. If contourStepSize is left at
  /// the default value of zero, only that single iso value is rendered as a
  /// surface.
  ///
  /// If surfaceValues has more than a single value defined then the
  /// contourStepSize field is ignored and surfaces corresponding to those 
  /// nominated values are rendered.
  ///
  /// For each isosurface extracted from the data set, a separate render
  /// style may be assigned using the renderStyle node. The rendering styles
  /// are taken from the renderStyles field corresponding to the index of the
  /// surface value defined. In the case where automatic contours are being
  /// extracted using the step size, the explicit surface value shall use the
  /// first declared render style, and then render styles are assigned 
  /// starting from the smallest iso value. In all cases, if there are
  /// insufficient render styles defined for the number of isosurfaces to be
  /// rendered, the last style shall be used for all surfaces that don't have
  /// an explicit style set
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
  ///   - <a href="../../x3d/ISOSurfaceVolumeData.x3d">ISOSurfaceVolumeData.x3d</a>
  ///     ( <a href="x3d/ISOSurfaceVolumeData.x3d.html">Source</a> )
  ///   - <a href="../../x3d/ISOSurfaceVolumeData2.x3d">ISOSurfaceVolumeData2.x3d</a>
  ///     ( <a href="x3d/ISOSurfaceVolumeData2.x3d.html">Source</a> )
  class MEDX3D_API ISOSurfaceVolumeData : public X3DVolumeNode {
    
  public:

    class MEDX3D_API MFVolumeRenderStyleNode: 
    public TypedMFNode< X3DVolumeRenderStyleNode > {
      virtual void onAdd( Node *n);      
      virtual void onRemove( Node *n);
    };
    
    /// Constructor.
    ISOSurfaceVolumeData( Inst< DisplayList >             _displayList = 0, 
                          Inst< SFVec3f >                 _dimensions  = 0,
                          Inst< SFNode  >                 _metadata    = 0,
                          Inst< MFVolumeRenderStyleNode > _renderStyle = 0,
                          Inst< SFTexture3DNode >         _voxels      = 0, // obs SF
                          Inst< SurfaceNormals >          _surfaceNormals = 0,
                          Inst< SFBound >                 _bound       = 0,
                          Inst< SFVec3f >                 _bboxCenter  = 0,
                          Inst< SFVec3f >                 _bboxSize    = 0,
                          Inst< MFFloat >                 _surfaceValues = 0,
                          Inst< SFFloat >                 _contourStepSize = 0,
                          Inst< SFFloat >                 _surfaceTolerance =0,
                          Inst< SFTexture3DNode >         _gradients   = 0 );
    
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

    /// The field containing the X3DVolumeRenderStyleNode node to be used when
    /// rendering the volume.
    /// 
    /// <b>Access type:</b> inputOutput \n
    auto_ptr< MFVolumeRenderStyleNode > renderStyle;

    /// The isovalues for the isosurfaces.
    /// 
    /// <b>Access type:</b> inputOutput \n
    auto_ptr< MFFloat > surfaceValues;

    /// If contourStepSize is non-zero, then 
    /// all isosurfaces that are multiples of that step size from
    /// the initial surface value is also rendered.
    /// 
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> 0 \n
    auto_ptr< SFFloat > contourStepSize;

    /// Epsilon value to determine if the gradient magnitude is 0.
    /// 
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> 0 \n
    auto_ptr< SFFloat > surfaceTolerance;

    /// The gradient field may be used to provide explicit per-voxel gradient
    /// direction information for determining surface boundaries rather than
    /// having it implicitly calculated by the implementation.
    /// 
    /// <b>Access type:</b> inputOutput \n
    auto_ptr< SFTexture3DNode > gradients;
    
    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;
  };
  
  typedef ISOSurfaceVolumeData IsoSurfaceVolumeData;
}

#endif
