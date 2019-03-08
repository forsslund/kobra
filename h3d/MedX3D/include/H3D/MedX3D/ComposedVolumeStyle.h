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
/// \file ComposedVolumeStyle.h
/// \brief Header file for ComposedVolumeStyle node
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __COMPOSEDVOLUMESTYLE_H__
#define __COMPOSEDVOLUMESTYLE_H__

#include <H3D/MedX3D/X3DComposableVolumeRenderStyleNode.h>

#include <H3D/X3DTextureNode.h>
#include <H3D/X3DTexture3DNode.h>
#include <H3D/DependentNodeFields.h>
#include <H3D/SFColorRGBA.h>

namespace H3D {

  /// \ingroup X3DNodes
  /// \class ComposedVolumeStyle
  /// A rendering style node that allows compositing multiple styles together
  /// into a single rendering pass. This is used, for example to render a
  /// simple image with both edge and silhouette styles.
  ///
  /// The renderStyle field contains a list of contributing style node
  /// references that can be applied to the object. Whether the styles should
  /// be strictly rendered in order or not is dependent on the ordered field
  /// value. If this field value is FALSE, then the implementation may apply
  /// the various styles in any order (or even in parallel if the underlying 
  /// implementation supports it). If the value is TRUE, then the 
  /// implementation shall apply each style strictly in the order declared,
  /// starting at index 0.
  ///
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../x3d/ComposedVolumeStyle.x3d">ComposedVolumeStyle.x3d</a>
  ///     ( <a href="x3d/ComposedVolumeStyle.x3d.html">Source</a> )
  ///
  class MEDX3D_API ComposedVolumeStyle : 
    public X3DComposableVolumeRenderStyleNode {
  public:

    class MEDX3D_API MFComposableVolumeRenderStyleNode: 
    public TypedMFNode< X3DComposableVolumeRenderStyleNode > {
      virtual void onAdd( Node *n);      
      virtual void onRemove( Node *n);
    };
    
    /// Constructor.
    ComposedVolumeStyle( Inst< DisplayList >  _displayList = 0,
                         Inst< SFBool >       _enabled     = 0,
                         Inst< SFNode >       _metadata    = 0,
                         Inst< SFBool >       _ordered     = 0,
        Inst< MFComposableVolumeRenderStyleNode > _renderStyle = 0 );
    
    /// Returns true if the style returns an associated color. An associated
    /// color means that the color that is output from the style is the 
    /// color pre-multiplied with the alpha value. We need to know this
    /// in order to choose the appropriate compositing formula.
    /// The input_assiciated input is true if the input to the style is
    /// an associated color.
    virtual bool producesAssociatedColor( bool input_associated );

    /// This function will be called once per scene-graph to possibly update 
    /// the value of uniform fields. 
    virtual void updateUniformFields( X3DVolumeNode *vd );
    
    /// Function for adding all the uniform variables to use in the shader.
    /// This is done by adding a field to the ComposedShader that is used,
    /// easiest done with the addUniformToFragmentShader help function,
    /// and returning a string of glsl uniform declarations to be used
    /// in the shader.
    virtual string addUniforms( ComposedShader *s );

    /// Virtual function for adding glsl fragment shader code to calculate
    /// color and opacity at a sample point within the raycaster loop.
    /// After the code returned by this function the glsl variable 
    /// 'sample_color' should be set for the color of the sample. See
    /// shader source traverseRay function to see what parameters are 
    /// available(INSIDE-LOOP).
    virtual string getShaderCode();

     virtual string getShaderCodeOpacityOnly();

    /// Virtual function for adding function to the fragment shader that
    /// can be called in other of the shader build functions.
    virtual string getShaderFunctions();

    /// Virtual function for adding glsl fragment shader code to the
    /// traverseRay function of the ray caster. This code is initialization
    /// code and will be run before the raycasting loop (PRE-LOOP)
    virtual string getShaderInitCode();
    
    /// Returns true if the style requires the default calculated normals
    /// for the voxels being rendered. If it returns true the variable
    /// "sample_default_normal" will be available in the shader loop.
    /// Implemented to return true if nor surfaceNormals have been specified.
    virtual bool requiresDefaultNormals();

    // For more info see X3DVolumeRenderStyleNode.h
    virtual bool requiresEnabledLights();

    /// Specifies if styles are ordered.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> FALSE \n
    auto_ptr<SFBool> ordered;
    
    /// The render styles.
    ///
    /// <b>Access type:</b> inputOutput \n
    auto_ptr<MFComposableVolumeRenderStyleNode> renderStyle;
    
    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

    /// The function returns
    /// true if the value range min_v to max_v can be empty, i.e. there
    /// exists a value in the range that causes the alpha value to be 0.
    /// The previous_empty value is true if a previous step has decided
    /// the range is possibly empty. 
    virtual bool isEmptySpace( H3DFloat min_v, 
                               H3DFloat max_v, 
                               bool previous_empty );

  };
}

#endif
