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
/// \file X3DVolumeRenderStyleNode.h
/// \brief Header file for X3DVolumeRenderStyleNode node
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __X3DVOLUMERENDERSTYLENODE_H__
#define __X3DVOLUMERENDERSTYLENODE_H__

#include <H3D/MedX3D/MedX3D.h>
#include <H3D/MedX3D/X3DVolumeNode.h>

#include <H3D/X3DNode.h>
#include <H3D/X3DTexture3DNode.h>
#include <H3D/SFBool.h>
#include <H3D/SFMatrix4f.h>
#include <H3D/SFFloat.h>
#include <H3D/MField.h>
#include <H3D/ComposedShader.h>


// Macros to facilitate adding variable names to shaders,
// the variable is concatenated with its ID 
#define UNIFORM_ID(arg) string( #arg ) + style_id
#define UNIFORM_ID_(arg) string( #arg ) + style_id + string( ", " )
#define VARIABLE_ID(arg) string( #arg ) + string( "_" ) + style_id
#define VARIABLE_ID_(arg) string( #arg ) + string( "_" ) + style_id + string( ", " )

namespace H3D {

  /// \ingroup AbstractNodes
  /// \class X3DVolumeRenderStyleNode
  /// This abstract node type is the base type for all node types which specify
  /// a specific visual rendering style to be used.
  ///
  /// The enabled field defines whether this rendering style should be
  /// currently applied to the volume data. If the field is set to FALSE, then
  /// the rendering shall not be applied at all. The render shall act as though
  /// no volume data is rendered when set to FALSE. Effectively, this allows
  /// the end user to turn on and off volume rendering of specific parts of the
  /// volume without needing to add or remove style definitions from the volume
  /// data node.
  ///
  class MEDX3D_API X3DVolumeRenderStyleNode : 
  public X3DNode,
    public H3DDisplayListObject{
  public:
    typedef DependentSFNode< X3DTexture3DNode,
                             FieldRef<H3DDisplayListObject,
                                      H3DDisplayListObject::DisplayList,
                                      &H3D::H3DDisplayListObject::displayList>,
                             true > SFTexture3DNode;

    /// Constructor.
    X3DVolumeRenderStyleNode( Inst< DisplayList > _displayList = 0,
                              Inst< SFBool >      _enabled     = 0,
                              Inst< SFNode >      _metadata    = 0 );
    
    /// Returns the default xml containerField attribute value.
    /// For this node it is "renderStyle".
    virtual string defaultXMLContainerField() {
      return "renderStyle";
    }
    
    /// Returns true if the style returns an associated color. An associated
    /// color means that the color that is output from the style is the 
    /// color pre-multiplied with the alpha value. We need to know this
    /// in order to choose the appropriate compositing formula.
    /// The input_assiciated input is true if the input to the style is
    /// an associated color.
    inline virtual bool producesAssociatedColor( bool input_associated ) {
      return input_associated;
    }

    /// This function will be called once per scene-graph to possibly update 
    /// the value of uniform fields. 
    virtual void updateUniformFields( X3DVolumeNode *vd );
    
    /// Function for adding all the uniform variables to use in the shader.
    /// This is done by adding a field to the ComposedShader that is used,
    /// easiest done with the addUniformToFragmentShader help function,
    /// and returning a string of glsl uniform declarations to be used
    /// in the shader.
    virtual string addUniforms( ComposedShader *shader );

    /// Virtual function for adding glsl fragment shader code to calculate
    /// color and opacity at a sample point within the raycaster loop.
    /// After the code returned by this function the glsl variable 
    /// 'sample_color' should be set for the color of the sample. See
    /// shader source traverseRay function to see what parameters are 
    /// available(INSIDE-LOOP).
    virtual string getShaderCode() { return ""; }

    /// Virtual function for adding glsl fragment shader code to 
    /// calculate opacity at a sample point within the raycaster loop.
    /// After the code returned by this function the glsl variable 
    /// 'sample_color' should be set for the color of the sample. See
    /// shader source traverseRay function to see what parameters are 
    /// available(INSIDE-LOOP).
    inline virtual string getShaderCodeOpacityOnly() { 
      return getShaderCode(); 
    }

    /// Virtual function for adding function to the fragment shader that
    /// can be called in other of the shader build functions.
    virtual string getShaderFunctions() { return ""; }

    /// Virtual function for adding glsl fragment shader code to the
    /// traverseRay function of the ray caster. This code is initialization
    /// code and will be run before the raycasting loop (PRE-LOOP)
    virtual string getShaderInitCode() { return ""; }

    /// Virtual function for adding glsl fragment shader code to the
    /// traverseRay function of the ray caster. This code will be run 
    /// after the raycasting loop (POST-LOOP)
    virtual string getShaderPostCode() { return ""; }

    /// Returns true if the style requires the default calculated normals
    /// for the voxels being rendered. If it returns true the variable
    /// "sample_default_normal" will be available in the shader loop.
    virtual bool requiresDefaultNormals() { return false; }
    
    /// Returns true if the styles requries the nr_enabled_lights constant
    /// and getEnabledLights macro in the shader.
    virtual bool requiresEnabledLights() { return false; }

    /// add field as uniform to fragment shader and the corresponding code
    string addUniformToFragmentShader( ComposedShader *s,
                                       const string &_name,
                                       const string &glsl_type,
                                       const Field::AccessType &access,
                                       Field *field,
                                       bool delete_unadded_field = true );
    
    /// Is this render style enabled.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> TRUE
    auto_ptr<SFBool> enabled;
    
    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

    /// If an event is sent to this field the shader of all X3DVolumeNode
    /// instances this style is part of will be rebuilt.
    auto_ptr< Field > rebuildShader;

    /// The function returns
    /// true if the value range min_v to max_v can be empty, i.e. there
    /// exists a value in the range that causes the alpha value to be 0.
    /// The previous_empty value is true if a previous step has decided
    /// the range is possibly empty. 
    virtual bool isEmptySpace( H3DFloat /*min_v*/, 
                               H3DFloat /*max_v*/, 
                               bool previous_empty ) {
      return previous_empty;
    }
    
  protected:
    // string to use as suffix for shader variable names
    string style_id;
    
  };
}

#endif
