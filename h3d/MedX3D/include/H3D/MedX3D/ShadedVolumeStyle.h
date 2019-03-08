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
/// \file ShadedVolumeStyle.h
/// \brief Header file for ShadedVolumeStyle node
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __SHADEDVOLUMESTYLE_H__
#define __SHADEDVOLUMESTYLE_H__

#include <H3D/MedX3D/MedX3D.h>
#include <H3D/MedX3D/X3DComposableVolumeRenderStyleNode.h>

#include <H3D/X3DTextureNode.h>
#include <H3D/X3DTexture3DNode.h>
#include <H3D/DependentNodeFields.h>

namespace H3D {
  
  /// \ingroup X3DNodes
  /// \class ShadedVolumeStyle
  /// \brief The shaded volume style applies traditional local illumination
  /// model that is used in polygonal rendering to volume rendering. In this
  /// style, the source voxel value ignored other than to determine if it is a
  /// surface that needs to be shaded or not and the normal at that surface.
  /// Typically this style is used combined with the ISOSurfaceVolumeData to
  /// extract surfaces from the data and render each surface with a different
  /// colour. Determination of whether the voxel should be shaded using this
  /// model is the responsibility of the volume data definition.
  ///
  /// Once a voxel has been determined to be a rendered, a colour and opacity
  /// is determined based on whether a value has been specified for the
  /// material field. If a material value is provided, this voxel is considered
  /// to be lit using the lighting equations below. If no material node is
  /// provided, it is considered to be unlit and the colour of the voxel
  /// completely transparent.
  ///
  /// When a material node is provided the voxel is lit using the the
  /// Blinn-Phong local Illumnation Model (which is similar to the model used
  /// for polygonal surfaces). The lighting equation is defined as:
  ///
  /// Cg = IFrgb * (1 -f0)
  ///      + f0 * (CE rgb + SUM( oni * attenuationi * spoti * ILrgb
  ///                                * (ambienti + diffusei + specular i)))
  /// Og = Ov(1 - X3DMaterialNode transparency)
  ///
  /// where:
  ///
  /// attenuationi = 1 / max(a1 + a2 * dL + a3 * dL^2 , 1 )
  /// ambienti = Iia * CDrgb * Ca
  /// diffusei = Ii * CDrgb * ( N · L )
  /// specular i = Ii * CSrgb * ( N · ((L + V) / |L + V|))shininess * 128
  /// 
  /// and:
  /// · = modified vector dot product:
  ///     if dot product < 0,then 0.0, otherwise, dot product
  /// a1 , a2, a3 = light i attenuation
  /// dV = distance from this voxel to viewer's position, in coordinate system
  ///      of current fog node
  /// dL = distance from light to voxel, in light's coordinate system
  /// f0 = fog interpolant
  /// IFrgb = currently bound fog's color
  /// I Lrgb = light i color
  /// Ii = light i intensity
  /// Iia = light i ambientIntensity
  /// L = (PointLight/SpotLight) normalized vector from this voxel to light
  ///     source i position
  /// L = (DirectionalLight) -direction of light source i
  /// N = normalized normal vector at this voxel (interpolated from vertex
  ///     normals specified by the surfaceNormals field or automatically
  ///     calculated.
  /// Ca = X3DMaterialNode ambientIntensity
  /// CDrgb = diffuse colour, from a node derived from X3DMaterialNode
  /// CErgb = X3DMaterialNode emissiveColor
  /// CSrgb = X3DMaterialNode specularColor
  /// on i = 1, if light source i affects this voxel,
  ///        0, if light source i does not affect this voxel. The following
  ///        conditions indicate that light source i does not affect this voxel:
  ///          a. if the voxel is farther away than radius for PointLight or
  ///             SpotLight;
  ///          b. if the volume is outside the enclosing X3DGroupingNode;
  ///             and/or
  ///          c. if the on field is FALSE.
  ///          d. if the lighting field of this volume is FALSE.
  /// shininess = X3DMaterialNode shininess
  /// spotAngle = arccosine(-L · spotDiri)
  /// spot BW = SpotLight i beamWidth
  /// spot CO = SpotLight i cutOffAngle
  /// spot i = spotlight factor.
  /// spotDiri = normalized SpotLight i direction
  /// SUM: sum over all light sources i
  /// V = normalized vector from the voxel to viewer's position 
  ///
  /// The lighting field controls whether the rendering should calculate and
  /// apply shading effects to the visual output. When shading is applied, the
  /// value of the surfaceNormals field can be used to provide pre-generated
  /// surface normals for lighting calculations. If lighting is not enabled,
  /// then flat shading using the surface colour is to be used.
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
  /// The shadows field controls whether the rendering should calculate and
  /// apply shadows to the visual output. A value of FALSE requires that no
  /// shadowing be applied. A value of TRUE requires that shadows be applied to
  /// the object. If the lighting field is set to FALSE, this field shall be
  /// ignore and no shadows generated. This field will also be ignored if the
  /// requested component level is less than 4.
  ///
  /// The phaseFunction field is used to define the scattering model for use in
  /// an implementation using global illumnation. The name defines the model
  /// type, based on standard algorithms externally defined to this
  /// specification. The default implementation is the Henyey-Greenstein phase
  /// function defined in [HENYEY].
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../x3d/ShadedVolumeStyle.x3d">ShadedVolumeStyle.x3d</a>
  ///     ( <a href="x3d/ShadedVolumeStyle.x3d.html">Source</a> )
  ///
  class MEDX3D_API ShadedVolumeStyle : public X3DComposableVolumeRenderStyleNode {
  public:
    /// typedefs
    typedef DependentSFNode< X3DTexture3DNode, 
                             FieldRef<H3DDisplayListObject,
                                      H3DDisplayListObject::DisplayList,
                                      &H3D::H3DDisplayListObject::displayList>,
                             true > SFTexture3DNode;

    /// typedefs
    typedef DependentSFNode< X3DMaterialNode, 
                             FieldRef<H3DDisplayListObject,
                                      H3DDisplayListObject::DisplayList,
                                      &H3D::H3DDisplayListObject::displayList>,
                             true > SFMaterialNodeBase;
    
    /// Specializing SFMaterialNode to rebuild shader when the material node is
    /// changed(but not when the field values in the contained X3DMateralNode
    /// are changed).
    class MEDX3D_API SFMaterialNode: public SFMaterialNodeBase  {
      virtual void onAdd( Node *n);      
      virtual void onRemove( Node *n);
    };

    class MEDX3D_API FloatAboveValue : public SFFloat {
    public:
      virtual void setValue( const H3DFloat &f, int id = 0 ) {
        if( f < 0.0001f ) {
          SFFloat::setValue( 0.0001f );
        } else {
          SFFloat::setValue( f, id );
        }
      }
    protected:
      virtual void update() {
        SFFloat::update();
        if( value < 0.0001f ) {
          value = 0.0001f;
        }
      }
    };

    /// Constructor.
    ShadedVolumeStyle( Inst< DisplayList >     _displayList    = 0,
                       Inst< SFBool >          _enabled        = 0,
                       Inst< SFNode >          _metadata       = 0,
                       Inst< SFMaterialNode >  _material       = 0,
                       Inst< SFString >        _phaseFunction  = 0,
                       Inst< SFBool >          _shadows        = 0,
                       Inst< SFBool >          _lighting       = 0,
                       Inst< SFTexture3DNode > _surfaceNormals = 0,
                       Inst< FloatAboveValue > _lightRayStepSize = 0 );
    
    // Set default transfer function and volumes normals if needed.
    // For more info check X3DVolumeRenderStyleNode.
    virtual void updateUniformFields( X3DVolumeNode *vd );
    
    // Add uniforms
    // For more info check X3DVolumeRenderStyleNode.
    virtual string addUniforms( ComposedShader *c );

    // Add shader code
    // For more info check X3DVolumeRenderStyleNode.
    virtual string getShaderCode();

    // Add shader code
    // For more info check X3DVolumeRenderStyleNode.
    virtual string getShaderCodeOpacityOnly();

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

    /// If a material value is provided, the voxels are considered to be lit.
    /// If no material node is provided, they are considered to be unlit and
    /// the colour of the voxels completely transparent.
    ///
    /// <b>Access type:</b> inputOutput
    auto_ptr< SFMaterialNode > material;

    /// The phaseFunction field is used to define the scattering model for use
    /// in an implementation using global illumnation. The name defines the
    /// model type, based on standard algorithms externally defined to this
    /// specification. The default implementation is the Henyey-Greenstein
    /// phase function.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> "Henyey-Greenstein"
    auto_ptr<SFString> phaseFunction;
    
    /// Specifies if shadows should be used or not. Only supported in 
    /// ray caster in current implementation.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> FALSE
    auto_ptr<SFBool> shadows;

    /// The lighting field controls whether the rendering should calculate and
    /// apply shading effects to the visual output. When shading is applied,
    /// the value of the surfaceNormals field can be used to provide
    /// pre-generated surface normals for lighting calculations. If lighting
    /// is not enabled, then flat shading using the surface colour is to be
    /// used.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> FALSE
    auto_ptr< SFBool > lighting;

    /// If provided, it should contain a X3DTexture3DNode with surface normals
    /// 
    /// <b>Access type:</b> inputOutput
    auto_ptr< SFTexture3DNode > surfaceNormals;

    /// The step size to use when the shadows field is set to true for tracing 
    /// the light that reaches each fragment(in texture coordinates)
    /// Only for use with ray casting.
    /// The given value will be capped to the allowed values range.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> 0.03
    /// <b>Allowed values:</b> [0.0001;,inf]
    auto_ptr< FloatAboveValue > lightRayStepSize;
  
    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:
    // updated in updateUniformFields. Set to true if the surfaceNormals
    // field is not NULL.
    bool had_normals;

    // below are fields used internally in the shader to represent
    // the values of the material.
    auto_ptr< SFColorRGBA > emissiveColor;
    auto_ptr< SFColorRGBA > diffuseColor;
    auto_ptr< SFColorRGBA > ambientColor;
    auto_ptr< SFColorRGBA > specularColor;
    auto_ptr< SFFloat     > shininess;

  };
}

#endif
