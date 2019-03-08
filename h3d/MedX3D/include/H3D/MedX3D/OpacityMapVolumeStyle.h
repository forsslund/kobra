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
/// \file OpacityMapVolumeStyle.h
/// \brief Header file for OpacityMapVolumeStyle node
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __OPACITYMAPVOLUMESTYLE_H__
#define __OPACITYMAPVOLUMESTYLE_H__

#include <H3D/MedX3D/X3DComposableVolumeRenderStyleNode.h>

#include <H3D/X3DTexture2DNode.h>
#include <H3D/DependentNodeFields.h>

namespace H3D {

  /// \ingroup X3DNodes
  /// \class OpacityMapVolumeStyle
  /// Renders the volume using the opacity mapped to a transfer function
  /// texture. This is the default rendering style if none is defined for
  /// the volume data.
  ///
  /// The transferFunction field holds a single texture representation in
  /// either two or three dimensions that map the voxel data values to a
  /// specific colour output. If no value is supplied for this field, the
  /// default implementation shall generate a 256x1 greyscale alpha-only image
  /// that blends from completely transparent at pixel 0 to fully opaque at
  /// pixel 255. The texture may be any number of dimensions and any number
  /// of components. The voxel values are used as a lookup coordinates into
  /// the transfer function texture, where the texel value represents the
  /// output colour.
  ///
  /// Components are mapped from the voxel data to the transfer function in
  /// a component-wise fashion. The first component of the voxel data is an
  /// index into the first dimension of the transferFunction texture (S) and
  /// so on (see Table 1). If there are more components defined in the voxel
  /// data than there dimensions in the transfer function, the extra
  /// components are ignored. If there are more dimensions in the transfer
  /// function texture than the voxel data, the extra dimensions in 
  /// the transfer function are ignored (effectively treating the voxel
  /// component data as a value of zero for the extra dimension). This mapping
  /// the locates the texel value in the texture, which is then used as the
  /// output for this style. The colour value is treated like a normal texture
  /// with the colour mapping as defined in Table 2.
  ///
  /// Table 1  Transfer function texture coordinate mapping
  /// <table>
  /// <tr><th>Voxel Components<th>Transfer Function Texture Coordinates
  /// <th></tr>
  /// <tr><td>Luminance<td>S</tr>
  /// <tr><td>Luminance Alpha<td>S,T</tr>
  /// <tr><td>RGB<td>S,T,R</tr>
  /// <tr><td>RGBA<td>S,T,R,Q</tr>
  /// </table>
  ///
  /// Table 2  Transfer function texture type to output colour mapping
  /// <table>
  /// <tr><th>Texture Components<th>Red<th>Green<th>Blue<th>Alpha<th></tr>
  /// <tr><td>Luminance (L)<td>L<td>L<td>L<td>1</tr>
  /// <tr><td>Luminance Alpha (LA)<td>L<td>L<td>L<td>A</tr>
  /// <tr><td>RGB<td>R<td>G<td>B<td>1</tr>
  /// <tr><td>RGBA<td>R<td>G<td>B<td>A</tr>
  /// </table>
  ///
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../x3d/OpacityMapVolumeStyle.x3d">OpacityMapVolumeStyle.x3d</a>
  ///     ( <a href="x3d/OpacityMapVolumeStyle.x3d.html">Source</a> )
  ///
  class MEDX3D_API OpacityMapVolumeStyle : public X3DComposableVolumeRenderStyleNode {
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
    struct MEDX3D_API DefaultTransferFunction : public SFTextureNode {
      virtual void update();
    };
    
    /// Field that calculates a 2D texture containing the pre-integrated values of
    /// the transfer function.
    /// routes_in[0] is the transferFunction field.
    /// routes_in[1] is the preintegratedIntegralStep field.
    /// routes_in[2] is the stepSize field.
    /// routes_in[3] is the type field.
    class MEDX3D_API SFPreIntegratedTextureNode: public TypedField< TypedSFNode< X3DTexture2DNode >,
      Types< SFTextureNode, SFFloat, SFFloat, SFString > > {
      virtual void update();

      struct FunctionData {
        H3DFloat sf, sb, d;
        Image *image;
      };

      /// Calculate the preintegration table using the integral functions
      /// ignoring self-attenuation within ray segment (see page 97 in
      /// "Real-time volume graphics book".
      Image * createPreIntegrationTableFast( Image *transfer_function,
                                             H3DInt32 input_dim,
                                             H3DFloat d,
                                             Image *output = NULL );

      Image * createPreIntegrationTable2( Image *transfer_function,
                                          H3DInt32 input_dim,
                                          H3DFloat d,
                                          Image *output = NULL );

      

      RGBA calculatePreIntegratedColor( FunctionData &data ); 
      static H3DFloat textureLookupFunctionAlpha( H3DFloat w, void *data );
      static RGB textureLookupFunctionRGB( H3DFloat w, void *data );
    };

    /// Constructor.
    OpacityMapVolumeStyle( Inst< DisplayList >     _displayList = 0,
                           Inst< SFBool >          _enabled  = 0,
                           Inst< SFNode >          _metadata = 0,
                           Inst< SFTextureNode >   _transferFunction = 0,
                           Inst< SFString >        _type = 0,
                           Inst< SFPreIntegratedTextureNode > _preintegratedTransferFunction = 0,
                           Inst< SFFloat >         _preintegratedIntegralStep = 0 );
    
    /// Returns true if the style returns an associated color. An associated
    /// color means that the color that is output from the style is the 
    /// color pre-multiplied with the alpha value. We need to know this
    /// in order to choose the appropriate compositing formula.
    /// The input_assiciated input is true if the input to the style is
    /// an associated color.
    inline virtual bool producesAssociatedColor( bool input_associated ) {
      const string &ts = type->getValue();
      bool is_preintegrated = enabled->getValue() && (ts == "preintegrated" || ts == "preintegrated_fast");
      if( is_preintegrated ) return true;
      else return input_associated;
    }

    // Pre render
    // For more info check X3DVolumeRenderStyleNode.
    virtual void updateUniformFields( X3DVolumeNode *vd );
    
    // Add shader code
    // For more info check X3DVolumeRenderStyleNode.
    virtual string getShaderCode();
    
    /// Add uniforms
    // For more info check X3DVolumeRenderStyleNode.
    virtual string addUniforms( ComposedShader *s );

    /// Traverse the scene graph.
    virtual void traverseSG( TraverseInfo &ti );

    /// Add variable for tracking last color when preintegrated
    /// rendering.
    virtual string getShaderInitCode() { 
      const string &ts = type->getValue();
      // add the texture of preintegrated values if needed
      if( ts == "preintegrated" || ts == "preintegrated_fast" ) {
        return string( "vec4 " ) +
          UNIFORM_ID(last_orig_sample_color) +
          " = vec4(0,0,0,0);\n";
      } else {
        return "";
      }
    }
    
    /// Contains a X3DTextureNode that map the voxel data values to a 
    /// specific colour output
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> NULL
    auto_ptr< SFTextureNode > transferFunction;

    /// The type field controls different ways to apply the transfer function. Possible
    /// values are:
    /// - "simple" - transfer function is applied directly to the values.
    /// - "preintegrated" - using preintegrated rendering to avoid sampling artifacts in 
    /// transfer functions(higher image quality, change of transfer function requires a
    /// calculation of preintegrated data). This is not optimized at the moment
    /// and takes a very long time to calculate.
    /// - "preintegrated_fast" - same as "preintegrated" but the calculation of preintegrated
    /// data is done in a simplified way. Much faster but not as accurate since it ignores
    /// self-attenuation within a ray segment. 
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> "simple" 
    auto_ptr< SFString > type;

    /// If using preintegrated texture this is a 2D texture containing the pre-integrated values
    /// of the transfer function. It is calculated based on the type, preintegratedImageDimension
    /// and preintegratedIntegralStep fields.
    ///
    /// <b>Access type:</b> outputOnly \n
    auto_ptr< SFPreIntegratedTextureNode > preintegratedTransferFunction;

    /// The step size to use in the internal calculations of integrals.
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> 0.001
    auto_ptr< SFFloat > preintegratedIntegralStep;

    // copy of field that is added to shader
    auto_ptr< SFTextureNode > transferFunction_glsl;
    
    /// The default transfer function
    auto_ptr< DefaultTransferFunction > defaultTransferFunction;
    
    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

    /// The function returns
    /// true if the value range min_v to max_v can be empty, i.e. there
    /// exists a value in the range that causes the alpha value to be 0.
    /// The previous_empty value is true if a previous step has decided
    /// the range is empty. 
    virtual bool isEmptySpace( H3DFloat min_v, 
                               H3DFloat max_v, 
                               bool previous_empty );
  protected:
    unsigned int nr_components_in_data;
    bool transfer_function_is_2d;
    /// The step size from the X3DVolumeNode this node is in.
    auto_ptr< SFFloat > stepSize;
    
    /// The value of the transferFunction field in the last call to 
    /// traverseSG
    Node *last_loop_tf;

  };
}

#endif
