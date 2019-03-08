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
/// \file BlendedVolumeStyle.h
/// \brief Header file for BlendedVolumeStyle node
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __BLENDEDVOLUMESTYLE_H__
#define __BLENDEDVOLUMESTYLE_H__

#include <H3D/MedX3D/X3DComposableVolumeRenderStyleNode.h>

#include <H3D/X3DTextureNode.h>
#include <H3D/X3DTexture3DNode.h>
#include <H3D/DependentNodeFields.h>
#include <H3D/SFColorRGBA.h>
#include <H3D/SFFloat.h>
#include <H3D/MedX3D/X3DVolumeNode.h>

namespace H3D {

  /// \ingroup X3DNodes
  /// \class BlendedVolumeStyle
  ///
  /// The BlendedVolumeStyle combines the rendering of two voxel data sets 
  /// into one by blending the values according to a weight function. The 
  /// first data set is the one that the style is being applied to. The 
  /// second data set and its render style is defined with the voxels and
  /// renderStyle fields just as in the VolumeData node.
  ///
  /// The fields weightConstant1, weightFunction1, and weightTransferFunction1
  /// apply to the target volume as specified in the parent X3DVolumeNode
  /// node. The fields weightConstant2, weightFunction2, and 
  /// weightTransferFunction2 apply to the voxels field specified in the
  /// BlendedVolumeStyle node.
  ///
  /// The final colour is determined by:
  /// Cg = clamp[0-1]( Cv * w1 + Cblend * w2 ) 
  /// Og = clamp[0-1]( Ov * w1 + Oblend * w2 ) 
  ///
  /// where Cblend and Oblend is the color and alpha value of the second
  /// data set after the rendering style has been applied. The values of w1 
  /// and w2 depends on the weightFunction1 and weightFunction2 fields, 
  /// respectively, as defined in Table 41.3 
  ///
  /// weightTransferFunction1 and weightTransferFunction2 specify a 
  /// 2-dimensional texture that is used to determine the weight values
  /// when the weight function is set to "TABLE". The output weight value
  /// depends on the number of components in the texture as specified 
  /// in Table 41.4. 
  ///
  /// Table 41.4 - Transfer function to weight mapping 
  /// Components           Weight
  /// Luminance(L)           L
  /// Luminance Alpha(LA)     L
  /// RGB                     R
  /// RGBA                   R
  ///
  ///
  /// Table 41.3 - Weight function types
  /// Value  Description
  /// "CONSTANT"  Use weightConstant1 if specified for weightFunction1, and weightConstant2 if specified for weightFunction2.
  /// "ALPHA1"  Use Ov
  /// "ALPHA2"  Use Oblend 
  /// "ONE_MINUS_ALPHA1"  Use 1 - Ov
  /// "ONE_MINUS_ALPHA2"  Use 1 - Oblend 
  /// "TABLE"   If specified for weightFunction1, lookup value for texture 
  /// coordinate (Ov,Oblend) in weightTransferFunction1 and map to weight
  /// value according to Table 41.4. If weightTransferFunction1 is NULL
  /// use Ov. If specified for weightFunction2, lookup value for texture
  /// coordinate (Oblend,Ov) in weightTransferFunction2 and map to weight
  /// value according to Table 41.4. If weightTransferFunction2 is NULL 
  /// use Oblend
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../x3d/BlendedVolumeStyle.x3d">BlendedVolumeStyle.x3d</a>
  ///     ( <a href="x3d/BlendedVolumeStyle.x3d.html">Source</a> )
  ///
  class MEDX3D_API BlendedVolumeStyle : 
    public X3DComposableVolumeRenderStyleNode {
  public:

    typedef X3DVolumeNode::SurfaceNormals SurfaceNormals;
    
    class MEDX3D_API SFComposableVolumeRenderStyleNode: 
    public DependentSFNode< X3DComposableVolumeRenderStyleNode, 
      FieldRef< H3DDisplayListObject,
      H3DDisplayListObject::DisplayList,
      &H3DDisplayListObject::displayList >, 
      true >  {

      typedef DependentSFNode< X3DComposableVolumeRenderStyleNode, 
        FieldRef< H3DDisplayListObject,
        H3DDisplayListObject::DisplayList,
        &H3DDisplayListObject::displayList >, 
        true > SFComposableVolumeRenderStyleNodeBase;
      virtual void onAdd( Node *n);      
      virtual void onRemove( Node *n);
    };

    typedef DependentSFNode< X3DTexture3DNode,
                             FieldRef<H3DDisplayListObject,
                                      H3DDisplayListObject::DisplayList,
                                      &H3D::H3DDisplayListObject::displayList>,
                             true > SFTexture3DNode;
    
    typedef DependentSFNode< X3DTexture2DNode,
                             FieldRef<H3DDisplayListObject,
                                      H3DDisplayListObject::DisplayList,
                                      &H3D::H3DDisplayListObject::displayList>,
                             true > SFTexture2DNode;


    /// Constructor.
    BlendedVolumeStyle( Inst< DisplayList >     _displayList = 0,
                        Inst< SFBool >          _enabled = 0,
                        Inst< SFNode >          _metadata = 0,
                        Inst< SFTexture3DNode > _voxels = 0,
                        Inst< SurfaceNormals  > _surfaceNormals = 0,
                        Inst< SFComposableVolumeRenderStyleNode > _renderStyle = 0,
                        Inst< SFFloat >         _weightConstant1 = 0,
                        Inst< SFFloat >         _weightConstant2 = 0,
                        Inst< SFString >        _weightFunction1 = 0,
                        Inst< SFString >        _weightFunction2 = 0,
                        Inst< SFTexture2DNode > _weightTransferFunction1 = 0,
                        Inst< SFTexture2DNode > _weightTransferFunction2 = 0 );
    /// The function returns
    /// true if the value range min_v to max_v can be empty, i.e. there
    /// exists a value in the range that causes the alpha value to be 0.
    /// The previous_empty value is true if a previous step has decided
    /// the range is possibly empty. 
    virtual bool isEmptySpace( H3DFloat min_v, 
                               H3DFloat max_v, 
                               bool previous_empty );


    /// Returns true if the style in the renderStyle field needs lights.
    virtual bool requiresEnabledLights();

    // add default transfer function and volumes normals if needed
    // For more info check X3DVolumeRenderStyleNode.
    virtual void updateUniformFields( X3DVolumeNode *vd );
    
    // add uniforms
    // For more info check X3DVolumeRenderStyleNode.
    virtual string addUniforms( ComposedShader *s );
    
    // add inside loop
    // For more info check X3DVolumeRenderStyleNode.
    virtual string getShaderCode();

    // add shader init code
    // For more info check X3DVolumeRenderStyleNode.
    virtual string getShaderInitCode();

    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> NULL \n
    auto_ptr<SFTexture3DNode> voxels;

    /// The field containing the automatically computed gradients
    /// for the voxel data set to be used for rendering styles that
    /// need a normal in its calculation.
    /// <b>Access type:</b> inputOutput
    auto_ptr< SurfaceNormals > surfaceNormals;

    /// The style to apply to voxels to produce the second blend component.
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> NULL \n
    auto_ptr<SFComposableVolumeRenderStyleNode> renderStyle;

    /// Weight to use for first blend component if weightFunction1
    /// is "CONSTANT".
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> 0.5 \n
    auto_ptr<SFFloat> weightConstant1;
    
    /// Weight to use for second blend component if weightFunction2
    /// is "CONSTANT".
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> 0.5 \n
    auto_ptr<SFFloat> weightConstant2;
    
    /// The function to use to calculate the weight for the first 
    /// blend component.
    /// 
    /// "CONSTANT"  Use weightConstant1 
    /// "ALPHA1"  Use Ov
    /// "ALPHA2"  Use Oblend 
    /// "ONE_MINUS_ALPHA1"  Use 1 - Ov
    /// "ONE_MINUS_ALPHA2"  Use 1 - Oblend 
    /// "TABLE"   Lookup value for texture coordinate (Ov,Oblend) in 
    /// weightTransferFunction1 and map to weight value according to 
    /// Table 41.4. 
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> "CONSTANT" \n
    /// <b>Valid values:</b> "CONSTANT", "ALPHA1", "ALPHA2", 
    /// "ONE_MINUS_ALPHA1", "ONE_MINUS_ALPHA2", "TABLE" \n
    auto_ptr<SFString> weightFunction1;

    /// The function to use to calculate the weight for the second 
    /// blend component.
    /// 
    /// "CONSTANT"  Use weightConstant2 
    /// "ALPHA1"  Use Ov
    /// "ALPHA2"  Use Oblend 
    /// "ONE_MINUS_ALPHA1"  Use 1 - Ov
    /// "ONE_MINUS_ALPHA2"  Use 1 - Oblend 
    /// "TABLE"   Lookup value for texture coordinate (Oblend,Ov) in 
    /// weightTransferFunction2 and map to weight value according to 
    /// Table 41.4. 
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> "CONSTANT" \n
    /// <b>Valid values:</b> "CONSTANT", "ALPHA1", "ALPHA2", 
    /// "ONE_MINUS_ALPHA1", "ONE_MINUS_ALPHA2", "TABLE" \n
    auto_ptr<SFString> weightFunction2;

    /// Transfer function to use for first blend component if weightFunction1
    /// is "TABLE".
    /// 
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> NULL \n
    auto_ptr<SFTexture2DNode> weightTransferFunction1;

    /// Transfer function to use for first blend component if weightFunction1
    /// is "TABLE".
    /// 
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> NULL \n
    auto_ptr<SFTexture2DNode> weightTransferFunction2;

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:
    /// This field is put between 
    /// surfaceNormals and the shader defaultNormal field and is needed
    /// to get the correct texture values in the shader.
    /// surfaceNormals->surfaceNormals_glsl->ComposedShader.defaultNormals
    auto_ptr< SFTexture3DNode > surfaceNormals_glsl;

  };
}

#endif
