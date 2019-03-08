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
/// \file EdgeEnhancementVolumeStyle.cpp
/// \brief CPP file for EdgeEnhancementVolumeStyle, X3D scene-graph node
///
//
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/MedX3D/EdgeEnhancementVolumeStyle.h>
#include <H3D/MedX3D/VolumeGradient.h>

#include <H3D/ShaderPart.h>
#include <H3D/PixelTexture.h>
#include <H3D/Pixel3DTexture.h>
#include <H3D/TextureProperties.h>
 
namespace H3D {
  
  H3DNodeDatabase 
  EdgeEnhancementVolumeStyle::database(
          "EdgeEnhancementVolumeStyle", 
          &(newInstance<EdgeEnhancementVolumeStyle>),
          typeid( EdgeEnhancementVolumeStyle ),
          &X3DComposableVolumeRenderStyleNode::database );
  
  namespace EdgeEnhancementVolumeStyleInternals {
    FIELDDB_ELEMENT( EdgeEnhancementVolumeStyle, edgeColor, INPUT_OUTPUT )
    FIELDDB_ELEMENT( EdgeEnhancementVolumeStyle,
                     gradientThreshold, INPUT_OUTPUT )
    FIELDDB_ELEMENT( EdgeEnhancementVolumeStyle,
                     surfaceNormals, INPUT_OUTPUT )
  }
  
  EdgeEnhancementVolumeStyle::
  EdgeEnhancementVolumeStyle( Inst<DisplayList>_displayList, 
                              Inst<SFBool> _enabled,
                              Inst<SFNode> _metadata,
                              Inst< SFColor > _edgeColor,
                              Inst< SFFloat > _gradientThreshold,
                              Inst< SFTexture3DNode > _surfaceNormals ) :
    X3DComposableVolumeRenderStyleNode( _displayList, _enabled, _metadata ),
    edgeColor( _edgeColor ),
    gradientThreshold( _gradientThreshold ),
    surfaceNormals( _surfaceNormals ),
    had_normals( false ) {
    type_name = "EdgeEnhancementVolumeStyle";
    database.initFields( this );
    
    // defaults
    edgeColor->setValue( RGB(0,0,0) );
    gradientThreshold->setValue( 0.4f );
    
    // routings
    edgeColor->route( displayList );
    gradientThreshold->route( displayList );
    surfaceNormals->route( displayList );
  }
  
  string EdgeEnhancementVolumeStyle::
  addUniforms( ComposedShader *s ) {
    return
    // add the fields to the shader
    addUniformToFragmentShader( s,
                                UNIFORM_ID(enabled),
                                "bool",
                                H3D::Field::INPUT_OUTPUT, 
                                copyAndRouteField(enabled) ) +
    addUniformToFragmentShader( s,
                                UNIFORM_ID(edgeColor),
                                "vec3",
                                H3D::Field::INPUT_OUTPUT,
                                copyAndRouteField(edgeColor) ) +    
    addUniformToFragmentShader( s,
                                UNIFORM_ID(gradientThreshold),
                                "float",
                                H3D::Field::INPUT_OUTPUT,
                                copyAndRouteField(gradientThreshold) ) +
    (surfaceNormals->getValue() ?
      addUniformToFragmentShader( s,
                                  UNIFORM_ID(surfaceNormals),
                                  "sampler3D",
                                  H3D::Field::INPUT_OUTPUT,
                                  copyAndRouteField( surfaceNormals) ) : "");
  }
  
  void EdgeEnhancementVolumeStyle::updateUniformFields( X3DVolumeNode *vd ) {
    // rebuild shader if normals have been added or removed since
    // we need to add/remove the uniform surfaceNormals then
    bool have_normals = surfaceNormals->getValue() != NULL;
    if( have_normals != had_normals ) rebuildShader->touch();
    had_normals = have_normals;
    
    X3DComposableVolumeRenderStyleNode::updateUniformFields( vd );
  }
  
  string EdgeEnhancementVolumeStyle::getShaderCode() {
    return
      string( "EdgeEnhancementVolumeStyle(" ) +
      string( "sample_color, " ) +
      string( "r0, " ) +
      string( "viewdir_tex, " ) + 
      UNIFORM_ID_(enabled) + 
      UNIFORM_ID_(edgeColor) +
      UNIFORM_ID_(gradientThreshold) +
      (surfaceNormals->getValue() ? 
       "normalizedNormalFromTexture( " + UNIFORM_ID(surfaceNormals) + ", r0 )" :
       "sample_default_normal" )+
      ");";
  }
  
}




  
