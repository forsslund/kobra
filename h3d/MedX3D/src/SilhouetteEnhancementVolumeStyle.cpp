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
/// \file SilhouetteEnhancementVolumeStyle.cpp
/// \brief CPP file for SilhouetteEnhancementVolumeStyle, X3D scene-graph node
///
//
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/MedX3D/SilhouetteEnhancementVolumeStyle.h>
#include <H3D/MedX3D/VolumeGradient.h>

#include <H3D/ShaderPart.h>
#include <H3D/PixelTexture.h>
#include <H3D/Pixel3DTexture.h>
#include <H3D/TextureProperties.h>
 
namespace H3D {
  
  H3DNodeDatabase 
  SilhouetteEnhancementVolumeStyle::database(
                "SilhouetteEnhancementVolumeStyle", 
                &(newInstance<SilhouetteEnhancementVolumeStyle>),
                typeid( SilhouetteEnhancementVolumeStyle ),
                &X3DComposableVolumeRenderStyleNode::database );
  
  namespace SilhouetteEnhancementVolumeStyleInternals {
    FIELDDB_ELEMENT( SilhouetteEnhancementVolumeStyle,
                     silhouetteBoundaryOpacity, INPUT_OUTPUT )
    FIELDDB_ELEMENT( SilhouetteEnhancementVolumeStyle,
                     silhouetteRetainedOpacity, INPUT_OUTPUT )
    FIELDDB_ELEMENT( SilhouetteEnhancementVolumeStyle,
                     silhouetteSharpness, INPUT_OUTPUT )
    FIELDDB_ELEMENT( SilhouetteEnhancementVolumeStyle,
                     surfaceNormals, INPUT_OUTPUT )
  }
  
  SilhouetteEnhancementVolumeStyle::
  SilhouetteEnhancementVolumeStyle( Inst<DisplayList>_displayList, 
                                    Inst<SFBool> _enabled,
                                    Inst<SFNode> _metadata,
                                    Inst< SFFloat > _silhouetteBoundaryOpacity,
                                    Inst< SFFloat > _silhouetteRetainedOpacity,
                                    Inst< SFFloat > _silhouetteSharpness,
                                    Inst< SFTexture3DNode > _surfaceNormals ) :
    X3DComposableVolumeRenderStyleNode( _displayList, _enabled, _metadata ),
    silhouetteBoundaryOpacity( _silhouetteBoundaryOpacity ),
    silhouetteRetainedOpacity( _silhouetteRetainedOpacity ),
    silhouetteSharpness( _silhouetteSharpness ),
    surfaceNormals( _surfaceNormals ),
    had_normals( false ) {
    type_name = "SilhouetteEnhancementVolumeStyle";
    database.initFields( this );
    
    // defaults
    silhouetteBoundaryOpacity->setValue( 0.0f );
    silhouetteRetainedOpacity->setValue( 1.0f );
    silhouetteSharpness->setValue( 0.5f );

    // routings
    silhouetteBoundaryOpacity->route( displayList );
    silhouetteRetainedOpacity->route( displayList );
    silhouetteSharpness->route( displayList );
    surfaceNormals->route( displayList );
  }
  
  string SilhouetteEnhancementVolumeStyle::addUniforms( ComposedShader *s ) {
    return 
    // add the fields to the shader
    addUniformToFragmentShader( s,
                                UNIFORM_ID(enabled),
                                "bool",
                                H3D::Field::INPUT_OUTPUT, 
                                copyAndRouteField(enabled) ) +
    addUniformToFragmentShader( s,
                                UNIFORM_ID(silhouetteBoundaryOpacity),
                                "float",
                                H3D::Field::INPUT_OUTPUT,
                                copyAndRouteField(silhouetteBoundaryOpacity) )+
    addUniformToFragmentShader( s,
                                UNIFORM_ID(silhouetteRetainedOpacity),
                                "float",
                                H3D::Field::INPUT_OUTPUT,
                                copyAndRouteField(silhouetteRetainedOpacity) )+
    addUniformToFragmentShader( s,
                                UNIFORM_ID(silhouetteSharpness),
                                "float",
                                H3D::Field::INPUT_OUTPUT,
                                copyAndRouteField(silhouetteSharpness) ) +
    (surfaceNormals->getValue() ?
      addUniformToFragmentShader( s,
                                  UNIFORM_ID(surfaceNormals),
                                  "sampler3D",
                                  H3D::Field::INPUT_OUTPUT,
                                  copyAndRouteField( surfaceNormals) ) : "");
  }
  
  void SilhouetteEnhancementVolumeStyle::
  updateUniformFields( X3DVolumeNode *vd ) {
    // rebuild shader if normals have been added or removed since
    // we need to add/remove the uniform surfaceNormals then
    bool have_normals = surfaceNormals->getValue() != NULL;
    if( have_normals != had_normals ) rebuildShader->touch();
    had_normals = have_normals;    
    
    X3DComposableVolumeRenderStyleNode::updateUniformFields( vd );
  }
  
  string SilhouetteEnhancementVolumeStyle::getShaderCode() {
    return
      string( "SilhouetteEnhancementVolumeStyle(" ) +
      string( "sample_color, " ) +
      string( "r0, " ) +
      string( "viewdir_tex, " ) + 
      UNIFORM_ID_(enabled) +
      UNIFORM_ID_(silhouetteBoundaryOpacity) +
      UNIFORM_ID_(silhouetteRetainedOpacity) +
      UNIFORM_ID_(silhouetteSharpness) +
      (surfaceNormals->getValue() ? 
       "normalizedNormalFromTexture( " + UNIFORM_ID(surfaceNormals) + ", r0 )" :
       "sample_default_normal" )+
      ");";
  }
}




