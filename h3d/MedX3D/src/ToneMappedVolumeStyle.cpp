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
/// \file ToneMappedVolumeStyle.cpp
/// \brief CPP file for ToneMappedVolumeStyle, X3D scene-graph node
///
//
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/MedX3D/ToneMappedVolumeStyle.h>
#include <H3D/MedX3D/VolumeGradient.h>

#include <H3D/ShaderPart.h>
#include <H3D/Pixel3DTexture.h>

namespace H3D {
  
  H3DNodeDatabase 
  ToneMappedVolumeStyle::database(
          "ToneMappedVolumeStyle", 
          &(newInstance<ToneMappedVolumeStyle>),
          typeid( ToneMappedVolumeStyle ),
          &X3DComposableVolumeRenderStyleNode::database );
  
  namespace ToneMappedVolumeStyleInternals {
    FIELDDB_ELEMENT( ToneMappedVolumeStyle, coolColor, INPUT_OUTPUT )
    FIELDDB_ELEMENT( ToneMappedVolumeStyle, warmColor, INPUT_OUTPUT )
    FIELDDB_ELEMENT( ToneMappedVolumeStyle, surfaceNormals, INPUT_OUTPUT )
  }
  
  ToneMappedVolumeStyle::ToneMappedVolumeStyle(
    Inst< DisplayList >_displayList, 
    Inst< SFBool > _enabled,
    Inst< SFNode > _metadata,
    Inst< SFColorRGBA > _coolColor,
    Inst< SFColorRGBA > _warmColor,
    Inst< SFTexture3DNode > _surfaceNormals ) :
    X3DComposableVolumeRenderStyleNode( _displayList, _enabled, _metadata ),
    coolColor( _coolColor ),
    warmColor( _warmColor ),
    surfaceNormals( _surfaceNormals ),
    had_normals( false ) {
    
    type_name = "ToneMappedVolumeStyle";
    database.initFields( this );
        
    // routings
    coolColor->route( displayList );
    warmColor->route( displayList );
    surfaceNormals->route( displayList );
    
    // defaults
    coolColor->setValue( RGBA( 0, 0, 1, 0 ) );
    warmColor->setValue( RGBA( 1, 1, 0, 0 ) );
  }
  
  string ToneMappedVolumeStyle::addUniforms( ComposedShader *s ) {
    return
    addUniformToFragmentShader( s,
                                UNIFORM_ID(enabled),
                                "bool",
                                H3D::Field::INPUT_OUTPUT, 
                                copyAndRouteField(enabled) ) +
    addUniformToFragmentShader( s,
                                UNIFORM_ID(coolColor),
                                "vec4",
                                H3D::Field::INPUT_OUTPUT,
                                copyAndRouteField(coolColor) ) +
    addUniformToFragmentShader( s,
                                UNIFORM_ID(warmColor),
                                "vec4",
                                H3D::Field::INPUT_OUTPUT,
                                copyAndRouteField(warmColor) ) +  
    (surfaceNormals->getValue() ?
      addUniformToFragmentShader( s,
                                  UNIFORM_ID(surfaceNormals),
                                  "sampler3D",
                                  H3D::Field::INPUT_OUTPUT,
                                  copyAndRouteField( surfaceNormals) ) : "");
  }
  
  void ToneMappedVolumeStyle::updateUniformFields( X3DVolumeNode *vd ) {
    // rebuild shader if normals have been added or removed since
    // we need to add/remove the uniform surfaceNormals then
    bool have_normals = surfaceNormals->getValue() != NULL;
    if( have_normals != had_normals ) rebuildShader->touch();
    had_normals = have_normals;
    
    X3DComposableVolumeRenderStyleNode::updateUniformFields( vd );
  }
  
  string ToneMappedVolumeStyle::getShaderCode() {
    return
      string( "ToneMappedVolumeStyle(" ) +
      string( "sample_color, " ) +
      string( "r0, " ) +
      string( "view_to_tex, " ) + 
      UNIFORM_ID_(enabled) +
      UNIFORM_ID_(coolColor) +
      UNIFORM_ID_(warmColor) +
       (surfaceNormals->getValue() ? 
       "normalizedNormalFromTexture( " + UNIFORM_ID(surfaceNormals) + ", r0 )" :
       "sample_default_normal" )+
      ");";
  }
}
