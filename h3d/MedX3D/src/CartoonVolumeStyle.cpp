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
/// \file CartoonVolumeStyle.cpp
/// \brief CPP file for CartoonVolumeStyle, X3D scene-graph node
///
//
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/MedX3D/CartoonVolumeStyle.h>
#include <H3D/MedX3D/VolumeGradient.h>
#include <H3D/ShaderPart.h>
#include <H3D/Pixel3DTexture.h>

namespace H3D {
  
  H3DNodeDatabase 
  CartoonVolumeStyle::database( "CartoonVolumeStyle", 
                                &(newInstance<CartoonVolumeStyle>),
                                typeid( CartoonVolumeStyle ),
                                &X3DComposableVolumeRenderStyleNode::database);
  
  namespace CartoonVolumeStyleInternals {
    FIELDDB_ELEMENT( CartoonVolumeStyle, parallelColor, INPUT_OUTPUT )
    FIELDDB_ELEMENT( CartoonVolumeStyle, orthogonalColor, INPUT_OUTPUT )
    FIELDDB_ELEMENT( CartoonVolumeStyle, colorSteps, INPUT_OUTPUT )
    FIELDDB_ELEMENT( CartoonVolumeStyle, surfaceNormals, INPUT_OUTPUT )
    
  }
  
  CartoonVolumeStyle::CartoonVolumeStyle( Inst< DisplayList >_displayList, 
                                          Inst< SFBool > _enabled,
                                          Inst< SFNode > _metadata,
                                          Inst< SFColorRGBA > _parallelColor,
                                          Inst< SFColorRGBA > _orthogonalColor,
                                          Inst< SFInt32     > _colorSteps, 
                                     Inst< SFTexture3DNode > _surfaceNormals) :
    X3DComposableVolumeRenderStyleNode( _displayList, _enabled, _metadata ),
    parallelColor( _parallelColor ),
    orthogonalColor( _orthogonalColor ),
    colorSteps( _colorSteps ),
    surfaceNormals( _surfaceNormals ),
    had_normals( false ) 
  {
    
    type_name = "CartoonVolumeStyle";
    database.initFields( this );
    
    // routings
    parallelColor->route( displayList );
    orthogonalColor->route( displayList );
    colorSteps->route( displayList );
    surfaceNormals->route( displayList );
    
    // defaults
    parallelColor->setValue( RGBA( 0, 0, 0, 1 ) );
    orthogonalColor->setValue( RGBA( 1, 1, 1, 1 ) );
    colorSteps->setValue( 4 );
  }
  
  string CartoonVolumeStyle::addUniforms( ComposedShader *s ) {
    return
      addUniformToFragmentShader( s,
                                  UNIFORM_ID(enabled),
                                  "bool",
                                  H3D::Field::INPUT_OUTPUT, 
                                  copyAndRouteField(enabled) ) +
    addUniformToFragmentShader( s,
                                UNIFORM_ID(parallelColor),
                                "vec4",
                                H3D::Field::INPUT_OUTPUT,
                                copyAndRouteField(parallelColor) ) +
    addUniformToFragmentShader( s,
                                UNIFORM_ID(orthogonalColor),
                                "vec4",
                                H3D::Field::INPUT_OUTPUT,
                                copyAndRouteField(orthogonalColor) ) +
    addUniformToFragmentShader( s,
                                UNIFORM_ID(colorSteps),
                                "int",
                                H3D::Field::INPUT_OUTPUT,
                                copyAndRouteField(colorSteps) ) +
    (surfaceNormals->getValue() ?
      addUniformToFragmentShader( s,
                                  UNIFORM_ID(surfaceNormals),
                                  "sampler3D",
                                  H3D::Field::INPUT_OUTPUT,
                                  copyAndRouteField( surfaceNormals) ) : "");
  }
  
  void CartoonVolumeStyle::updateUniformFields( X3DVolumeNode *vd ) {
    // rebuild shader if normals have been added or removed since
    // we need to add/remove the uniform surfaceNormals then
    bool have_normals = surfaceNormals->getValue() != NULL;
    if( have_normals != had_normals ) rebuildShader->touch();
    had_normals = have_normals;
    
    X3DComposableVolumeRenderStyleNode::updateUniformFields( vd );
  }
  
  string CartoonVolumeStyle::getShaderInitCode() {
    return 
      string( "vec4 " ) +
      VARIABLE_ID(parallelColorHSV) +
      string( " = RGBToHSV(" ) +
      UNIFORM_ID(parallelColor) + 
      string( ");\n" ) +
      string( "vec4 " ) +
      VARIABLE_ID(orthogonalColorHSV) +
      string( " = RGBToHSV(" ) +
      UNIFORM_ID(orthogonalColor) + 
      string( ");" );
  }

  string CartoonVolumeStyle::getShaderCode() {
    return
      string( "CartoonVolumeStyle(" ) +
      string( "sample_color, " ) +
      string( "r0, " ) +
      string( "viewdir_tex, " ) + 
      UNIFORM_ID_(enabled) +
      VARIABLE_ID_(parallelColorHSV) +
      VARIABLE_ID_(orthogonalColorHSV) +
      UNIFORM_ID_(colorSteps) +
      (surfaceNormals->getValue() ? 
       "normalizedNormalFromTexture( " + UNIFORM_ID(surfaceNormals) + ", r0 )" :
       "sample_default_normal" )+
      ");";
  }
}
