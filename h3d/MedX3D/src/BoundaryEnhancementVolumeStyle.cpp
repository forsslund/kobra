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
/// \file BoundaryEnhancementVolumeStyle.cpp
/// \brief CPP file for BoundaryEnhancementVolumeStyle, X3D scene-graph node
///
//
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/MedX3D/BoundaryEnhancementVolumeStyle.h>
#include <H3D/MedX3D/VolumeGradient.h>

#include <H3D/ShaderPart.h>
#include <H3D/PixelTexture.h>
#include <H3D/Pixel3DTexture.h>
#include <H3D/TextureProperties.h>
 
namespace H3D {
  
  H3DNodeDatabase 
  BoundaryEnhancementVolumeStyle::database( 
              "BoundaryEnhancementVolumeStyle", 
              &(newInstance<BoundaryEnhancementVolumeStyle>),
              typeid( BoundaryEnhancementVolumeStyle ),
              &X3DComposableVolumeRenderStyleNode::database );
  
  namespace BoundaryEnhancementVolumeStyleInternals {
    FIELDDB_ELEMENT( BoundaryEnhancementVolumeStyle, retainedOpacity,
                     INPUT_OUTPUT )
    FIELDDB_ELEMENT( BoundaryEnhancementVolumeStyle, boundaryOpacity,
                     INPUT_OUTPUT )
    FIELDDB_ELEMENT( BoundaryEnhancementVolumeStyle, opacityFactor,
                     INPUT_OUTPUT )
    FIELDDB_ELEMENT( BoundaryEnhancementVolumeStyle, surfaceNormals,
                     INPUT_OUTPUT )
  }
  
  BoundaryEnhancementVolumeStyle::
  BoundaryEnhancementVolumeStyle( Inst<DisplayList>_displayList, 
                                  Inst<SFBool> _enabled,
                                  Inst<SFNode> _metadata,
                                  Inst< SFFloat > _retainedOpacity,
                                  Inst< SFFloat > _boundaryOpacity,
                                  Inst< SFFloat > _opacityFactor,
                                  Inst< SFTexture3DNode > _surfaceNormals ) :
    X3DComposableVolumeRenderStyleNode( _displayList, _enabled, _metadata ),
    retainedOpacity( _retainedOpacity ),
    boundaryOpacity( _boundaryOpacity ),
    opacityFactor( _opacityFactor ),
    surfaceNormals( _surfaceNormals ),
    had_normals( false )
  {
    type_name = "BoundaryEnhancementVolumeStyle";
    database.initFields( this );
    
    // defaults
    retainedOpacity->setValue( 1.0 );
    boundaryOpacity->setValue( 0.0 );
    opacityFactor->setValue( 1.0 );
    
    // routings
    retainedOpacity->route( displayList );
    boundaryOpacity->route( displayList );
    opacityFactor->route( displayList );
    surfaceNormals->route( displayList );
  }
  
  void BoundaryEnhancementVolumeStyle::
  updateUniformFields( X3DVolumeNode *vd ) {
    // rebuild shader if normals have been added or removed since
    // we need to add/remove the uniform surfaceNormals then
    bool have_normals = surfaceNormals->getValue() != NULL;
    if( have_normals != had_normals ) rebuildShader->touch();
    had_normals = have_normals;
    
    X3DComposableVolumeRenderStyleNode::updateUniformFields( vd );
  }
  
  string BoundaryEnhancementVolumeStyle::
  addUniforms( ComposedShader *s ) {
    
    return
    // add the fields to the shader
    addUniformToFragmentShader( s,
                                UNIFORM_ID(enabled),
                                "bool",
                                H3D::Field::INPUT_OUTPUT, 
                                copyAndRouteField(enabled) ) +
    addUniformToFragmentShader( s,
                                UNIFORM_ID(retainedOpacity),
                                "float",
                                H3D::Field::INPUT_OUTPUT,
                                copyAndRouteField(retainedOpacity) ) +
    addUniformToFragmentShader( s,
                                UNIFORM_ID(boundaryOpacity),
                                "float",
                                H3D::Field::INPUT_OUTPUT,
                                copyAndRouteField(boundaryOpacity) ) +
    addUniformToFragmentShader( s,
                                UNIFORM_ID(opacityFactor),
                                "float",
                                H3D::Field::INPUT_OUTPUT,
                                copyAndRouteField(opacityFactor) ) +
    (surfaceNormals->getValue() ?
      addUniformToFragmentShader( s,
                                  UNIFORM_ID(surfaceNormals),
                                  "sampler3D",
                                  H3D::Field::INPUT_OUTPUT,
                                  copyAndRouteField( surfaceNormals) ) : "");
    
  }
  
  string BoundaryEnhancementVolumeStyle::
  getShaderCode() {
    return 
      string( "BoundaryEnhancementVolumeStyle(" ) +
      string( "sample_color, " ) +
      string( "r0, " ) +
      UNIFORM_ID_(enabled) + 
      UNIFORM_ID_(retainedOpacity) +
      UNIFORM_ID_(boundaryOpacity) +
      UNIFORM_ID_(opacityFactor) +
      (surfaceNormals->getValue() ? 
       "normalizedNormalFromTexture( " + UNIFORM_ID(surfaceNormals) + ", r0 )" :
       "sample_default_normal" )+
      ");";
  }
}




