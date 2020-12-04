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
/// \file ShadedVolumeStyle.cpp
/// \brief CPP file for ShadedVolumeStyle, X3D scene-graph node
///
//
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/MedX3D/ShadedVolumeStyle.h>
#include <H3D/MedX3D/VolumeGradient.h>

#include <H3D/ShaderPart.h>
#include <H3D/PixelTexture.h>
#include <H3D/Pixel3DTexture.h>
#include <H3D/TextureProperties.h>
#include <H3D/SFFloat.h>

namespace H3D {
  
  H3DNodeDatabase 
  ShadedVolumeStyle::database( "ShadedVolumeStyle", 
                               &(newInstance<ShadedVolumeStyle>),
                               typeid( ShadedVolumeStyle ),
                               &X3DComposableVolumeRenderStyleNode::database );
  
  namespace ShadedVolumeStyleInternals {
    FIELDDB_ELEMENT( ShadedVolumeStyle, material, INPUT_OUTPUT )
    FIELDDB_ELEMENT( ShadedVolumeStyle, phaseFunction, INPUT_OUTPUT )
    FIELDDB_ELEMENT( ShadedVolumeStyle, shadows, INPUT_OUTPUT )
    FIELDDB_ELEMENT( ShadedVolumeStyle, lighting, INPUT_OUTPUT )
    FIELDDB_ELEMENT( ShadedVolumeStyle, surfaceNormals, INPUT_OUTPUT )
    FIELDDB_ELEMENT( ShadedVolumeStyle, lightRayStepSize, INPUT_OUTPUT )
  }
  
  ShadedVolumeStyle::
  ShadedVolumeStyle( Inst< DisplayList >_displayList, 
                     Inst< SFBool > _enabled,
                     Inst< SFNode > _metadata,
                     Inst< SFMaterialNode > _material,
                     Inst< SFString > _phaseFunction,
                     Inst< SFBool > _shadows,
                     Inst< SFBool > _lighting,
                     Inst< SFTexture3DNode > _surfaceNormals,
                     Inst< FloatAboveValue > _lightRayStepSize ) :
    X3DComposableVolumeRenderStyleNode( _displayList, _enabled, _metadata ),
    material( _material ),    
    phaseFunction( _phaseFunction ),
    shadows( _shadows ),
    lighting( _lighting ),
    surfaceNormals( _surfaceNormals ),
    lightRayStepSize( _lightRayStepSize ),
    had_normals( false ),
    emissiveColor( new SFColorRGBA ),
    diffuseColor( new SFColorRGBA ),
    ambientColor( new SFColorRGBA ),
    specularColor( new SFColorRGBA ),
    shininess( new SFFloat ) {
    type_name = "ShadedVolumeStyle";
    database.initFields( this );
   
    // initialize internal fields
    emissiveColor->setOwner( this );
    diffuseColor->setOwner( this );
    ambientColor->setOwner( this );
    specularColor->setOwner( this );
    shininess->setOwner( this );

    emissiveColor->setName( "emissiveColor" );
    diffuseColor->setName( "diffuseColor" );
    ambientColor->setName( "ambientColor" );
    specularColor->setName( "specularColor" );
    shininess->setName( "shininess" );

    // defaults
    lighting->setValue( false );
    phaseFunction->setValue( "Henyey-Greenstein" );
    shadows->setValue( false );
    lightRayStepSize->setValue( (H3DFloat)0.03 );
    
    // routings
    material->route( displayList );
    phaseFunction->route( displayList );
    shadows->route( displayList );
    surfaceNormals->route( displayList );
    lightRayStepSize->route( displayList );

    shadows->route( rebuildShader );
  }
  
  string ShadedVolumeStyle::addUniforms( ComposedShader *s ) {
    string return_string = "";

    if( shadows->getValue() ) {
      // add the ray step to the shader    
      return_string +=addUniformToFragmentShader( s,
                                                  UNIFORM_ID(lightRayStepSize),
                                                  "float",
                                                  H3D::Field::INPUT_OUTPUT, 
                                                  copyAndRouteField( lightRayStepSize ) );
    }

    // add the fields to the shader
    return_string += addUniformToFragmentShader( s,
                                                 UNIFORM_ID(enabled),
                                                 "bool",
                                                 H3D::Field::INPUT_OUTPUT, 
                                                 copyAndRouteField(enabled) );
    return_string += addUniformToFragmentShader( s,
                                                 UNIFORM_ID(lighting),
                                                 "bool",
                                                 H3D::Field::INPUT_OUTPUT, 
                                                 copyAndRouteField(lighting) ) ;


    if( material->getValue() ) {
      // add material properties, update in updateUniformFields
      return_string += addUniformToFragmentShader( s,
                                                   UNIFORM_ID(emissiveColor), 
                                                   "vec4",
                                                   H3D::Field::INPUT_OUTPUT,
                                                   copyAndRouteField(emissiveColor ) );
      return_string += addUniformToFragmentShader( s,
                                                   UNIFORM_ID(ambientColor), 
                                                   "vec4",
                                                   H3D::Field::INPUT_OUTPUT,
                                                   copyAndRouteField(ambientColor ) );
      
      return_string += addUniformToFragmentShader( s,
                                                   UNIFORM_ID(diffuseColor), 
                                                   "vec4",
                                                   H3D::Field::INPUT_OUTPUT,
                                                   copyAndRouteField(diffuseColor) );
      
      return_string += addUniformToFragmentShader( s,
                                                   UNIFORM_ID(specularColor),
                                                   "vec4",
                                                   H3D::Field::INPUT_OUTPUT,
                                                   copyAndRouteField(specularColor) );
      
      return_string += addUniformToFragmentShader( s,
                                                   UNIFORM_ID(shininess), 
                                                   "float",
                                                   H3D::Field::INPUT_OUTPUT,
                                                   copyAndRouteField(shininess) );
    }
    /*addUniformToFragmentShader( s,
                                UNIFORM_ID(shadows),
                                "bool",
                                H3D::Field::INPUT_OUTPUT,
                                copyAndRouteField(shadows) ) +*/
    return_string += (surfaceNormals->getValue() ?
                      addUniformToFragmentShader( s,
                                                  UNIFORM_ID(surfaceNormals),
                                                  "sampler3D",
                                                  H3D::Field::INPUT_OUTPUT,
                                                  copyAndRouteField( surfaceNormals) ) : "");
    return return_string;
  }
  
  void ShadedVolumeStyle::updateUniformFields( X3DVolumeNode *vd ) {
    // rebuild shader if normals have been added or removed since
    // we need to add/remove the uniform surfaceNormals then
    bool have_normals = surfaceNormals->getValue() != NULL;
    if( have_normals != had_normals ) rebuildShader->touch();
    had_normals = have_normals;
 
    // render material to make it available in shader
    X3DMaterialNode *m = material->getValue();
    if ( m ) {
      m->displayList->callList();
      GLfloat c[4];
      glGetMaterialfv( GL_FRONT, GL_EMISSION, c );
      emissiveColor->setValue( RGBA( c[0], c[1], c[2], c[3] ) );
      glGetMaterialfv( GL_FRONT, GL_DIFFUSE, c );
      diffuseColor->setValue( RGBA( c[0], c[1], c[2], c[3] ) );
      glGetMaterialfv( GL_FRONT, GL_AMBIENT, c );
      ambientColor->setValue( RGBA( c[0], c[1], c[2], c[3] ) );
      glGetMaterialfv( GL_FRONT, GL_SPECULAR, c );
      specularColor->setValue( RGBA( c[0], c[1], c[2], c[3] ) );
      glGetMaterialfv( GL_FRONT, GL_SHININESS, c );
      shininess->setValue( c[0] );
    }

    X3DComposableVolumeRenderStyleNode::updateUniformFields( vd );
  }
  
  string ShadedVolumeStyle::getShaderCode( ) {
    X3DMaterialNode *m = material->getValue();

    if( shadows->getValue() ) {
      return 
        string( "ShadedVolumeStyleWithShadows(" ) +
        string( "sample_color, " ) +
        (m ? UNIFORM_ID_(emissiveColor) : "vec4(0,0,0,0),") +
        (m ? UNIFORM_ID_(diffuseColor) : "vec4(sample_color.rgb, 1 ),")   +
        (m ? UNIFORM_ID_(ambientColor) : "vec4(0,0,0,0),")  +
        (m ? UNIFORM_ID_(specularColor): "vec4(0,0,0,0),")  +
        (m ? UNIFORM_ID_(shininess): "0,") +
        string( "r0, " ) +
        string( "viewdir_tex, " ) + 
        string( "view_to_tex, " ) + 
        UNIFORM_ID_(enabled) +
        UNIFORM_ID_(lighting) +
        (surfaceNormals->getValue() ? 
         "normalizedNormalFromTexture( " + UNIFORM_ID(surfaceNormals) + ", r0 )," :
         "sample_default_normal," )+
        UNIFORM_ID(lightRayStepSize) +
        ");";
    } else {
      return  
        string( "ShadedVolumeStyle(" ) +
        string( "sample_color, " ) +
        (m ? UNIFORM_ID_(emissiveColor) : "vec4(0,0,0,0),") +
        (m ? UNIFORM_ID_(diffuseColor) : "vec4(sample_color.rgb, 1),")+
        (m ? UNIFORM_ID_(ambientColor) : "vec4(0,0,0,0),")+
        (m ? UNIFORM_ID_(specularColor): "vec4(0,0,0,0),") +
        (m ? UNIFORM_ID_(shininess): "0,") +
        string( "r0, " ) +
        string( "viewdir_tex, " ) + 
        string( "view_to_tex, " ) + 
        UNIFORM_ID_(enabled) +
        UNIFORM_ID_(lighting) +
        (surfaceNormals->getValue() ? 
         "normalizedNormalFromTexture( " + UNIFORM_ID(surfaceNormals) + ", r0 )" :
         "sample_default_normal" )+
        ");";
    }
  }

  string ShadedVolumeStyle::getShaderCodeOpacityOnly( ) {
    X3DMaterialNode *m = material->getValue();

    stringstream s;
    if( 1 || diffuseColor->getValue().a != 1 ) {
      s << "if( sample_default_normal.a > 0.001 ) { \n";
      if( m ) {
        s << "sample_color.a *= " 
          << UNIFORM_ID(diffuseColor) 
          << ".a; " << endl;
      }
      s << "} else { sample_color.a = 0.0; } \n";
    }
    return s.str();
  }

  void ShadedVolumeStyle::SFMaterialNode::onAdd( Node *n) {
    SFMaterialNodeBase::onAdd( n );
    ShadedVolumeStyle *vd = static_cast< ShadedVolumeStyle * >( getOwner() );
    vd->rebuildShader->touch();    
  }
  
  void ShadedVolumeStyle::SFMaterialNode::onRemove( Node *n) {
    ShadedVolumeStyle *vd = static_cast< ShadedVolumeStyle * >( getOwner() );
    vd->rebuildShader->touch();
    SFMaterialNodeBase::onRemove( n );
  }

}





