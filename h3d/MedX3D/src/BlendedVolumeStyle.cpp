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
/// \file BlendedVolumeStyle.cpp
/// \brief CPP file for BlendedVolumeStyle, X3D scene-graph node
///
//
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/MedX3D/BlendedVolumeStyle.h>
#include <H3D/MedX3D/VolumeGradient.h>

#include <H3D/ShaderPart.h>
#include <H3D/PixelTexture.h>
#include <H3D/Pixel3DTexture.h>
#include <H3D/TextureProperties.h>
 
using namespace H3D;
  
H3DNodeDatabase 
BlendedVolumeStyle::database( 
                             "BlendedVolumeStyle", 
                             &(newInstance<BlendedVolumeStyle>),
                             typeid( BlendedVolumeStyle ),
                             &X3DComposableVolumeRenderStyleNode::database );
  
namespace BlendedVolumeStyleInternals {
  FIELDDB_ELEMENT( BlendedVolumeStyle, voxels, INPUT_OUTPUT )
  FIELDDB_ELEMENT( BlendedVolumeStyle, renderStyle, INPUT_OUTPUT )
  FIELDDB_ELEMENT( BlendedVolumeStyle, weightConstant1, INPUT_OUTPUT )
  FIELDDB_ELEMENT( BlendedVolumeStyle, weightConstant2, INPUT_OUTPUT )
  FIELDDB_ELEMENT( BlendedVolumeStyle, weightFunction1, INPUT_OUTPUT )
  FIELDDB_ELEMENT( BlendedVolumeStyle, weightFunction2, INPUT_OUTPUT )
  FIELDDB_ELEMENT( BlendedVolumeStyle, weightTransferFunction1, INPUT_OUTPUT )
  FIELDDB_ELEMENT( BlendedVolumeStyle, weightTransferFunction2, INPUT_OUTPUT )
}
  
BlendedVolumeStyle::
BlendedVolumeStyle( Inst<DisplayList>_displayList, 
                    Inst<SFBool> _enabled,
                    Inst<SFNode> _metadata,
                    Inst< SFTexture3DNode > _voxels,
                    Inst< SurfaceNormals > _surfaceNormals,
                    Inst< SFComposableVolumeRenderStyleNode > _renderStyle,
                    Inst< SFFloat >         _weightConstant1,
                    Inst< SFFloat >         _weightConstant2,
                    Inst< SFString >        _weightFunction1,
                    Inst< SFString >        _weightFunction2,
                    Inst< SFTexture2DNode > _weightTransferFunction1,
                    Inst< SFTexture2DNode > _weightTransferFunction2 ):
  X3DComposableVolumeRenderStyleNode( _displayList, _enabled, _metadata ),
  voxels( _voxels ),
  surfaceNormals( _surfaceNormals ),
  renderStyle( _renderStyle ),
  weightConstant1( _weightConstant1 ),
  weightConstant2( _weightConstant2 ),
  weightFunction1( _weightFunction1 ),
  weightFunction2( _weightFunction2 ),
  weightTransferFunction1( _weightTransferFunction1 ),
  weightTransferFunction2( _weightTransferFunction2 ),
  surfaceNormals_glsl( new SFTexture3DNode )
{
  type_name = "BlendedVolumeStyle";
  database.initFields( this );
    
  // defaults
  weightConstant1->setValue( 0.5 );
  weightConstant2->setValue( 0.5 );

  weightFunction1->addValidValue( "CONSTANT" );
  weightFunction1->addValidValue( "TABLE" );
  weightFunction1->addValidValue( "ALPHA1" );
  weightFunction1->addValidValue( "ALPHA2" );
  weightFunction1->addValidValue( "ONE_MINUS_ALPHA1" );
  weightFunction1->addValidValue( "ONE_MINUS_ALPHA2" );

  weightFunction2->addValidValue( "CONSTANT" );
  weightFunction2->addValidValue( "TABLE" );
  weightFunction2->addValidValue( "ALPHA1" );
  weightFunction2->addValidValue( "ALPHA2" );
  weightFunction2->addValidValue( "ONE_MINUS_ALPHA1" );
  weightFunction2->addValidValue( "ONE_MINUS_ALPHA2" );

  weightFunction1->setValue( "CONSTANT" );
  weightFunction2->setValue( "CONSTANT" );
    
  // routings
  voxels->route( displayList );
  renderStyle->route( displayList );
  weightConstant1->route( displayList );
  weightConstant2->route( displayList );
  weightFunction1->route( displayList );
  weightFunction2->route( displayList );
  weightTransferFunction1->route( displayList );
  weightTransferFunction2->route( displayList );
  voxels->route( surfaceNormals );
  surfaceNormals->route( surfaceNormals_glsl );

  weightFunction1->route( rebuildShader );
  weightFunction2->route( rebuildShader );
  voxels->route( rebuildShader );
}
  
void BlendedVolumeStyle::
updateUniformFields( X3DVolumeNode *vd ) {
  //    // rebuild shader if normals have been added or removed since
  //    // we need to add/remove the uniform surfaceNormals then
  //    bool have_normals = surfaceNormals->getValue() != NULL;
  //    if( have_normals != had_normals ) rebuildShader->touch();
  //    had_normals = have_normals;
  //    
  X3DComposableVolumeRenderStyleNode *style = renderStyle->getValue();
  if( style ) style->updateUniformFields( vd );
  X3DComposableVolumeRenderStyleNode::updateUniformFields( vd );
}

string BlendedVolumeStyle::getShaderInitCode() {
  X3DComposableVolumeRenderStyleNode *style = renderStyle->getValue();
  if( style ) return style->getShaderInitCode();
  else return "";
}
  
string BlendedVolumeStyle::addUniforms( ComposedShader *s ) {
  stringstream os;

  X3DComposableVolumeRenderStyleNode *style = renderStyle->getValue();
  if( style ) os << style->addUniforms( s );

  // add normals if required
  if( style && style->requiresDefaultNormals() ) {
    os << addUniformToFragmentShader( s,
                                      UNIFORM_ID(defaultNormals), 
                                      "sampler3D",
                                      H3D::Field::INPUT_OUTPUT,
                                      copyAndRouteField( surfaceNormals_glsl ) );
  } 

  os << addUniformToFragmentShader( s,
                                    UNIFORM_ID(voxels), 
                                    "sampler3D",
                                    H3D::Field::INPUT_OUTPUT,
                                    copyAndRouteField( voxels ) );

  os << addUniformToFragmentShader( s,
                                    UNIFORM_ID(enabled),
                                    "bool",
                                    H3D::Field::INPUT_OUTPUT, 
                                    copyAndRouteField(enabled) );
  const string &f1 = weightFunction1->getValue();
  if( f1 == "CONSTANT" ) {
    os <<  addUniformToFragmentShader( s,
                                       UNIFORM_ID(weightConstant1),
                                       "float",
                                       H3D::Field::INPUT_OUTPUT,
                                       copyAndRouteField(weightConstant1) );
  } else if( f1 == "TABLE" ) {
    os << addUniformToFragmentShader( s,
                                      UNIFORM_ID(weightTransferFunction1), 
                                      "sampler2D",
                                      H3D::Field::INPUT_OUTPUT,
                                      copyAndRouteField( weightTransferFunction1 ) );
  } else if( f1 != "ALPHA1" && f1 != "ALPHA2" &&
             f1 != "ONE_MINUS_ALPHA1" && f1 != "ONE_MINUS_ALPHA2" ) {
    Console(4) << "Warning: Invalid value for weightFunction1 field: \""
               << f1 << "\"(in BlendedVolumeStyle)." << endl;
  }
  

  const string &f2 = weightFunction2->getValue();
  if( f2 == "CONSTANT" ) {
    os <<  addUniformToFragmentShader( s,
                                       UNIFORM_ID(weightConstant2),
                                       "float",
                                       H3D::Field::INPUT_OUTPUT,
                                       copyAndRouteField(weightConstant2) );
  } else if( f2 == "TABLE" ) {
    os << addUniformToFragmentShader( s,
                                      UNIFORM_ID(weightTransferFunction2), 
                                      "sampler2D",
                                      H3D::Field::INPUT_OUTPUT,
                                      copyAndRouteField( weightTransferFunction2 ) );
  } else if( f2 != "ALPHA1" && f2 != "ALPHA2" &&
             f2 != "ONE_MINUS_ALPHA1" && f2 != "ONE_MINUS_ALPHA2" ) {
    Console(4) << "Warning: Invalid value for weightFunction1 field: \""
               << f2 << "\"(in BlendedVolumeStyle)." << endl;
  }

  return os.str();
}
  
string BlendedVolumeStyle::getShaderCode() {
  X3DTexture3DNode *tex = voxels->getValue();
  if( !tex ) return "";

  X3DComposableVolumeRenderStyleNode *style = renderStyle->getValue();
  stringstream s;

  s << "if( " << UNIFORM_ID( enabled ) << ") { " << endl;

  s << "vec4 blend_sample_color = texture3D( " << UNIFORM_ID( voxels ) 
    << ", r0.xyz );" << endl;

  Image *image = tex ? tex->image->getValue() : NULL;
  if( image ) {
    Image::PixelType type = image->pixelType();
    if( type == Image::LUMINANCE ) {  
      s << "blend_sample_color.a = blend_sample_color.r;" << endl;
    }
  }

  if( style ) {
    s << "{" << endl;
    s << "   vec4 orig_sample_color = blend_sample_color; " << endl;
    s << "   vec4 sample_color = orig_sample_color; " << endl;
    if( style->requiresDefaultNormals() ) {
      s << "vec4 sample_default_normal = normalizedNormalFromTexture( " 
        << UNIFORM_ID(defaultNormals) << ", r0 );" << endl;
    }
    s << style->getShaderCode() << endl;
    s << "   blend_sample_color = sample_color; " << endl;
    s << "}" << endl;
  }

  const string &f1 = weightFunction1->getValue();
  if( f1 == "CONSTANT" ) {
    s << "float w1 = " << UNIFORM_ID( weightConstant1 ) << ";" << endl;
  } else if( f1 == "TABLE" ) {
    s << "float w1 = texture2D( " << UNIFORM_ID( weightTransferFunction1 ) 
      << ", vec2( sample_color.a, blend_sample_color.a ) );" << endl;
  } else if( f1 == "ALPHA1" ) {
    s << "float w1 = sample_color.a;" << endl;
  } else if( f1 == "ALPHA2" ) {
    s << "float w1 = blend_sample_color.a;" << endl;
  } else if( f1 == "ONE_MINUS_ALPHA1" ) {
    s << "float w1 = 1.0 - sample_color.a;" << endl;
  } else if( f1 == "ONE_MINUS_ALPHA2" ) {
    s << "float w1 = 1.0 - blend_sample_color.a;" << endl;
  }

  const string &f2 = weightFunction2->getValue();
  if( f2 == "CONSTANT" ) {
    s << "float w2 = " << UNIFORM_ID( weightConstant2 ) << ";" << endl;
  } else if( f2 == "TABLE" ) {
    s << "float w2 = texture2D( " << UNIFORM_ID( weightTransferFunction2 ) 
      << ", vec2( blend_sample_color.a, sample_color.a ) );" << endl;
  } else if( f2 == "ALPHA1" ) {
    s << "float w2 = sample_color.a;" << endl;
  } else if( f2 == "ALPHA2" ) {
    s << "float w2 = blend_sample_color.a;" << endl;
  } else if( f2 == "ONE_MINUS_ALPHA1" ) {
    s << "float w2 = 1.0 - sample_color.a;" << endl;
  } else if( f2 == "ONE_MINUS_ALPHA2" ) {
    s << "float w2 = 1.0 - blend_sample_color.a;" << endl;
  }

  s << "sample_color = w1 * sample_color + w2 * blend_sample_color;";
  s << "sample_color = clamp( sample_color, 0.0, 1.0 );";

  s << "}" << endl;

  return s.str();
}

void BlendedVolumeStyle::SFComposableVolumeRenderStyleNode::onAdd( Node *n) {
  SFComposableVolumeRenderStyleNodeBase::onAdd( n );
  X3DComposableVolumeRenderStyleNode *cs = 
    dynamic_cast< X3DComposableVolumeRenderStyleNode * >( n );
  BlendedVolumeStyle *vd = 
    static_cast< BlendedVolumeStyle * >( getOwner() );
  cs->rebuildShader->route( vd->rebuildShader );
}

void BlendedVolumeStyle::SFComposableVolumeRenderStyleNode::onRemove( Node *n) {
  X3DComposableVolumeRenderStyleNode *cs = 
    dynamic_cast< X3DComposableVolumeRenderStyleNode * >( n );
  BlendedVolumeStyle *vd = 
    static_cast< BlendedVolumeStyle * >( getOwner() );
  cs->rebuildShader->unroute( vd->rebuildShader );
  SFComposableVolumeRenderStyleNodeBase::onRemove( n );
}

bool BlendedVolumeStyle::requiresEnabledLights() {
  X3DComposableVolumeRenderStyleNode *style = 
    renderStyle->getValue();

  if( style ) return style->requiresEnabledLights();
  else return false;
}

bool BlendedVolumeStyle::isEmptySpace( H3DFloat min_v, 
                                       H3DFloat max_v, 
                                       bool previous_empty ) {
  X3DComposableVolumeRenderStyleNode *style = 
    renderStyle->getValue();

  if( style ) 
    return previous_empty && style->isEmptySpace( min_v, max_v, previous_empty );
  else return previous_empty;
}



