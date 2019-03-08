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
/// \file OpacityMapVolumeStyle.cpp
/// \brief CPP file for OpacityMapVolumeStyle, X3D scene-graph node
///
//
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/MedX3D/OpacityMapVolumeStyle.h>

#include <H3D/ShaderPart.h>
#include <H3D/PixelTexture.h>
#include <H3D/TextureProperties.h>
#include <H3D/PixelTexture.h>
#include <H3D/MedX3D/Integrator.h>

using namespace H3D;
  
  // Generates the default transfer function tf(x) = x
  void OpacityMapVolumeStyle::DefaultTransferFunction::update() {
    unsigned char data[256][4];
    for(int i=0; i<256; ++i) 
      data[i][0] = data[i][1] = data[i][2] = data[i][3] = i;
    PixelTexture *pt = new PixelTexture;
    pt->image->setValue( new PixelImage( 256,1,1,
                                         32,
                                         Image::RGBA,
                                         Image::UNSIGNED,
                                         &data[0][0], true) );
    TextureProperties *tp = new TextureProperties;
    tp->boundaryModeS->setValue("CLAMP_TO_EDGE");
    tp->boundaryModeT->setValue("CLAMP_TO_EDGE");
    tp->boundaryModeR->setValue("CLAMP_TO_EDGE");
    tp->minificationFilter->setValue("AVG_PIXEL");
    tp->magnificationFilter->setValue("AVG_PIXEL");
    tp->textureCompression->setValue("DEFAULT");
    pt->textureProperties->setValue(tp);
    
    value = pt;
  }
  
  H3DNodeDatabase OpacityMapVolumeStyle::database(
    "OpacityMapVolumeStyle",
    &(newInstance<OpacityMapVolumeStyle>),
    typeid( OpacityMapVolumeStyle ),
    &X3DComposableVolumeRenderStyleNode::database );
  
  namespace OpacityMapVolumeStyleInternals {
    FIELDDB_ELEMENT( OpacityMapVolumeStyle, transferFunction, INPUT_OUTPUT )
    FIELDDB_ELEMENT( OpacityMapVolumeStyle, type, INPUT_OUTPUT )

    FIELDDB_ELEMENT( OpacityMapVolumeStyle, preintegratedTransferFunction, OUTPUT_ONLY )
    FIELDDB_ELEMENT( OpacityMapVolumeStyle, preintegratedIntegralStep, INPUT_OUTPUT )
  }
  
  OpacityMapVolumeStyle::
  OpacityMapVolumeStyle( Inst< DisplayList >_displayList, 
                         Inst< SFBool > _enabled,
                         Inst< SFNode > _metadata,
                         Inst< SFTextureNode > _transferFunction,
                         Inst< SFString >        _type,
                         Inst< SFPreIntegratedTextureNode > _preintegratedTransferFunction,
                         Inst< SFFloat >         _preintegratedIntegralStep) :
    X3DComposableVolumeRenderStyleNode( _displayList, _enabled, _metadata ),
    transferFunction( _transferFunction ),
    type( _type ),
    preintegratedTransferFunction( _preintegratedTransferFunction ),
    preintegratedIntegralStep( _preintegratedIntegralStep ),
    transferFunction_glsl( new SFTextureNode ),
    defaultTransferFunction( new DefaultTransferFunction ),
    nr_components_in_data( 0 ),
    transfer_function_is_2d( true ),
    stepSize( new SFFloat ),
    last_loop_tf( NULL )
  {
    
    type_name = "OpacityMapVolumeStyle";
    database.initFields( this );
    
    // ownership
    displayList->setOwner(this);
    stepSize->setName( "stepSize" );
    stepSize->setOwner( this );

    // default values
    type->addValidValue( "simple" );
    type->addValidValue( "preintegrated" );
    type->addValidValue( "preintegrated_fast" );
    type->setValue( "simple" );

    preintegratedIntegralStep->setValue( (H3DFloat) 0.001 );
    stepSize->setValue( (H3DFloat) 0.01 );

    // routings
    transferFunction->route( displayList );

    transferFunction_glsl->route( preintegratedTransferFunction, id );
    preintegratedIntegralStep->route( preintegratedTransferFunction, id );
    stepSize->route( preintegratedTransferFunction, id );
    type->route( preintegratedTransferFunction, id );

    type->route( rebuildShader );
    enabled->route( rebuildShader );
  }
  
  void OpacityMapVolumeStyle::updateUniformFields( X3DVolumeNode *vd ) {
    // check if we should use the default transfer function
    X3DTextureNode *tex = NULL;
    if( transferFunction->getValue()==0 ) {
      if( defaultTransferFunction->getValue()==0 ) {
        defaultTransferFunction->update();
      }
      tex = defaultTransferFunction->getValue();
    }
    else {
      tex = transferFunction->getValue();
    }
    
    if( tex != transferFunction_glsl->getValue() )
      transferFunction_glsl->setValue( tex );

    bool was_2d = transfer_function_is_2d;
    X3DTexture2DNode *tft = 
      dynamic_cast< X3DTexture2DNode *>(transferFunction_glsl->getValue() );
    transfer_function_is_2d = (tft != NULL);

    X3DTexture3DNode *data = vd->voxels->getValue();
    unsigned int last_loop_components = nr_components_in_data;
    nr_components_in_data = 0;
    if( data ) {
      Image *image = data->image->getValue();
      if( image ) {
        Image::PixelType _type = image->pixelType();
        if( _type == Image::LUMINANCE ) nr_components_in_data = 1;
        else if( _type == Image::LUMINANCE_ALPHA ) nr_components_in_data = 2;
        else if( _type == Image::RGBA ) nr_components_in_data = 4;
        else nr_components_in_data = 3;
      } 
    }

    if( was_2d != transfer_function_is_2d ||
        last_loop_components != nr_components_in_data ) {
      rebuildShader->touch();
    }

    X3DComposableVolumeRenderStyleNode::updateUniformFields( vd );    
  }
  
  string OpacityMapVolumeStyle::addUniforms( ComposedShader *s ) {
    X3DTextureNode *t = transferFunction->getValue();

    string uniform_string = "";

    const string &ts = type->getValue();
    bool is_preintegrated = ts == "preintegrated" || ts == "preintegrated_fast";


    // add the texture of preintegrated values if needed
    if( is_preintegrated ) {
       SFNode* f = new TypedSFNode< X3DTexture2DNode >();
       preintegratedTransferFunction->route( f );
       uniform_string += addUniformToFragmentShader( s,
                                                     UNIFORM_ID(preintegratedTransferFunction),
                                                     "sampler2D",
                                                     H3D::Field::INPUT_OUTPUT,
                                                     f );
    }

    // the enabled field
    if( t || is_preintegrated ) {
      uniform_string += addUniformToFragmentShader( s,
                                                    UNIFORM_ID(enabled),
                                                    "bool",
                                                    H3D::Field::INPUT_OUTPUT, 
                                                    copyAndRouteField(enabled) );
    }
     
    // the transferFunction field
    if( t && !is_preintegrated ) {
      string sampler_string = 
        dynamic_cast< X3DTexture2DNode * >( t ) ? "sampler2D" : "sampler3D";

 
    
      // add the transfer function to the shader
      uniform_string += addUniformToFragmentShader( s,
                                                    UNIFORM_ID(transferFunction),
                                                    sampler_string,
                                                    H3D::Field::INPUT_OUTPUT,
                                                    copyAndRouteField( transferFunction_glsl ) );
    }

    return uniform_string;
  }

  
  string OpacityMapVolumeStyle::getShaderCode() {
    X3DTextureNode *t = transferFunction->getValue();

    const string &ts = type->getValue();
    // add the texture of preintegrated values if needed
    if( ts == "preintegrated" || ts == "preintegrated_fast" ) {
      return string( "OpacityMapVolumeStylePreIntegrated(" ) +
             string( "sample_color, " ) +
             string( "orig_sample_color, " ) +
             string( "r0, " ) +
             UNIFORM_ID_(enabled) +
             UNIFORM_ID_(last_orig_sample_color) +
             UNIFORM_ID(preintegratedTransferFunction) + ");\n" +
             UNIFORM_ID(last_orig_sample_color) +
             " = orig_sample_color;\n";
    }



    if( !t ) return "";

    stringstream components;
    components << nr_components_in_data << ", ";

    if( dynamic_cast< X3DTexture2DNode * >( t ) ) {
      return string( "OpacityMapVolumeStyle2D(" ) +
        string( "sample_color, " ) +
        string( "orig_sample_color, " ) +
        string( "r0, " ) +
        UNIFORM_ID_(enabled) +
        string( components.str() ) +
        UNIFORM_ID(transferFunction) + ");";
    } else {
      return string( "OpacityMapVolumeStyle3D(" ) +
             string( "sample_color, " ) +
             string( "orig_sample_color, " ) +
             string( "r0, " ) +
             UNIFORM_ID_(enabled) +
             string( components.str() ) +
             UNIFORM_ID(transferFunction) + ");";
    }
  }
 
  
  void OpacityMapVolumeStyle::SFPreIntegratedTextureNode::update() {
    OpacityMapVolumeStyle *style = static_cast< OpacityMapVolumeStyle * >( getOwner() );

    X3DTextureNode *t = static_cast< SFTextureNode * >( routes_in[0] )->getValue();
    H3DFloat integral_step_size = static_cast< SFFloat * >( routes_in[1] )->getValue();
    H3DFloat step_size = static_cast< SFFloat * >( routes_in[2] )->getValue();
    string _type = static_cast< SFString * >( routes_in[3] )->getValue();

    if( !value.get() ) {
      PixelTexture *new_pt = new PixelTexture;
      TextureProperties *tp = new TextureProperties;
      tp->boundaryModeS->setValue( "CLAMP_TO_EDGE" );
      tp->boundaryModeT->setValue( "CLAMP_TO_EDGE" );
      tp->boundaryModeR->setValue( "CLAMP_TO_EDGE" );
      tp->magnificationFilter->setValue("AVG_PIXEL");
      tp->minificationFilter->setValue("AVG_PIXEL");
      tp->textureCompression->setValue( "DEFAULT" );
      new_pt->textureProperties->setValue( tp );
      value = new_pt;
    }
    
    PixelTexture *pt = static_cast<PixelTexture *>(value.get());
    PixelImage *piOut = 
      dynamic_cast< PixelImage * >( pt->image->getValue() );


    TimeStamp start;
    Console(4) << "Calculating pre-integrated values.. ";

    Image *transfer_function_image = NULL;

    if( X3DTexture2DNode *t2d = dynamic_cast< X3DTexture2DNode * >( t ) ) {
      transfer_function_image = t2d->image->getValue();
    } else if( X3DTexture3DNode *t3d = dynamic_cast< X3DTexture3DNode * >( t ) ) {
      transfer_function_image = t3d->image->getValue();
    }

    unsigned int dim;
    if ( transfer_function_image )
      dim = transfer_function_image->width();
    else
      dim = 256;

    // if wrong dimensions create image with correct dimensions.
    if( !piOut  || piOut->width() != dim || piOut->height() != dim || piOut->depth() != 1 ) {
      unsigned char *data = new unsigned char[ dim*dim*4 ];
      piOut = new PixelImage( dim, dim, 1, 32, Image::RGBA, Image::UNSIGNED, data );
      pt->image->setValue( piOut );
    }

    if ( transfer_function_image ) {
      if ( _type == "preintegrated" ) {
        createPreIntegrationTable2( transfer_function_image,
                                    dim,
                                    1.0,
                                    piOut );
      }
      else if ( _type == "preintegrated_fast" ) {
        createPreIntegrationTableFast( transfer_function_image,
                                       dim,
                                       1.0,
                                       piOut );
      }
    } else {
      // No lookup table exist
      createPreIntegrationTable2( NULL,
                                  dim,
                                  1.0,
                                  piOut );
    }

    pt->image->touch();
    Console(4) << "done! " << TimeStamp() - start << endl;

}


  void OpacityMapVolumeStyle::traverseSG( TraverseInfo &ti ) {
    X3DVolumeNode *vn = NULL;
    ti.getUserData( "VolumeNode", (void * *)&vn ); 
    if( vn ) {
      H3DFloat step_size = vn->rayStep->getValue();
      if( step_size != stepSize->getValue() ){
        stepSize->setValue( step_size );
      }
    }
    
    // we want the shader to be rebuild when we change the actual texture
    // node and not when it is being updated internally. Because of this
    // we cannot route transferFunction directly to rebuildShader. Instead
    // we excplitly check here if the node has changed and rebuild the shader
    // if it has.
    Node *tf = transferFunction->getValue();
    if( tf != last_loop_tf ) rebuildShader->touch();
    last_loop_tf = tf;
  }

 Image *OpacityMapVolumeStyle::SFPreIntegratedTextureNode::createPreIntegrationTableFast( Image *transfer_function,
                                                                                          H3DInt32 input_dim,
                                                                                          H3DFloat d,
                                                                                          Image *output  ) {

    if( !output ) {
      unsigned char *data = new unsigned char[input_dim * input_dim*4 ];
      output = new PixelImage( input_dim, input_dim, 
                               1, 32, Image::RGBA, Image::UNSIGNED, data );
    }
    
    RGBA col0 = transfer_function->getPixel( 0 );
    RGBA col1;

    H3DFloat r = 0, g = 0, b = 0, a = 0;
    H3DFloat *r_int = new H3DFloat[input_dim];
    H3DFloat *g_int = new H3DFloat[input_dim];
    H3DFloat *b_int = new H3DFloat[input_dim];
    H3DFloat *a_int = new H3DFloat[input_dim];
    r_int[0] = 0; g_int[0] = 0; b_int[0] = 0; a_int[0] = 0;
    
    // compute integral functions
    for( H3DInt32 i = 1; i < input_dim; ++i ) {
      col1 = transfer_function->getPixel( i );
      H3DFloat tauc = (col0.a + col1.a) / 2;
      r += tauc * (col0.r + col1.r ) / 2;

      g += tauc * (col0.g + col1.g ) / 2;
      b += tauc * (col0.b + col1.b ) / 2;
      a +=  tauc;
      col0 = col1;
      r_int[i] = r; g_int[i] = g; b_int[i] = b; a_int[i] = a;
    }

    // compute table
    H3DInt32 smin, smax;
    for( H3DInt32 sb = 0; sb < input_dim; ++sb ) {
      for( H3DInt32 sf = 0; sf < input_dim; ++sf ) {
        if( sb < sf ) {
          smin = sb; smax = sf;
        } else {
          smin = sf; smax = sb;
        }

        RGBA c;        
        if( smax != smin ) {
          H3DFloat factor = d / (smax - smin );
          c.r = (r_int[smax] - r_int[smin]) * factor;
          c.g = (g_int[smax] - g_int[smin]) * factor;
          c.b = (b_int[smax] - b_int[smin]) * factor;
          c.a = 1 - H3DExp( -(a_int[smax] - a_int[smin] )*factor ); 
        } else {
          H3DFloat factor = d;
          RGBA tc = transfer_function->getPixel( smin );
          c.r = tc.r * tc.a * factor;
          c.g = tc.g * tc.a * factor;
          c.b = tc.b * tc.a * factor;
          c.a = 1 - H3DExp( -tc.a *factor ); 
        }
        
        // clamp to [0,1]
        if( c.r < 0 ) c.r = 0;
        else if ( c.r > 1 ) c.r = 1;
        if( c.g < 0 ) c.g = 0;
        else if ( c.g > 1 ) c.g = 1;
        if( c.b < 0 ) c.b = 0;
        else if ( c.b > 1 ) c.b = 1;
        if( c.a < 0 ) c.a = 0;
        else if ( c.a > 1 ) c.a = 1;

        output->setPixel( c, sf, sb );
      }
    }
    delete [] r_int;
    delete [] g_int;
    delete [] b_int;
    delete [] a_int;
    return output;
  }

  Image *OpacityMapVolumeStyle::SFPreIntegratedTextureNode::createPreIntegrationTable2( Image *transfer_function,
                                                                                        H3DInt32 input_dim,
                                                                                        H3DFloat d,
                                                                                        Image *output  )
  {

    H3DFloat step = (H3DFloat) 1.0 / input_dim; 

    if( !output ) {

        unsigned char *data_buffer = new unsigned char[input_dim * input_dim*4 ];
        output = new PixelImage( input_dim, input_dim, 
                               1, 32, Image::RGBA, Image::UNSIGNED, data_buffer );
    }

    FunctionData data;

    data.image = transfer_function;

    for( int y = 0; y < input_dim; ++y ) {
        for( int x = 0; x < input_dim; ++x ) {
            data.sf = step / 2 + x*step;
            data.sb = step / 2 + y*step;
            data.d  = d;

            RGBA color;
            if (data.image)
            {
                color = calculatePreIntegratedColor( data );
            }
            else
            {
                H3DFloat fMean = (data.sf + data.sb)/2.0f;
                color.a = (H3DFloat) 1.0 - H3DExp(-fMean);
                
                color.r = color.g = color.b = fMean*color.a;
            }

            output->setPixel( color, x, y ); 
        }
    }

    return output;
  }

  RGBA OpacityMapVolumeStyle::SFPreIntegratedTextureNode::calculatePreIntegratedColor( FunctionData &data ) {
    H3DFloat v;
    RGBA color;

    // calculate alpha according to equation 5 in preintegrated volume rendering paper
    v = CIntegrator<H3DFloat, H3DFloat>::RiemannMean( OpacityMapVolumeStyle::SFPreIntegratedTextureNode::textureLookupFunctionAlpha,
                                             0,
                                             1,
                                             10,
                                             &data );

    // Calculate alpha
    v = 1 - H3DExp( -v );
    if( v > 1 ) v = 1;
    if( v < 0 ) v = 0;
    color.a = v;

    RGB c;

    // Calculate color according to  equation 6 in preintegrated volume rendering paper
    c = CIntegrator<H3DFloat, RGB >::RiemannMean( OpacityMapVolumeStyle::SFPreIntegratedTextureNode::textureLookupFunctionRGB,
                                            0,
                                            1,
                                            10,
                                            &data );

    return RGBA( c.r,c.g,c.b, color.a);
  }

  H3DFloat OpacityMapVolumeStyle::SFPreIntegratedTextureNode::textureLookupFunctionAlpha( H3DFloat w, void *d ) {
    FunctionData *data = static_cast< FunctionData * >( d );

    H3DFloat s = (1-w) * data->sf + w * data->sb;
    H3DFloat alpha = data->image->getSample( s, 0.5, 0.5 ).a;
    return alpha;
  } 

  RGB OpacityMapVolumeStyle::SFPreIntegratedTextureNode::textureLookupFunctionRGB( H3DFloat w, 
                                                                                   void *d ) {
    FunctionData *data = static_cast< FunctionData * >( d );

    H3DFloat s = (1-w) * data->sf + w * data->sb;
    RGBA rgba = data->image->getSample( s, 0.5, 0.5 );
    RGB rgb( rgba.r, rgba.g, rgba.b );

    H3DFloat extinction_factor = 1.0;
    H3DFloat mean;
    if ( w > 0.0 )
    {
        mean = CIntegrator<H3DFloat, H3DFloat>::RiemannMean( OpacityMapVolumeStyle::SFPreIntegratedTextureNode::textureLookupFunctionAlpha,
            0, w, 7, d );

        extinction_factor = H3DExp(-mean*w);
    }

    H3DFloat alpha = textureLookupFunctionAlpha(w,d); 

    return rgb * alpha * extinction_factor;
  }




bool OpacityMapVolumeStyle::isEmptySpace( H3DFloat min_v, 
                                          H3DFloat max_v, 
                                          bool previous_empty ) {
  X3DTextureNode *t = transferFunction->getValue();
  Image *image = NULL;
  if( X3DTexture2DNode *t2d = dynamic_cast< X3DTexture2DNode * >( t ) ) {
    image = t2d->image->getValue();
  } else if( X3DTexture3DNode *t3d = dynamic_cast< X3DTexture3DNode * >( t ) ) {
    image = t3d->image->getValue();
  }

  // no transfer function image, ignore this stage
  if( !image ) return previous_empty;

  unsigned int low =  (unsigned int) H3DFloor( min_v * (image->width() - 1) );
  unsigned int high = (unsigned int) H3DCeil( max_v * (image->width() - 1)) ;

  bool v = true;

  // go through every pixel of the transfer function in the range
  // and see if alpha is 0 somewhere in the range
  for( unsigned int i = low; i <= high; ++i ) {
    if( image->getPixel( i, 0, 0 ).a > 0 ) {
      v = false;
      break;
    }
  }

  return v;
}
