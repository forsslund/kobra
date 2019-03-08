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
/// \file WindowFunctionTexture.cpp
/// \brief CPP file for WindowFunctionTexture, X3D scene-graph node
///
//
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/MedX3D/WindowFunctionTexture.h>

using namespace H3D;

// Add this node to the H3DNodeDatabase system.
H3DNodeDatabase WindowFunctionTexture::database( "WindowFunctionTexture", 
                                            &(newInstance<WindowFunctionTexture>), 
                                            typeid( WindowFunctionTexture ),
                                            &X3DTexture2DNode::database );

namespace WindowFunctionTextureInternals {
  FIELDDB_ELEMENT( WindowFunctionTexture, windowCenter, INPUT_OUTPUT )
  FIELDDB_ELEMENT( WindowFunctionTexture, windowWidth, INPUT_OUTPUT )
}


WindowFunctionTexture::WindowFunctionTexture( 
                           Inst< DisplayList > _displayList,
                           Inst< SFNode      > _metadata,
                           Inst< SFImage     > _image,
                           Inst< SFTextureProperties > _textureProperties,
                           Inst< SFInt32     > _windowCenter,
                           Inst< SFInt32     > _windowWidth ) :
  X3DTexture2DNode( _displayList, _metadata, 0, 0,
                    0, _image, _textureProperties ),
  windowCenter( _windowCenter ),
  windowWidth( _windowWidth ) {

  type_name = "WindowFunctionTexture";
  database.initFields( this );

  windowCenter->route( image );
  windowWidth->route( image );

  repeatS->setValue( false );
  repeatT->setValue( false );
  windowCenter->setValue( 128 );
  windowWidth->setValue( 256 );
}

void WindowFunctionTexture::SFImage::update() {
  vector< unsigned char > data( 512, 0 );
  int window_center = static_cast< SFInt32 * >(routes_in[0])->getValue(); 
  int window_width = static_cast< SFInt32 * >(routes_in[1])->getValue(); 
  
  int start_pos = window_center - window_width /2;
   
  for( int i = start_pos < 0 ? 0: start_pos; 
       i < start_pos + window_width && i < 256; 
       ++i ) {
    data[2*i] = 255 * (i - start_pos) / (window_width-1) ;
    data[2*i+1] =  data[2*i];
  }
  for( int i = start_pos + window_width; i < 256; ++i ) {
    data[2*i] = 255;
    data[2*i+1] = data[2*i];
  }
  
  value = new PixelImage( 256, 1, 1, 16, 
                          PixelImage::LUMINANCE_ALPHA,
                          PixelImage::UNSIGNED, &data[0], true );
}


