//////////////////////////////////////////////////////////////////////////////
//    Copyright 2004-2019, SenseGraphics AB
//
//    This file is part of H3DUtil.
//
//    H3DUtil is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    H3DUtil is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with H3DUtil; if not, write to the Free Software
//    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
//    A commercial license is also available. Please contact us at 
//    www.sensegraphics.com for more information.
//
//
/// \file PixelImage.cpp
/// \brief .cpp file for PixelImage.
///
//
//
//////////////////////////////////////////////////////////////////////////////

#include "H3DUtil/PixelImage.h"


H3DUtil::PixelImage::PixelImage( unsigned int _width,
                         unsigned int _height,
                         unsigned int _depth,
                         unsigned int _bits_per_pixel,
                         PixelType _pixel_type,
                         PixelComponentType _pixel_component_type,
                         unsigned char *data,
                         bool copy_data,
                         const Vec3f &_pixel_size,
                         CompressionType _compression_type ):
   w( _width ),
   h( _height ),
   d( _depth ),
   bits_per_pixel( _bits_per_pixel ),
   pixel_type( _pixel_type ),
   pixel_component_type( _pixel_component_type ),
   pixel_size( _pixel_size ),
   compression_type( _compression_type ) {
   if( copy_data ) {
     unsigned int size = (w * h * d * bits_per_pixel)/8;
     image_data = new unsigned char[ size ];
     memcpy( image_data, data, size );
   }
   else{ 
     image_data = data;
   }
 }

H3DUtil::PixelImage::PixelImage( unsigned int _width,
                        unsigned int _height,
                        unsigned int _depth,
                        unsigned int _bits_per_pixel,
                        PixelType _pixel_type,
                        PixelComponentType _pixel_component_type,
                        const Vec3f &_pixel_size,
                        CompressionType _compression_type ):
  w( _width ),
  h( _height ),
  d( _depth ),
  bits_per_pixel( _bits_per_pixel ),
  pixel_type( _pixel_type ),
  pixel_component_type( _pixel_component_type ),
  pixel_size( _pixel_size ),
  compression_type( _compression_type ) {
  unsigned int size = (w * h * d * bits_per_pixel)/8;
  image_data = new unsigned char[ size ];
} 

H3DUtil::PixelImage::PixelImage( Image *image,
                        unsigned int new_width,
                        unsigned int new_height,
                        unsigned int new_depth ) :
  w( 0 ),
  h( 0 ),
  d( 0 ),
  bits_per_pixel( 0 ),
  pixel_type( LUMINANCE ),
  pixel_component_type( SIGNED ),
  compression_type( NO_COMPRESSION ),
  image_data( NULL ) {

  if( image && image->compressionType() == NO_COMPRESSION ) {
    unsigned int width = image->width ();
    unsigned int height = image->height();
    unsigned int depth = image->depth();
    bits_per_pixel = image->bitsPerPixel();
    
    if( new_width == width && 
        new_height == height &&
        new_depth == depth ) {
      w = new_width;
      h = new_height;
      d = new_depth;
      pixel_type = image->pixelType();
      pixel_component_type = image->pixelComponentType();
      pixel_size = image->pixelSize();
      unsigned int size = (w * h * d * bits_per_pixel)/8;
      image_data = new unsigned char[ size ];
      memcpy( image_data, image->getImageData(), size );
    } else {
      unsigned int size = 
        (new_width * new_height * new_depth * bits_per_pixel)/8;
      unsigned int bytes_per_pixel = bits_per_pixel / 8;
      unsigned char *data = new unsigned char[ size ];
      H3DFloat z_step = 1.0f / new_depth;
      H3DFloat y_step = 1.0f / new_height;
      H3DFloat x_step = 1.0f / new_width;
      
      for( unsigned int z = 0; z < new_depth; ++z ) {
        H3DFloat dt = (z_step / 2) + z_step *z;
        for( unsigned int y = 0; y < new_height; ++y ) {
          H3DFloat ht =  (y_step / 2) + y_step *y;
          for( unsigned int x = 0; x < new_width; ++x ) {
            H3DFloat wt =  (x_step / 2) + x_step *x; 
            image->getSample( data + 
                              ( ( z * new_height + y ) * new_width + x ) * 
                              bytes_per_pixel,
                              wt, ht, dt );
          }
        }
      }
      w = new_width;
      h = new_height;
      d = new_depth;
      pixel_type = image->pixelType();
      pixel_component_type = image->pixelComponentType();
      pixel_size = image->pixelSize();
      compression_type = image->compressionType();
      image_data = data;
    }
  }
}

H3DUtil::PixelImage::PixelImage( PixelImage* other ) : PixelImage( other->width(),
                                                                   other->height(),
                                                                   other->depth(),
                                                                   other->bitsPerPixel(),
                                                                   other->pixelType(),
                                                                   other->pixelComponentType(),
                                                                   static_cast< unsigned char* >( other->getImageData() ),
                                                                   true, //copy the data
                                                                   other->pixelSize(),
                                                                   other->compressionType() ) {}
