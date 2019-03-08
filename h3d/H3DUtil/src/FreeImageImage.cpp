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
/// \file FreeImageImage.cpp
/// \brief .cpp file for FreeImageImage.
///
//
//
//////////////////////////////////////////////////////////////////////////////

#include "H3DUtil/FreeImageImage.h"

using namespace H3DUtil;

#ifdef HAVE_FREEIMAGE

#include <FreeImage.h>

  /// Thrown when the FIBITMAP * we encapsulate has a color type we 
  /// don't support.
  H3D_VALUE_EXCEPTION( FREE_IMAGE_COLOR_TYPE, 
                       UnsupportedFreeImageColorType ); 
  /// Thrown when the FIBITMAP * we encapsulate has a image type we 
  /// don't support.
  H3D_VALUE_EXCEPTION( FREE_IMAGE_TYPE, UnsupportedFreeImageImageType ); 

FreeImageImage::PixelComponentType FreeImageImage::pixelComponentType() {

  return pixel_component_type;
  
}

FreeImageImage::PixelType FreeImageImage::pixelType() {

  return pixel_type;
  
}

/// Destructor.
FreeImageImage::~FreeImageImage() {
  if( bitmap ) FreeImage_Unload( bitmap );
}

void *FreeImageImage::getImageData() {
  //return FreeImage_GetBits( bitmap );
  return image_data;
}

unsigned int FreeImageImage::bitsPerPixel() {
  return bits_per_pixel;
}

/// Returns the height of the image in pixels.
unsigned int FreeImageImage::height() {
  return h;
}

/// Returns the width of the image in pixels.
unsigned int FreeImageImage::width() {
  return w;
}

namespace FreeImageImageInternals {
  unsigned DLL_CALLCONV
  _ReadProc(void *buffer, unsigned size, unsigned count, fi_handle handle) {
    std::istream& is= *static_cast<std::istream*>(handle);

    std::streampos a= is.tellg();
    is.read ( static_cast<char*>(buffer), size*count );
    std::streampos b= is.tellg();

    return static_cast<unsigned int>(b-a);
  }

  inline int DLL_CALLCONV
  _SeekProc(fi_handle handle, long offset, int origin) {
    std::istream& is= *static_cast<std::istream*>(handle);
    switch ( origin ) {
    case SEEK_SET:
      is.seekg ( offset );
      break;
    case SEEK_CUR:
      is.seekg ( offset,std::ios_base::cur );
      break;
    case SEEK_END:
      is.seekg ( offset,std::ios_base::end );
      break;
    }
 
    return -1;
  }
 
  inline long DLL_CALLCONV
  _TellProc(fi_handle handle) {
    std::istream& is= *static_cast<std::istream*>(handle);
    return static_cast<long>(is.tellg());
  }
}

FreeImageIO* FreeImageImage::getIStreamIO () {
  static FreeImageIO io;
  io.read_proc= FreeImageImageInternals::_ReadProc;
  io.write_proc= NULL;
  io.seek_proc= FreeImageImageInternals::_SeekProc;
  io.tell_proc= FreeImageImageInternals::_TellProc;
  return &io;
}

void FreeImageImage::updateImageProperties(){

  if( bitmap==NULL ) {
    Console(LogLevel::Error)<<"Warning(FreeImageImage): Not able to updateImageProperties as"
      << " no valid bitmap is available, please check if related image loader is"
      << " correctly working" << std::endl;
    return;
  }
  
  // update pixel type
  FREE_IMAGE_COLOR_TYPE c_t = FreeImage_GetColorType( bitmap );

  switch( c_t ) {
  case FIC_MINISBLACK: 
  case FIC_MINISWHITE: pixel_type = LUMINANCE; break;
#if defined(FREEIMAGE_COLORORDER) && FREEIMAGE_COLORORDER == FREEIMAGE_COLORORDER_RGB
  case FIC_RGB: pixel_type = RGB; break;
  case FIC_RGBALPHA: pixel_type = RGBA; break;
#else
  case FIC_RGB: pixel_type = BGR; break;
  case FIC_RGBALPHA: pixel_type = BGRA; break;
#endif
  default: 
    throw UnsupportedFreeImageColorType( c_t, "", H3D_FULL_LOCATION );
  }
  // update pixel component type
  FREE_IMAGE_TYPE t = FreeImage_GetImageType( bitmap );
  switch( t ) {
  case FIT_RGB16:
  case FIT_RGBA16:
  case FIT_BITMAP:
  case FIT_UINT16: 
  case FIT_UINT32: pixel_component_type = UNSIGNED; break;
  case FIT_INT16: 
  case FIT_INT32: pixel_component_type = SIGNED; break;
  case FIT_DOUBLE:
  case FIT_FLOAT:
  case FIT_RGBF:
  case FIT_RGBAF: pixel_component_type = RATIONAL; break;
  default: 
    throw UnsupportedFreeImageImageType( t, "", H3D_FULL_LOCATION );
  }
  // update image data
  image_data = FreeImage_GetBits( bitmap );
  // update image width
  w = FreeImage_GetWidth( bitmap );
  // update image height
  h = FreeImage_GetHeight( bitmap );
  // update bit per pixel
  bits_per_pixel = FreeImage_GetBPP( bitmap );

}

#endif
