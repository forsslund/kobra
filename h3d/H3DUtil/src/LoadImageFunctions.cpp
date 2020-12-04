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
/// \file LoadImageFunctions.cpp
/// \brief .cpp file for functions to load images
///
//
//////////////////////////////////////////////////////////////////////////////
#include <H3DUtil/LoadImageFunctions.h>
#include <H3DUtil/ThreadPool.h>
#ifdef HAVE_ZLIB
#include <zlib.h>
#endif

#ifdef HAVE_FREEIMAGE
#include <H3DUtil/FreeImageImage.h>
#include <FreeImage.h>
#endif // HAVE_FREEIMAGE

#ifdef HAVE_TEEM
#include <H3DUtil/PixelImage.h>

//#define TEEM_STATIC
#include <teem/nrrd.h>
#include <teem/air.h>
#endif // HAVE_TEEM

#ifdef HAVE_DCMTK
#ifndef H3D_WINDOWS
// for Unix platforms this has to be defined before including anything
// from dcmtk
#define HAVE_CONFIG_H
#endif
#include <dcmtk/dcmdata/dcmetinf.h>
#include <dcmtk/dcmdata/dcdatset.h>
#include <dcmtk/dcmdata/dcdeftag.h>

#ifndef H3D_WINDOWS 
#include <dirent.h>
#endif
#include <algorithm>

#endif // HAVE_DCMTK

#ifdef HAVE_OPENEXR

#if _MSC_VER >= 1500
#pragma warning( disable : 4244 )
#pragma warning( disable : 4275 )
#pragma warning( disable : 4515 )
#pragma warning( disable : 4800 )
#endif

#include <OpenEXR/ImfOutputFile.h>
#include <OpenEXR/ImfInputFile.h>
#include <OpenEXR/ImfChannelList.h>
#if defined(OPENEXR_VERSION_MAJOR) && defined(H3D_WINDOWS)
#include <OpenEXR/ImfMisc.h>
#else
// pixelTypeSize actually exists in ImfMisc.h in some debian releases.
// However, the ImfMisc.h can not be included because it includes some file not included in the release.
namespace Imf {
  int pixelTypeSize( const Imf::PixelType &pt ) {
    switch( pt ) {
      case Imf::UINT:
  return 4;
      case Imf::HALF:
    return 2;
      case Imf::FLOAT:
  return 4;
      default: {
        H3DUtil::Console(H3DUtil::LogLevel::Error) << "Error: Unknown pixel type of OpenEXR image! Image will most likely behave weirdly." << std::endl;
        return 0;
      }
    }
  }
}
#endif

#if _MSC_VER >= 1500
#pragma warning( default : 4244 )
#pragma warning( default : 4275 )
#pragma warning( default : 4515 )
#pragma warning( default : 4800 )
#endif

#endif // HAVE_OPENEXF

#include <H3DUtil/PixelImage.h>
#include <H3DUtil/DicomImage.h>
#include <fstream>
#include <memory>

using namespace H3DUtil;
using namespace std;

#ifdef HAVE_FREEIMAGE
Image *loadFreeImageInternal( FIBITMAP* bm, const string& url= "" ) {
  // Take care of the case of 32 bit RGB images( alpha ignored ) that seems to
  // happen once in a while with png images
  if( FreeImage_GetColorType( bm ) == FIC_RGB && FreeImage_GetBPP( bm ) == 32 ) {
    FIBITMAP *old = bm;
    bm = FreeImage_ConvertTo24Bits( bm );
    FreeImage_Unload( old );
  }

  FREE_IMAGE_COLOR_TYPE t = FreeImage_GetColorType( bm );
  switch( t ) {
  case FIC_PALETTE: {
    // We have a palatted image. Convert to RGB or RGBA.
    RGBQUAD *palette = FreeImage_GetPalette( bm );
    BYTE *transparency_table = FreeImage_GetTransparencyTable( bm );
    bool is_transparent = FreeImage_GetTransparencyCount( bm ) > 0;

    Image::PixelType pixel_type = is_transparent ? Image::RGBA : Image::RGB;
    Image::PixelComponentType pixel_component_type = Image::UNSIGNED;
    unsigned int width =  FreeImage_GetWidth( bm );
    unsigned int height =  FreeImage_GetHeight( bm );
    unsigned int depth =  1;
    unsigned int bytes_per_pixel = is_transparent ? 4 : 3;
    unsigned int size = width * height * depth * bytes_per_pixel;

    // build the new pixel data
    unsigned char *data = new unsigned char[ size ];
    for( unsigned int y = 0; y < height; ++y ) {
      for( unsigned int x = 0; x < width; ++x ) {
        unsigned int i = (x + y * width) * bytes_per_pixel;
        BYTE index;
        FreeImage_GetPixelIndex( bm, x, y, &index );
        data[ i ] = palette[index].rgbRed;
        data[ i + 1 ] = palette[index].rgbGreen;
        data[ i + 2 ] = palette[index].rgbBlue;
        if( is_transparent ) {
          data[ i + 3 ] = transparency_table[index];
        }
      }
    }

    FreeImage_Unload( bm );

    return new PixelImage( width,
                           height,
                           depth,
                           bytes_per_pixel * 8,
                           pixel_type,
                           pixel_component_type,
                           data,
                           false,
                           Vec3f( 1, 1, 1 ) );

  }
  case FIC_MINISBLACK: 
  case FIC_MINISWHITE:
  case FIC_RGB:
  case FIC_RGBALPHA: break;
  default: {
    Console(LogLevel::Warning) << "Warning: UnsupportedFreeImageColorType " << t << ". ";
    if ( url.empty() ) {
      Console(LogLevel::Warning) << "Cannot load file from stream." << endl;
    } else {
      Console(LogLevel::Warning) << "File " << url << " can not be loaded. "
                 << "File name might be the name of a downloaded temporary file. "
                 << endl;
    }

    FreeImage_Unload( bm );
    return NULL;
  }
  }

  return new FreeImageImage( bm );
}

Image *H3DUtil::loadFreeImage( const string &url ) {
  FREE_IMAGE_FORMAT format = FreeImage_GetFileType( url.c_str() );
  if( format == FIF_UNKNOWN ) {
    format = FreeImage_GetFIFFromFilename( url.c_str() );
  }

  if( format != FIF_UNKNOWN && FreeImage_FIFSupportsReading( format ) ) { 
    FIBITMAP *bm = FreeImage_Load( format, url.c_str() );
    if ( bm ) {
      return loadFreeImageInternal ( bm, url );
    }
  }

  return NULL;
}

Image *H3DUtil::loadFreeImage( istream &is ) {
  FREE_IMAGE_FORMAT format = FreeImage_GetFileTypeFromHandle ( FreeImageImage::getIStreamIO(), static_cast<fi_handle>(&is) );

  if( format != FIF_UNKNOWN && FreeImage_FIFSupportsReading( format ) ) { 
    FIBITMAP *bm = FreeImage_LoadFromHandle( format, FreeImageImage::getIStreamIO(), static_cast<fi_handle>(&is) );
    if ( bm ) {
      return loadFreeImageInternal ( bm );
    }
  }

  return NULL;
}

bool H3DUtil::saveFreeImagePNG( const string &url,
                                Image& image, bool disable_alpha, bool async ) {

  struct SaveImageParams
  {
    std::string url;
    bool disable_alpha;
    std::unique_ptr<PixelImage> pixel_image;
    SaveImageParams(const string& _url, Image& _image, const bool& _disable_alpha ):
    url( _url ),
    disable_alpha( _disable_alpha ),
    pixel_image( new PixelImage( &_image, _image.width(), _image.height(), _image.depth() ) ) {
    }
  };


  if( async ) {
    auto saveFreeImagePNGCallBack = []( void* saveImageParams )-> void* {
      SaveImageParams* params = static_cast<SaveImageParams*>(saveImageParams);
      bool saved = saveFreeImagePNGInternal( params->url, *(params->pixel_image), params->disable_alpha );
      if( !saved ) {
        Console( LogLevel::Error ) << "Saving to url " << params->url
          << " failed. Make sure that the url is valid and that the application has the required permissions.\n";
      }
      delete params;
      return nullptr;
    };
    if ( ThreadPool::global_pool.size() == 0 ) {
      ThreadPool::global_pool.resize( 2 );
    }
    SaveImageParams* save_image_params = new SaveImageParams( url, image, disable_alpha );

    ThreadPool::global_pool.executeFunction( saveFreeImagePNGCallBack, save_image_params );

    return true;
  } else {
    return saveFreeImagePNGInternal( url, image, disable_alpha );
  }
}

bool H3DUtil::saveFreeImagePNGInternal( const std::string &url, Image& image, bool disable_alpha ) {
  // Can't use image.bitsPerPixel() because not all image objects use 32bpp.
  int bits_per_pixel = 32;

  FIBITMAP *free_image = FreeImage_Allocate(image.width(), image.height(), bits_per_pixel);
  RGBQUAD pixelcolor;
  RGBA c;

  // Transfer values from the image
  for( unsigned int x = 0; x < image.width(); ++x ) {
    for( unsigned y = 0; y < image.height(); ++y ) {
      c= image.getPixel ( x, y );
      pixelcolor.rgbRed = BYTE( c.r*255 );
      pixelcolor.rgbGreen = BYTE( c.g*255 );
      pixelcolor.rgbBlue = BYTE( c.b*255 );
      pixelcolor.rgbReserved = BYTE( disable_alpha ? 255 : c.a*255 );
      FreeImage_SetPixelColor(free_image,x,y,&pixelcolor);
    }
  }

  if( FreeImage_Save(FIF_PNG,free_image,url.c_str(),0) ) {
    FreeImage_Unload(free_image);
    return true;
  } else {
    FreeImage_Unload(free_image);
    return false;
  }
}

#endif

Image *H3DUtil::loadRawImage( const string &url,
                              RawImageInfo &raw_image_info ) {
  Image::PixelType pixel_type;
  if( raw_image_info.pixel_type_string == "LUMINANCE" )
    pixel_type = Image::LUMINANCE;
  else if( raw_image_info.pixel_type_string == "LUMINANCE_ALPHA" )
    pixel_type = Image::LUMINANCE_ALPHA;
  else if( raw_image_info.pixel_type_string == "RGB" ) 
    pixel_type = Image::RGB;
  else if( raw_image_info.pixel_type_string == "RGBA" ) 
    pixel_type = Image::RGBA;
  else if( raw_image_info.pixel_type_string == "BGR" ) 
    pixel_type = Image::BGR;
  else if( raw_image_info.pixel_type_string == "BGRA" ) 
    pixel_type = Image::BGRA;
  else if( raw_image_info.pixel_type_string == "VEC3" ) 
    pixel_type = Image::VEC3;
  else {
    Console(LogLevel::Warning) << "Warning: Invalid pixelType value \"" << raw_image_info.pixel_type_string
               << "\" in  RawImageLoader. " << endl;
    return NULL;
  }

  Image::PixelComponentType pixel_component_type;
  if( raw_image_info.pixel_component_type_string == "SIGNED" ) 
    pixel_component_type = Image::SIGNED; 
  else if( raw_image_info.pixel_component_type_string == "UNSIGNED" )
    pixel_component_type = Image::UNSIGNED;
  else if( raw_image_info.pixel_component_type_string == "RATIONAL" )
    pixel_component_type = Image::RATIONAL;
  else {
    Console(LogLevel::Warning) << "Warning: Invalid pixelComponentType value \"" 
               << raw_image_info.pixel_component_type_string
               << "\" in  RawImageLoader. " << endl;
    return NULL;
  }
    
  unsigned int expected_size = 
    raw_image_info.width * raw_image_info.height * raw_image_info.depth *
    ( raw_image_info.bits_per_pixel / 8 );

  unsigned char * data = new unsigned char[expected_size];
  
  ifstream is( url.c_str(), ios::in | ios::binary );
  if( !is.good() ) {
    return NULL;
  }
  
  is.read( (char *)data, expected_size );
  unsigned int actual_size = (unsigned int) is.gcount();
  is.close();
  
#ifdef HAVE_ZLIB
  
  if( actual_size < expected_size ){
    
    unsigned char * data2 = new unsigned char[expected_size];
    
    int err;
    z_stream strm;

    // allocate inflate state
    strm.next_in = data;               // next input byte
    strm.avail_in = (uInt)actual_size; // number of bytes available at next_in
    strm.total_in = 0;                 // total number of input bytes read so far
    strm.next_out = data2;             // next output byte will go here
    strm.avail_out = expected_size;    // remaining free space at next_out
    strm.total_out = 0;                // total number of bytes output so far
    strm.zalloc = Z_NULL;
    strm.zfree = Z_NULL;
    strm.opaque = Z_NULL;
    
    err = inflateInit2(&strm,47);
    
    if( err == Z_MEM_ERROR ){
      Console(LogLevel::Warning) << "Warning: zlib memory error." << endl;
      delete[] data;
      delete[] data2;
      return NULL;
    }
    if( err == Z_VERSION_ERROR ){
      Console(LogLevel::Warning) << "Warning: zlib version error." << endl;
      delete[] data;
      delete[] data2;
      return NULL;
    }
    
    err = inflate(&strm,Z_FINISH);
    
    if( err == Z_DATA_ERROR ){
      Console(LogLevel::Warning) << "Warning: zlib unrecognizable data error." << endl;
      delete[] data;
      delete[] data2;
      return NULL;
    }
    if( err == Z_STREAM_ERROR ){
      Console(LogLevel::Warning) << "Warning: zlib stream error." << endl;
      delete[] data;
      delete[] data2;
      return NULL;
    }
    if( err == Z_BUF_ERROR ){
      Console(LogLevel::Warning) << "Warning: zlib out of memory error." << endl;
      delete[] data;
      delete[] data2;
      return NULL;
    }
    
    err = inflateEnd(&strm);
    
    if( err == Z_STREAM_ERROR ){
      Console(LogLevel::Warning) << "Warning: zlib stream error." << endl;
      delete[] data;
      delete[] data2;
      return NULL;
    }
    
    Console(LogLevel::Debug) << "Inflated compressed raw file." << endl;
    delete[] data;
    data = data2;
  }
  
#endif
  
  return new PixelImage( raw_image_info.width,
                         raw_image_info.height,
                         raw_image_info.depth,
                         raw_image_info.bits_per_pixel,
                         pixel_type,
                         pixel_component_type,
                         data,
                         false,
                         raw_image_info.pixel_size );
}

#ifdef HAVE_TEEM
Image *H3DUtil::loadNrrdFile( const string &url ) {
  Nrrd *nin;
  
  /* create a new nrrd */
  nin = nrrdNew();
  
  /* tell nrrdLoad to only read the header, not the data */
  NrrdIoState *nio = nrrdIoStateNew();
  nrrdIoStateSet(nio, nrrdIoStateSkipData, AIR_TRUE);

  // read in the nrrd header from file , not the data.
  // The reason for this approach is that H3DUtil will allocate the data later
  // to make sure that there are no problems with allocating data over
  // boundaries for shared libraries in Windows.
  if( nrrdLoad( nin, url.c_str(), nio ) ) {
    nio = nrrdIoStateNix(nio);
    nrrdNuke(nin);
    return NULL;
  }
  
  /* we're done with the nrrdIoState, this sets it to NULL */
  nio = nrrdIoStateNix(nio);

  Image::PixelType pixel_type = Image::LUMINANCE;
  Image::PixelComponentType component_type;
  unsigned int bits_per_pixel;

  if( nin->type == nrrdTypeChar ) {
    component_type = Image::SIGNED;
    bits_per_pixel = 8;
  } else if( nin->type == nrrdTypeUChar ) {
    component_type = Image::UNSIGNED;
    bits_per_pixel = 8;
  } else if( nin->type == nrrdTypeShort ) {
    component_type = Image::SIGNED;
    bits_per_pixel = 16;
  } else if( nin->type == nrrdTypeUShort ) {
    component_type = Image::UNSIGNED;
    bits_per_pixel = 16;
  } else if( nin->type == nrrdTypeInt ) {
    component_type = Image::SIGNED;
    bits_per_pixel = 32;
  } else if( nin->type == nrrdTypeUInt ) {
    component_type = Image::UNSIGNED;
    bits_per_pixel = 32;
  } else if( nin->type == nrrdTypeLLong ) {
    component_type = Image::SIGNED;
    bits_per_pixel = 64;
  } else if( nin->type == nrrdTypeULLong ) {
    component_type = Image::UNSIGNED;
    bits_per_pixel = 64;
  } else if( nin->type == nrrdTypeFloat ) {
    component_type = Image::RATIONAL;
    bits_per_pixel = 32;
  } else  if( nin->type == nrrdTypeDouble ) {
    component_type = Image::RATIONAL;
    bits_per_pixel = 64;
  } else {
    nrrdNuke(nin);
    return NULL;
  }
  
  unsigned int width = 1, height = 1, depth = 1;

  Vec3f spacing = Vec3f( 0.0003f, 0.0003f, 0.0003f );

  unsigned w_axis = 0, h_axis = 1, d_axis = 2;
  
  if( nin->dim == 4 ) {
    // if dimension == 4, we assume the first dimension is used for each 
    // voxel value
    w_axis = 1;
    h_axis = 2;
    d_axis = 3;

    size_t nr_components = nin->axis[0].size;
    if( nr_components == 2 ) {
      pixel_type = Image::LUMINANCE_ALPHA;
      bits_per_pixel *= 2;
    } else if( nr_components == 3 ) {
      pixel_type = Image::RGB;
      bits_per_pixel *= 3;
    } else if( nr_components == 4 ) {
      pixel_type = Image::RGBA;
      bits_per_pixel *= 4;
    }
  }
  
  const char* MM = "mm";
  const char* CM = "cm";
  const float mmFactor = 0.001f;
  const float cmFactor = 0.01f;

  if( nin->dim >= 3 ) {
    depth = (unsigned int) nin->axis[d_axis].size;
    if(!airIsNaN(nin->axis[d_axis].spacing)) {
      spacing.z = (H3DFloat)( nin->axis[d_axis].spacing );
    } else if( !airIsNaN(nin->axis[d_axis].spaceDirection[d_axis])) {
      // The space direction contains the spacings
      spacing.z = H3DAbs((H3DFloat)( nin->axis[d_axis].spaceDirection[d_axis] ));
      // Check for space units
      if(nin->spaceUnits[d_axis]) {
        if(strcmp(nin->spaceUnits[d_axis],MM) == 0) {
          spacing.z *= mmFactor;
        } else if(strcmp(nin->spaceUnits[d_axis],CM) == 0) {
          spacing.z *= cmFactor;
        }
      }
    } else {
      Console(LogLevel::Warning) << "Warning: NRRD file " << url
         << " lacks spacing information in axis 2. Sets to default 0.0003\n" << endl;
    }
  }

  if( nin->dim >= 2 ) {
    height = (unsigned int) nin->axis[h_axis].size;
    if(!airIsNaN(nin->axis[h_axis].spacing)) {
      spacing.y = (H3DFloat)( nin->axis[h_axis].spacing );
    } else if( !airIsNaN(nin->axis[h_axis].spaceDirection[h_axis])) {
      // The space direction contains the spacings
      spacing.y = H3DAbs((H3DFloat)( nin->axis[h_axis].spaceDirection[h_axis] ));
      if(nin->spaceUnits[h_axis]) {
        if(strcmp(nin->spaceUnits[h_axis],MM) == 0) {
          spacing.y *= mmFactor;
        } else if(strcmp(nin->spaceUnits[h_axis],CM) == 0) {
          spacing.y *= cmFactor;
        }
      }
    } else {
      Console(LogLevel::Warning) << "Warning: NRRD file " << url
         << " lacks spacing information in axis 1. Sets to default 0.0003\n" << endl;
    }
  }

  if( nin->dim >= 1 ) {
    width = (unsigned int) nin->axis[w_axis].size;
    if(!airIsNaN(nin->axis[w_axis].spacing)) {
      spacing.x = (H3DFloat)( nin->axis[w_axis].spacing );
    } else if( !airIsNaN(nin->axis[w_axis].spaceDirection[w_axis])) {
      // The space direction contains the spacings
      spacing.x = H3DAbs((H3DFloat)( nin->axis[w_axis].spaceDirection[w_axis] ));
      if(nin->spaceUnits[w_axis]) {
        if(strcmp(nin->spaceUnits[w_axis],MM) == 0) {
          spacing.x *= mmFactor;
        } else if(strcmp(nin->spaceUnits[w_axis],CM) == 0) {
          spacing.x *= cmFactor;
        }
      }
    } else {
      Console(LogLevel::Warning) << "Warning: NRRD file " << url
         << " lacks spacing information in axis 0. Sets to default 0.0003\n" << endl;
    }
  }

  // Allocate the data.
  // The reason for allocating the memory for the data ourself instead of
  // letting the teem library take care of that is to make sure that there
  // are no problems with deleting data from H3DUtil that is allocated
  // in another shared library on a Windows system.
  unsigned bytes_per_pixel = 
  bits_per_pixel % 8 == 0 ? 
  bits_per_pixel / 8 : bits_per_pixel / 8 + 1;

  unsigned int size = (width * height * depth * bytes_per_pixel);
  unsigned char * data = 0;
  try {
    data = new unsigned char[ size ];
  } catch ( bad_alloc& ) {
    Console(LogLevel::Warning) << "Error: NRRD file " << url
         << " contains to many data points to allocate (bad_alloc)\n" << endl;
    nrrdNuke(nin);
    return NULL;
  }

  nin->data = data;
  if( nrrdLoad( nin, url.c_str(), NULL ) ) {
    // free nrrd struct memory but not data.
    nrrdNix(nin);
    delete [] data;
    return NULL;
  }
  // We assume this will work well on other systems, note that this is not
  // yet tested properly.
  Image *image =  new PixelImage( width, height, depth, bits_per_pixel,
                                  pixel_type, component_type,
                                  (unsigned char *)nin->data,
                                  false, spacing );
  // free nrrd struct memory but not data.
  nrrdNix(nin);
  return image;
}

int H3DUtil::saveImageAsNrrdFile( const string &filename,
                                  Image *image ) {
    Nrrd *nin;
  
  /* create a new nrrd */
  nin = nrrdNew();

  unsigned int bits_per_pixel = image->bitsPerPixel();
 
  Image::PixelComponentType component_type = image->pixelComponentType();
  Image::PixelType pixel_type = image->pixelType();
  const Vec3f &voxel_size = image->pixelSize(); 

  // dimensions
  unsigned int nr_components = 1;
  if( pixel_type == Image::LUMINANCE ) {
    nr_components = 1;
  } else if( pixel_type == Image::LUMINANCE_ALPHA ) {
    nr_components = 2;
  } else if( pixel_type == Image::BGR || pixel_type == Image::RGB || 
             pixel_type == Image::VEC3) {
    nr_components = 3;
  } else if( pixel_type == Image::RGBA || pixel_type == Image::BGRA ) {
    nr_components = 4;
  } else {
    return -1;
  }

  int bytes_per_component =  bits_per_pixel / (nr_components * 8);
  if( bits_per_pixel % (nr_components * 8) > 0 ) ++bytes_per_component;

  // save the type
  if( component_type == Image::SIGNED ) {
    if( bytes_per_component == 1 ) nin->type = nrrdTypeChar;
    else if( bytes_per_component == 2 ) nin->type = nrrdTypeShort;
    else if( bytes_per_component == 4 ) nin->type = nrrdTypeInt;
    else if( bytes_per_component == 8 ) nin->type = nrrdTypeLLong;
    else return -1; 
  } else if( component_type == Image::UNSIGNED ) {
    if( bytes_per_component == 1 ) nin->type = nrrdTypeUChar;
    else if( bytes_per_component == 2 ) nin->type = nrrdTypeUShort;
    else if( bytes_per_component == 4 ) nin->type = nrrdTypeUInt;
    else if( bytes_per_component == 8 ) nin->type = nrrdTypeULLong;
    else return -1; 
  } else  if( component_type == Image::RATIONAL ) {
    if(  bytes_per_component == 4 ) nin->type = nrrdTypeFloat;
    else if( bytes_per_component == 8 ) nin->type = nrrdTypeDouble;
    else return -1; 
  } else {
    return -1; 
  }



  nin->dim = 4;
  nin->axis[0].size = nr_components;
  nin->axis[1].size = image->width();
  nin->axis[2].size = image->height();
  nin->axis[3].size = image->depth();

  // Fallback if voxel size is invalid.
  if(epsilonCompare(voxel_size.x, 0.0f) ||
     epsilonCompare(voxel_size.y, 0.0f) ||
     epsilonCompare(voxel_size.z, 0.0f)) {

    nin->axis[1].spacing = 1.0f;
    nin->axis[2].spacing = 1.0f;
    nin->axis[3].spacing = 1.0f;
  } else {
    nin->axis[1].spacing = voxel_size.x;
    nin->axis[2].spacing = voxel_size.y;
    nin->axis[3].spacing = voxel_size.z;
  }

  nin->data = image->getImageData();

  int res = nrrdSave( filename.c_str(), nin, NULL );

  // free nrrd struct memory but not data.
  nrrdNix( nin );

  return res;
}
#endif

#ifdef HAVE_DCMTK
Image * loadSingleDicomFile( const string &url ) {
  try {
    // dicom data is specified from topleft corner. we have to convert it so
    // it is specified from the bottomleft corner
    auto_ptr< DicomImage > slice_2d( new DicomImage( url ) );

    // get properties from dicom image
    unsigned int width  = slice_2d->width();
    unsigned int height = slice_2d->height();
    // depth will be incremented below depending on the number of slices
    // used
    unsigned int depth = slice_2d->depth();
    Vec3f pixel_size = slice_2d->pixelSize();
    Image::PixelType pixel_type = slice_2d->pixelType();
    unsigned int bits_per_pixel = slice_2d->bitsPerPixel();
    Image::PixelComponentType component_type = 
      slice_2d->pixelComponentType();    

    unsigned bytes_per_pixel = 
      ( bits_per_pixel % 8 == 0 ) ? 
      ( bits_per_pixel / 8 ) : ( bits_per_pixel / 8 + 1 );

    // allocate and copy data into correct order.
    unsigned char *data = 
      new unsigned char[ width * height * depth * bytes_per_pixel ];

    unsigned char * slice_data = (unsigned char *)slice_2d->getImageData();
    for( unsigned int d = 0; d < depth; ++d ) {
      for( unsigned int row = 0; row < height; row++ ) {
        memcpy( data + (d * height + row ) *width* bytes_per_pixel,
                slice_data + (d * height + height - row - 1 ) * width * bytes_per_pixel,
                width * bytes_per_pixel );
      }
    }

    // return new image with the correct row order.
    return new PixelImage( width, height, depth, 
                           bits_per_pixel, pixel_type, component_type,
                           data, false, pixel_size );

  } catch( const DicomImage::CouldNotLoadDicomImage &e ) {
    Console(LogLevel::Warning) << e << endl;
    return NULL;
  }
}

H3DUTIL_API Image *H3DUtil::loadDicomFile( const string &url,
                                           bool load_single_file ) {
  if( load_single_file ) {
    return loadSingleDicomFile(url);
  } else {
    // the filenames for all files to compose.
    vector< string > filenames;

    // divide url into path and filename
    size_t found = url.find_last_of("/\\");
    string path, filename;

    if( found != string::npos ) {
      path = url.substr(0,found);
      filename = url.substr(found+1);
    } else {
      path = "";
      filename = url;
    }

    // find files in the same directory as the original file that starts
    // with the same characters.
#ifdef H3D_WINDOWS
    LPWIN32_FIND_DATA find_data = new WIN32_FIND_DATA;
    HANDLE handle = FindFirstFile( 
      (path + "/" + filename.substr( 0,3 ) +"*" ).c_str(), find_data );
    if( handle != INVALID_HANDLE_VALUE ) {
      string name = path + "\\" + find_data->cFileName;
      filenames.push_back( name );
      //Console(LogLevel::Warning) << name << endl;
      while( FindNextFile(handle, find_data) ) {
        //Console(LogLevel::Warning) << find_data->cFileName << endl;
        filenames.push_back( string( path + "\\" + find_data->cFileName ) );
      }
    }
    delete find_data;
#else
    string prefix = filename.substr( 0,3 );
    DIR *dp;
    struct dirent *dirp;
    if((dp  = opendir(path.c_str())) != NULL) {
      while ((dirp = readdir(dp)) != NULL) {
        string name = string(dirp->d_name);
        if( prefix == name.substr( 0,3 ) ) {
          filenames.push_back( string( path.c_str() ) + "/" + name );
        }
      }
      closedir(dp);
    }
#endif

    // There is a dicom format version which uses only one file and store all the
    // slices in it. This will hopefully take care of that case always.
    if( filenames.size() == 1 ) {
      return loadSingleDicomFile( url );
    }

    // sort them in alphabetical order
    std::sort( filenames.begin(), filenames.end() );

    if( filenames.empty() ) return NULL;

    auto_ptr< DicomImage > slice_2d;
    
    // read the original slice in order to get image information
    try {
      slice_2d.reset( new DicomImage( url ) );
    } catch( const DicomImage::CouldNotLoadDicomImage &e ) {
      Console(LogLevel::Warning) << e << endl;
      return NULL;
    }

    unsigned int width  = slice_2d->width();
    unsigned int height = slice_2d->height();
    // depth will be incremented below depending on the number of slices
    // used
    unsigned int depth = 0;
    Vec3f pixel_size = slice_2d->pixelSize();
    Image::PixelType pixel_type = slice_2d->pixelType();
    unsigned int bits_per_pixel = slice_2d->bitsPerPixel();
    Image::PixelComponentType component_type = 
      slice_2d->pixelComponentType();    

    unsigned bytes_per_pixel = 
      bits_per_pixel % 8 == 0 ? 
      bits_per_pixel / 8 : bits_per_pixel / 8 + 1;

    unsigned char *data = NULL;
    try {
      data = new unsigned char[ width * height * filenames.size() * bytes_per_pixel ];
    } catch( std::bad_alloc& ) {
      Console(LogLevel::Error) << "H3DUtil::loadDicomFile failed to allocate enough data." << endl;
      return NULL;
    }

    DcmDataset *orig_data_set = slice_2d->getDicomFileInfo().getDataset();

    // get the series unique id for the original file
    OFString orig_series_instance_UID;
    OFCondition res = orig_data_set->findAndGetOFString( DCM_SeriesInstanceUID, 
                                                         orig_series_instance_UID );
   

    bool use_all_files = (res != EC_Normal || orig_series_instance_UID == "" );

    // Checking the z value of the ImagePositionPatient to know in which
    // order the dicom files should be read.
    // In the future these might be changed to Vec3f and we check
    // the entire vector to know which order to read the images.
    // init patient_pos1 to just remove warning, it will actually be properly init later
    H3DFloat patient_pos1 = 0.0;
    H3DFloat patient_pos2 = 0.0;
    H3DFloat patient_orn[6];
    bool first_patient_pos_set = false;
    bool second_patient_pos_set = false;
    // Iterate through filename and get the two first valid ones and try
    // to read information from them. If information is read from two, then
    // break. This could perhaps be coupled with the other for loop.
    for( unsigned int i = 0; i < filenames.size(); ++i ) {
      DcmFileFormat fileformat;
      if( fileformat.loadFile(filenames[i].c_str()).good() ) {
        OFString string_value;
        DcmDataset * dataset = fileformat.getDataset();
        res = dataset->findAndGetOFString( DCM_SeriesInstanceUID,
                                           string_value );
        // only use the files that match the series instance of the original 
        // file.
        if( use_all_files ||
            string_value == orig_series_instance_UID ) {
          res = dataset->findAndGetOFString(
            DCM_ImagePositionPatient, string_value, 2 );

#ifdef H3D_WINDOWS
          // Get locale string, used to test what is used as
          // decimal separator . or ,
          // DICOM always use . as decimal separator so we need to make sure that
          // it is used properly.
          string locale_string( setlocale( LC_NUMERIC, NULL ) );
          if( locale_string.length() < 7 || locale_string.substr( 0, 7 ) != "English" ) {
            H3DFloat test_data_comma = H3DFloat( atof( "0,2" ) );
            if( test_data_comma > 0.1 ) {
              for( unsigned int j = 0; j < string_value.size(); ++j ) {
                if( string_value[j] == '.' )
                  string_value[j] = ',';
              }
            }
          }
#endif
          if( res == EC_Normal ) {
            if( first_patient_pos_set ) {
              patient_pos2 = (H3DFloat)( atof( string_value.c_str() ) );
              second_patient_pos_set = true;
              OFString string_value2;
              OFCondition res2 = dataset->findAndGetOFStringArray(
                DCM_ImageOrientationPatient, string_value2 );
              if( res2 == EC_Normal ) {
                size_t start_pos = 0;
                for( unsigned int j = 0; j < 6; ++j ) {
                  size_t pos = string_value2.find( "\\", start_pos, 2 );
                  patient_orn[j] = (H3DFloat)( atof( string_value2.substr(
                    start_pos, pos - start_pos ).c_str() ) );
                  start_pos = pos + 1;
                }
                if( patient_pos2 > patient_pos1 ) {
                  // Sort in reverse alphabetical order.
                  vector< string > tmp_filenames;
                  tmp_filenames.reserve( filenames.size() );
                  for( int j = (int)filenames.size() - 1; j >= 0; --j )
                    tmp_filenames.push_back( filenames[j] );
                  filenames = tmp_filenames;
                }

                if( H3DAbs( patient_orn[0] - 1 ) > Constants::f_epsilon ||
                    H3DAbs( patient_orn[1] - 0 ) > Constants::f_epsilon ||
                    H3DAbs( patient_orn[2] - 0 ) > Constants::f_epsilon ||
                    H3DAbs( patient_orn[3] - 0 ) > Constants::f_epsilon ||
                    H3DAbs( patient_orn[4] - 1 ) > Constants::f_epsilon ||
                    H3DAbs( patient_orn[5] - 0 ) > Constants::f_epsilon ) {
                  Console(LogLevel::Warning) << "Warning: ImageOrientationPatient is not "
                             << "the assumed default. Dicom image might not "
                             << "be read correctly." << endl;
                }
                break;
              }
            } else {
              // Set patient_pos1 and break.
              patient_pos1 = (H3DFloat)( atof( string_value.c_str() ) );
              first_patient_pos_set = true;
            }
          }
        }
      }
    }

    // use ImagePositionPatient if available to set pixel_size z.
    // If it does not exist SliceThickness is used.
    if( second_patient_pos_set ) {
      H3DFloat slice_distance( H3DAbs(patient_pos1 - patient_pos2) );
      pixel_size.z = slice_distance * H3DFloat( 1e-3 ); // to metres
    }

    // read all files and compose them into one image.
    for( unsigned int i = 0; i < filenames.size(); ++i ) {
      DcmFileFormat fileformat;
      OFString series_instance_UID;

      if (fileformat.loadFile(filenames[i].c_str()).good()) {
        DcmDataset *dataset = fileformat.getDataset();
        res = dataset->findAndGetOFString( DCM_SeriesInstanceUID,
                                                       series_instance_UID );
      }
     
      // only use the files that match the seris instance of the original 
      // file.
      if( use_all_files || series_instance_UID == orig_series_instance_UID ) {
        try {
          slice_2d.reset( new DicomImage( filenames[i] ) );
        } catch( const DicomImage::CouldNotLoadDicomImage &e ) {
          Console(LogLevel::Warning) << e << endl;
          delete [] data;
          return NULL;
        }

        // dicom data is specified from topleft corner. we have to convert it
        // so it is specified from the bottomleft corner
        unsigned char * slice_data = (unsigned char *)slice_2d->getImageData();
        for( unsigned int row = 0; row < height; ++row ) {
          memcpy( data + (width * height * depth  + row*width)*
                         bytes_per_pixel,
                  slice_data + (height - row - 1 ) * width * bytes_per_pixel,
                  width * bytes_per_pixel );
        }
        ++depth;
      }
    }
   
    return new PixelImage( width, height, depth, 
                           bits_per_pixel, pixel_type, component_type,
                           data, false, pixel_size );
  }
}
#endif

#ifdef HAVE_OPENEXR
H3DUTIL_API bool H3DUtil::saveOpenEXRImage( const string &url,
  Image& image ) {

  if( image.pixelComponentType() != Image::RATIONAL ) {
    Console( LogLevel::Error ) << "Cannot save image as OpenEXR, pixel component type must be RATIONAL! "
      "Only float and half-float images are supported!" << endl;
    return false;
  }

  // channel offset from base (-1 if channel is not used)
  int offsets[ 4 ] = { -1, -1, -1, -1 };

  // number of channels used
  int nr_channel = 0;

  int bytes_per_pixel = image.bitsPerPixel() / 8;

  switch( image.pixelType() ) {
  case Image::LUMINANCE:
    offsets[ 0 ] = 0;
    nr_channel = 1;
    break;
  case Image::R:
    offsets[ 0 ] = 0;
    nr_channel = 1;
    break;
  case Image::LUMINANCE_ALPHA:
    offsets[ 0 ] = 0;
    offsets[ 3 ] = 1;
    nr_channel = 2;
    break;
  case Image::RGB:
    offsets[ 0 ] = 0;
    offsets[ 1 ] = 1;
    offsets[ 2 ] = 2;
    nr_channel = 3;
    break;
  case Image::RGBA:
    offsets[ 0 ] = 0;
    offsets[ 1 ] = 1;
    offsets[ 2 ] = 2;
    offsets[ 3 ] = 3;
    nr_channel = 4;
    break;
  case Image::BGR:
    offsets[ 0 ] = 2;
    offsets[ 1 ] = 1;
    offsets[ 2 ] = 0;
    nr_channel = 3;
    break;
  case Image::BGRA:
    offsets[ 0 ] = 2;
    offsets[ 1 ] = 1;
    offsets[ 2 ] = 0;
    offsets[ 3 ] = 3;
    nr_channel = 4;
    break;
  case Image::VEC3:
    offsets[ 0 ] = 0;
    offsets[ 1 ] = 1;
    offsets[ 2 ] = 2;
    nr_channel = 3;
    break;
  case Image::RG:
    offsets[ 0 ] = 0;
    offsets[ 1 ] = 1;
    nr_channel = 2;
    break;
  }

  Imf::PixelType pixel_type;
  int bytes_per_channel = bytes_per_pixel / nr_channel;
  if( bytes_per_channel == Imf::pixelTypeSize( Imf::FLOAT ) ) {
    pixel_type = Imf::FLOAT;
  } else if( bytes_per_channel == Imf::pixelTypeSize( Imf::HALF ) ) {
    pixel_type = Imf::HALF;
  } else {
    Console( LogLevel::Error ) <<
      "Cannot save image as OpenEXR, only float and half-float images are supported!" << endl;
    return false;
  }

  try {
    Imf::Header header( image.width(), image.height() );
    if( offsets[ 0 ] != -1 ) header.channels().insert( "R", Imf::Channel( pixel_type ) );
    if( offsets[ 1 ] != -1 ) header.channels().insert( "G", Imf::Channel( pixel_type ) );
    if( offsets[ 2 ] != -1 ) header.channels().insert( "B", Imf::Channel( pixel_type ) );
    if( offsets[ 3 ] != -1 ) header.channels().insert( "A", Imf::Channel( pixel_type ) );

    Imf::OutputFile file( url.c_str(), header );

    Imf::FrameBuffer frameBuffer;

    char* image_data = (char*)image.getImageData();
    int image_width = image.width();
    int image_height = image.height();
    char* image_data_flipped = new char[ image_width*image_height*bytes_per_pixel ];

    for( int i = 0; i < image_width*bytes_per_pixel; ++i ) {
      for( int j = 0; j < image_height; ++j ) {
        image_data_flipped[ i + j * image_width*bytes_per_pixel ] =
          image_data[ i + (image_height - 1 - j)*bytes_per_pixel*image_width ];
      }
    }


    if( offsets[ 0 ] != -1 ) {
      frameBuffer.insert( 
        "R",                                                      // name
        Imf::Slice( 
          pixel_type,                                             // type
          image_data_flipped + offsets[ 0 ] * bytes_per_channel,  // base
          bytes_per_pixel,                                        // xStride
          bytes_per_pixel * image.width() ) );                    // yStride
    }

    if( offsets[ 1 ] != -1 ) {
      frameBuffer.insert( 
        "G",
        Imf::Slice(
          pixel_type,
          image_data_flipped + offsets[ 1 ] * bytes_per_channel,
          bytes_per_pixel,
          bytes_per_pixel * image.width() ) );
    }

    if( offsets[ 2 ] != -1 ) {
      frameBuffer.insert( 
        "B",
        Imf::Slice(
          pixel_type,
          image_data_flipped + offsets[ 2 ] * bytes_per_channel,
          bytes_per_pixel,
          bytes_per_pixel * image.width() ) );
    }

    if( offsets[ 3 ] != -1 ) {
      frameBuffer.insert( 
        "A",
        Imf::Slice( 
          pixel_type,
          image_data_flipped + offsets[ 3 ] * bytes_per_channel,
          bytes_per_pixel,
          bytes_per_pixel * image.width() ) );
    }

    file.setFrameBuffer( frameBuffer );
    file.writePixels( image.height() );
    delete[] image_data_flipped;
  } catch( const std::exception& e ) {
    Console( LogLevel::Error ) << e.what() << endl;
    return false;
  }

  return true;
}


H3DUTIL_API Image* H3DUtil::loadOpenEXRImage( const string &url ) {
  try {
    Imf::InputFile file( url.c_str() );
    // The InputFile class cause something that looks like memory leaks.
    // It is most probably just some static intialization that is caught a bit to early when exiting.
    // Either way the "memory leak" seems to not grow when exiting. It might for programs that frequently
    // load and unload H3DUtil but H3DUtil is not intended for that.
    // See http://lists.nongnu.org/archive/html/openexr-devel/2013-11/msg00003.html

    Imath::Box2i dw = file.header().dataWindow();

    int width = dw.max.x - dw.min.x + 1;
    int height = dw.max.y - dw.min.y + 1;

    Imf::FrameBuffer frameBuffer;

    const Imf::Channel* r = file.header().channels().findChannel( "R" );
    const Imf::Channel* g = file.header().channels().findChannel( "G" );
    const Imf::Channel* b = file.header().channels().findChannel( "B" );
    const Imf::Channel* a = file.header().channels().findChannel( "A" );

    Image::PixelType pixel_type;
    if( r && !g && !b && !a ) {
      pixel_type = Image::R;
    } else if( r && g && !b && !a ) {
      pixel_type = Image::RG;
    } else if( r && g && b && !a ) {
      pixel_type = Image::RGB;
    } else if( r && g && b && a ) {
      pixel_type = Image::RGBA;
    } else if( r && !g && !b && a ) {
      pixel_type = Image::LUMINANCE_ALPHA;
    } else {
      Console( LogLevel::Error ) << "Error: unsupported image channels!" << endl;
      return NULL;
    }

    int bytes_per_pixel = 0;

    if( r ) bytes_per_pixel += Imf::pixelTypeSize( r->type );
    if( g ) bytes_per_pixel += Imf::pixelTypeSize( g->type );
    if( b ) bytes_per_pixel += Imf::pixelTypeSize( b->type );
    if( a ) bytes_per_pixel += Imf::pixelTypeSize( a->type );

    char *data =
      new char[ width * height * bytes_per_pixel ];

    if( r ) {
      frameBuffer.insert( 
        "R",                            // name
        Imf::Slice( 
          r->type,                      // type
          data,                         // base
          bytes_per_pixel,              // xStride
          bytes_per_pixel * width ) );  // fillValue
    }
    if( g && r ) {
      frameBuffer.insert(
        "G",
        Imf::Slice(
          r->type,
          data + Imf::pixelTypeSize( r->type ),
          bytes_per_pixel,
          bytes_per_pixel * width ) );
    }
    if( b && g && r ) {
      frameBuffer.insert( "B",
        Imf::Slice(
          r->type,
          data + Imf::pixelTypeSize( r->type )
          + Imf::pixelTypeSize( g->type ),
          bytes_per_pixel,
          bytes_per_pixel * width ) );
    }
    if( a && b && g && r) {
      frameBuffer.insert( "A",
        Imf::Slice( 
          r->type,
          data + Imf::pixelTypeSize( r->type )
          + Imf::pixelTypeSize( g->type )
          + Imf::pixelTypeSize( b->type ),
          bytes_per_pixel,
          bytes_per_pixel * width ) );
    }
    file.setFrameBuffer( frameBuffer );
    file.readPixels( dw.min.y, dw.max.y );


    // openexr read data from top to bottom in y direction, 
    // need to flip the data in y direction
    char* data_flipped = new char[ width * height * bytes_per_pixel ];

    for( int i = 0; i < width*bytes_per_pixel; ++i ) {
      for( int j = 0; j < height; ++j ) {
        data_flipped[ i + j * width*bytes_per_pixel ] =
          data[ i + (height - 1 - j)*bytes_per_pixel*width ];
      }
    }
    delete[] data;
    return new PixelImage( 
      width,
      height,
      1,
      bytes_per_pixel * 8,
      pixel_type,
      Image::RATIONAL,
      (unsigned char*)data_flipped );

  } catch( const std::exception& e ) {
    Console( LogLevel::Error ) << e.what() << endl;
    return NULL;
  }
}
#endif

// Structures from DDS file format

struct DDSPixelFormat {
  int size;
  int flags;
  int fourCC;
  int rGBBitCount;
  int rBitMask, gBitMask, bBitMask;
  int rGBAlphaBitMask;
};

struct DDSCaps {
  int caps1;
  int caps2;
  int reserved[2];
};

struct DDSHeader {
  int size;
  int flags;
  int height;
  int width;
  int pitchOrLinearSize;
  int depth;
  int mipMapCount;
  int reserved1[11];
  DDSPixelFormat pixelFormat;
  DDSCaps caps;
  int reserved2;
};

struct DDSHeaderDX10 {
  int dxgiFormat;
  int resourceDimension;
  int miscFlag;
  int arraySize;
  int miscFlags2;
};

H3DUTIL_API Image* H3DUtil::loadDDSImage( const std::string &url ) {
  ifstream is( url.c_str(), ios::in | ios::binary );
  if( !is ) {
    Console( LogLevel::Error ) << "loadDDSImage(): Cannot open file " << url << endl;
    return NULL;
  }

  return loadDDSImage( is, url );
}

H3DUTIL_API Image* H3DUtil::loadDDSImage( std::istream &is, const std::string& url ) {
  // DDS contants

  // Magic number
  const unsigned int dds_magic_no = 0x20534444;

  // Flag indicating compression is used
  const unsigned int dds_fourcc = 0x00000004;

  // Various types of compression stored in fourcc flag
  const unsigned int dds_fourcc_dxt1 = 0x31545844;
  const unsigned int dds_fourcc_dxt3 = 0x33545844;
  const unsigned int dds_fourcc_dxt5 = 0x35545844;
  const unsigned int dds_fourcc_dx10 = 0x30315844;
  const unsigned int dds_fourcc_ati1 = 0x31495441;

  // Pixel format flags
  const unsigned int ddpf_alphapixels = 0x1;

  // Varius types of compression used in DX10 header
  const unsigned int dxgi_format_bc4_typeless = 79;
  const unsigned int dxgi_format_bc4_unorm = 80;
  const unsigned int dxgi_format_bc4_snorm = 81;

  const unsigned int dxgi_format_bc5_typeless = 82;
  const unsigned int dxgi_format_bc5_unorm = 83;
  const unsigned int dxgi_format_bc5_snorm = 84;

  const unsigned int dxgi_format_bc6h_typeless = 94;
  const unsigned int dxgi_format_bc6h_uf16 = 95;
  const unsigned int dxgi_format_bc6h_sf16 = 96;

  const unsigned int dxgi_format_bc7_typeless = 97;
  const unsigned int dxgi_format_bc7_unorm = 98;
  const unsigned int dxgi_format_bc7_unorm_srgb = 99;

  // Check magic number
  int magic_no;
  is.read( (char*)&magic_no, sizeof( magic_no ) );
  if( !is || magic_no != dds_magic_no ) {
    // Not a DDS file
    Console( LogLevel::Error ) << "loadDDSImage(): Not a DSS file " << url << endl;
    return NULL;
  }

  // Read header
  DDSHeader header;
  is.read( (char*)&header, sizeof( header ) );

  // Determine format
  Image::CompressionType type = Image::NO_COMPRESSION;
  Image::PixelType pixel_type = Image::RGBA;
  Image::PixelComponentType pixel_component_type = Image::UNSIGNED;
  int block_size = 0;

  if( header.pixelFormat.flags & dds_fourcc ) {
    switch( header.pixelFormat.fourCC ) {

    case dds_fourcc_dxt1:
      type = Image::BC1;
      if( header.pixelFormat.flags & ddpf_alphapixels ) {
        pixel_type = Image::RGBA;
      } else {
        pixel_type = Image::RGB;
      }
      pixel_component_type = Image::UNSIGNED;
      block_size = 8;
      break;

    case dds_fourcc_dxt3:
      type = Image::BC2;
      pixel_type = Image::RGBA;
      pixel_component_type = Image::UNSIGNED;
      block_size = 16;
      break;

    case dds_fourcc_dxt5:
      type = Image::BC3;
      pixel_type = Image::RGBA;
      pixel_component_type = Image::UNSIGNED;
      block_size = 16;
      break;

    case dds_fourcc_ati1:
      // BC4
      type = Image::BC4;
      pixel_type = Image::R;
      pixel_component_type = Image::UNSIGNED;
      block_size = 8;
      break;

    case dds_fourcc_dx10:
      // In this case we read the extra Dx10 header for more info
      DDSHeaderDX10 dx10_header;
      is.read( (char*)&dx10_header, sizeof( dx10_header ) );

      if(
        dx10_header.dxgiFormat == dxgi_format_bc4_typeless ||
        dx10_header.dxgiFormat == dxgi_format_bc4_unorm ) {
        // BC4
        type = Image::BC4;
        pixel_type = Image::R;
        pixel_component_type = Image::UNSIGNED;
        block_size = 8;

      } else if(
        dx10_header.dxgiFormat == dxgi_format_bc4_snorm ) {
        type = Image::BC4;
        pixel_type = Image::R;
        pixel_component_type = Image::SIGNED;
        block_size = 8;

      } else if(
        dx10_header.dxgiFormat == dxgi_format_bc5_typeless ||
        dx10_header.dxgiFormat == dxgi_format_bc5_unorm ) {
        // BC5
        type = Image::BC5;
        pixel_type = Image::RG;
        pixel_component_type = Image::UNSIGNED;
        block_size = 16;

      } else if(
        dx10_header.dxgiFormat == dxgi_format_bc5_snorm ) {
        type = Image::BC5;
        pixel_type = Image::RG;
        pixel_component_type = Image::SIGNED;
        block_size = 16;

      } else if(
        dx10_header.dxgiFormat == dxgi_format_bc6h_typeless ||
        dx10_header.dxgiFormat == dxgi_format_bc6h_uf16 ) {
        // BC6
        type = Image::BC6;
        pixel_type = Image::RGB;
        pixel_component_type = Image::RATIONAL_UNSIGNED;
        block_size = 16;

      } else if(
        dx10_header.dxgiFormat == dxgi_format_bc6h_sf16 ) {
        type = Image::BC6;
        pixel_type = Image::RGB;
        pixel_component_type = Image::RATIONAL;
        block_size = 16;

      } else if(
        dx10_header.dxgiFormat == dxgi_format_bc7_typeless ||
        dx10_header.dxgiFormat == dxgi_format_bc7_unorm ) {
        // BC7
        type = Image::BC7_RGB;
        pixel_type = Image::RGB;
        pixel_component_type = Image::RATIONAL_UNSIGNED;
        block_size = 16;

      } else if(
        dx10_header.dxgiFormat == dxgi_format_bc7_unorm_srgb ) {
        type = Image::BC7_SRGB;
        if( header.pixelFormat.flags & ddpf_alphapixels ) {
          pixel_type = Image::RGBA;
        } else {
          pixel_type = Image::RGB;
        }
        pixel_component_type = Image::RATIONAL;
        block_size = 16;

      } else {
        Console( LogLevel::Error )
          << "loadDDSImage(): Unhandled compressed DX10 format: " << dx10_header.dxgiFormat << " in " << url << endl;
      }
      break;
    default:

      char buf[5];
      buf[0] = header.pixelFormat.fourCC & 255;
      buf[1] = (header.pixelFormat.fourCC >> 8) & 255;
      buf[2] = (header.pixelFormat.fourCC >> 16) & 255;
      buf[3] = (header.pixelFormat.fourCC >> 24) & 255;
      buf[4] = 0;

      Console( LogLevel::Error )
        << "loadDDSImage(): Unhandled compressed format: " << buf << " (0x" << hex << header.pixelFormat.fourCC << ") in " << url << endl;
      return NULL;
    }
  } else {
    // Only compressed formats are handled now
    Console( LogLevel::Error ) << "Unhandled format in " << url << endl;
    return NULL;
  }

  // Read pixel data
  int size = ((header.width + 3) / 4)*((header.height + 3) / 4) * block_size;
  char* buffer = new char[size];
  is.read( buffer, size );

  int bits_per_pixel = int( 8 * (float( size ) / (header.width*header.height)) );

  return new PixelImage(
    header.width,
    header.height,
    1,
    bits_per_pixel,
    pixel_type,
    pixel_component_type,
    (unsigned char*)buffer,
    false, Vec3f( 0, 0, 0 ), type );
}
