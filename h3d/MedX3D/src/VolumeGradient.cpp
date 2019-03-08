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
/// \file VolumeGradient.cpp
/// \brief cpp file for the VolumeGradient class.
///
//
//////////////////////////////////////////////////////////////////////////////
#include <H3D/MedX3D/VolumeGradient.h>

#include <H3D/H3DTypes.h>
namespace H3D {
  
  namespace VolumeGradientInternals {
    
    // wrapper template image class
    template<class Type, unsigned int N>    
    class VolumeGradientImage {
    public:
      // the tuple at each pixel
      struct NTuple {
        Type tuple[N];
      };
      
      // the data
      NTuple *data;
      
      // the dimensions
      int W, H, D;
      
      // constructor
      VolumeGradientImage(int _W, int _H, int _D, void *_data) :
        W(_W), H(_H), D(_D), data((NTuple*) _data) {}
      
      // access operator
      Type& operator()(int x, int y, int z, int k=0) {
        return data[(z*H+y)*W+x].tuple[k];
      }
      
      // the intensity
      H3DFloat getIntensity(int x, int y, int z) {
        switch( N ) {
          // for 1 or 2 components we take the first as intensity
          // (LUMINANCE, LUMINANCE_ALPHA)
          case 1:
          case 2:
            return H3DFloat( data[(z*H + y)*W + x].tuple[0] );

          // for 3 or 4 components we take the sum R+G+B as intensity
          // (RGB, BGR, RGBA, BGRA)
          case 3:
          case 4:
            return 
            H3DFloat(data[(z*H + y)*W + x].tuple[0])+
            H3DFloat(data[(z*H + y)*W + x].tuple[1])+
            H3DFloat(data[(z*H + y)*W + x].tuple[2]);
          default:
            return 0.0f;
        }
      }
      
      // the intensity gradient using centered differences
      Vec3f getGradient(int x, int y, int z) {
        Vec3f g;
        g.x = getIntensity(x+1,y,z)-getIntensity(x-1,y,z);
        g.y = getIntensity(x,y+1,z)-getIntensity(x,y-1,z);
        g.z = getIntensity(x,y,z+1)-getIntensity(x,y,z-1);
        return g;
      }
      
    };
    
    // compute the gradients of image input
    template<class Type, unsigned int N> 
    Image* computeGradients(Image *input) {
      // allocate output image data (RGBA, 8*4=32 bits per pixel, UNSIGNED)
      int w = input->width();
      int h = input->height();
      int d = input->depth();
      unsigned char *data = new unsigned char[w*h*d*4];
      
      // store gradient magnitude for normalization
      H3DFloat *gradmag = new H3DFloat[w*h*d];
      
      // wrap to VolumeGradientImage
      VolumeGradientImage<Type, N>          in(w,h,d,input->getImageData());
      VolumeGradientImage<unsigned char, 4> out(w,h,d,data);
      VolumeGradientImage<H3DFloat, 1>      gm(w,h,d,gradmag);
      
      // compute gradients
      H3DFloat maxmag=0.0;
      for(int z=0; z<d; ++z) {
        for(int y=0; y<h; ++y) {
          for(int x=0; x<w; ++x) {
            // skip border
            if( x==0 || y==0 || z==0 || x==w-1 || y==h-1 || z==d-1 ) {
              out(x,y,z,0) = 0;
              out(x,y,z,1) = 0;
              out(x,y,z,2) = 0;
              out(x,y,z,3) = 0;
            } else {
              // gradient
              Vec3f g = -in.getGradient(x,y,z);

              // magnitude
              H3DFloat mag = g.length();
              maxmag = H3DMax(mag,maxmag);

              // normalize
              g.normalizeSafe();

              // convert from [-1,1] to [0,255] 
              g.x = 255.0f*0.5f*(g.x+1.0f);
              g.x = H3DMax(g.x, 0.0f); g.x = H3DMin(g.x, 255.0f); 
              g.y = 255.0f*0.5f*(g.y+1.0f);
              g.y = H3DMax(g.y, 0.0f); g.y = H3DMin(g.y, 255.0f); 
              g.z = 255.0f*0.5f*(g.z+1.0f);
              g.z = H3DMax(g.z, 0.0f); g.z = H3DMin(g.z, 255.0f);

              // set the RGB components
              out(x,y,z,0) = (unsigned char) ( g.x );
              out(x,y,z,1) = (unsigned char) ( g.y );
              out(x,y,z,2) = (unsigned char) ( g.z );

              // the magnitude
              gm(x,y,z) = mag;
            }
          }
        }
      }
      
      // normalize gradient magnitude so that maximum gradient magnitude
      // becomes 255 (1.0)
      if( maxmag > 0.0 ) {
        for(int z=0; z<d; ++z) {
          for(int y=0; y<h; ++y) {
            for(int x=0; x<w; ++x) {
              H3DFloat mag = 255.0f*gm(x,y,z)/maxmag;
              mag = H3DMax(mag, 0.0f); mag = H3DMin(mag, 255.0f);
              out(x,y,z,3) = (unsigned char) mag;
            }
          }
        }
      }
      
      delete [] gradmag;
      
      // create PixelImage from data and return
      PixelImage *output = new PixelImage( w, h, d,
                                           8*4, // RGBA
                                           Image::RGBA,
                                           Image::UNSIGNED,
                                           data,
                                           false,
                                           input->pixelSize());
      
      return output;
    }
  }
  
  void VolumeGradient::execute() {
    if( input==0 ) {
      Console(3) << __FUNCTION__ << " --- No input!" << endl;
      return;
    }
    
    if( output ) {
      delete output;
      output = 0;
    }
    
    // compute gradients
    switch( input->pixelType() ) {
      case Image::LUMINANCE: {
        switch( input->pixelComponentType() ) {
          case Image::UNSIGNED: 
            switch( input->bitsPerPixel() ) {
              case 8:  
                output = VolumeGradientInternals::
                           computeGradients<unsigned char, 1>(input);
                return;
              case 16: 
                output = VolumeGradientInternals::
                           computeGradients<unsigned short, 1>(input);
                return;
              case 32: 
                output = VolumeGradientInternals::
                           computeGradients<unsigned int, 1>(input);
                return;
              default: 
                Console(3) << "VolumeGradient: Unsupported pixel type" << endl;
                return;
            }
          case Image::SIGNED:
            switch( input->bitsPerPixel() ) {
              case 8:  
                output = VolumeGradientInternals::
                           computeGradients<char, 1>(input);
                return;
              case 16: 
                output = VolumeGradientInternals::
                           computeGradients<short, 1>(input);
                return;
              case 32: 
                output = VolumeGradientInternals::
                           computeGradients<int, 1>(input);
                return;
              default: 
                Console(3) << "VolumeGradient: Unsupported pixel type" << endl;
                return;
            }
          case Image::RATIONAL_UNSIGNED:
          case Image::RATIONAL:
            switch( input->bitsPerPixel() ) {
              case 32: 
                output = VolumeGradientInternals::
                           computeGradients<float, 1>(input);
                return;
              default: 
                Console(3) << "VolumeGradient: Unsupported pixel type" << endl;
                return;
            }
        }
      }
      case Image::LUMINANCE_ALPHA: {
        switch( input->pixelComponentType() ) {
          case Image::UNSIGNED:
            switch( input->bitsPerPixel() ) {
              case 16:  
                output = VolumeGradientInternals::
                           computeGradients<unsigned char, 2>(input);
                return;
              case 32: 
                output = VolumeGradientInternals::
                           computeGradients<unsigned short, 2>(input);
                return;
              case 64: 
                output = VolumeGradientInternals::
                           computeGradients<unsigned int, 2>(input);
                return;
              default: 
                Console(3) << "VolumeGradient: Unsupported pixel type" << endl;
                return;
            }
          case Image::SIGNED:
            switch( input->bitsPerPixel() ) {
              case 16:  
                output = VolumeGradientInternals::
                           computeGradients<char, 2>(input);
                return;
              case 32: 
                output = VolumeGradientInternals::
                           computeGradients<short, 2>(input);
                return;
              case 64: 
                output = VolumeGradientInternals::
                           computeGradients<int, 2>(input);
                return;
              default: 
                Console(3) << "VolumeGradient: Unsupported pixel type" << endl;
              return;
            }
          case Image::RATIONAL_UNSIGNED:
          case Image::RATIONAL:
            switch( input->bitsPerPixel() ) {
              case 64: 
                output = VolumeGradientInternals::
                           computeGradients<float, 2>(input);
                return;
              default: 
                Console(3) << "VolumeGradient: Unsupported pixel type" << endl;
                return;
            }
        }
      }
      case Image::RGB:
      case Image::BGR:
      {
        switch( input->pixelComponentType() ) {
          case Image::UNSIGNED: 
            switch( input->bitsPerPixel() ) {
              case 24:  
                output = VolumeGradientInternals::
                           computeGradients<unsigned char, 3>(input);
                return;
              case 48: 
                output = VolumeGradientInternals::
                           computeGradients<unsigned short, 3>(input);
                return;
              case 96: 
                output = VolumeGradientInternals::
                           computeGradients<unsigned int, 3>(input);
                return;
              default:
                Console(3) << "VolumeGradient: Unsupported pixel type" << endl;
                return;
            }
          case Image::SIGNED:
            switch( input->bitsPerPixel() ) {
              case 24:  
                output = VolumeGradientInternals::
                           computeGradients<char, 3>(input);
                return;
              case 48:
                output = VolumeGradientInternals::
                           computeGradients<short, 3>(input);
                return;
              case 96:
                output = VolumeGradientInternals::
                           computeGradients<int, 3>(input);
                return;
              default: 
                Console(3) << "VolumeGradient: Unsupported pixel type" << endl;
                return;
            }
          case Image::RATIONAL_UNSIGNED:
          case Image::RATIONAL:
            switch( input->bitsPerPixel() ) {
              case 96: 
                output = VolumeGradientInternals::
                           computeGradients<float, 3>(input);
                return;
              default: 
                Console(3) << "VolumeGradient: Unsupported pixel type" << endl;
                return;
            }
        }
      }
      case Image::RGBA:
      case Image::BGRA:
      {
        switch( input->pixelComponentType() ) {
          case Image::UNSIGNED: 
            switch( input->bitsPerPixel() ) {
              case 32:
                output = VolumeGradientInternals::
                           computeGradients<unsigned char, 4>(input);
                return;
              case 64:
                output = VolumeGradientInternals::
                           computeGradients<unsigned short, 4>(input);
                return;
              case 128:
                output = VolumeGradientInternals::
                           computeGradients<unsigned int, 4>(input);
                return;
              default: 
                Console(3) << "VolumeGradient: Unsupported pixel type" << endl;
                return;
            }

          case Image::SIGNED:
            switch( input->bitsPerPixel() ) {
              case 32:
                output = VolumeGradientInternals::
                           computeGradients<char, 4>(input);
                return;
              case 64:
                output = VolumeGradientInternals::
                           computeGradients<short, 4>(input);
                return;
              case 128:
                output = VolumeGradientInternals::
                           computeGradients<int, 4>(input);
                return;
              default:
                Console(3) << "VolumeGradient: Unsupported pixel type" << endl;
                return;
            }

          case Image::RATIONAL_UNSIGNED:
          case Image::RATIONAL:
            switch( input->bitsPerPixel() ) {
              case 128: 
                output = VolumeGradientInternals::
                           computeGradients<float, 4>(input);
                return;
              default: 
                Console(3) << "VolumeGradient: Unsupported pixel type" << endl;
              return;
            }
        }
      }
      default: 
        Console(3) << "VolumeGradient: Unsupported pixel type" << endl;
    }
  }
}
