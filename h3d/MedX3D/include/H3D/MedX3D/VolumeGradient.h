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
/// \file VolumeGradient.h
/// \brief Header file for the VolumeGradient class.
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __VOLUMEGRADIENT_H__
#define __VOLUMEGRADIENT_H__

#include <H3D/MedX3D/MedX3D.h>

#include <H3DUtil/PixelImage.h>
#include <H3D/H3DTypes.h>

namespace H3D {

  /// \ingroup MedX3DClasses
  /// \class VolumeGradient
  /// Computes the gradient of the input volume image and stores 
  /// it as RGBA components in the output image. The RGB components contain
  /// the *negative* gradient vector such that the interval
  /// [0,0.5] means [-1.0,0] and [0.5,1.0] means [0.0,1.0].
  /// The A component contain the magnitude of the gradient 
  /// normalized to [0,1]. The normal is defined in *texture coordinates*.
  ///
  /// GLSL code To obtain the interpolated unit normal in volume coordinate r0:
  ///
  ///   vec4 normal = texture3D(surfaceNormals,r0.xyz);
  ///   normal.xyz = 2.0*normal.xyz-1.0;
  ///   // normalize if needed, remember to check length
  ///   if( length(normal.xyz) > 0.001 )
  ///     normal.xyz = normalize(normal.xyz);
  /// 
  ///   // view normal
  ///   vec3 viewnormal = gl_NormalMatrix*vec3(normal.x,-normal.y,-normal.z);
  /// 
  class MEDX3D_API VolumeGradient {
  public:
    VolumeGradient() :
      input(0),
      output(0),
      output_owner(true)
    {}
    
    ~VolumeGradient() {
      if( output_owner )
        delete output;
    }
    
    /// Set input image
    void setInput(Image *_input) {
      input = _input;
    }
    
    /// Set output image
    void setOutput(Image *_output) {
      output = _output;
      output_owner = false;
    }
    
    /// Get output image
    Image* getOutput(bool release=true) {
      output_owner = !release;
      return output;
    }
    
    /// Execute computation
    void execute();
    
  protected:
    Image *input;
    Image *output;
    bool output_owner;
  };
  
}
  
#endif
