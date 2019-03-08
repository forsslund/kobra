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
/// \file OpacityMapVolumeStyle_FragmentShader.glsl
/// \brief shader file for OpacityMapVolumeStyle, not used by default.
///
//
//
//////////////////////////////////////////////////////////////////////////////
#define eps 1.0e-6

// UNIFORMS - X3DVolumeRenderStyleNode
uniform sampler3D voxels;
uniform mat4 textureMatrix;
uniform mat4 textureMatrixInverse;
uniform float rayStep;

// UNIFORMS - OpacityMapVolumeStyle
uniform sampler2D transferFunction;

// VARYINGS - from X3DRenderStyleNode_VertexShader
varying vec3 rayStart;
varying vec3 rayDir;

// FUNCTIONS
// compute t such that r0+t*dir is the exit point of the [0,1] box
float rayexit(vec3 r0, vec3 dir) {
  vec3 q = 1.0/(dir+eps);
  vec3 t0 = -r0*q;
  vec3 tmax = max(t0,q+t0);
  return min(tmax.x,min(tmax.y,tmax.z));
}

// struct to return from traverseRay, 
// the resulting color and the point to use for depth calculation 
struct RayResult {
  vec4 color;
  vec4 zpoint;
};

// traverse ray
// Inputs:
//   r0  - ray start (xyz), ray exit time (w)
//   dir - ray direction (xyz), step length (-w)
// Output:
//   RayResult.color  - the computed RGBA color for this fragment
//   RayResult.zpoint - the point to use for depth calculations,
//                      if zpoint.x<0, no depth is computed
RayResult traverseRay(vec4 r0, vec4 dir) {
  // return value
  RayResult rr;
  
  // initial color of this fragment is black with zero alpha
  rr.color = vec4(0.0, 0.0, 0.0, 0.0);
  
  // the depth is fragment depth
  rr.zpoint = vec4(-1.0, 0.0, 0.0, 0.0);
  
  // compositing loop
  while( rr.color.a<0.95 && r0.w>=0.0 ) {
    // obtain sample value and look up color and opacity in transfer function
    float val = texture3D(voxels, r0.xyz).r;
    vec4 sample = texture2D(transferFunction, vec2(val,0.5));
    
    // front-to-back compositing
    sample.rgb *= sample.a; // opacity-weighted color or associated color        
    rr.color += (1.0-rr.color.a)*sample;
    
    // step forward along ray
    r0 += dir;
  }
  
  // return result
  return rr;
}

// main function
void main() {
  // initialize ray
  vec4 raydir = vec4(normalize(rayDir), -1.0);
  float texit = rayexit(rayStart,raydir.xyz);
  raydir = rayStep*raydir; 
  vec4 raypos = vec4(rayStart, texit);  
  
  // traverse ray
  RayResult rr = traverseRay(raypos, raydir);
  
  // set color
  gl_FragColor = rr.color;
  
  // set depth if computed
  if( rr.zpoint.x>=0.0 ) {
    // set depth
    vec4 tmp =
      gl_ModelViewProjectionMatrix*textureMatrixInverse*rr.zpoint;
    // in window coordinates
    gl_FragDepth = 0.5*(gl_DepthRange.diff*tmp.z/tmp.w +
			gl_DepthRange.near+gl_DepthRange.far);
  }
}

