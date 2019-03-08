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
/// \file ToneMappedVolumeStyle_FragmentShader.glsl
/// \brief shader file for ToneMappedVolumeStyle, not used by default.
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

// UNIFORMS - ToneMappedVolumeStyle
uniform sampler3D surfaceNormals;
uniform vec4 coolColor;
uniform vec4 warmColor;

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

  // the depth is entry point
  rr.zpoint = vec4(r0.xyz,1.0);
  
  mat4 tex_to_view =  gl_ModelViewMatrix*textureMatrixInverse;

  // compositing loop
  while( rr.color.a<0.95 && r0.w>=0.0 ) {

    // values
    float sample = texture3D(voxels, r0.xyz).r;

    // the normal at the sample point  
    // normal vector (0-0.5 -> -1.0-0.0 and 0.5-1.0 -> 0.0-1.0) 
    vec3 normal = 2.0*texture3D(surfaceNormals,r0.xyz).xyz-1.0;
    normal.y = -normal.y;		
    normal.z = -normal.z;     
    normal = normalize( gl_NormalMatrix * normal );     
    //normal = normalize( normal );
    
    // sample position in view coordinates
    vec3 sample_pos = (tex_to_view * vec4(r0.xyz,1.0)).xyz;

    vec4 L0_pos = gl_LightSource[0].position;
    vec3 Li = normalize(L0_pos.xyz-sample_pos); 
    
    // formula according to X3D-spec
    float cc = (1.0+dot(Li,normal)) * 0.5;
    vec4 Cg = cc * warmColor + (1.0-cc) * coolColor;

    vec4 src = vec4( Cg.xyz, sample );
    src.rgb *= src.a;
    rr.color += (1.0-rr.color.a)*src;

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

