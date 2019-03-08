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
/// \file ISOSurfaceVolumeStyle_FragmentShader.glsl
/// \brief shader file for ISOSurfaceVolumeStyle, not used by default.
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

// UNIFORMS - ISOSurfaceVolumeStyle
uniform float isovalue;
uniform sampler2D transferFunction;
uniform bool lighting;
uniform sampler3D surfaceNormals;
uniform bool shadows;

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
  // return color
  vec4 col=vec4(0,0,0,0), tfcol;
  
  float v, v1;
  vec3 r, r1;
  bool found_surf=false;
  while( r0.w>=0.0 && found_surf==false ) {
    r = r0.xyz;
    r1 = r0.xyz+dir.xyz;
    v = texture3D(voxels, r).r;
    v1 = texture3D(voxels, r1).r; 
    found_surf = (min(v,v1)<=isovalue && isovalue<=max(v,v1));
    r0 += dir;
  }
  
  // dicard this fragment if we did not find any surface
  if( !found_surf ) discard;
  
  // interpolate position
  float a = clamp( (isovalue-v)/(v1-v), 0.0, 1.0 );
  r = mix(r,r1,a);
    
  // lookup transfer function
  tfcol = texture2D(transferFunction,vec2(isovalue,0.5));
  
  // set the opacity
  col.a = tfcol.a;
  
  // lighting
  if( lighting ) {
    // sample position in view coordinates
    vec3 pos = (gl_ModelViewMatrix*textureMatrixInverse*
		vec4(r.xyz,1.0)).xyz;
    
    // lightsource position (only one lightsource so far)
    vec4 light_pos = gl_LightSource[0].position;
      
    // shadowing
    bool shdw=false;
    if( shadows ) {
      vec4 light_pos_tex = textureMatrix*gl_ModelViewMatrixInverse*light_pos;
      vec4 ldir = vec4( normalize(light_pos_tex.xyz-r.xyz), -1.0 );
      float lexit = rayexit(r.xyz, ldir.xyz);
      ldir *= 3.0*rayStep;
      vec4 lpos = vec4(r.xyz, lexit);
      lpos += ldir;
      
      while( lpos.w>=0.0 && shdw==false ) {
	float lv = texture3D(voxels, lpos.xyz).r;
	float lv1 = texture3D(voxels, lpos.xyz+ldir.xyz).r;
	shdw = (min(lv,lv1)<=isovalue && isovalue<=max(lv,lv1));
	lpos += ldir;
      }
    }
    
    // ambient
    col.rgb += vec3(.1,.1,.1);
    
    if( !shdw ) {
      // normal vector (0-0.5 -> -1.0-0.0 and 0.5-1.0 -> 0.0-1.0) 
      vec3 normal = 2.0*texture3D(surfaceNormals,r.xyz).xyz-1.0;
      normal = normalize(gl_NormalMatrix*vec3(normal.x,-normal.y,-normal.z));
      
      // view vector
      vec3 viewv = normalize(-pos);
      
      // re-orient normal
      if( dot(viewv,normal)<0.0 )
	normal = -normal;
      
      // lightsource vector
      vec3 lightv = normalize(light_pos.xyz-pos);
      float ndotl = dot(normal,lightv);
      
      // diffuse
      col.rgb +=  tfcol.rgb * max(ndotl,0.0);      
      
      // specular
      if (ndotl > 0.0) {
	vec3 reflv = normalize( reflect(-lightv,normal) );
	float rdotv = dot(reflv,viewv);
	col.rgb += vec3(.6,.6,.6) * pow(max(rdotv,0.0), 50.0);
      }
    } // shdw
    
  }
  
  // return result 
  return RayResult( col, vec4(r.xyz,1.0) );
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

