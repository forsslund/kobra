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
/// \file X3DVolumeRenderStyleNode_FS.glsl
/// \brief This is the fragment shader program for volume rendering with glsl
/// shaders in the MedX3D package.
///
/// The different render styles add code to this shader program through the
/// functions addUniforms and addShaderCode. This code is not used by default.
//
//////////////////////////////////////////////////////////////////////////////

//BEGIN UNIFORMS
//END UNIFORMS

//BEGIN VARYINGS

varying vec3 rayStart;
varying vec3 rayDir;
//END VARYINGS

//BEGIN MISC FUNCTIONS
// compute t such that r0+t*dir is the exit point of the [0,1] box
float rayexit(vec3 r0, vec3 dir) {
  vec3 q = 1.0/(dir+1.0e-6);
  vec3 t0 = -r0*q;
  vec3 tmax = max(t0,q+t0);
  return min(tmax.x,min(tmax.y,tmax.z));
}

// Get the normal from a 3D texture where, for RGB,
// the interval [0,0.5] means [-1.0,0] and [0.5,1.0] means [0.0,1.0].
// The A component contain the normalized gradient magnitude.
vec4 normalFromTexture(sampler3D normals, vec4 pos) {
  vec4 n = texture3D(normals, pos.xyz);
  n.xyz = 2.0*n.xyz-1.0;
  return n;
}
vec4 normalizedNormalFromTexture(sampler3D normals, vec4 pos) {
  vec4 n = texture3D(normals, pos.xyz);
  n.xyz = 2.0*n.xyz-1.0;
  if( length(n.xyz) > 0.001 )
    n.xyz = normalize(n.xyz);
  return n;
}

// convert from rgb to hsv 
vec4 RGBToHSV( vec4 rgba ) { 
  vec4 return_value; 
  float v, x, f, i; 
  float R = rgba.r, G = rgba.g, B = rgba.b; 
  x = min(R, min( G, B ) ); 
  v = max(R, max( G, B ) ); 
  if(v == x) 
    return_value = vec4(0, 0, v, rgba.a); 
  else { 
    f = (R == x) ? G - B : ((G == x) ? B - R : R - G); 
    i = (R == x) ? 3.0 : ((G == x) ? 5.0 : 1.0); 
    return_value = vec4(i - f /(v - x), (v - x)/v, v, rgba.a); 
  } 
  return return_value; 
}

vec3 RGBToHSV( vec3 rgb ) {
  return RGBToHSV( vec4(rgb,1.0) ).rgb;
}

// convert from hsv to rgb 
vec4 HSVToRGB( vec4 hsva ) { 
  vec4 return_value; 
  float h = hsva.x, s = hsva.y, v = hsva.z, m, n, f; 
  float i;   
  if( h == 0.0 ) 
    return_value = vec4(v, v, v, hsva.a); 
  else { 
    i = floor(h); 
    f = h - i; 
    float t = i / 2.0; 
    if( t - floor( t ) >  0.1 ) 
      f = 1.0 - f; // if i is even 
    m = v * (1.0 - s); 
    n = v * (1.0 - s * f); 
    if( i == 6.0 || i == 0.0 ) return_value = vec4(v, n, m, hsva.a); 
    else if( i == 1.0 ) return_value = vec4(n, v, m, hsva.a); 
    else if( i == 2.0 ) return_value = vec4(m, v, n, hsva.a); 
    else if( i == 3.0 ) return_value = vec4(m, n, v, hsva.a); 
    else if( i == 4.0 ) return_value = vec4(n, m, v, hsva.a); 
    else if( i == 5.0 ) return_value = vec4(v, m, n, hsva.a); 
    // should never happen 
    else return_value = vec4( 0, 0, 0, 1 ); 
  }
  return return_value; 
}

vec3 HSVToRGB( vec3 hsv ) {
  return HSVToRGB( vec4(hsv,1.0) ).rgb;
}
//END MISC FUNCTIONS

//BEGIN STYLE FUNCTIONS
// OpacityMapVolumeStyle
// Notes:
void OpacityMapVolumeStyle2D(inout vec4 sample_color,
           vec4 orig_color,
           vec4 pos,
           bool enabled,
           int nr_components,
           sampler2D transferFunction) {
  if( enabled ) {
    vec2 tex_coord = vec2( 0.0, 0.0 );
    if( nr_components == 1 ) {
      tex_coord.s = orig_color.r;
    } else if( nr_components == 2 ) {
      tex_coord.s = orig_color.r;
      tex_coord.t = orig_color.a;
    } else {
      tex_coord.s = orig_color.r;
      tex_coord.t = orig_color.g;
    }           
    sample_color = texture2D(transferFunction, tex_coord );
  }
}

void OpacityMapVolumeStyle3D(inout vec4 sample_color,
           vec4 orig_color,
           vec4 pos,
           bool enabled,
            int nr_components,
           sampler3D transferFunction) {


  if( enabled ) {
    vec3 tex_coord = vec3( 0.0, 0.0, 0.0 );  
   
    if( nr_components == 1 ) {
      tex_coord.s = orig_color.r;
    } else if( nr_components == 2 ) {
      tex_coord.s = orig_color.r;
      tex_coord.t = orig_color.a;
    } else {
      tex_coord.s = orig_color.r;
      tex_coord.t = orig_color.g;
      tex_coord.r = orig_color.b;
    } 

    sample_color = texture3D(transferFunction, tex_coord);
  }
}



// EdgeEnhancementVolumeStyle
// Notes: 
void EdgeEnhancementVolumeStyle(inout vec4 current_color,
        vec4 pos,
        vec4 viewdir,
        bool enabled,
        vec3 edgeColor,
        float gradientThreshold,
        vec4 normal) {
  if( enabled ) {
    if( normal.a > 0.001 ) {
      //float cosgt = 1.0-cos(gradientThreshold);
      //float cosnv = 1.0-abs(dot(normal.xyz,viewdir.xyz));
      //current_color += contribution*cosnv*step(cosgt,cosnv)*edgeColor;
      
      // according to updated spec.
      // Note: In the spec it says that the gradientThreshold
      // is the minimum angle, but the formula compares the dot product,
      // i.e., cosine of the angle.
      float nv = abs( dot( normal.xyz, viewdir.xyz ) );
      if( nv < cos(gradientThreshold) ) {
        current_color.rgb = mix(current_color.rgb, edgeColor, 1.0-nv);
      }
    }
  }
}

// BoundaryEnhancementVolumeStyle
void BoundaryEnhancementVolumeStyle(inout vec4 current_color,
            vec4 pos,
            bool enabled,
            float retainedOpacity,
            float boundaryOpacity,
            float opacityFactor,
            vec4 normal ) {
  if( enabled ) {
    current_color.a *= 
      (retainedOpacity + boundaryOpacity*pow(normal.a,opacityFactor));
  }
 
  
}

// SilhouetteEnhancementVolumeStyle
// Notes: In the updated spec, the text is wrong. silhouetteFactor
// is not a member of this style 
void SilhouetteEnhancementVolumeStyle(inout vec4 current_color,
              vec4 pos,
              vec4 viewdir,
              bool enabled,
              float silhouetteBoundaryOpacity,
              float silhouetteRetainedOpacity,
              float silhouetteSharpness,
              vec4 normal ) {
  if( enabled ) {
    if( normal.a > 0.001 ) {
      float a = 1.0-abs(dot(normal.xyz,viewdir.xyz));
      float b = 
  silhouetteRetainedOpacity + 
  silhouetteBoundaryOpacity*pow(a,silhouetteSharpness);
      current_color.a *= b;
    } else {
      current_color.a = 0.0;
    }
  }
}

// ToneMappedVolumeStyle
void ToneMappedVolumeStyle(inout vec4 current_color,
         vec4 pos,
         mat4 view_to_tex,
         bool enabled,
         vec4 coolColor,
         vec4 warmColor,
         vec4 normal) {
  if( enabled ) {
    if( normal.a > 0.001 ) {
      current_color.rgb = vec3( 0, 0, 0 );
      for( int i = 0; i < nr_enabled_lights; i++ ) {
  getEnabledLight( light, i );
  vec4 lightpos = view_to_tex * light.position;
  vec3 L;
  if(lightpos.w == 0.0 ) L = normalize(lightpos.xyz);
  else L =  normalize(lightpos.xyz-pos.xyz);

  // according to X3D spec
  float cc = (1.0+dot(L,normal.xyz))*0.5;
  current_color.rgb += mix(coolColor.rgb, warmColor.rgb, cc); 
      }
    } else {
      current_color.rgb = mix(coolColor.rgb, warmColor.rgb, 0.5);
      current_color.a = 0.0;
    }
  }    
}

// CartoonVolumeStyle
// Notes:
void CartoonVolumeStyle(inout vec4 current_color,
      vec4 pos,
      vec4 viewdir,
      bool enabled,
      vec3 parallelColorHSV,
      vec3 orthogonalColorHSV,
      int colorSteps,
      vec4 normal ) {
  
  if( enabled && colorSteps >= 1 ) {
    if( normal.a > 0.001 ) {
      float cos_a = dot( normal.xyz, viewdir.xyz );
      if( cos_a < 0.0 ) {
  current_color.rgb = vec3( 0.0, 0.0, 0.0 );
      } else {
  float step = 1.0 / float(colorSteps);
  float interval = floor( cos_a / step );
  if( interval >= float(colorSteps) )
    interval = float(colorSteps) - 1.0;
  float w = interval * step;
  current_color.rgb = 
    HSVToRGB( orthogonalColorHSV * w + parallelColorHSV * (1.0-w) );
  // 2 -> 0-0.5-1           0.5
  // 3 -> 0-0.33-0.66-1     0.33
  // 4 -> 0-0.25-0.5-0.75-1 0.25
      }
    } else {
      current_color.a = 0.0;
    }
  }
  
}

// MIPVolumeStyle
// Notes:
void MIPVolumeStyle(inout float maxvalue,
        inout vec4 maxpos,
        vec4 pos,
        bool enabled) {
  
  if( enabled ) {
    vec4 col = texture3D(voxels, pos.xyz);
    float val = max( col.r, max(col.g,col.b) );
    if( val > maxvalue ) {
      maxvalue = val;
      maxpos = vec4(pos.xyz,1.0);
    }
  }
  
}
void MIPVolumeStyle_POST(inout vec4 finalcolor,
       inout vec4 finalpos,
       float maxvalue,
       vec4 maxpos,
       bool enabled,
       sampler2D transferFunction) {
  
  if( enabled ) {
    finalcolor = texture2D(transferFunction, vec2(maxvalue,0.5));
    finalpos = maxpos;
  }
  
}

vec4 PhongLightingModel( vec3 normal,
       vec3 viewdir, 
       vec3 lightdir,
       gl_LightSourceParameters light ) {

  // ambient
  vec3 ambient = light.ambient.rgb * gl_FrontMaterial.ambient.rgb;

  float ndotl = dot(normal, lightdir);
    
  // diffuse
  vec3 diffuse = 
    light.diffuse.rgb * gl_FrontMaterial.diffuse.rgb * max(ndotl,0.0);
    
  vec3 specular = vec3(0,0,0);
  // specular
  /*  if (ndotl > 0.0) {
    vec3 reflv = normalize( reflect(-lightdir,normal) );
    float rdotv = dot(reflv,viewdir);
    if( rdotv > 0.0 ) {
      specular = light.specular.rgb*gl_FrontMaterial.specular.rgb * pow( rdotv, gl_FrontMaterial.shininess);
    }
  }
  */

  vec4 return_color;  
  return_color.rgb = gl_FrontMaterial.emission.rgb + ambient + diffuse + specular; 
  return_color.a = gl_FrontMaterial.diffuse.a;

  return return_color;
}

void ShadedVolumeStyle(inout vec4 current_color,
           vec4 pos,
           vec4 viewdir,
           mat4 view_to_tex,
           bool enabled,
           bool shadows,
           bool lighting,
           vec4 normal) {
  if( enabled ) {
    if(lighting) {  
      // re-orient normal
      if( dot(viewdir.xyz,normal.xyz)<0.0 )
        normal.xyz = -normal.xyz;

      if( normal.a > 0.001 ) {
  current_color.rgb = gl_FrontMaterial.emission.rgb;
        for( int i = 0; i < nr_enabled_lights; i++ ) {
    getEnabledLight( light, i );
    vec4 lightpos = view_to_tex * light.position;
    vec3 L;
    if(lightpos.w == 0.0 ) L = normalize(lightpos.xyz);
    else L =  normalize(lightpos.xyz-pos.xyz);

    current_color.rgb += PhongLightingModel( normal.xyz,
               viewdir.xyz,
               L,
               light ).rgb;

  }
  current_color.a *= gl_FrontMaterial.diffuse.a;
      } else {
  current_color = vec4( 0, 0, 0, 0 );
      }
    } else {
      current_color.rgb = gl_FrontMaterial.diffuse.rgb;
      current_color.a *= gl_FrontMaterial.diffuse.a;
    }    
    }
}


// ISOSurfaceVolumeStyle
// Notes:
void ISOSurfaceVolumeStyle(inout vec4 current_color,
         vec4 dir,
         vec3 r,
         vec3 r1,
         float v,
         float v1,
         vec4 viewdir,
         vec4 lightpos,
         float isovalue,
         bool shadows,
         sampler3D surfaceNormals) {

      // interpolate position
      float a = clamp( (isovalue-v)/(v1-v), 0.0, 1.0 );
      r = mix(r,r1,a);
      
      // lookup transfer function
      vec4 tfcol = vec4(0.5, 0.5, 0.5, 1 ); //texture2D(transferFunction,vec2(isovalue,0.5));
      
      // set the opacity
      current_color.a = tfcol.a;
      
      // lighting
      if( true /*lighting*/ ) {
  vec4 ldir = vec4( normalize(lightpos.xyz-r.xyz), 1.0 );
  
  // shadowing
  bool shdw=false;
  if( shadows ) {
    float lexit = rayexit(r.xyz, ldir.xyz);
    vec4 ldir2 = 3.0*dir.w*ldir;
    vec4 lpos = vec4(r.xyz, lexit);
    lpos += ldir2;
    
    while( lpos.w>=0.0 && shdw==false ) {
      float lv = texture3D(voxels, lpos.xyz).r;
      float lv1 = texture3D(voxels, lpos.xyz+ldir2.xyz).r;
      shdw = (min(lv,lv1)<=isovalue && isovalue<=max(lv,lv1));
      lpos += ldir2;
    }
  }
  
  if( !shdw ) {
    vec4 normal = 
      normalizedNormalFromTexture(surfaceNormals,vec4(r,1.0));
    // re-orient normal
    if( dot(viewdir.xyz,normal.xyz)<0.0 )
      normal.xyz = -normal.xyz;

    vec3 L;
    if(lightpos.w == 0.0 ) L = normalize(lightpos.xyz);
    else L = ldir.xyz;

    current_color = PhongLightingModel( normal.xyz,
                viewdir.xyz,
                L,
                gl_LightSource[0]);
  } // shdw
  
      } // lighting 
      
}
//END STYLE FUNCTIONS

// --------------------------------------------------------
// Here follows the raycasting engine and the main function
// --------------------------------------------------------
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
  //useful stuff (hopefully not computed if not needed...?)
  mat4 view_to_tex = textureMatrix*gl_ModelViewMatrixInverse;
  mat4 tex_to_view = gl_ModelViewMatrix*textureMatrixInverse;
  vec4 viewdir_tex = vec4( -normalize(dir.xyz), 0.0 );
  
  // return value
  RayResult rr;
  
  // initial color of this fragment is black with zero alpha
  rr.color = vec4(0.0, 0.0, 0.0, 0.0);
  // initial depth is not defined
  rr.zpoint = vec4(-1.0, 0.0, 0.0, 0.0);
  
  //BEGIN PRE-LOOP
  //END PRE-LOOP
  
  // compositing loop
  while( rr.color.a<0.95 && r0.w>=0.0 ) {
         
    vec4 orig_sample_color = texture3D(voxels, r0.xyz);

    //ORIG_SAMPLE_MANIP

    // color of this sample
    vec4 sample_color = orig_sample_color;  
    
    //BEGIN INSIDE-LOOP
    //END INSIDE-LOOP
    
    //BEGIN COMPOSITING
    //END COMPOSITING
    
    // step forward along ray
    r0 += dir;
  }
  
  //BEGIN POST-LOOP
  //END POST-LOOP

  if( rr.color.a == 0.0 ) discard;
  
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
  /*
  // set depth if computed
  if( rr.zpoint.x>=0.0 ) {
    // set depth
    vec4 tmp =
      gl_ModelViewProjectionMatrix*textureMatrixInverse*rr.zpoint;
    // in window coordinates
    gl_FragDepth = 0.5*(gl_DepthRange.diff*tmp.z/tmp.w +
      gl_DepthRange.near+gl_DepthRange.far);
  } else {
    gl_FragDepth = gl_FragCoord.z;
  }*/
  ;
}
