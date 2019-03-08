//////////////////////////////////////////////////////////////////////////////
//    Copyright 2012-2019, SenseGraphics AB
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
//////////////////////////////////////////////////////////////////////////////
//
/// \file StyleFunctions.glsl
/// \brief This is the fragment shader program for volume rendering with glsl
/// shaders in the MedX3D package.
///
/// The different render styles add code to this shader program through the
/// functions addUniforms and addShaderCode. NOTE: This code can be used by
/// changing a few lines in X3DVolumeNode.cpp in order for faster development
/// cycles of how the shading part of MedX3D is done.
//
//////////////////////////////////////////////////////////////////////////////

//BEGIN UNIFORMS
//END UNIFORMS

// compute t such that r0+t*dir is the exit point of the [0,1] box
// t is returned
float rayexitOrig(vec3 r0, vec3 dir) {
  // cube side length is 1 in texture coordinates 
  // q is the number of ray steps in each component to
  // go a full side. 
  vec3 q = 1.0/(dir+1.0e-6);

  // r0 [0-1] 
  // Since the texture coordinates of the cube side is 0-1, r0 will be
  // the fraction of the cube side
  // t0 is the number of steps required for each component to reach
  // 0 from r0  
  vec3 t0 = -r0*q;

  // if t0 is negative in a component, dir does not go towards 0
  // in that case we want t in that component to be the direction
  // towards 1 instead.
  vec3 tmax = max(t0,q+t0);
 
  // choose t to be the component that requires the least number of
  // steps to reach the cube boundary.
  return min(tmax.x,min(tmax.y,tmax.z));
}

//BEGIN MISC FUNCTIONS
// compute t such that r0+t*dir is the exit point of the [0,1] box
float rayexit(vec3 r0, vec3 dir) {
  // this works just as rayexitOrig with the difference that we first
  // transform the texture space with the texture matrix.
  vec3 r0_local = (gl_TextureMatrixInverse[0] * vec4( r0, 1 )).xyz;
  vec3 dir_local = (gl_TextureMatrixInverse[0] * vec4( dir, 0 )).xyz;

  vec3 q = 1.0/(dir_local+1.0e-6);
  //  q.x = 0.5 / (dir.x+1.0e-6);
  vec3 t0 = -r0_local*q;
  //  t0.x = -r0.x/0.5 *q.x;
  vec3 tmax = max(t0,q+t0);
  float t_local = min(tmax.x,min(tmax.y,tmax.z));

  //  return length((textureMatrix * vec4( t_local, 0, 0, 0 )).xyz);
  return t_local;
}

/// tex_coord - texture coordinate to evaluate color at. 
/// tex_source - the source texture 
/// tex_hg - filter offsets and weights 
/// e_x - texel size in x direction 
/// e_y - texel size in y direction 
/// e_z - texel size in z direction 
/// size_source - source texture size in pixels 
vec4 filterTriCubic( vec3 tex_coord, 
                     sampler3D tex_source, 
                     sampler2D tex_hg, 
                     vec3 e_x, 
                     vec3 e_y, 
                     vec3 e_z, 
                     vec3 size_source ) { 
  // calculate filter texture coordinates where [0,1] is a single texel. 
  vec3 coord_hg = tex_coord * size_source - vec3( 0.5, 0.5, 0.5 ); 
 
  // fetch offsets and weights from filter texture 
  vec3 hg_x = texture2D( tex_hg, vec2(coord_hg.s, 0.5) ).xyz; 
  vec3 hg_y = texture2D( tex_hg, vec2(coord_hg.t, 0.5) ).xyz; 
  vec3 hg_z = texture2D( tex_hg, vec2(coord_hg.r, 0.5) ).xyz; 
 
  // determine linear sampling coordinates 
  vec3 t000 = tex_coord - hg_x.y * e_x - hg_y.y * e_y - hg_z.y * e_z; 
  vec3 t100 = tex_coord + hg_x.x * e_x - hg_y.y * e_y - hg_z.y * e_z; 
  vec3 t010 = tex_coord - hg_x.y * e_x + hg_y.x * e_y - hg_z.y * e_z; 
  vec3 t110 = tex_coord + hg_x.x * e_x + hg_y.x * e_y - hg_z.y * e_z; 
 
  vec3 t001 = tex_coord - hg_x.y * e_x - hg_y.y * e_y + hg_z.x * e_z; 
  vec3 t101 = tex_coord + hg_x.x * e_x - hg_y.y * e_y + hg_z.x * e_z; 
  vec3 t011 = tex_coord - hg_x.y * e_x + hg_y.x * e_y + hg_z.x * e_z; 
  vec3 t111 = tex_coord + hg_x.x * e_x + hg_y.x * e_y + hg_z.x * e_z; 
 
  // fetch linearly interpolated inputs. 
  vec4 c000 = texture3D( tex_source, t000 ); 
  vec4 c100 = texture3D( tex_source, t100 ); 
  vec4 c010 = texture3D( tex_source, t010 ); 
  vec4 c110 = texture3D( tex_source, t110 ); 
  vec4 c001 = texture3D( tex_source, t001 ); 
  vec4 c101 = texture3D( tex_source, t101 ); 
  vec4 c011 = texture3D( tex_source, t011 ); 
  vec4 c111 = texture3D( tex_source, t111 ); 
 
  // weight along z direction 
  vec4 c00 = mix( c000, c001, hg_z.z ); 
  vec4 c10 = mix( c100, c101, hg_z.z ); 
  vec4 c01 = mix( c010, c011, hg_z.z ); 
  vec4 c11 = mix( c110, c111, hg_z.z ); 
 
  // weight along y direction 
  vec4 c0 = mix( c00, c01, hg_y.z ); 
  vec4 c1 = mix( c10, c11, hg_y.z ); 
 
  // weight along x direction 
  vec4 c = mix( c0, c1, hg_x.z ); 
 
  return c; 
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
void OpacityMapVolumeStyle2D( inout vec4 sample_color,
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

void OpacityMapVolumeStyle3D( inout vec4 sample_color,
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

void OpacityMapVolumeStylePreIntegrated(inout vec4 sample_color, 
                                        vec4 orig_color, 
                                        vec4 pos, 
                                        bool enabled, 
                                        vec4 last_step_orig_color, 
                                        sampler2D preintegrated_values ) { 

  if( enabled ) { 
    vec2 tex_coord = vec2( last_step_orig_color.r, orig_color.r );  
    sample_color = texture2D(preintegrated_values, tex_coord); 
  }
}


// EdgeEnhancementVolumeStyle
// Notes: 
void EdgeEnhancementVolumeStyle( inout vec4 current_color,
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
void BoundaryEnhancementVolumeStyle( inout vec4 current_color,
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
void SilhouetteEnhancementVolumeStyle( inout vec4 current_color,
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
void ToneMappedVolumeStyle( inout vec4 current_color,
                            vec4 pos,
                            mat4 view_to_tex,
                            bool enabled,
                            vec4 coolColor,
                            vec4 warmColor,
                            vec4 normal) {
  if( enabled ) {
    if( normal.a > 0.001 ) {
      current_color.rgb = vec3( 0, 0, 0 );
      for( int i = 0; i < nr_enabled_lights; ++i ) {
        getEnabledLight( lightpos, light_diffuse,
                         light_ambient, light_specular, i );
        lightpos = view_to_tex * lightpos;
        vec3 L;
        if(lightpos.w == 0.0 ) L = normalize(lightpos.xyz);
        else L =  normalize(lightpos.xyz-pos.xyz);

        // according to X3D spec
        float cc = (1.0+dot(L,normal.xyz))*0.5;
        current_color.rgb += mix(coolColor.rgb, warmColor.rgb, cc); 
      }
    } else {
      current_color = mix(coolColor, warmColor, 0.5);
      //current_color.rgb = mix(coolColor.rgb, warmColor.rgb, 0.5);
      //current_color.a = 0.0;
    }
  }
}

// CartoonVolumeStyle
// Notes:
void CartoonVolumeStyle( inout vec4 current_color,
                         vec4 pos,
                         vec4 viewdir,
                         bool enabled,
                         vec4 parallelColorHSV,
                         vec4 orthogonalColorHSV,
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
        HSVToRGB( orthogonalColorHSV * w + parallelColorHSV * (1.0-w) ).rgb;
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
/*void MIPVolumeStyle( inout float maxvalue,
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
void MIPVolumeStyle_POST( inout vec4 finalcolor,
                          inout vec4 finalpos,
                          float maxvalue,
                          vec4 maxpos,
                          bool enabled,
                          sampler2D transferFunction) {
  
  if( enabled ) {
    finalcolor = texture2D(transferFunction, vec2(maxvalue,0.5));
    finalpos = maxpos;
  }
  
}*/

vec4 PhongLightingModel( vec3 normal,
                         vec3 viewdir, 
                         vec4 material_diffuse, 
                         vec4 material_ambient, 
                         vec4 material_specular, 
                         vec4 material_emission, 
                         float material_shininess, 
                         vec3 lightdir,
                         vec4 light_diffuse,
                         vec4 light_ambient,
                         vec4 light_specular ) {

  // ambient
  vec3 ambient = light_ambient.rgb * material_ambient.rgb;

  float ndotl = dot(normal, lightdir);

  // diffuse
  vec3 diffuse = 
    light_diffuse.rgb * material_diffuse.rgb * max(ndotl,0.0);

  vec3 specular = vec3(0,0,0);
  // specular
  if (ndotl > 0.0) {
    vec3 reflv = normalize( reflect(-lightdir,normal) );
    float rdotv = dot(reflv,viewdir);
    if( rdotv > 0.0 ) {
      specular = light_specular.rgb*material_specular.rgb * pow( rdotv, material_shininess);
    }
  }
  

  vec4 return_color;  
  return_color.rgb = material_emission.rgb + ambient + diffuse + specular; 
  return_color.a = material_diffuse.a;

  return return_color;
}

void ShadedVolumeStyle( inout vec4 current_color,
                        vec4 emissive_color, 
                        vec4 diffuse_color, 
                        vec4 ambient_color, 
                        vec4 specular_color, 
                        float shininess, 
                        vec4 pos,
                        vec4 viewdir,
                        mat4 view_to_tex,
                        bool enabled,
                        bool lighting,
                        vec4 normal ) {
  if( enabled ) {
    if(lighting) {  
      // re-orient normal
      if( dot(viewdir.xyz,normal.xyz)<0.0 )
        normal.xyz = -normal.xyz;

      if( normal.a > 0.001 ) {
        current_color.rgb = emissive_color.rgb;
        for( int i = 0; i < nr_enabled_lights; ++i ) {
          getEnabledLight( lightpos, light_diffuse, light_ambient, light_specular, i );
          lightpos = view_to_tex * lightpos;
          vec3 L;
          if(lightpos.w == 0.0 ) L = normalize(lightpos.xyz);
          else L =  normalize(lightpos.xyz-pos.xyz);

          current_color.rgb += PhongLightingModel( normal.xyz,
                                                   viewdir.xyz,
                                                   diffuse_color,
                                                   ambient_color,
                                                   specular_color,
                                                   emissive_color,
                                                   shininess,
                                                   L,
                                                   light_diffuse,
                                                   light_ambient,
                                                   light_specular ).rgb;

        }
        current_color.a *= diffuse_color.a;
      } else {
      current_color = vec4( 0, 0, 0, 0 );
      }
    } else {
      current_color.rgb = diffuse_color.rgb;
      current_color.a *= diffuse_color.a;
    }
  }
}



#if 0
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
                                        gl_FrontMaterial.diffuse,
                                        gl_FrontMaterial.ambient,
                                        gl_FrontMaterial.specular,
                                        gl_FrontMaterial.emission,
                                        gl_FrontMaterial.shininess,
                                        L,
                                        gl_LightSource[0].diffuse,
                                        gl_LightSource[0].ambient,
                                        gl_LightSource[0].specular );
  } // shdw
  
      } // lighting 
      
}
#endif
//END STYLE FUNCTIONS
