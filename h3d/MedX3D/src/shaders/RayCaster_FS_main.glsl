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
/// \file RayCaster_FS_main.glsl
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

//BEGIN VARYINGS
varying vec3 rayStart;
varying vec3 rayDir;
//END VARYINGS


// --------------------------------------------------------
// Here follows the raycasting engine and the main function
// --------------------------------------------------------
// struct to return from traverseRay,
// the resulting color and the point to use for depth calculation
struct RayResult {
  vec4 color;
  vec4 zpoint;
};

// given a 3d position in texture space and a texture with the 
// depth buffer value we return the position in texture space that
// corresponds to the position at the same screen coordinate as 
// the given coordinate but at the depth of the depth texture.
vec3 getDepthBufferValue( vec3 pos, //in texture space
                           sampler2D depth_texture ) {
  // transform to normalized screen coordinates
  vec4 screen_coords = gl_ModelViewProjectionMatrix *
                           textureMatrixInverse *
                           gl_TextureMatrixInverse[0] * vec4( pos, 1 );
  screen_coords = screen_coords / screen_coords.w;

  // to range [0,1]
  screen_coords.xyz = screen_coords.xyz * 0.5 + 0.5;

  // get new z value from depth texture
  screen_coords.z = texture2D( depth_texture, screen_coords.xy ).r;
  screen_coords.w = 1.0;

  // to range [-1,1]
  screen_coords.xyz = screen_coords.xyz * 2.0 - 1.0;

  // back to texture space
  screen_coords = gl_ModelViewProjectionMatrixInverse * screen_coords;
  screen_coords = screen_coords / screen_coords.w;
  screen_coords = textureMatrix * gl_TextureMatrix[0] * screen_coords;

  return screen_coords.xyz;
}


int getNrSteps( vec3 r0,
                vec3 ray_sign,
                vec3 dir,
                inout vec3 empty_space_pos,
                vec3 empty_space_step,
                sampler3D emptySpaceMinMaxTexture,
                sampler2D emptySpaceClassificationTexture ) {
  int nr_steps = 1;

  if( ((ray_sign.x == 1.0 && r0.x > empty_space_pos.x ) || 
      (ray_sign.y == 1.0 && r0.y > empty_space_pos.y ) ||
      (ray_sign.z == 1.0 && r0.z > empty_space_pos.z ) ||
      (ray_sign.x == -1.0 && r0.x < empty_space_pos.x ) || 
      (ray_sign.y == -1.0 && r0.y < empty_space_pos.y ) ||
      (ray_sign.z == -1.0 && r0.z < empty_space_pos.z ) ) ){
    // we are in a new empty space grid cell
      
    empty_space_pos.x += 
      empty_space_step.x * (floor( abs(empty_space_pos.x - r0.x) / abs(empty_space_step.x) ) + 1.0 ) * step( 0.0, ray_sign.x*(r0.x - empty_space_pos.x ));
    empty_space_pos.y += 
      empty_space_step.y * (floor( abs(empty_space_pos.y - r0.y) / abs(empty_space_step.y) ) + 1.0 ) * step( 0.0, ray_sign.y*(r0.y - empty_space_pos.y ));

    empty_space_pos.z += 
      empty_space_step.z * (floor( abs(empty_space_pos.z - r0.z) / abs(empty_space_step.z) ) + 1.0 ) * step( 0.0, ray_sign.z*(r0.z - empty_space_pos.z ));
    
    /*
    // find the next position in x that is outside the current
    // grid cell
    while( (ray_sign.x == 1  && r0.x > empty_space_pos.x) ||
           (ray_sign.x == -1 && r0.x < empty_space_pos.x) ) 
      empty_space_pos.x += empty_space_step.x;

    // find the next position in y that is outside the current
    // grid cell
    while( (ray_sign.y == 1  && r0.y > empty_space_pos.y) ||
           (ray_sign.y == -1 && r0.y < empty_space_pos.y) ) 
      empty_space_pos.y += empty_space_step.y;
    
    // find the next position in y that is outside the current
    // grid cell
    while( (ray_sign.z == 1  && r0.z > empty_space_pos.z) ||
           (ray_sign.z == -1 && r0.z < empty_space_pos.z) ) 
      empty_space_pos.z += empty_space_step.z;
    */  
    // lookup min and max value in current grid cell.
    vec4 minmax = texture3D( emptySpaceMinMaxTexture, r0.xyz );
    
    // classify it
    vec4 classification = texture2D( emptySpaceClassificationTexture, 
                                     minmax.ra );
    if( classification.r < 0.5 ) {
      // cell is empty
      //
      // find out how many ray steps we will have to step in each direction
      // to come to a new grid cell.
      float x_steps = floor( abs(empty_space_pos.x - r0.x) / abs(dir.x) ) + 1.0;
      float y_steps = floor( abs(empty_space_pos.y - r0.y) / abs(dir.y) ) + 1.0;
      float z_steps = floor( abs(empty_space_pos.z - r0.z) / abs(dir.z) ) + 1.0;  
      // step the number of steps that move to a new grid
      // cell
      nr_steps = int( max( min( x_steps, min( y_steps, z_steps ) ), 1.0 ) );
      } 
  } 

  //  nr_steps = 1;
  return nr_steps;
}



// traverse ray through volume calculating how much alpha is
// left after following the ray through the volume. Used
// by e.g. ShadedVolumeStyleWithShadows that needs to shoot
// a ray from fragment to light to determin how much light 
// reaches the fragment. 
//
// Inputs:
//   r0  - ray start (xyz), ray exit time (w)
//   dir - ray direction (xyz), step length (-w)
// Output:
//   RayResult.color.a  - the computed alpha for the ray

RayResult traverseRayOpacity(vec4 r0, vec4 dir) {
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
  
  // the number of ray steps to take each loop. Normally this is 1 but
  // can be changed by various optimization techniques such as 
  // empty space skipping in order to jump over empty regions of the
  // volume.
  int nr_steps = 1;

  //OPACITY BEGIN PRE-LOOP
  //OPACITY END PRE-LOOP
  
  // compositing loop
  while( rr.color.a<0.95 && r0.w>=0.0 ) {

    //OPACITY GET_ORIG_SAMPLE_COLOR

    //OPACITY ORIG_SAMPLE_MANIP

    // color of this sample
    vec4 sample_color = orig_sample_color;  

    //OPACITY BEGIN INSIDE-LOOP
    //OPACITY END INSIDE-LOOP
    
    //OPACITY BEGIN COMPOSITING
    //OPACITY END COMPOSITING
    

    // step forward along ray
    r0 += dir * float(nr_steps);
  }
  
  //OPACITY BEGIN POST-LOOP
  //OPACITY END POST-LOOP
  
  // return result
  return rr;
}

// pos is position of current sample
// light_pos is position of light
// light_dir is normalized direction from pos to light_pos
float lightAbsorbed( vec4 pos, vec4 light_pos, 
                     vec3 light_dir, float step_size ) {
  // shoot ray from sample toward light and see how such is 
  // absorbed along the way.
  float lexit = rayexit(pos.xyz, light_dir.xyz);
  vec4 step = vec4( step_size * light_dir, -step_size );
  vec4 lpos = vec4(pos.xyz, lexit);

  // ignore current position
  lpos += step;                

  RayResult res = traverseRayOpacity( lpos,
                                      step );
  return res.color.a;
}

void ShadedVolumeStyleWithShadows(inout vec4 current_color,
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
                                  vec4 normal,
                                  float step_size ) {
  if( enabled ) {
    if(lighting) {  
      // re-orient normal
      if( dot(viewdir.xyz,normal.xyz)<0.0 )
        normal.xyz = -normal.xyz;

      if( normal.a > 0.001 ) {
        current_color.rgb = emissive_color.rgb;
        for( int i = 0; i < nr_enabled_lights; i++ ) {
          getEnabledLight( lightpos, light_diffuse, light_ambient, light_specular, i );
          lightpos = view_to_tex * lightpos;
          vec3 L;
          if(lightpos.w == 0.0 ) L = normalize(lightpos.xyz);
          else L =  normalize(lightpos.xyz-pos.xyz);
          
          // ray trace light to see how much reaches the current point.
          float absorbed_light = lightAbsorbed( pos, lightpos, L, step_size );
          light_diffuse  *= (1.0 - absorbed_light);
          light_ambient  *= (1.0 - absorbed_light);
          light_specular *= (1.0 - absorbed_light);

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
          //current_color.rgb = vec3( absorbed_light, 0, 0 );
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
  
  // the number of ray steps to take each loop. Normally this is 1 but
  // can be changed by various optimization techniques such as 
  // empty space skipping in order to jump over empty regions of the
  // volume.
  int nr_steps = 1;

  //BEGIN PRE-LOOP
  //END PRE-LOOP
  
  // compositing loop
  while( rr.color.a<0.95 && r0.w>=0.0 ) {

    //GET_ORIG_SAMPLE_COLOR

    //ORIG_SAMPLE_MANIP

    // color of this sample
    vec4 sample_color = orig_sample_color;  

    //BEGIN INSIDE-LOOP
    //END INSIDE-LOOP
    
    //BEGIN COMPOSITING
    //END COMPOSITING
    

    // step forward along ray
    r0 += dir * float(nr_steps);
  }
  
  //BEGIN POST-LOOP
  //END POST-LOOP

  // moved discard from here because of problems with AMD GPUs\n"
  //if( rr.color.a == 0.0 ) discard;\n"
  if( rr.color.a == 0.0 )
    rr.color.a = -1.0;
  
  // return result
  return rr;
}


// main function
void main() {
  // initialize ray(in texture space)
  vec4 raydir = vec4(normalize(rayDir), -1.0);
  vec3 raystart = rayStart;
  //BEGIN RAY-INITIALIZATION
  //END RAY-INITIALIZATION

  float texit = rayexit(raystart,raydir.xyz);
  raydir = vec4( rayStep * raydir.xyz, raydir.w * abs( rayStep ) );
  vec4 raypos = vec4(raystart, texit);
  
  // traverse ray
  RayResult rr = traverseRay(raypos, raydir);
  
  // if there nothing along the ray discard, else set color
  // n.b. moved discard from previous position because of weird AMD bug
  if( rr.color.a == -1.0 )
    discard;
  else
    gl_FragColor = rr.color;
  
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
  };
}
