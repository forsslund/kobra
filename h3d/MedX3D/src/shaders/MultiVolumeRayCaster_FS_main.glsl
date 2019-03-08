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
varying vec2 screenPos;
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

// transformation matrix from local space to view space(camera space)
mat4 view_matrix;

// transformation matrix from view space(camera space) to local space
mat4 view_matrix_inverse;

// for each volume the depth buffer value for the entry point into
// the proxy geometry.  
// Only valid after initArrays has been called.
float volume_entry_points[ nr_volumes ];

// for each volume the depth buffer value for the exit point from
// the proxy geometry. 
// Only valid after initArrays has been called.
float volume_exit_points[ nr_volumes ];

// for each volume this specifies in the traversal of the ray there the
// ray is in relation to the volume.
// 0 - not entered volume
// 1 - inside volume
// 2 - exited volume
// Only valid after initArrays has been called.
int volume_current_layer[ nr_volumes ];

// The direction of the ray in texture coordinates for each volume.
// Used during ray traversal.
vec3 volume_ray_dir[ nr_volumes ];

// The direction of the ray in texture coordinates for each volume.
// Used during ray traversal.
vec3 volume_ray_pos[ nr_volumes ];

// Transformation matrix from view space(camera space) to texture space for
// each volume. Only valid after initArrays has been called.
mat4 view_to_tex_mat[ nr_volumes ];

// Transformation matrix from texture space to  view space(camera space)for 
// each volume. Only valid after initArrays has been called.
mat4 tex_to_view_mat[ nr_volumes ];


// given a 3d position in texture space and a texture with the 
// depth buffer value we return the position in texture space that
// corresponds to the position at the same screen coordinate as 
// the given coordinate but at the depth of the depth texture.

//tex_to_global = model * textureInverse
//global_to_tex = texture * modelInverse


// view = modelview * textureInverse * global_to_tex
// view_inverse = textureInverse * tex_to_global * modelviewInverse 

vec3 getFrameBufferTexCoords( vec3 pos, int index ) {//in texture space

  // transform to normalized screen coordinates
  vec4 screen_coords = 
    gl_ProjectionMatrix *
    view_matrix *  
    vec4( pos, 1 );

  screen_coords = screen_coords / screen_coords.w;

  // to range [0,1]
  screen_coords.xyz = screen_coords.xyz * 0.5 + 0.5;

  return screen_coords.xyz;
}

vec3 frameBufferCoordToTexCoord( vec3 pos, int index ) {
  // to range [-1, 1 ]
  vec3 tex_coord = (pos.xyz - 0.5 ) * 2;

  // to texture space
    vec4 tc = 
    gl_TextureMatrix[0] *
    localToTexSpace[index] *
    globalToLocalSpace[index] *
    view_matrix_inverse *
    gl_ProjectionMatrixInverse * vec4( tex_coord, 1 );

  /*   vec4 tc = 
    gl_TextureMatrix[0] *
   textureMatrix *
    gl_ModelViewProjectionMatrixInverse * vec4( tex_coord, 1 );
  */
  tc = tc / tc.w;

  return tc.xyz;
}



vec3 frameBufferCoordToGlobalSpace( vec3 pos ) {
  // to range [-1, 1 ]
  vec3 tex_coord = (pos.xyz - 0.5 ) * 2;

  // to global space
  vec4 tc = 
    view_matrix_inverse *
    gl_ProjectionMatrixInverse * vec4( tex_coord, 1 );

  tc = tc / tc.w;

  return tc.xyz;
}

vec3 globalToTexSpace( vec3 pos, int index ) {
  vec4 c = 
    gl_TextureMatrix[0] * 
    localToTexSpace[index] * 
    globalToLocalSpace[index] * 
    vec4( pos, 1.0 );
  return c.xyz;
}

vec3 globalToTexSpaceVec( vec3 vec, int index ) {
  vec4 c = 
    gl_TextureMatrix[0] * 
    localToTexSpace[index] * 
    globalToLocalSpace[index] * 
    vec4( vec, 0.0 );
  return c.xyz;
}

// given a 3d position in global space and a texture with the 
// depth buffer value we return the position in global space that
// corresponds to the position at the same screen coordinate as 
// the given coordinate but at the depth of the depth texture.
vec3 getDepthBufferValue( vec3 pos, //in global space
                           sampler2D depth_texture ) {
  // transform to normalized screen coordinates

  // Note: On Daniels MacBook Pro using the view_to_tex_mat array here
  // halves the frame rate(even when using a constant instead of volumeIndex)
  // Seems to work fine on other computers though. Any other matrix multiplications
  // seems to work fine. 
  vec4 screen_coords = 
    gl_ProjectionMatrix *
    view_matrix *
    vec4( pos, 1 );

  screen_coords = screen_coords / screen_coords.w;

  // to range [0,1]
  screen_coords.xyz = screen_coords.xyz * 0.5 + 0.5;

  // get new z value from depth texture
  screen_coords.z = texture2D( depth_texture, screen_coords.xy ).r;
  screen_coords.w = 1.0;

  // to range [-1,1]
  screen_coords.xyz = screen_coords.xyz * 2.0 - 1.0;

  // back to texture space
  screen_coords = gl_ProjectionMatrixInverse * screen_coords;
  screen_coords = screen_coords / screen_coords.w;
  
   screen_coords =  
     view_matrix_inverse *
     screen_coords;
  
  return screen_coords.xyz;
}

void initArrays( vec2 tc ) {
 

  for( int i = 0; i < nr_volumes; i++ ) {
    volume_entry_points[i] = 
      texture2DArray( volumeDepthTexture, vec3( tc, i*2 ) ).r;
    volume_exit_points[i] = 
      texture2DArray( volumeDepthTexture, vec3( tc, i*2 + 1 ) ).r;    
    if( volume_exit_points[i] == 1.0 ) {
      volume_current_layer[i] = 2;
    } else {
      volume_current_layer[i] = 0;
    }
    view_to_tex_mat[i] = gl_TextureMatrix[0] * localToTexSpace[i] * globalToLocalSpace[i] * view_matrix_inverse;
    tex_to_view_mat[i] = view_matrix * localToGlobalSpace[i] *  texToLocalSpace[i] * gl_TextureMatrixInverse[0];
  }
}

void updateRayStartPositions( vec3 pos_global ) {
  for( int i = 0; i < nr_volumes; i++ ) {
    volume_ray_pos[i] = globalToTexSpace( pos_global, i );
  }
}

void updateRayDirections( vec3 dir_global ) {
  for( int i = 0; i < nr_volumes; i++ ) {
    volume_ray_dir[i] = globalToTexSpaceVec( dir_global, i );
  }
}

void stepRayAllVolumes( int nr_steps ) {
  for( int i = 0; i < nr_volumes; i++ ) {
    volume_ray_pos[i].xyz += volume_ray_dir[i].xyz * nr_steps;
  }
}

bool isInsideVolume( int index ) {
  return volume_current_layer[index] == 1;
}

bool isInsideAnyVolume() {
  for( int i = 0; i < nr_volumes; i++ ) {
    if( isInsideVolume( i ) ) return true;
  }
  return false;
}

float getVolumeDepth( int volume_index ) {
  if( volume_current_layer[ volume_index ] == 1 ) {
    return volume_exit_points[ volume_index ];
  } else if( volume_current_layer[ volume_index ] == 0 ) {
    return volume_entry_points[ volume_index ];
  } else {
    return 2.0;
  }
}

int findClosest() {
  float closest_dist = 0; 
  int closest_index = -1;
  for( int i = 0; i < nr_volumes; i++ ) {
    float d = getVolumeDepth( i );
    if( ( closest_index == -1 || d < closest_dist) && d != 2.0 ) {
      closest_dist = d;
      closest_index = i;
    }
  }
  return closest_index;
}

void peelLayer( int index ) {
  volume_current_layer[index]++;
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
  //  mat4 view_to_tex = textureMatrix*gl_ModelViewMatrixInverse;
  //  mat4 tex_to_view = gl_ModelViewMatrix*textureMatrixInverse;
  //  vec4 viewdir_tex = vec4( -normalize(dir.xyz), 0.0 );
  
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
  /*
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
  */
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
RayResult traverseRay(vec4 r0, vec4 dir, vec4 color, vec4 zpoint ) {
  //useful stuff (hopefully not computed if not needed...?)
  //  mat4 view_to_tex = textureMatrix*gl_ModelViewMatrixInverse;
  //  mat4 tex_to_view = gl_ModelViewMatrix*textureMatrixInverse;
  //  vec4 viewdir_tex = vec4( -normalize(dir.xyz), 0.0 );
  
  // return value
  RayResult rr;
  
  // initial color of this fragment is black with zero alpha
  rr.color = color;
  // initial depth is not defined
  rr.zpoint = zpoint;
  
  // the number of ray steps to take each loop. Normally this is 1 but
  // can be changed by various optimization techniques such as 
  // empty space skipping in order to jump over empty regions of the
  // volume.
  int nr_steps = 1;

  //BEGIN PRE-LOOP
  //END PRE-LOOP
  
  float step_length =  abs(dir.w);

  // compositing loop
  while( rr.color.a<0.95 && r0.w>=-step_length) {
    {
      //BEGIN INSIDE-LOOP
      //END INSIDE-LOOP

    
      /*      if( r0.x >= 0.0 && r0.x <= 1.0 &&
	  r0.y >= 0.0 && r0.y <= 1.0 &&
	  r0.z >= 0.0 && r0.z <= 1.0 ) {
      */
      //BEGIN COMPOSITING
      //END COMPOSITING
	
      //}
    }

    // step forward along ray
    r0 += dir * float(nr_steps);
    stepRayAllVolumes( nr_steps );
  }
  
  //BEGIN POST-LOOP
  //END POST-LOOP

  // return result
  return rr;
}

// main function
void main() {

  // initialize variables
  view_matrix = gl_ModelViewMatrix * globalToLocalSpace[volumeIndex] ; 
  view_matrix_inverse = localToGlobalSpace[volumeIndex] * gl_ModelViewMatrixInverse;

  // initialize ray(in global space)
  vec4 raydir = vec4(normalize(rayDir), -1.0);
  vec3 raystart = rayStart;

  // get the screen texture coordinate for the ray start position.
  vec2 tc = getFrameBufferTexCoords( raystart, volumeIndex ).xy;

  // initialize array variables
  initArrays( tc.xy );

  //BEGIN RAY-INITIALIZATION
  //END RAY-INITIALIZATION

  raydir = vec4( rayStep * raydir.xyz, raydir.w * abs( rayStep ) );
  vec4 raypos = vec4(raystart, 0.0);
  
  // initialize local ray directions and start positions
  // for all volumes.
  updateRayDirections( raydir.xyz );
  updateRayStartPositions( raystart );

  RayResult rr;

  // initial color of this fragment is black with zero alpha
  rr.color = vec4(0.0, 0.0, 0.0, 0.0);
  // initial depth is not defined
  rr.zpoint = vec4(-1.0, 0.0, 0.0, 0.0);  

  // find closest
  int closest = findClosest();

  // if the closest volume is not the one currently being rendered
  // we abort. The ray should always start at the first volume encountered.
  if( closest != volumeIndex ) {
    discard;
    return;
  }

  // previous_depth is the depth of the previous intersection point of
  // any volume. We initialize it to the depth of the closest volume.
  float previous_depth = getVolumeDepth( closest );
  
  // Peel a layer for the closest volume.
  peelLayer( closest );

  // Find the new closest volume.
  closest = findClosest();

  // Traverse ray through the volumes one ray segment at a time,
  // We define the ray segments to be the path between two following
  // volume intersection points.
  while( closest != -1 ) {
    // the depth of the closest volume.
    float current_depth = getVolumeDepth( closest );

    // global position of ray-volume intersection point
    vec3 current_pos =  frameBufferCoordToGlobalSpace( vec3( tc.xy, current_depth) );

    // calculate distance(in global coordinates) from previous rayposition
    // to next to get ray segment length.
    vec3 diff = current_pos - raypos.xyz ;
    float t = length( diff );
    raypos.w = t;

    // traverse ray segment
    rr = traverseRay(raypos, raydir, rr.color, rr.zpoint );

    // move to next ray segment. Use old segment end point as new segment
    // start points.
    raypos.xyz = current_pos;
    updateRayStartPositions( raypos.xyz );
    
    // peel a layer for the closest volume
    peelLayer( closest );

    // find the new closest volume.
    closest = findClosest();

    // we have moved into empty space in between volumes so skip the current
    // segment
    if( closest != -1 && !isInsideAnyVolume() ) {
      previous_depth = getVolumeDepth( closest );
      raypos.xyz = frameBufferCoordToGlobalSpace( vec3( tc.xy, previous_depth) );
      updateRayStartPositions( raypos.xyz );

      peelLayer( closest );
      closest = findClosest();
      
    }
  }

  // if final color is fully transparent, we discard the fragment.
  if( rr.color.a == 0.0 ) discard;
  
  // set color
  gl_FragColor = rr.color;
  

  /*
  // set depth if computed
  if( rr.zpoint.x>=0.0 ) {
    // set depth
    vec4 tmp =
      gl_ProjectionMatrix*  tex_to_view_mat[volumeIndex] *  gl_TextureMatrix[0] * rr.zpoint;
    // in window coordinatess
    gl_FragDepth = 0.5*(gl_DepthRange.diff*tmp.z/tmp.w +
  			gl_DepthRange.near+gl_DepthRange.far);
  } else {
    gl_FragDepth = gl_FragCoord.z;
    };*/
}
