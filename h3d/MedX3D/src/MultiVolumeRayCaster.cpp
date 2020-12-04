//////////////////////////////////////////////////////////////////////////////
//    Copyright 2004-2019, SenseGraphics AB
//
//    This file is part of H3D API.
//
//    H3D API is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    H3D API is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with H3D API; if not, write to the Free Software
//    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
//    A commercial license is also available. Please contact us at 
//    www.sensegraphics.com for more information.
//
//
/// \file MultiVolumeRayCaster.cpp
/// \brief CPP file for MultiVolumeRayCaster.
///
//
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/ClipPlane.h>

#include <H3D/MedX3D/MultiVolumeRayCaster.h>
#include <H3D/MedX3D/FrameBufferTexture.h>
#include <H3D/MedX3D/X3DVolumeNode.h>
#include <H3D/Pixel3DTexture.h>
#include <H3D/PixelTexture.h>
#include <H3D/ShaderPart.h>
#include <H3D/ShaderFunctions.h>

using namespace H3D;

H3DNodeDatabase MultiVolumeRayCaster::database( "MultiVolumeRayCaster", 
                                     &newInstance< MultiVolumeRayCaster >, 
                                     typeid( MultiVolumeRayCaster ),
                                     &H3DVolumeRendererNode::database );
namespace MultiVolumeRayCasterInternals {
  FIELDDB_ELEMENT( MultiVolumeRayCaster, stopRaysAtGeometries, INPUT_OUTPUT )
  FIELDDB_ELEMENT( MultiVolumeRayCaster, useStochasticJittering, INPUT_OUTPUT )
  FIELDDB_ELEMENT( MultiVolumeRayCaster, rayStep, INPUT_OUTPUT )

  // The glsl vertex shader for perspective projection. Ray direction goes from camera 
  // position towards ray start if perspective projection and constant direction
  // for all points( (0 0 -1) in camera space) if orthographic projection
  //
   const string vertexshader_raycaster =
    "#version 120\n"
    "#extension GL_EXT_texture_array : enable\n"
    "#extension GL_EXT_gpu_shader4 : enable\n"
    "//BEGIN UNIFORMS\n"
    "//END UNIFORMS\n"
    "\n"
    "//BEGIN VARYINGS\n"
    "varying vec3 rayStart;\n"
    "varying vec3 rayDir;\n"
    "varying vec2 screenPos;\n"
    "//END VARYINGS\n"
    "\n"
    "void main() {\n"
    "  rayStart = (localToGlobalSpace[volumeIndex]*gl_Vertex).xyz;\n"
    "  mat4 camera_to_global_space =localToGlobalSpace[volumeIndex]*gl_ModelViewMatrixInverse; \n"
    " if( gl_ProjectionMatrix[2][3] == 0.0 ) { \n"
    "    // orthographic projection. \n"
    "  rayDir = (camera_to_global_space*vec4(0,0,-1,1)).xyz - (camera_to_global_space*vec4(0,0,0,1)).xyz;\n"
    " } else { \n" 
    "    // perspective projection. \n" 
    "    rayDir = rayStart-(camera_to_global_space*vec4(0,0,0,1)).xyz;\n"
    " } \n"
    "  gl_Position = gl_ModelViewProjectionMatrix*gl_Vertex;\n"
    "  screenPos = gl_Position.xy * 0.5 + 0.5; \n"
    "  gl_ClipVertex = gl_ModelViewMatrix*gl_Vertex;\n"
    "}\n";


 
  // "(MEDX3D_DIRECTORY)/src/shaders/MultiVolumeRayCaster_FS_main.glsl"

  const string raycaster_fs_main = 
    "//BEGIN UNIFORMS\n"
    "//END UNIFORMS\n"
    "\n"
    "//BEGIN VARYINGS\n"
    "varying vec3 rayStart;\n"
    "varying vec3 rayDir;\n"
    "varying vec2 screenPos;\n"
    "//END VARYINGS\n"
    "\n"
    "\n"
    "// --------------------------------------------------------\n"
    "// Here follows the raycasting engine and the main function\n"
    "// --------------------------------------------------------\n"
    "// struct to return from traverseRay,\n"
    "// the resulting color and the point to use for depth calculation\n"
    "struct RayResult {\n"
    "  vec4 color;\n"
    "  vec4 zpoint;\n"
    "};\n"
    "\n"
    "// transformation matrix from local space to view space(camera space)\n"
    "mat4 view_matrix;\n"
    "\n"
    "// transformation matrix from view space(camera space) to local space\n"
    "mat4 view_matrix_inverse;\n"
    "\n"
    "// for each volume the depth buffer value for the entry point into\n"
    "// the proxy geometry.  \n"
    "// Only valid after initArrays has been called.\n"
    "float volume_entry_points[ nr_volumes ];\n"
    "\n"
    "// for each volume the depth buffer value for the exit point from\n"
    "// the proxy geometry. \n"
    "// Only valid after initArrays has been called.\n"
    "float volume_exit_points[ nr_volumes ];\n"
    "\n"
    "// for each volume this specifies in the traversal of the ray there the\n"
    "// ray is in relation to the volume.\n"
    "// 0 - not entered volume\n"
    "// 1 - inside volume\n"
    "// 2 - exited volume\n"
    "// Only valid after initArrays has been called.\n"
    "int volume_current_layer[ nr_volumes ];\n"
    "\n"
    "// The direction of the ray in texture coordinates for each volume.\n"
    "// Used during ray traversal.\n"
    "vec3 volume_ray_dir[ nr_volumes ];\n"
    "\n"
    "// The direction of the ray in texture coordinates for each volume.\n"
    "// Used during ray traversal.\n"
    "vec3 volume_ray_pos[ nr_volumes ];\n"
    "\n"
    "// Transformation matrix from view space(camera space) to texture space for\n"
    "// each volume. Only valid after initArrays has been called.\n"
    "mat4 view_to_tex_mat[ nr_volumes ];\n"
    "\n"
    "// Transformation matrix from texture space to  view space(camera space)for \n"
    "// each volume. Only valid after initArrays has been called.\n"
    "mat4 tex_to_view_mat[ nr_volumes ];\n"
    "\n"
    "\n"
    "// given a 3d position in texture space and a texture with the \n"
    "// depth buffer value we return the position in texture space that\n"
    "// corresponds to the position at the same screen coordinate as \n"
    "// the given coordinate but at the depth of the depth texture.\n"
    "\n"
    "//tex_to_global = model * textureInverse\n"
    "//global_to_tex = texture * modelInverse\n"
    "\n"
    "\n"
    "// view = modelview * textureInverse * global_to_tex\n"
    "// view_inverse = textureInverse * tex_to_global * modelviewInverse \n"
    "\n"
    "vec3 getFrameBufferTexCoords( vec3 pos, int index ) {//in texture space\n"
    "\n"
    "  // transform to normalized screen coordinates\n"
    "  vec4 screen_coords = \n"
    "    gl_ProjectionMatrix *\n"
    "    view_matrix *  \n"
    "    vec4( pos, 1 );\n"
    "\n"
    "  screen_coords = screen_coords / screen_coords.w;\n"
    "\n"
    "  // to range [0,1]\n"
    "  screen_coords.xyz = screen_coords.xyz * 0.5 + 0.5;\n"
    "\n"
    "  return screen_coords.xyz;\n"
    "}\n"
    "\n"
    "vec3 frameBufferCoordToTexCoord( vec3 pos, int index ) {\n"
    "  // to range [-1, 1 ]\n"
    "  vec3 tex_coord = (pos.xyz - 0.5 ) * 2;\n"
    "\n"
    "  // to texture space\n"
    "    vec4 tc = \n"
    "    gl_TextureMatrix[0] *\n"
    "    localToTexSpace[index] *\n"
    "    globalToLocalSpace[index] *\n"
    "    view_matrix_inverse *\n"
    "    gl_ProjectionMatrixInverse * vec4( tex_coord, 1 );\n"
    "\n"
    "  /*   vec4 tc = \n"
    "    gl_TextureMatrix[0] *\n"
    "   textureMatrix *\n"
    "    gl_ModelViewProjectionMatrixInverse * vec4( tex_coord, 1 );\n"
    "  */\n"
    "  tc = tc / tc.w;\n"
    "\n"
    "  return tc.xyz;\n"
    "}\n"
    "\n"
    "\n"
    "\n"
    "vec3 frameBufferCoordToGlobalSpace( vec3 pos ) {\n"
    "  // to range [-1, 1 ]\n"
    "  vec3 tex_coord = (pos.xyz - 0.5 ) * 2;\n"
    "\n"
    "  // to global space\n"
    "  vec4 tc = \n"
    "    view_matrix_inverse *\n"
    "    gl_ProjectionMatrixInverse * vec4( tex_coord, 1 );\n"
    "\n"
    "  tc = tc / tc.w;\n"
    "\n"
    "  return tc.xyz;\n"
    "}\n"
    "\n"
    "vec3 globalToTexSpace( vec3 pos, int index ) {\n"
    "  vec4 c = \n"
    "    gl_TextureMatrix[0] * \n"
    "    localToTexSpace[index] * \n"
    "    globalToLocalSpace[index] * \n"
    "    vec4( pos, 1.0 );\n"
    "  return c.xyz;\n"
    "}\n"
    "\n"
    "vec3 globalToTexSpaceVec( vec3 vec, int index ) {\n"
    "  vec4 c = \n"
    "    gl_TextureMatrix[0] * \n"
    "    localToTexSpace[index] * \n"
    "    globalToLocalSpace[index] * \n"
    "    vec4( vec, 0.0 );\n"
    "  return c.xyz;\n"
    "}\n"
    "\n"
    "// given a 3d position in global space and a texture with the \n"
    "// depth buffer value we return the position in global space that\n"
    "// corresponds to the position at the same screen coordinate as \n"
    "// the given coordinate but at the depth of the depth texture.\n"
    "vec3 getDepthBufferValue( vec3 pos, //in global space\n"
    "                           sampler2D depth_texture ) {\n"
    "  // transform to normalized screen coordinates\n"
    "\n"
    "  // Note: On Daniels MacBook Pro using the view_to_tex_mat array here\n"
    "  // halves the frame rate(even when using a constant instead of volumeIndex)\n"
    "  // Seems to work fine on other computers though. Any other matrix multiplications\n"
    "  // seems to work fine. \n"
    "  vec4 screen_coords = \n"
    "    gl_ProjectionMatrix *\n"
    "    view_matrix *\n"
    "    vec4( pos, 1 );\n"
    "\n"
    "  screen_coords = screen_coords / screen_coords.w;\n"
    "\n"
    "  // to range [0,1]\n"
    "  screen_coords.xyz = screen_coords.xyz * 0.5 + 0.5;\n"
    "\n"
    "  // get new z value from depth texture\n"
    "  screen_coords.z = texture2D( depth_texture, screen_coords.xy ).r;\n"
    "  screen_coords.w = 1.0;\n"
    "\n"
    "  // to range [-1,1]\n"
    "  screen_coords.xyz = screen_coords.xyz * 2.0 - 1.0;\n"
    "\n"
    "  // back to texture space\n"
    "  screen_coords = gl_ProjectionMatrixInverse * screen_coords;\n"
    "  screen_coords = screen_coords / screen_coords.w;\n"
    "  \n"
    "   screen_coords =  \n"
    "     view_matrix_inverse *\n"
    "     screen_coords;\n"
    "  \n"
    "  return screen_coords.xyz;\n"
    "}\n"
    "\n"
    "void initArrays( vec2 tc ) {\n"
    " \n"
    "\n"
    "  for( int i = 0; i < nr_volumes; ++i ) {\n"
    "    volume_entry_points[i] = \n"
    "      texture2DArray( volumeDepthTexture, vec3( tc, i*2 ) ).r;\n"
    "    volume_exit_points[i] = \n"
    "      texture2DArray( volumeDepthTexture, vec3( tc, i*2 + 1 ) ).r;    \n"
    "    if( volume_exit_points[i] == 1.0 ) {\n"
    "      volume_current_layer[i] = 2;\n"
    "    } else {\n"
    "      volume_current_layer[i] = 0;\n"
    "    }\n"
    "    view_to_tex_mat[i] = gl_TextureMatrix[0] * localToTexSpace[i] * globalToLocalSpace[i] * view_matrix_inverse;\n"
    "    tex_to_view_mat[i] = view_matrix * localToGlobalSpace[i] *  texToLocalSpace[i] * gl_TextureMatrixInverse[0];\n"
    "  }\n"
    "}\n"
    "\n"
    "void updateRayStartPositions( vec3 pos_global ) {\n"
    "  for( int i = 0; i < nr_volumes; ++i ) {\n"
    "    volume_ray_pos[i] = globalToTexSpace( pos_global, i );\n"
    "  }\n"
    "}\n"
    "\n"
    "void updateRayDirections( vec3 dir_global ) {\n"
    "  for( int i = 0; i < nr_volumes; ++i ) {\n"
    "    volume_ray_dir[i] = globalToTexSpaceVec( dir_global, i );\n"
    "  }\n"
    "}\n"
    "\n"
    "void stepRayAllVolumes( int nr_steps ) {\n"
    "  for( int i = 0; i < nr_volumes; ++i ) {\n"
    "    volume_ray_pos[i].xyz += volume_ray_dir[i].xyz * nr_steps;\n"
    "  }\n"
    "}\n"
    "\n"
    "bool isInsideVolume( int index ) {\n"
    "  return volume_current_layer[index] == 1;\n"
    "}\n"
    "\n"
    "bool isInsideAnyVolume() {\n"
    "  for( int i = 0; i < nr_volumes; ++i ) {\n"
    "    if( isInsideVolume( i ) ) return true;\n"
    "  }\n"
    "  return false;\n"
    "}\n"
    "\n"
    "float getVolumeDepth( int volume_index ) {\n"
    "  if( volume_current_layer[ volume_index ] == 1 ) {\n"
    "    return volume_exit_points[ volume_index ];\n"
    "  } else if( volume_current_layer[ volume_index ] == 0 ) {\n"
    "    return volume_entry_points[ volume_index ];\n"
    "  } else {\n"
    "    return 2.0;\n"
    "  }\n"
    "}\n"
    "\n"
    "int findClosest() {\n"
    "  float closest_dist = 0; \n"
    "  int closest_index = -1;\n"
    "  for( int i = 0; i < nr_volumes; ++i ) {\n"
    "    float d = getVolumeDepth( i );\n"
    "    if( ( closest_index == -1 || d < closest_dist) && d != 2.0 ) {\n"
    "      closest_dist = d;\n"
    "      closest_index = i;\n"
    "    }\n"
    "  }\n"
    "  return closest_index;\n"
    "}\n"
    "\n"
    "void peelLayer( int index ) {\n"
    "  ++(volume_current_layer[index]);\n"
    "}\n"
    "\n"
    "\n"
    "\n"
    "int getNrSteps( vec3 r0,\n"
    "                vec3 ray_sign,\n"
    "                vec3 dir,\n"
    "                inout vec3 empty_space_pos,\n"
    "                vec3 empty_space_step,\n"
    "                sampler3D emptySpaceMinMaxTexture,\n"
    "                sampler2D emptySpaceClassificationTexture ) {\n"
    "  int nr_steps = 1;\n"
    "\n"
    "  if( ((ray_sign.x == 1.0 && r0.x > empty_space_pos.x ) || \n"
    "      (ray_sign.y == 1.0 && r0.y > empty_space_pos.y ) ||\n"
    "      (ray_sign.z == 1.0 && r0.z > empty_space_pos.z ) ||\n"
    "      (ray_sign.x == -1.0 && r0.x < empty_space_pos.x ) || \n"
    "      (ray_sign.y == -1.0 && r0.y < empty_space_pos.y ) ||\n"
    "      (ray_sign.z == -1.0 && r0.z < empty_space_pos.z ) ) ){\n"
    "    // we are in a new empty space grid cell\n"
    "      \n"
    "    empty_space_pos.x += \n"
    "      empty_space_step.x * (floor( abs(empty_space_pos.x - r0.x) / abs(empty_space_step.x) ) + 1.0 ) * step( 0.0, ray_sign.x*(r0.x - empty_space_pos.x ));\n"
    "    empty_space_pos.y += \n"
    "      empty_space_step.y * (floor( abs(empty_space_pos.y - r0.y) / abs(empty_space_step.y) ) + 1.0 ) * step( 0.0, ray_sign.y*(r0.y - empty_space_pos.y ));\n"
    "\n"
    "    empty_space_pos.z += \n"
    "      empty_space_step.z * (floor( abs(empty_space_pos.z - r0.z) / abs(empty_space_step.z) ) + 1.0 ) * step( 0.0, ray_sign.z*(r0.z - empty_space_pos.z ));\n"
    "    \n"
    "    /*\n"
    "    // find the next position in x that is outside the current\n"
    "  // grid cell\n"
    "    while( (ray_sign.x == 1  && r0.x > empty_space_pos.x) ||\n"
    "           (ray_sign.x == -1 && r0.x < empty_space_pos.x) ) \n"
    "      empty_space_pos.x += empty_space_step.x;\n"
    "\n"
    "    // find the next position in y that is outside the current\n"
    "    // grid cell\n"
    "    while( (ray_sign.y == 1  && r0.y > empty_space_pos.y) ||\n"
    "           (ray_sign.y == -1 && r0.y < empty_space_pos.y) ) \n"
    "      empty_space_pos.y += empty_space_step.y;\n"
    "    \n"
    "    // find the next position in y that is outside the current\n"
    "    // grid cell\n"
    "    while( (ray_sign.z == 1  && r0.z > empty_space_pos.z) ||\n"
    "           (ray_sign.z == -1 && r0.z < empty_space_pos.z) ) \n"
    "      empty_space_pos.z += empty_space_step.z;\n"
    "    */  \n"
    "    // lookup min and max value in current grid cell.\n"
    "    vec4 minmax = texture3D( emptySpaceMinMaxTexture, r0.xyz );\n"
    "    \n"
    "    // classify it\n"
    "    vec4 classification = texture2D( emptySpaceClassificationTexture, \n"
    "                                     minmax.ra );\n"
    "    if( classification.r < 0.5 ) {\n"
    "      // cell is empty\n"
    "      //\n"
    "      // find out how many ray steps we will have to step in each direction\n"
    "      // to come to a new grid cell.\n"
    "      float x_steps = floor( abs(empty_space_pos.x - r0.x) / abs(dir.x) ) + 1.0;\n"
    "      float y_steps = floor( abs(empty_space_pos.y - r0.y) / abs(dir.y) ) + 1.0;\n"
    "      float z_steps = floor( abs(empty_space_pos.z - r0.z) / abs(dir.z) ) + 1.0;  \n"
    "      // step the number of steps that move to a new grid\n"
    "      // cell\n"
    "      nr_steps = int( max( min( x_steps, min( y_steps, z_steps ) ), 1.0 ) );\n"
    "      } \n"
    "  } \n"
    "\n"
    "  //  nr_steps = 1;\n"
    "  return nr_steps;\n"
    "}\n"
    "\n"
    "\n"
    "\n"
    "// traverse ray through volume calculating how much alpha is\n"
    "// left after following the ray through the volume. Used\n"
    "// by e.g. ShadedVolumeStyleWithShadows that needs to shoot\n"
    "// a ray from fragment to light to determin how much light \n"
    "// reaches the fragment. \n"
    "//\n"
    "// Inputs:\n"
    "//   r0  - ray start (xyz), ray exit time (w)\n"
    "//   dir - ray direction (xyz), step length (-w)\n"
    "// Output:\n"
    "//   RayResult.color.a  - the computed alpha for the ray\n"
    "\n"
    "RayResult traverseRayOpacity(vec4 r0, vec4 dir) {\n"
    "  //useful stuff (hopefully not computed if not needed...?)\n"
    "  //  mat4 view_to_tex = textureMatrix*gl_ModelViewMatrixInverse;\n"
    "  //  mat4 tex_to_view = gl_ModelViewMatrix*textureMatrixInverse;\n"
    "  //  vec4 viewdir_tex = vec4( -normalize(dir.xyz), 0.0 );\n"
    "  \n"
    "  // return value\n"
    "  RayResult rr;\n"
    "  \n"
    "  // initial color of this fragment is black with zero alpha\n"
    "  rr.color = vec4(0.0, 0.0, 0.0, 0.0);\n"
    "  // initial depth is not defined\n"
    "  rr.zpoint = vec4(-1.0, 0.0, 0.0, 0.0);\n"
    "  \n"
    "  // the number of ray steps to take each loop. Normally this is 1 but\n"
    "  // can be changed by various optimization techniques such as \n"
    "  // empty space skipping in order to jump over empty regions of the\n"
    "  // volume.\n"
    "  int nr_steps = 1;\n"
    "  /*\n"
    "  //OPACITY BEGIN PRE-LOOP\n"
    "  //OPACITY END PRE-LOOP\n"
    "  \n"
    "  // compositing loop\n"
    "  while( rr.color.a<0.95 && r0.w>=0.0 ) {\n"
    "\n"
    "    //OPACITY GET_ORIG_SAMPLE_COLOR\n"
    "\n"
    "    //OPACITY ORIG_SAMPLE_MANIP\n"
    "\n"
    "    // color of this sample\n"
    "    vec4 sample_color = orig_sample_color;  \n"
    "\n"
    "    //OPACITY BEGIN INSIDE-LOOP\n"
    "    //OPACITY END INSIDE-LOOP\n"
    "    \n"
    "    //OPACITY BEGIN COMPOSITING\n"
    "    //OPACITY END COMPOSITING\n"
    "    \n"
    "\n"
    "    // step forward along ray\n"
    "    r0 += dir * float(nr_steps);\n"
    "  }\n"
    "  \n"
    "  //OPACITY BEGIN POST-LOOP\n"
    "  //OPACITY END POST-LOOP\n"
    "  */\n"
    "  // return result\n"
    "  return rr;\n"
    "}\n"
    "\n"
    "// pos is position of current sample\n"
    "// light_pos is position of light\n"
    "// light_dir is normalized direction from pos to light_pos\n"
    "float lightAbsorbed( vec4 pos, vec4 light_pos, \n"
    "                     vec3 light_dir, float step_size ) {\n"
    "  // shoot ray from sample toward light and see how such is \n"
    "  // absorbed along the way.\n"
    "  float lexit = rayexit(pos.xyz, light_dir.xyz);\n"
    "  vec4 step = vec4( step_size * light_dir, -step_size );\n"
    "  vec4 lpos = vec4(pos.xyz, lexit);\n"
    "\n"
    "  // ignore current position\n"
    "  lpos += step;                \n"
    "\n"
    "  RayResult res = traverseRayOpacity( lpos,\n"
    "                                      step );\n"
    "  return res.color.a;\n"
    "}\n"
    "\n"
    "void ShadedVolumeStyleWithShadows(inout vec4 current_color,\n"
    "                                  vec4 emissive_color,\n"
    "                                  vec4 diffuse_color,\n"
    "                                  vec4 ambient_color,\n"
    "                                  vec4 specular_color,\n"
    "                                  float shininess,\n"
    "                                  vec4 pos,\n"
    "                                  vec4 viewdir,\n"
    "                                  mat4 view_to_tex,\n"
    "                                  bool enabled,\n"
    "                                  bool lighting,\n"
    "                                  vec4 normal,\n"
    "                                  float step_size ) {\n"
    "  if( enabled ) {\n"
    "    if(lighting) {  \n"
    "      // re-orient normal\n"
    "      if( dot(viewdir.xyz,normal.xyz)<0.0 )\n"
    "        normal.xyz = -normal.xyz;\n"
    "\n"
    "      if( normal.a > 0.001 ) {\n"
    "        current_color.rgb = emissive_color.rgb;\n"
    "        for( int i = 0; i < nr_enabled_lights; ++i ) {\n"
    "          getEnabledLight( lightpos, light_diffuse, light_ambient, light_specular, i );\n"
    "          lightpos = view_to_tex * lightpos;\n"
    "          vec3 L;\n"
    "          if(lightpos.w == 0.0 ) L = normalize(lightpos.xyz);\n"
    "          else L =  normalize(lightpos.xyz-pos.xyz);\n"
    "          \n"
    "          // ray trace light to see how much reaches the current point.\n"
    "          float absorbed_light = lightAbsorbed( pos, lightpos, L, step_size );\n"
    "          light_diffuse  *= (1.0 - absorbed_light);\n"
    "          light_ambient  *= (1.0 - absorbed_light);\n"
    "          light_specular *= (1.0 - absorbed_light);\n"
    "\n"
    "          current_color.rgb += PhongLightingModel( normal.xyz,\n"
    "                                                   viewdir.xyz,\n"
    "                                                   diffuse_color,\n"
    "                                                   ambient_color,\n"
    "                                                   specular_color,\n"
    "                                                   emissive_color,\n"
    "                                                   shininess,\n"
    "                                                   L,\n"
    "                                                   light_diffuse,\n"
    "                                                   light_ambient,\n"
    "                                                   light_specular ).rgb;\n"
    "          //current_color.rgb = vec3( absorbed_light, 0, 0 );\n"
    "        }\n"
    "        current_color.a *= diffuse_color.a;\n"
    "      } else {\n"
    "        current_color = vec4( 0, 0, 0, 0 );\n"
    "      }\n"
    "    } else {\n"
    "      current_color.rgb = diffuse_color.rgb;\n"
    "      current_color.a *= diffuse_color.a;\n"
    "    }    \n"
    "  }\n"
    "}\n"
    "\n"
    "// traverse ray\n"
    "// Inputs:\n"
    "//   r0  - ray start (xyz), ray exit time (w)\n"
    "//   dir - ray direction (xyz), step length (-w)\n"
    "// Output:\n"
    "//   RayResult.color  - the computed RGBA color for this fragment\n"
    "//   RayResult.zpoint - the point to use for depth calculations,\n"
    "//                      if zpoint.x<0, no depth is computed\n"
    "RayResult traverseRay(vec4 r0, vec4 dir, vec4 color, vec4 zpoint ) {\n"
    "  //useful stuff (hopefully not computed if not needed...?)\n"
    "  //  mat4 view_to_tex = textureMatrix*gl_ModelViewMatrixInverse;\n"
    "  //  mat4 tex_to_view = gl_ModelViewMatrix*textureMatrixInverse;\n"
    "  //  vec4 viewdir_tex = vec4( -normalize(dir.xyz), 0.0 );\n"
    "  \n"
    "  // return value\n"
    "  RayResult rr;\n"
    "  \n"
    "  // initial color of this fragment is black with zero alpha\n"
    "  rr.color = color;\n"
    "  // initial depth is not defined\n"
    "  rr.zpoint = zpoint;\n"
    "  \n"
    "  // the number of ray steps to take each loop. Normally this is 1 but\n"
    "  // can be changed by various optimization techniques such as \n"
    "  // empty space skipping in order to jump over empty regions of the\n"
    "  // volume.\n"
    "  int nr_steps = 1;\n"
    "\n"
    "  //BEGIN PRE-LOOP\n"
    "  //END PRE-LOOP\n"
    "  \n"
    "  float step_length =  abs(dir.w);\n"
    "\n"
    "  // compositing loop\n"
    "  while( rr.color.a<0.95 && r0.w>=-step_length) {\n"
    "    {\n"
    "      //BEGIN INSIDE-LOOP\n"
    "      //END INSIDE-LOOP\n"
    "\n"
    "    \n"
    "      /*      if( r0.x >= 0.0 && r0.x <= 1.0 &&\n"
    "    r0.y >= 0.0 && r0.y <= 1.0 &&\n"
    "    r0.z >= 0.0 && r0.z <= 1.0 ) {\n"
    "      */\n"
    "      //BEGIN COMPOSITING\n"
    "      //END COMPOSITING\n"
    "  \n"
    "      //}\n"
    "    }\n"
    "\n"
    "    // step forward along ray\n"
    "    r0 += dir * float(nr_steps);\n"
    "    stepRayAllVolumes( nr_steps );\n"
    "  }\n"
    "  \n"
    "  //BEGIN POST-LOOP\n"
    "  //END POST-LOOP\n"
    "\n"
    "  // return result\n"
    "  return rr;\n"
    "}\n"
    "\n"
    "// main function\n"
    "void main() {\n"
    "\n"
    "  // initialize variables\n"
    "  view_matrix = gl_ModelViewMatrix * globalToLocalSpace[volumeIndex] ; \n"
    "  view_matrix_inverse = localToGlobalSpace[volumeIndex] * gl_ModelViewMatrixInverse;\n"
    "\n"
    "  // initialize ray(in global space)\n"
    "  vec4 raydir = vec4(normalize(rayDir), -1.0);\n"
    "  vec3 raystart = rayStart;\n"
    "\n"
    "  // get the screen texture coordinate for the ray start position.\n"
    "  vec2 tc = getFrameBufferTexCoords( raystart, volumeIndex ).xy;\n"
    "\n"
    "  // initialize array variables\n"
    "  initArrays( tc.xy );\n"
    "\n"
    "  //BEGIN RAY-INITIALIZATION\n"
    "  //END RAY-INITIALIZATION\n"
    "\n"
    "  raydir = vec4( rayStep * raydir.xyz, raydir.w * abs( rayStep ) );\n"
    "  vec4 raypos = vec4(raystart, 0.0);\n"
    "  \n"
    "  // initialize local ray directions and start positions\n"
    "  // for all volumes.\n"
    "  updateRayDirections( raydir.xyz );\n"
    "  updateRayStartPositions( raystart );\n"
    "\n"
    "  RayResult rr;\n"
    "\n"
    "  // initial color of this fragment is black with zero alpha\n"
    "  rr.color = vec4(0.0, 0.0, 0.0, 0.0);\n"
    "  // initial depth is not defined\n"
    "  rr.zpoint = vec4(-1.0, 0.0, 0.0, 0.0);  \n"
    "\n"
    "  // find closest\n"
    "  int closest = findClosest();\n"
    "\n"
    "  // if the closest volume is not the one currently being rendered\n"
    "  // we abort. The ray should always start at the first volume encountered.\n"
    "  if( closest != volumeIndex ) {\n"
    "    discard;\n"
    "    return;\n"
    "  }\n"
    "\n"
    "  // previous_depth is the depth of the previous intersection point of\n"
    "  // any volume. We initialize it to the depth of the closest volume.\n"
    "  float previous_depth = getVolumeDepth( closest );\n"
    "  \n"
    "  // Peel a layer for the closest volume.\n"
    "  peelLayer( closest );\n"
    "\n"
    "  // Find the new closest volume.\n"
    "  closest = findClosest();\n"
    "\n"
    "  // Traverse ray through the volumes one ray segment at a time,\n"
    "  // We define the ray segments to be the path between two following\n"
    "  // volume intersection points.\n"
    "  while( closest != -1 ) {\n"
    "    // the depth of the closest volume.\n"
    "    float current_depth = getVolumeDepth( closest );\n"
    "\n"
    "    // global position of ray-volume intersection point\n"
    "    vec3 current_pos =  frameBufferCoordToGlobalSpace( vec3( tc.xy, current_depth) );\n"
    "\n"
    "    // calculate distance(in global coordinates) from previous rayposition\n"
    "    // to next to get ray segment length.\n"
    "    vec3 diff = current_pos - raypos.xyz ;\n"
    "    float t = length( diff );\n"
    "    raypos.w = t;\n"
    "\n"
    "    // traverse ray segment\n"
    "    rr = traverseRay(raypos, raydir, rr.color, rr.zpoint );\n"
    "\n"
    "    // move to next ray segment. Use old segment end point as new segment\n"
    "    // start points.\n"
    "    raypos.xyz = current_pos;\n"
    "    updateRayStartPositions( raypos.xyz );\n"
    "    \n"
    "    // peel a layer for the closest volume\n"
    "    peelLayer( closest );\n"
    "\n"
    "    // find the new closest volume.\n"
    "    closest = findClosest();\n"
    "\n"
    "    // we have moved into empty space in between volumes so skip the current\n"
    "    // segment\n"
    "    if( closest != -1 && !isInsideAnyVolume() ) {\n"
    "      previous_depth = getVolumeDepth( closest );\n"
    "      raypos.xyz = frameBufferCoordToGlobalSpace( vec3( tc.xy, previous_depth) );\n"
    "      updateRayStartPositions( raypos.xyz );\n"
    "\n"
    "      peelLayer( closest );\n"
    "      closest = findClosest();\n"
    "      \n"
    "    }\n"
    "  }\n"
    "\n"
    "  // if final color is fully transparent, we discard the fragment.\n"
    "  if( rr.color.a == 0.0 ) discard;\n"
    "  \n"
    "  // set color\n"
    "  gl_FragColor = rr.color;\n"
    "  \n"
    "\n"
    "  /*\n"
    "  // set depth if computed\n"
    "  if( rr.zpoint.x>=0.0 ) {\n"
    "    // set depth\n"
    "    vec4 tmp =\n"
    "      gl_ProjectionMatrix*  tex_to_view_mat[volumeIndex] *  gl_TextureMatrix[0] * rr.zpoint;\n"
    "    // in window coordinatess\n"
    "    gl_FragDepth = 0.5*(gl_DepthRange.diff*tmp.z/tmp.w +\n"
    "        gl_DepthRange.near+gl_DepthRange.far);\n"
    "  } else {\n"
    "    gl_FragDepth = gl_FragCoord.z;\n"
    "    };*/\n"
    "}\n";


  // The volume geometry is used to build a proxy geometry to draw
  // the volume for ray casting. It is a box by default but can be cut
  // into other geometry by cutting the box by clip planes. 
  struct VolumeGeometry {
    // Constructor. Starts out as a box.
    VolumeGeometry( const Vec3f &size ) {
      float xp = size.x/2.0f;
      float yp = size.y/2.0f;
      float zp = size.z/2.0f;
      
      // set up geometry as box

      // front
      normals.push_back( Vec3f(0,0,1) );
      vector< Vec3f > front_points;
      front_points.push_back( Vec3f(-xp, -yp, zp) );
      front_points.push_back( Vec3f( xp, -yp, zp) );
      front_points.push_back( Vec3f( xp,  yp, zp) );
      front_points.push_back( Vec3f(-xp,  yp, zp) );
      sides.push_back( front_points );
      
      // back
      normals.push_back( Vec3f(0,0,-1) );
      vector< Vec3f > back_points;
      back_points.push_back( Vec3f(-xp,  yp, -zp) );  
      back_points.push_back( Vec3f( xp,  yp, -zp) );
      back_points.push_back( Vec3f( xp, -yp, -zp) );
      back_points.push_back( Vec3f(-xp, -yp, -zp) );
      sides.push_back( back_points );    
      
      // left
      normals.push_back( Vec3f(-1,0,0) );
      vector< Vec3f > left_points;
      left_points.push_back( Vec3f(-xp, -yp, -zp) ); 
      left_points.push_back( Vec3f(-xp, -yp,  zp) );
      left_points.push_back( Vec3f(-xp,  yp,  zp) );
      left_points.push_back( Vec3f(-xp,  yp, -zp) );
      sides.push_back( left_points );    
      
      // right
      normals.push_back( Vec3f(1,0,0) );
      vector< Vec3f > right_points;
      right_points.push_back( Vec3f(xp,  yp, -zp) );
      right_points.push_back( Vec3f(xp,  yp,  zp) );
      right_points.push_back( Vec3f(xp, -yp,  zp) );
      right_points.push_back( Vec3f(xp, -yp, -zp) );
      sides.push_back( right_points );    
      
      // top
      normals.push_back( Vec3f( 0,1,0) );
      vector< Vec3f > top_points;
      top_points.push_back( Vec3f(-xp, yp,  zp) );
      top_points.push_back( Vec3f( xp, yp,  zp) );
      top_points.push_back( Vec3f( xp, yp, -zp) );
      top_points.push_back( Vec3f(-xp, yp, -zp) );
      sides.push_back( top_points );    
      
      // bottom
      normals.push_back( Vec3f(0,-1,0) );
      vector< Vec3f > bottom_points;
      bottom_points.push_back( Vec3f(-xp, -yp, -zp) );  
      bottom_points.push_back( Vec3f( xp, -yp, -zp) );
      bottom_points.push_back( Vec3f( xp, -yp,  zp) );
      bottom_points.push_back( Vec3f(-xp, -yp,  zp) );
      sides.push_back( bottom_points );    
    }

    // Cut the geometry with a plane creating a new geometry.
    // This is not optimized in any way and does much more intersection
    // tests than needed.
    void cutGeometry( const Vec3f &plane_point,
                      const Vec3f &plane_normal ) {
      HAPI::Collision::Plane plane( plane_point, plane_normal );

      // the edges of the new side resulting from the cut.
      list< pair< Vec3f, Vec3f > > new_side_edges;
      
      // check each side for intersection
      for( SidesContainer::iterator i = sides.begin();
           i != sides.end(); ++i ) {
        
        vector< Vec3f > &points = *i; 

        // skip empty polygons.
        if( points.size() == 0 ) continue;
        
        Vec3f last_point = points.back();
        // current point behind plane
        bool behind_plane = (points.front() - plane.point)*plane.normal < 0;
        
        bool had_intersection = false;
        HAPI::Collision::IntersectionInfo last_intersection;
        
        // the points for the new side.
        vector< Vec3f > new_points;
        new_points.reserve( points.size() + 1 );

        for( vector< Vec3f >::iterator p = points.begin();
             p != points.end(); ++p ) {
          HAPI::Collision::IntersectionInfo info;
          bool intersect = plane.lineIntersect( last_point, *p, info );
          
          if( had_intersection ) {
            if( intersect ) {
              new_points.push_back( (Vec3f) info.point );
              if( behind_plane ) {
                new_side_edges.push_back( make_pair( (Vec3f)info.point,
                                                     (Vec3f)last_intersection.point ) ); 
              } else {
                new_side_edges.push_back( make_pair( (Vec3f)last_intersection.point,
                                                     (Vec3f)info.point ) );
              }
              // we are behind the plane if line goes from front to 
              // back.
              behind_plane = info.face == HAPI::Collision::FRONT;
            } 

            // only add a point if it is in front of the plane.
            if( !behind_plane ) {
              new_points.push_back( *p );
            }
            
          } else {
            if( intersect ) {
              had_intersection = true;
              last_intersection = info;
              new_points.push_back( (Vec3f) info.point );
              // we are behind the plane if line goes from front to 
              // back.
              behind_plane = info.face == HAPI::Collision::FRONT;
            } 
            // only add a point if it is in front of the plane.
            if( !behind_plane ) {
              new_points.push_back( *p );
            }
            
          }
          last_point = *p;
        }
        (*i) = new_points;
      }
      
      // add new side.
      if( new_side_edges.size() > 0 ) {
        // go through all edges and find how they are connected in order
        // to create a new polygon.
        list< pair< Vec3f, Vec3f > >::iterator i = new_side_edges.begin();
        bool connected = false;
        Vec3f first_point = (*i).first;
        Vec3f last_point = (*i).second;
        
        vector< Vec3f > side_points;
        side_points.reserve( new_side_edges.size() + 1 );
        side_points.push_back( (*i).first );
        side_points.push_back( (*i).second );
        new_side_edges.erase( i );
        
        while( !connected ) {
          // find edge that connects with this.
          for( i = new_side_edges.begin(); i != new_side_edges.end(); ++i ) {
            if( (*i).first == last_point ) {
              last_point = (*i).second;
              connected = last_point == first_point;
              if( !connected )
                side_points.push_back( last_point );
              break;
            } else if( (*i).second == last_point ) {
              last_point = (*i).first;
              connected = last_point == first_point;
              if( !connected )
                side_points.push_back( last_point );
              break;
            }
          }
          
          if( i == new_side_edges.end() ) {
            // should not happen.
            break;
          }
          
          // remove the edge we just used.
          new_side_edges.erase( i );
        }
      
        normals.push_back( -plane_normal );
        sides.push_back( side_points );
      }


    }

    // Returns the number of sides in the geometry.
    inline unsigned int nrSides() {
      return (unsigned int) sides.size();
    }

    // Returns the number of polygons for a side.
    inline unsigned int nrPolygons( unsigned int side ) {
      return (unsigned int) sides[ side ].size();
    }

    // Get the point of a given index in the polygon for a side.
    inline const Vec3f &getPoint( unsigned int side,
                                  unsigned int index ) {
      return sides[side][index];
    }

    // Get the normal for a side of the geometry.
    inline const Vec3f &getNormal( unsigned int side ) {
      return normals[side];
    }
 
    // Draw the geometry with OpenGL.
    void render() {
      for( unsigned int i = 0; i < nrSides(); ++i ) {
        const Vec3f &n = getNormal( i );
        glNormal3f( n.x, n.y, n.z );
        glBegin( GL_POLYGON );
        for( unsigned int j = 0; j < nrPolygons( i ); ++j ) {
          const Vec3f &v = getPoint( i, j );
          glVertex3f( v.x, v.y, v.z );
        }
        glEnd();
      }
    }
    
  protected:
    vector< Vec3f > normals;
    typedef vector< vector< Vec3f > > SidesContainer;
    SidesContainer sides;
  };


}



MultiVolumeRayCaster::MultiVolumeRayCaster( Inst< SFNode>  _metadata,
              Inst< SFFloatNoZero > _rayStep,
              Inst< SFBool > _stopRaysAtGeometries,
              Inst< SFBool > _useStochasticJittering ):
  H3DVolumeRendererNode( _metadata ),
  rayStep( _rayStep ),
  stopRaysAtGeometries( _stopRaysAtGeometries ),
  useStochasticJittering( _useStochasticJittering ),
  stochasticJitteringTexture( new SFTexture2DNode ),
  stochasticJitteringTextureDimension( new SFInt32 ),
  depthBufferTexture( new SFNode ),
  texToLocalSpace( new MFMatrix4f ),
  localToTexSpace( new MFMatrix4f ),
  localToGlobalSpace( new MFMatrix4f ),
  globalToLocalSpace( new MFMatrix4f ),
  volumeIndex( new SFInt32 ),
  ignore_depth_texture( false ),
  rebuildShader( new RebuildShader ) {

  type_name = "MultiVolumeRayCaster";
  database.initFields( this );

  volumeIndex->setOwner( this );
  volumeIndex->setName( "volumeIndex" );
  stochasticJitteringTexture->setOwner( this );
  stochasticJitteringTexture->setName( "stochasticJitteringTexture" );
  stochasticJitteringTextureDimension->setOwner( this );
  stochasticJitteringTextureDimension->setName( "stochasticJitteringTextureDimension" );
  rebuildShader->setOwner( this );
  rebuildShader->setName( "updateShader" );
  
  // set default values
  rayStep->setValue(0.01f);
  stopRaysAtGeometries->setValue( true );
  depthBufferTexture->setValue( new FrameBufferTexture );
  useStochasticJittering->setValue( false );
  stochasticJitteringTextureDimension->setValue( 32 );
  volumeIndex->setValue( 0 );

  // setup routes
  stopRaysAtGeometries->route( rebuildShader );
  useStochasticJittering->route( rebuildShader );

  stopRaysAtGeometries->route( paramsChanged );
  useStochasticJittering->route( paramsChanged );
  rayStep->route( paramsChanged );

  FrameBufferTextureGenerator *gen =  new FrameBufferTextureGenerator;
  gen->outputTextureType->setValue( "2D_ARRAY" );
  gen->generateDepthTexture->setValue( true );
  gen->setRenderCallback( renderDepthPeelCallback, this );
  depth_texture_generator.reset( gen );
  gen->samples->setValue( 0 );

}



void MultiVolumeRayCaster::render( X3DVolumeNode *volume ) {
 

  if( current_loop_volumes.empty() || 
      current_loop_volumes[0].first != volume ) {
    return;
  }
  
  glMatrixMode( GL_MODELVIEW );

  Matrix4f model_inverse = current_loop_volumes[0].second.inverse();
  GLfloat m_inv[16] = { model_inverse[0][0], model_inverse[1][0], model_inverse[2][0], model_inverse[3][0],
      model_inverse[0][1], model_inverse[1][1], model_inverse[2][1], model_inverse[3][1],
      model_inverse[0][2], model_inverse[1][2], model_inverse[2][2], model_inverse[3][2],
      model_inverse[0][3], model_inverse[1][3], model_inverse[2][3], model_inverse[3][3] };

  glPushMatrix();
  // multiply by model matrix inverse leaving only the view matrix in GL_MODELVIEW
  glMultMatrixf( m_inv );
  
  X3DShapeNode::GeometryRenderMode old_mode = X3DShapeNode::geometry_render_mode;
  if ( old_mode != X3DShapeNode::ALL &&
    old_mode != X3DShapeNode::SOLID ) {
    X3DShapeNode::geometry_render_mode = X3DShapeNode::ALL;
  }

  depth_texture_generator->render();
  X3DShapeNode::geometry_render_mode = old_mode;

  // drawVolumeBoxes()
  glMatrixMode( GL_MODELVIEW ); 
  glPopMatrix();

  // get the shader for the volume to render.
  VolumeShaderMap::iterator i = volume_shaders.find( volume );
  ComposedShader *shader = NULL;
  if( i == volume_shaders.end() ) {
    Console(4) << "Assertion error: MultiVolumeRayCaster::render( volume ) called with invalid volume" << endl;
    return;
  } else {
    shader = (*i).second->shader.get();
  }

#ifdef H3D_OSX
  // Due to some strange error on OSX (MacBook Pro) we need to add a glBegin/glEnd
  // pair in order for the rays to stop in the right place if we have a scene
  // with just a volume data node in it and no other geometries. Otherwise 
  // it seems like the rays are stopped at the wrong position in some places.
  if( stopRaysAtGeometries->getValue() ) {
    glBegin( GL_POINTS );
    glEnd();
  }
#endif

  rebuildShader->upToDate();

 // select and pre render shader
  shader->setSelected( true );
  glPushAttrib( shader->getAffectedGLAttribs() );
  shader->preRender();
  
  // render style
  shader->displayList->callList();

  // if filterMode is something else than default, we want the voxels texture to
  // be rendered with tri-linear interpolation. Filtering will be taken care
  // of in the shader.
  if( volume->filterType->getValue() == "NEAREST" ) {
    // voxel texture is always at texture 0
    glActiveTextureARB(GL_TEXTURE0_ARB );
    glTexParameteri( GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_NEAREST );
    glTexParameteri( GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_NEAREST );
  } else if( volume->filterType->getValue() != "DEFAULT" ) {
    // voxel texture is always at texture 0
    glActiveTextureARB(GL_TEXTURE0_ARB );
    glTexParameteri( GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
    glTexParameteri( GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
  }

  // Here follows an explanation about why multisample is disabled.
  // The reason is that when multisampling is enabled, the background
  // "bleeds" through at the borders of two volumes from certain angles.
  // The angles that give this effect are such that the edge of the bounding
  // box of one volume is seen without looking into that volume. This edge
  // must be in front of the second volume for the effect to be shown.
  // The reason is that in this case, some pixels at the edge of the first
  // volume are multisampled (multi sample anti aliasing) and the multi sampling
  // affects not only the colors, but also the alpha value. The alpha value
  // can be shown as only 25 % of the original value, which means quiet a low
  // alpha which means that the background is shown. The multi volume ray caster
  // will then, when rendering the same pixel using the second volume which is behind the
  // first one, use discard since rendering of the first volume already set the color for
  // a certain pixel and that color included a ray tracing through the second volume.
  // For all intersecting pixels they are visited by each volume. Only the volume which is
  // detected as being closest to the first ray will be used to set the color
  // of that pixel, all other volumes will do discard. The important fact to remember
  // is that this behaviour is correct and that the multisample behaviour is what screws
  // up the algorithm due to how it works.
  // The only case in which disabling multi sampling will look ugly are for volumes which
  // are completely solid at the edges, since this is not the case most of the time (in my
  // experience) we decide that shutting of multisample is sufficient (as opposed to
  // do some msaa by ourselves inside the shader).
  // Check out GL_SAMPLE_SHADING (glEnable) and the centroid keyword, one of those
  // might solve the problem.
  GLboolean multisample_enabled;
  glGetBooleanv( GL_MULTISAMPLE, &multisample_enabled );
  glDisable( GL_MULTISAMPLE );
  // render volume bounding boxes as proxy geometry
  drawVolumeBoxes( shader );
  if( multisample_enabled )
    glEnable( GL_MULTISAMPLE );

  shader->postRender();
  glPopAttrib();
}

void MultiVolumeRayCaster::drawVolumeBox( const Vec3f &dim ) {
    
  // A box representing the volume, used as proxy geometry in
  // the shader based raycasting.
    
  // dimensions
  float xp = dim.x/2.0f;
  float yp = dim.y/2.0f;
  float zp = dim.z/2.0f;

  GLfloat pm[16];
  GLfloat mv[16];
  GLint max_nr_clip_planes;
  glGetFloatv( GL_PROJECTION_MATRIX, pm );
  glGetFloatv( GL_MODELVIEW_MATRIX, mv );
  glGetIntegerv( GL_MAX_CLIP_PLANES, &max_nr_clip_planes ); 

  float max_dim = H3DMax( xp, H3DMax( yp, zp ) );
 
  // find the position of the near plane.
  Matrix4f pm_matrix( pm[0], pm[4], pm[8],  pm[12],
                      pm[1], pm[5], pm[9],  pm[13],
                      pm[2], pm[6], pm[10], pm[14],
                      pm[3], pm[7], pm[11], pm[15] );
  Matrix4f pm_matrix_inv = pm_matrix.inverse();
  Vec3f v  = pm_matrix_inv * Vec3f(0, 0, -1 );
  
  Matrix4f mv_matrix( mv[0], mv[4], mv[8],  mv[12],
                      mv[1], mv[5], mv[9],  mv[13],
                      mv[2], mv[6], mv[10], mv[14],
                      mv[3], mv[7], mv[11], mv[15] );
  Matrix4f mv_matrix_inv = mv_matrix.inverse();
  Matrix4f mv_matrix_tp = mv_matrix.transpose();
  
  // create the geometry.
  MultiVolumeRayCasterInternals::VolumeGeometry geom(dim);

  // cut with the near plane.
  Vec3f lp = mv_matrix_inv * Vec3f( 0, 0, v.z );
  Vec3f ln =  mv_matrix_inv.getScaleRotationPart() *Vec3f( 0, 0, -1 );
  ln.normalizeSafe();
  // don't know how long to move the plane in order for it not to be cut.
  // this seems to work by trying different values.
  geom.cutGeometry( lp + ln * max_dim / 1e2, ln );
  
  // cut clipplanes if active 
  if( ClipPlane::nr_active_clip_planes > 0 ) {
    for( int i = 0; i <max_nr_clip_planes; ++i ) {
      if( glIsEnabled( GL_CLIP_PLANE0 + i ) ) {
        GLdouble e[4];
        glGetClipPlane( GL_CLIP_PLANE0 + i, e );
        Vec4f eq_eye( (H3DFloat)e[0], (H3DFloat)e[1],
                      (H3DFloat)e[2], (H3DFloat)e[3] );

        // Normal vectors aren't transformed in the same way as points
        // between different coordinate spaces. To transform a plane
        // equation from model coordinates to eye coordinates the matrix
        // (MV^-1)^t is used instead of MV. To transform in the other
        // direction MV^t is used:
        Vec4f eq_world = mv_matrix_tp * eq_eye;
        Vec3f n( eq_world.x, eq_world.y, eq_world.z );
        Vec3f p( n * -eq_world.w );

        p = p + n * max_dim / 1e2;
        geom.cutGeometry( p, n );
      }
    }
  } 
  // render the geometry.
  geom.render();
}

void MultiVolumeRayCaster::buildShader( X3DVolumeNode * /*in_volume*/ ) {
  // get the shader for the volume to render. We ignore the input volume 
  // since we always want to use the shader of the first volume as the
  // shader to run for all volumes. This function will be called when
  // any volume with the same MultiVolumeRaycaster receives a rebuildShader
  // event.
  if( current_loop_volumes.empty() ) return;
  X3DVolumeNode *volume = current_loop_volumes[0].first;
  VolumeShaderMap::iterator i = volume_shaders.find( volume );
  ComposedShader *shader = NULL;
  MFString *fragmentShaderString = NULL;
  if( i == volume_shaders.end() ) {
    Console(4) << "Assertion error: MultiVolumeRayCaster::buildShader( volume ) called with invalid volume" << endl;
    return;
  } else {
    shader = (*i).second->shader.get();
    fragmentShaderString = (*i).second->fragmentShaderString.get();
  }

  shader->uniformFields.clear();
  shader->clearFields();
  shader->parts->clear();

  // setup fragment shader part
  ShaderPart *fs = new ShaderPart;
  fs->type->setValue( "FRAGMENT" );
  fs->setName( getName() + "_fragmentshader" );
  fragmentShaderString->route( fs->url );
  shader->parts->push_back( fs );

  // setup vertex shader part
  ShaderPart *vs = new ShaderPart;    
  vs->type->setValue( "VERTEX" );
  vs->setName( getName() + "_vertexshader" );
  vs->url->push_back( "glsl:"+
                      MultiVolumeRayCasterInternals::vertexshader_raycaster );

  shader->parts->push_back( vs );
    
  // setup the fragment shader code
  fragmentShaderString->resize(0);
    
  string fragmentshader;

  // TEMPORARY
  // read fragment shader from file
  /*string stylefunction_file = 
    "C:/H3D/MedX3D/src/shaders/StyleFunctions.glsl";*/
  fragmentshader += H3DVolumeRendererNode::style_function; //readShaderFromFile( stylefunction_file );

  /*string main_file = 
    "C:/H3D/MedX3D/src/shaders/MultiVolumeRayCaster_FS_main.glsl"; */
  fragmentshader +=  MultiVolumeRayCasterInternals::raycaster_fs_main; //readShaderFromFile( main_file );

  stringstream main_s;
  main_s << "glsl:#version 120" << endl;
  main_s << "#extension GL_EXT_texture_array : enable" << endl;
  main_s << "#extension GL_EXT_gpu_shader4 : enable" << endl;
  main_s << "const int nr_volumes = " << current_loop_volumes.size() << ";" << endl;
  main_s << fragmentshader;

  fragmentShaderString->push_back( main_s.str() );
  // END TEMPORARY

  stringstream s;
    

  s << "vec4 composed_color = vec4( 0, 0, 0, 0 ); " << endl;

  for( unsigned int j = 0; j < current_loop_volumes.size(); ++j ) {
    X3DVolumeNode *_volume = current_loop_volumes[j].first;
     s << "    if( isInsideVolume( " << j << ") ) { " << endl;
     s << "      mat4  view_to_tex = view_to_tex_mat["<<j<< "];  " << endl;
     s << "      mat4 tex_to_view = tex_to_view_mat[" << j << "]; " << endl;
     s << "      vec4 viewdir_tex = vec4( -normalize( volume_ray_dir[" << j << "]), 0.0 ); " << endl;
     // add the texture sampling function.
     s << "vec3 orig_r0 = r0.xyz; " << endl;
     s << "r0.xyz = volume_ray_pos[" << j << "];" << endl;
    //   s << "vec3 orig_r0 = r0.xyz; " << endl;
    // s << "r0.xyz = globalToTexSpace( orig_r0.xyz, " << j << " );" << endl;
     //s <<  "rr.color = vec4( r0.xyz ,1); "<< endl;
     //s << "  return rr;" << endl;
     s << "// getOrigSampleColor" << endl;
     s << _volume->getOrigSampleColor() << endl;
     s << "// sample manipulators(ORIG_SAMPLE_MANIP)" << endl;
     X3DTexture3DNode *tex = _volume->voxels->getValue();
     Image *image = tex ? tex->image->getValue() : NULL;
     if( image ) {
       Image::PixelType type = image->pixelType();
       if( type == Image::LUMINANCE ) {
         s << "orig_sample_color.a = orig_sample_color.r;\n" << endl;
       }
     }
     
     if( _volume->requiresDefaultNormals() ) {
       s << "vec4 sample_default_normal = normalizedNormalFromTexture( " << _volume->uniqueShaderName("defaultNormals") << ", r0 );" << endl;
     }

     s << "vec4 sample_color = orig_sample_color; " << endl;
     s << "// getShaderCode()" << endl;
     s << _volume->getShaderCode() << endl;
     s << "composed_color += sample_color;" << endl;
     s << "r0.xyz = orig_r0; " << endl;
     s << "}" << endl;
  }
  
  s << "vec4 sample_color = composed_color; " << endl;

 // add shader code
  insertFragmentShaderCode(fragmentShaderString, "//END INSIDE-LOOP", s.str() );

  // add the ray initialization code
  insertFragmentShaderCode(fragmentShaderString, "//END RAY-INITIALIZATION", getRayInitializationCode() );
  
  // add the volume node uniforms
  string uniform_s = addUniforms();
  insertFragmentShaderCode(fragmentShaderString, "//END UNIFORMS", uniform_s );
  insertFragmentShaderCode(vs->url.get(), "//END UNIFORMS", uniform_s );

  // add the functions
  insertFragmentShaderCode(fragmentShaderString, "//END UNIFORMS", volume->getShaderFunctions() );

  // add shader code
  insertFragmentShaderCode(fragmentShaderString, "//END PRE-LOOP", getShaderInitCode() );
  
  // add shader compositing code
  insertFragmentShaderCode( fragmentShaderString,
                            "//END COMPOSITING", 
                            volume->getShaderCompositingCode() );

  insertFragmentShaderCode( fragmentShaderString,"//END POST-LOOP", getShaderPostCode() );
  /*
  // add the code for the general traverseRayGeneral function used by
  // e.g. the ShadedVolumeStyle with shadows enabled.
  if( image ) {
    Image::PixelType type = image->pixelType();
    if( type == Image::LUMINANCE ) {  
      insertFragmentShaderCode(fragmentShaderString,
                               "//OPACITY ORIG_SAMPLE_MANIP", 
                               "orig_sample_color.a = orig_sample_color.r;\n" );
    }
  }
  
  if( volume->requiresDefaultNormals() ) {
    insertFragmentShaderCode(fragmentShaderString,
                             "//OPACITY ORIG_SAMPLE_MANIP", 
                             "vec4 sample_default_normal = normalizedNormalFromTexture( defaultNormals, r0 );\n" );
  }
  
  insertFragmentShaderCode(fragmentShaderString, "//OPACITY GET_ORIG_SAMPLE_COLOR", getOrigSampleColor( volume ) );
  
  // add shader code
  insertFragmentShaderCode(fragmentShaderString, "//OPACITY END INSIDE-LOOP", volume->getShaderCodeOpacityOnly() );
    
  // add shader code
  // ignore the depth texture in the light traverse ray, since it has
  // been rendered from another viewpoint.
  bool prev = ignore_depth_texture;
  ignore_depth_texture = true;
  insertFragmentShaderCode(fragmentShaderString, "//OPACITY END PRE-LOOP", getShaderInitCode( volume ) );
  ignore_depth_texture = prev;
  
  insertFragmentShaderCode( fragmentShaderString, "//OPACITY END COMPOSITING",  
                            volume->getShaderCompositingCode() );
  //                              " rr.color.a += sample_color.a *(1.0 - rr.color.a);\n" );
    
  insertFragmentShaderCode( fragmentShaderString,"//OPACITY END POST-LOOP", volume->getShaderPostCode() );
  */
  // TEMPORARY
  // save final fragment shader to file
     /* string saveshaderfile = 
    //"c:/tmp" + getName() +".glsl";
  writeShaderToFile( fragmentShaderString->getValue()[0], 
  saveshaderfile );*/
  // END TEMPORARY

  // force relinking
  shader->activate->setValue( true );
}


string MultiVolumeRayCaster::addUniforms() {
  // get the shader for the volume to render.
  if( current_loop_volumes.empty() ) return "";

  X3DVolumeNode *volume = current_loop_volumes[0].first;
  VolumeShaderMap::iterator i = volume_shaders.find( volume );
  ComposedShader *shader = NULL;
  if( i == volume_shaders.end() ) {
    Console(4) << "Assertion error: MultiVolumeRayCaster::addUniforms( volume ) called with invalid volume" << endl;
    return "";
  } else {
    shader = (*i).second->shader.get();
  }

  stringstream s;

  for( unsigned int j = 0; j < current_loop_volumes.size(); ++j ) {
    X3DVolumeNode *_volume = current_loop_volumes[j].first;
    s << _volume->addUniforms( shader );
  }

  s << addUniformToFragmentShader( shader,
           "volumeIndex",
           "int",
           H3D::Field::INPUT_OUTPUT, 
           copyAndRouteField( volumeIndex ) );

  // add the volume peel texture to the shader    
  s << addUniformToFragmentShader( shader,
                                   "volumeDepthTexture",
                                   "sampler2DArray",
                                   H3D::Field::INPUT_OUTPUT, 
                                   copyAndRouteField( depth_texture_generator->depthTexture ) );

  // add the ray step to the shader    
  s << addUniformToFragmentShader( shader,
                                   "rayStep",
                                   "float",
                                   H3D::Field::INPUT_OUTPUT, 
                                   copyAndRouteField( rayStep ) );
  
  // add the depth buffer texture
  if( stopRaysAtGeometries->getValue() ) {
    s <<  addUniformToFragmentShader( shader,
                                      "depthTexture",
                                      "sampler2D",
                                      H3D::Field::INPUT_OUTPUT, 
                                      copyAndRouteField( depthBufferTexture ) );
  }
  
  // add the matrices for transforming between texture space and local space
  s << addUniformToFragmentShader( shader,
                                   "localToTexSpace",
                                   "mat4",
                                   H3D::Field::INPUT_OUTPUT, 
                                   copyAndRouteField( localToTexSpace ),
           localToTexSpace->size() );

  s << addUniformToFragmentShader( shader,
                                   "texToLocalSpace",
                                   "mat4",
                                   H3D::Field::INPUT_OUTPUT, 
                                   copyAndRouteField( texToLocalSpace ),
           texToLocalSpace->size() );

  s << addUniformToFragmentShader( shader,
                                   "localToGlobalSpace",
                                   "mat4",
                                   H3D::Field::INPUT_OUTPUT, 
                                   copyAndRouteField( localToGlobalSpace ),
           localToGlobalSpace->size() );
  
  s << addUniformToFragmentShader( shader,
                                   "globalToLocalSpace",
                                   "mat4",
                                   H3D::Field::INPUT_OUTPUT, 
                                   copyAndRouteField( globalToLocalSpace ),
           globalToLocalSpace->size() );

  if( useStochasticJittering->getValue() ) {
    int dim = stochasticJitteringTextureDimension->getValue();
    if( !stochasticJitteringTexture->getValue() ) {
      stochasticJitteringTexture->setValue( 
                                           generateStochasticJitteringTexture( dim, dim ) );
    }
    s << addUniformToFragmentShader( shader,
                                     "stochasticJitteringTexture", 
                                     "sampler2D",
                                     H3D::Field::INPUT_OUTPUT,
                                     copyAndRouteField( stochasticJitteringTexture ) );
    s << addUniformToFragmentShader( shader,
                                     "stochasticJitteringTextureDimension", 
                                     "int",
                                     H3D::Field::INPUT_OUTPUT,
                                     copyAndRouteField( stochasticJitteringTextureDimension ) );
  }

  return s.str();
}


X3DTexture2DNode *MultiVolumeRayCaster::generateStochasticJitteringTexture( unsigned int width,
                                                                 unsigned int height ) {

  unsigned char *data = new unsigned char[width*height];
  for( unsigned int i = 0; i < width*height; ++i ) {
    data[i] = (unsigned char)( 255.*rand()/(float)RAND_MAX);
  }

  PixelTexture *pt = new PixelTexture;
  PixelImage *pi = new PixelImage( width, height, 1, 8, 
                                   Image::LUMINANCE, Image::UNSIGNED, data );
  pt->image->setValue( pi );
  return pt;
}

string MultiVolumeRayCaster::getShaderInitCode() {
  stringstream s;

  if( stopRaysAtGeometries->getValue() && 
      !ignore_depth_texture ) {
    s << "vec3 depth_buffer_value = getDepthBufferValue( r0.xyz, depthTexture );" << endl;
    s << "r0.w = min( length(depth_buffer_value - r0.xyz), r0.w );" << endl;
  } 

  for( unsigned int i = 0; i < current_loop_volumes.size(); ++i ) {
    X3DVolumeNode *volume = current_loop_volumes[i].first;
    s << volume->getShaderInitCode();
  }

  return s.str();
}


void MultiVolumeRayCaster::traverseSG( X3DVolumeNode *volume, 
               TraverseInfo &ti ) {
  H3DVolumeRendererNode::traverseSG( volume, ti );
  if( current_loop_volumes.empty() ) {
    Scene::addCallback( resetVolumesCallback, this );
    ti.addPostTraverseCallback( checkIfVolumesChangedCallback, this );
  }

  current_loop_volumes.push_back( make_pair( volume, 
               ti.getAccForwardMatrix() ) );
}


Scene::CallbackCode MultiVolumeRayCaster::resetVolumesCallback( void *data ){
  MultiVolumeRayCaster *raycaster = 
    static_cast< MultiVolumeRayCaster * >( data );
  raycaster->last_loop_volumes.clear();
  raycaster->last_loop_volumes.swap( raycaster->current_loop_volumes );
  return Scene::CALLBACK_DONE;
}

void MultiVolumeRayCaster::checkIfVolumesChangedCallback( TraverseInfo &/*ti*/, 
                void *data ) {
  MultiVolumeRayCaster *raycaster = 
    static_cast< MultiVolumeRayCaster * >( data );

  // make sure we have enough children in the generator to generate one depth buffer 
  // layer for front facing and one for back facing polygons per volume being rendered. 
  raycaster->depth_texture_generator->children->resize( 2 * raycaster->current_loop_volumes.size(), NULL ); 
  
  // check if the volumes being rendered have changed since last loop. if they have
  // rebuild the shader.
  if( !raycaster->current_loop_volumes.empty() &&
      raycaster->last_loop_volumes.size() != raycaster->current_loop_volumes.size() ) {
    // sizes changed
    raycaster->buildShader( raycaster->current_loop_volumes[0].first );
  } else {
    for( unsigned int i = 0; i < raycaster->current_loop_volumes.size(); ++i ) {

      if( raycaster->current_loop_volumes[i].first != raycaster->last_loop_volumes[i].first ) {
  // values in array changed
  raycaster->buildShader( raycaster->current_loop_volumes[0].first );
  break;
      }
    }
  }

  // set the transform matrices.
  raycaster->localToTexSpace->clear();
  raycaster->texToLocalSpace->clear();
  raycaster->localToGlobalSpace->clear();
  raycaster->globalToLocalSpace->clear();


  for( unsigned int i = 0; i < raycaster->current_loop_volumes.size(); ++i ) {
    X3DVolumeNode *volume = raycaster->current_loop_volumes[i].first;
    volume->updateDimensions->upToDate();
    Matrix4f tex_to_local = volume->textureMatrixInverse->getValue();
    Matrix4f local_to_tex = volume->textureMatrix->getValue();
    Matrix4f local_to_global = raycaster->current_loop_volumes[i].second;
    Matrix4f global_to_local = local_to_global.inverse();

    raycaster->localToTexSpace->push_back( local_to_tex );
    raycaster->texToLocalSpace->push_back( tex_to_local );
    raycaster->localToGlobalSpace->push_back( local_to_global );
    raycaster->globalToLocalSpace->push_back( global_to_local );
    
  }
}


void MultiVolumeRayCaster::renderDepthPeelCallback( FrameBufferTextureGenerator * /*gen*/, int i, void *data ) {
  
  MultiVolumeRayCaster *ray_caster = 
    static_cast< MultiVolumeRayCaster * >( data );
  glClear( GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT );

    //GLdouble mvmatrix[16];
     
      //  glGetDoublev( GL_MODELVIEW_MATRIX, mvmatrix );
  //ray_caster->drawVolumeBox( Vec3f( 0.1, 0.1, 0.1 ) );
  
  glPushAttrib( GL_POLYGON_BIT );
  glEnable( GL_CULL_FACE );

  // render front facing polygons on even indices, back facing on odd.
  if( i % 2 == 0 ) {
    glCullFace( GL_BACK );
  } else {
    glCullFace( GL_FRONT );
  }
  
  glMatrixMode( GL_MODELVIEW );  


  glPushMatrix();
  int volume_index = i / 2;
  Matrix4f m = ray_caster->current_loop_volumes[volume_index].second;
  GLfloat m_gl[16] = { m[0][0], m[1][0], m[2][0], m[3][0],
           m[0][1], m[1][1], m[2][1], m[3][1],
           m[0][2], m[1][2], m[2][2], m[3][2],
           m[0][3], m[1][3], m[2][3], m[3][3] };
  glMultMatrixf( m_gl );
  ray_caster->drawVolumeBox( ray_caster->current_loop_volumes[volume_index].first->dimensions->getValue() );
  glMatrixMode( GL_MODELVIEW );
  glPopMatrix();

  glPopAttrib();
  
}

string MultiVolumeRayCaster::getShaderPostCode() {
  stringstream s;

  for( unsigned int i = 0; i < current_loop_volumes.size(); ++i ) {
    X3DVolumeNode *volume = current_loop_volumes[i].first;
    s << volume->getShaderPostCode();
  }
  return s.str();
}

string MultiVolumeRayCaster::getRayInitializationCode() {
  stringstream s;

 for( unsigned int i = 0; i < current_loop_volumes.size(); ++i ) {
   X3DVolumeNode *volume = current_loop_volumes[i].first;
   s << volume->getRayInitializationCode();
 }

 if( useStochasticJittering->getValue() ) {
    s << 
      "   raystart = raystart + raydir.xyz * rayStep * \n"
      "              texture2D( stochasticJitteringTexture, \n"
      "                         vec2(gl_FragCoord.x / float(stochasticJitteringTextureDimension), \n"
      "                         gl_FragCoord.y / float(stochasticJitteringTextureDimension) ) ).r; \n";
 }
 return s.str();
}

void MultiVolumeRayCaster::drawVolumeBoxes( ComposedShader *shader ) {
  GLhandleARB program_handle = shader->getProgramHandle();
  if( current_loop_volumes.size() > 0 ) {
    Matrix4f volume0_global_to_local = globalToLocalSpace->getValueByIndex( 0 );
    for( unsigned int i = 0; i < current_loop_volumes.size(); ++i ) {
      glMatrixMode( GL_MODELVIEW );
      glPushMatrix();
      Matrix4f m = volume0_global_to_local * current_loop_volumes[i].second;
      GLfloat m_gl[16] = { m[0][0], m[1][0], m[2][0], m[3][0],
         m[0][1], m[1][1], m[2][1], m[3][1],
         m[0][2], m[1][2], m[2][2], m[3][2],
       m[0][3], m[1][3], m[2][3], m[3][3] };
      glMultMatrixf( m_gl );
      volumeIndex->setValue( i );
      Shaders::setGLSLUniformVariableValue( program_handle, 
              volumeIndex.get() );
      drawVolumeBox( current_loop_volumes[i].first->dimensions->getValue() );
      //      Console(4) << i << ": " << m << endl <<  current_loop_volumes[i].first->dimensions->getValue() << endl;
      glMatrixMode( GL_MODELVIEW );
      glPopMatrix();
    }  
  }
} 

bool MultiVolumeRayCaster::requiresEnabledLights() {
 for( unsigned int i = 0; i < current_loop_volumes.size(); ++i ) {
   if( current_loop_volumes[i].first->requiresEnabledLights() )
     return true;
 }

 return false;
}

bool MultiVolumeRayCaster::removeVolume( X3DVolumeNode *volume ) {
  for( vector< pair< X3DVolumeNode *, Matrix4f > >::iterator j = last_loop_volumes.begin();
       j != last_loop_volumes.end(); ++j ) {
    if( (*j).first == volume ) {
      last_loop_volumes.erase( j );
      if( last_loop_volumes.empty() )
        break;
      j = last_loop_volumes.begin();
    }
  }
  for( vector< pair< X3DVolumeNode *, Matrix4f > >::iterator j = current_loop_volumes.begin();
       j != current_loop_volumes.end(); ++j ) {
    if( (*j).first == volume ) {
      current_loop_volumes.erase( j );
      if( current_loop_volumes.empty() )
        break;
      j = current_loop_volumes.begin();
    }
  }
  return H3DVolumeRendererNode::removeVolume( volume );
}

