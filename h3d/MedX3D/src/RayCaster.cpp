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
/// \file RayCaster.cpp
/// \brief CPP file for RayCaster.
///
//
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/ClipPlane.h>

#include <H3D/MedX3D/RayCaster.h>
#include <H3D/MedX3D/FrameBufferTexture.h>
#include <H3D/MedX3D/X3DVolumeNode.h>
#include <H3D/Pixel3DTexture.h>
#include <H3D/PixelTexture.h>
#include <H3D/ShaderPart.h>

using namespace H3D;

H3DNodeDatabase RayCaster::database( "RayCaster", 
                                     &newInstance< RayCaster >, 
                                     typeid( RayCaster ),
                                     &H3DVolumeRendererNode::database );

namespace RayCasterInternals {
  FIELDDB_ELEMENT( RayCaster, rayStep, INPUT_OUTPUT )
  FIELDDB_ELEMENT( RayCaster, stopRaysAtGeometries, INPUT_OUTPUT )
  FIELDDB_ELEMENT( RayCaster, useEmptySpaceSkipping, INPUT_OUTPUT )
  FIELDDB_ELEMENT( RayCaster, showNonEmptySpace, INPUT_OUTPUT )
  FIELDDB_ELEMENT( RayCaster, useStochasticJittering, INPUT_OUTPUT )

  // The glsl vertex shader for perspective projection. Ray direction goes from camera 
  // position towards ray start if perspective projection and constant direction
  // for all points( (0 0 -1) in camera space) if orthographic projection
  //
   const string vertexshader_raycaster =
    "//BEGIN UNIFORMS\n"
    "uniform mat4 textureMatrix;\n"
    "uniform mat4 textureMatrixInverse;\n"
    "//END UNIFORMS\n"
    "\n"
    "//BEGIN VARYINGS\n"
    "varying vec3 rayStart;\n"
    "varying vec3 rayDir;\n"
    "//END VARYINGS\n"
    "\n"
    "void main() {\n"
    "  rayStart = (gl_TextureMatrix[0] * textureMatrix*gl_Vertex).xyz;\n"
    "  mat4 camera_to_tex_space =gl_TextureMatrix[0] * textureMatrix*gl_ModelViewMatrixInverse; \n"
    " if( gl_ProjectionMatrix[2][3] == 0.0 ) { \n"
    "    // orthographic projection. \n"
    "  rayDir = (camera_to_tex_space*vec4(0,0,-1,1)).xyz - (camera_to_tex_space*vec4(0,0,0,1)).xyz;\n"
    " } else { \n" 
    "    // perspective projection. \n" 
    "    rayDir = rayStart-(camera_to_tex_space*vec4(0,0,0,1)).xyz;\n"
    " } \n"
    "  gl_Position = gl_ModelViewProjectionMatrix*gl_Vertex;\n"
    "  gl_ClipVertex = gl_ModelViewMatrix*gl_Vertex;\n"
    "}\n";


 
  // "(MEDX3D_DIRECTORY)/src/shaders/RayCaster_FS_main.glsl"


  const string raycaster_fs_main = 
    "//////////////////////////////////////////////////////////////////////////////\n"
    "//    Copyright 2004-2019, SenseGraphics AB\n"
    "//\n"
    "//    This file is part of H3D API.\n"
    "//\n"
    "//    H3D API is free software; you can redistribute it and/or modify\n"
    "//    it under the terms of the GNU General Public License as published by\n"
    "//    the Free Software Foundation; either version 2 of the License, or\n"
    "//    (at your option) any later version.\n"
    "//\n"
    "//    H3D API is distributed in the hope that it will be useful,\n"
    "//    but WITHOUT ANY WARRANTY; without even the implied warranty of\n"
    "//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the\n"
    "//    GNU General Public License for more details.\n"
    "//\n"
    "//    You should have received a copy of the GNU General Public License\n"
    "//    along with H3D API; if not, write to the Free Software\n"
    "//    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA\n"
    "//\n"
    "//    A commercial license is also available. Please contact us at \n"
    "//    www.sensegraphics.com for more information.\n"
    "//\n"
    "//////////////////////////////////////////////////////////////////////////////\n"
    "//\n"
    "// This is the fragment shader program for volume rendering with glsl\n"
    "// shaders in the MedX3D package.\n"
    "// \n"
    "// The different render styles add code to this shader program through the\n"
    "// functions addUniforms and addShaderCode.\n"
    "//\n"
    "//////////////////////////////////////////////////////////////////////////////\n"
    "\n"
    "//BEGIN UNIFORMS\n"
    "//END UNIFORMS\n"
    "\n"
    "//BEGIN VARYINGS\n"
    "varying vec3 rayStart;\n"
    "varying vec3 rayDir;\n"
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
    "// given a 3d position in texture space and a texture with the \n"
    "// depth buffer value we return the position in texture space that\n"
    "// corresponds to the position at the same screen coordinate as \n"
    "// the given coordinate but at the depth of the depth texture.\n"
    "vec3 getDepthBufferValue( vec3 pos, //in texture space\n"
    "                           sampler2D depth_texture ) {\n"
    "  // transform to normalized screen coordinates\n"
    "  vec4 screen_coords = gl_ModelViewProjectionMatrix *\n"
    "                           textureMatrixInverse * vec4( pos, 1 );\n"
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
    "  screen_coords = gl_ModelViewProjectionMatrixInverse * screen_coords;\n"
    "  screen_coords = screen_coords / screen_coords.w;\n"
    "  screen_coords = textureMatrix * screen_coords;\n"
    "\n"
    "  return screen_coords.xyz;\n"
    "}\n"
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
    "      empty_space_step.x * (floor( abs(empty_space_pos.x - r0.x) / abs(empty_space_step.x) ) + 1.0) * step( 0.0, ray_sign.x*(r0.x - empty_space_pos.x ));\n"
    "    empty_space_pos.y += \n"
    "      empty_space_step.y * (floor( abs(empty_space_pos.y - r0.y) / abs(empty_space_step.y) ) + 1.0) * step( 0.0, ray_sign.y*(r0.y - empty_space_pos.y ));\n"
    "\n"
    "    empty_space_pos.z += \n"
    "      empty_space_step.z * (floor( abs(empty_space_pos.z - r0.z) / abs(empty_space_step.z) ) + 1.0) * step( 0.0, ray_sign.z*(r0.z - empty_space_pos.z ));\n"
    "    \n"
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
    "  mat4 view_to_tex = textureMatrix*gl_ModelViewMatrixInverse;\n"
    "  mat4 tex_to_view = gl_ModelViewMatrix*textureMatrixInverse;\n"
    "  vec4 viewdir_tex = vec4( -normalize(dir.xyz), 0.0 );\n"
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
    "\n"
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
    "  \n"
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
    "// traverse ray\n"
    "// Inputs:\n"
    "//   r0  - ray start (xyz), ray exit time (w)\n"
    "//   dir - ray direction (xyz), step length (-w)\n"
    "// Output:\n"
    "//   RayResult.color  - the computed RGBA color for this fragment\n"
    "//   RayResult.zpoint - the point to use for depth calculations,\n"
    "//                      if zpoint.x<0, no depth is computed\n"
    "RayResult traverseRay(vec4 r0, vec4 dir) {\n"
    "  //useful stuff (hopefully not computed if not needed...?)\n"
    "  mat4 view_to_tex = textureMatrix*gl_ModelViewMatrixInverse;\n"
    "  mat4 tex_to_view = gl_ModelViewMatrix*textureMatrixInverse;\n"
    "  vec4 viewdir_tex = vec4( -normalize(dir.xyz), 0.0 );\n"
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
    "\n"
    "  //BEGIN PRE-LOOP\n"
    "  //END PRE-LOOP\n"
    "  \n"
    "  // compositing loop\n"
    "  while( rr.color.a<0.95 && r0.w>=0.0 ) {\n"
    "         \n"
    "    //GET_ORIG_SAMPLE_COLOR\n"
    "\n"
    "    //ORIG_SAMPLE_MANIP\n"
    "\n"
    "    // color of this sample\n"
    "    vec4 sample_color = orig_sample_color;  \n"
    "    \n"
    "    //BEGIN INSIDE-LOOP\n"
    "    //END INSIDE-LOOP\n"
    "    \n"
    "    //BEGIN COMPOSITING\n"
    "    //END COMPOSITING\n"
    "    \n"
    "    // step forward along ray\n"
    "    r0 += dir * float(nr_steps);\n"
    "  }\n"
    "  \n"
    "  //BEGIN POST-LOOP\n"
    "  //END POST-LOOP\n"
    "\n"
    "  // moved discard from here because of problems with AMD GPUs\n"
    "  //if( rr.color.a == 0.0 ) discard;\n"
    "  if( rr.color.a == 0.0 )\n"
    "    rr.color.a = -1.0;\n"
    "  \n"
    "  // return result\n"
    "  return rr;\n"
    "}\n"
    "\n"
    "// main function\n"
    "void main() {\n"
    "  // initialize ray\n"
    "  vec4 raydir = vec4(normalize(rayDir), -1.0);\n"
    "  vec3 raystart = rayStart; \n"
    "  //BEGIN RAY-INITIALIZATION \n"
    "  //END RAY-INITIALIZATION \n"
    " \n"
    "  float texit = rayexit(raystart,raydir.xyz); \n"
    "  raydir = vec4( rayStep * raydir.xyz, raydir.w * abs( rayStep ) );\n"
    "  vec4 raypos = vec4(raystart, texit);\n"
    "  \n"
    "  // traverse ray\n"
    "  RayResult rr = traverseRay(raypos, raydir);\n"
    "  \n"
    "  // if there nothing along the ray discard, else set color\n"
    "  // n.b. moved discard from previous position because of weird AMD bug\n"
    "  if( rr.color.a == -1.0 )\n"
    "    discard;\n"
    "  else\n"
    "    gl_FragColor = rr.color;\n"
    "  \n"
    "  // set depth if computed\n"
#if 0 // Set to 1 if compiling for ATI and using old ATI drivers.
    "  /*if( rr.zpoint.x>=0.0 ) {\n"
    "    // set depth\n"
    "    vec4 tmp =\n"
    "      gl_ModelViewProjectionMatrix*textureMatrixInverse*rr.zpoint;\n"
    "    // in window coordinates\n"
    "    gl_FragDepth = 0.5*(gl_DepthRange.diff*tmp.z/tmp.w +\n"
    "      gl_DepthRange.near+gl_DepthRange.far);\n"
    "  } else {\n"
    "    gl_FragDepth = gl_FragCoord.z;\n"
    "  }\n"*/
#else
    "  if( rr.zpoint.x>=0.0 ) {\n"
    "    // set depth\n"
    "    vec4 tmp =\n"
    "      gl_ModelViewProjectionMatrix*textureMatrixInverse*rr.zpoint;\n"
    "    // in window coordinates\n"
    "    gl_FragDepth = 0.5*(gl_DepthRange.diff*tmp.z/tmp.w +\n"
    "      gl_DepthRange.near+gl_DepthRange.far);\n"
    "  } else {\n"
    "    gl_FragDepth = gl_FragCoord.z;\n"
    "  }\n"
#endif
    "  ;\n"
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



RayCaster::RayCaster( Inst< SFNode>  _metadata,
                      Inst< SFFloatNoZero > _rayStep,
                      Inst< SFBool  > _stopRaysAtGeometries,
                      Inst< SFBool  > _useEmptySpaceSkipping,
                      Inst< SFBool  > _showNonEmptySpace,
                      Inst< SFBool  > _useStochasticJittering ):
  H3DVolumeRendererNode( _metadata ),
  rayStep( _rayStep ),
  stopRaysAtGeometries( _stopRaysAtGeometries ),
  useEmptySpaceSkipping( _useEmptySpaceSkipping ),
  showNonEmptySpace( _showNonEmptySpace ),
  useStochasticJittering( _useStochasticJittering ),
  stochasticJitteringTexture( new SFTexture2DNode ),
  stochasticJitteringTextureDimension( new SFInt32 ),
  depthBufferTexture( new SFNode ),
  ignore_depth_texture( false ),
  rebuildShader( new RebuildShader ) {

  type_name = "RayCaster";
  database.initFields( this );

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
  useEmptySpaceSkipping->setValue( false );
  showNonEmptySpace->setValue( false );
  useStochasticJittering->setValue( false );
  stochasticJitteringTextureDimension->setValue( 32 );

  // setup routes
  stopRaysAtGeometries->route( rebuildShader );
  useEmptySpaceSkipping->route( rebuildShader );
  useStochasticJittering->route( rebuildShader );

  stopRaysAtGeometries->route( paramsChanged );
  useEmptySpaceSkipping->route( paramsChanged );
  useStochasticJittering->route( paramsChanged );
  showNonEmptySpace->route( paramsChanged );
  rayStep->route( paramsChanged );

//  forceRebuildShader->route( rebuildShader );
  
}

void renderBox( RGB color, Vec3f min, Vec3f max ) {
  glDisable( GL_LIGHTING );
  glColor3f( color.r, color.g, color.b );

  glBegin( GL_LINE_STRIP );
  glVertex3f( min.x, min.y, min.z );
  glVertex3f( min.x, max.y, min.z );
  glVertex3f( max.x, max.y, min.z );
  glVertex3f( max.x, min.y, min.z );
  glVertex3f( min.x, min.y, min.z );
  glEnd();

  glBegin( GL_LINE_STRIP );
  glVertex3f( min.x, min.y, max.z );
  glVertex3f( min.x, max.y, max.z );
  glVertex3f( max.x, max.y, max.z );
  glVertex3f( max.x, min.y, max.z );
  glVertex3f( min.x, min.y, max.z );
  glEnd();

  glBegin( GL_LINES );
  glVertex3f( min.x, min.y, max.z );
  glVertex3f( min.x, min.y, min.z );
  glVertex3f( max.x, min.y, max.z );
  glVertex3f( max.x, min.y, min.z );
  glVertex3f( min.x, max.y, max.z );
  glVertex3f( min.x, max.y, min.z );
  glVertex3f( max.x, max.y, max.z );
  glVertex3f( max.x, max.y, min.z );
  glEnd();
  glEnable( GL_LIGHTING );
}


void RayCaster::render( X3DVolumeNode *volume ) {

  // get the shader for the volume to render.
  VolumeShaderMap::iterator i = volume_shaders.find( volume );
  ComposedShader *shader = NULL;
  if( i == volume_shaders.end() ) {
    Console(4) << "Assertion error: RayCaster::render( volume ) called with invalid volume" << endl;
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

  if( showNonEmptySpace->getValue() ) {
    X3DTexture3DNode *t_minmax = volume->emptySpaceMinMaxTexture->getValue();
    X3DTexture2DNode *t_class = volume->emptySpaceClassificationTexture->getValue();
    Image *i_minmax = NULL;
    Image *i_class = NULL;
    if( t_minmax ) i_minmax = t_minmax->image->getValue();
    if( t_class ) i_class = t_class->image->getValue();
    if( i_minmax && i_class ) {
      H3DInt32 resolution = volume->emptySpaceSkippingRes->getValue();
      Vec3f dim = volume->dimensions->getValue();
      
      H3DFloat step = 1.0f / resolution;

      for( H3DInt32 z = 0; z < resolution; ++z ) {
        for( H3DInt32 y = 0; y < resolution; ++y ) {
          for( H3DInt32 x = 0; x < resolution; ++x ) {
            RGBA c = i_minmax->getSample( x * step + step / 2,
                                          y * step + step / 2,
                                          z * step + step / 2,
                                          Image::NEAREST );
            RGBA empty_c = 
              i_class->getSample( c.r, c.a, 0.5f, Image::NEAREST );
            if( empty_c.r > 0.5f ) {

              Vec3f min, max;
              min = Vec3f( (x * step - 0.5f) * dim.x,
                           (y * step - 0.5f) * dim.y,
                           (z * step - 0.5f) * dim.z );
              max = Vec3f( ((x+1) * step - 0.5f) * dim.x,
                           ((y+1) * step - 0.5f) * dim.y,
                           ((z+1) * step - 0.5f) * dim.z ); 
              renderBox( RGB( 0, 0, 1 ), min, max );
            } 
          }
        }
      }
    }
  }

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

  // render volume bounding box as proxy geometry
  drawVolumeBox( volume->dimensions->getValue() );

  shader->postRender();
  glPopAttrib();
}

void RayCaster::drawVolumeBox( const Vec3f &dim ) {
    
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
  RayCasterInternals::VolumeGeometry geom(dim);

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

void RayCaster::buildShader( X3DVolumeNode *volume ) {
 // get the shader for the volume to render.
  VolumeShaderMap::iterator i = volume_shaders.find( volume );
  ComposedShader *shader = NULL;
  MFString *fragmentShaderString = NULL;
  if( i == volume_shaders.end() ) {
    Console(4) << "Assertion error: RayCaster::buildShader( volume ) called with invalid volume" << endl;
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
                      RayCasterInternals::vertexshader_raycaster );

  shader->parts->push_back( vs );
    
  // setup the fragment shader code
  fragmentShaderString->resize(0);
    
  string fragmentshader;

  // TEMPORARY
  // read fragment shader from file
  /*string stylefunction_file = 
    "C:/H3D/MedX3D/src/shaders/StyleFunctions.glsl";*/
  fragmentshader += H3DVolumeRendererNode::style_function;//readShaderFromFile( stylefunction_file );

  /*string main_file = 
    "C:/H3D/MedX3D/src/shaders/RayCaster_FS_main.glsl";*/
  fragmentshader += RayCasterInternals::raycaster_fs_main;//readShaderFromFile( main_file );

  fragmentShaderString->push_back( "glsl:" + fragmentshader );
  // END TEMPORARY
    
  X3DTexture3DNode *tex = volume->voxels->getValue();
  Image *image = tex ? tex->image->getValue() : NULL;
  if( image ) {
    Image::PixelType type = image->pixelType();
    if( type == Image::LUMINANCE ) {  
      insertFragmentShaderCode(fragmentShaderString,
                               "//ORIG_SAMPLE_MANIP", 
                               "orig_sample_color.a = orig_sample_color.r;\n" );
    }
  }

  if( volume->requiresDefaultNormals() ) {
    insertFragmentShaderCode(fragmentShaderString,
                             "//ORIG_SAMPLE_MANIP", 
                             "vec4 sample_default_normal = normalizedNormalFromTexture( " + volume->uniqueShaderName( "defaultNormals" ) + ", r0 );\n" );
  }


  // add the ray initialization code
  insertFragmentShaderCode(fragmentShaderString, "//END RAY-INITIALIZATION", getRayInitializationCode( volume ) );
  
  // add the texture sampling function.
  insertFragmentShaderCode(fragmentShaderString, "//GET_ORIG_SAMPLE_COLOR", getOrigSampleColor( volume ) );

  // add the volume node uniforms
  insertFragmentShaderCode(fragmentShaderString, "//END UNIFORMS", addUniforms( volume ) );

  // add the functions 
  insertFragmentShaderCode(fragmentShaderString, "//END UNIFORMS", volume->getShaderFunctions() );

  // add shader code
  insertFragmentShaderCode(fragmentShaderString, "//END INSIDE-LOOP", volume->getShaderCode() );

  // add shader code
  insertFragmentShaderCode(fragmentShaderString, "//END PRE-LOOP", getShaderInitCode( volume ) );
  
  // add shader compositing code
  insertFragmentShaderCode( fragmentShaderString,
                            "//END COMPOSITING", 
                            volume->getShaderCompositingCode() );

  insertFragmentShaderCode( fragmentShaderString,"//END POST-LOOP", volume->getShaderPostCode() );

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
                              "vec4 sample_default_normal = normalizedNormalFromTexture( " + volume->uniqueShaderName( "defaultNormals" ) + ", r0 );\n" );
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

  // TEMPORARY
  // save final fragment shader to file
  /*  string saveshaderfile = 
    "c:/tmp" + getName() +".glsl";
  writeShaderToFile( fragmentShaderString->getValue()[0], 
         saveshaderfile );
  */// END TEMPORARY

  // force relinking
  shader->activate->setValue( true );
}


string RayCaster::addUniforms( X3DVolumeNode *volume ) {
  // get the shader for the volume to render.
  VolumeShaderMap::iterator i = volume_shaders.find( volume );
  ComposedShader *shader = NULL;
  if( i == volume_shaders.end() ) {
    Console(4) << "Assertion error: RayCaster::addUniforms( volume ) called with invalid volume" << endl;
    return "";
  } else {
    shader = (*i).second->shader.get();
  }

  stringstream s;

  s << volume->addUniforms( shader );

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
  
  // add the empty space skipping textures
  if( useEmptySpaceSkipping->getValue() ) {
    SFNode* f = new TypedSFNode< X3DTexture3DNode >();
    volume->emptySpaceMinMaxTexture->route( f );
    s <<  addUniformToFragmentShader( shader,
                                      "emptySpaceMinMaxTexture",
                                      "sampler3D",
                                      H3D::Field::INPUT_OUTPUT, 
                                      f );
    
    SFNode* f2 = new TypedSFNode< X3DTexture2DNode >();
    volume->emptySpaceClassificationTexture->route( f2 );
    s <<  addUniformToFragmentShader( shader,
                                      "emptySpaceClassificationTexture",
                                      "sampler2D",
                                      H3D::Field::INPUT_OUTPUT, 
                                      f2 );
  }
  
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


X3DTexture2DNode *RayCaster::generateStochasticJitteringTexture( unsigned int width,
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

string RayCaster::getShaderInitCode( X3DVolumeNode *volume ) {
  stringstream s;

  if( useEmptySpaceSkipping->getValue() ) {
    
    s << "  // the sign for each axis of the ray direction" << endl;
    s << "  vec3 ray_sign = vec3( sign( dir.x ), sign( dir.y ), sign( dir.z ) );" << endl;
    s << "" << endl;
    s << "  // the position the ray was at the last time we checked for empty space." << endl;
    s << "  vec3 empty_space_pos = -2.0*ray_sign; //vec3( 0.0, 0.0, 0.0 );" << endl;
    s << "" << endl;
    s << "  // the size of each subvolume of the empty space min/max texture in " << endl;
    s << "  // texture space. The sign of the value is the same as the" << endl;
    s << "  // sign of the ray direction." << endl;
    H3DInt32 res = volume->emptySpaceSkippingRes->getValue();
    s << "  vec3 empty_space_step = vec3( ray_sign.x * 1.0 / " << res << ".0," << endl;
    s << "                                ray_sign.y * 1.0 / " << res << ".0," << endl;
    s << "                                ray_sign.z * 1.0 / " << res << ".0 );" << endl;
  }

  if( stopRaysAtGeometries->getValue() && 
      !ignore_depth_texture ) {
    s << "vec3 depth_buffer_value = getDepthBufferValue( r0.xyz, depthTexture );" << endl;
    s << "r0.w = min( length(depth_buffer_value - r0.xyz), r0.w );" << endl;
  } 

  s << volume->getShaderInitCode();

  return s.str();
}


string RayCaster::getOrigSampleColor( X3DVolumeNode *volume ) {
  stringstream s;

  s << volume->getOrigSampleColor();

  if( useEmptySpaceSkipping->getValue() ) {
    s << "nr_steps = getNrSteps( r0.xyz, ray_sign, dir.xyz, " << endl;
    s << "                       empty_space_pos, empty_space_step," << endl;
    s << "                       emptySpaceMinMaxTexture," << endl;
    s << "                       emptySpaceClassificationTexture );" << endl;
  }

  return s.str();
}

string RayCaster::getRayInitializationCode( X3DVolumeNode *volume ) {
  stringstream s;

  s << volume->getRayInitializationCode();

  if( useStochasticJittering->getValue() )
    return 
      "   raystart = raystart + raydir.xyz * rayStep * \n"
      "              texture2D( stochasticJitteringTexture, \n"
      "                         vec2(gl_FragCoord.x / float(stochasticJitteringTextureDimension), \n"
      "                         gl_FragCoord.y / float(stochasticJitteringTextureDimension) ) ).r; \n";
  else
    return "";
}
