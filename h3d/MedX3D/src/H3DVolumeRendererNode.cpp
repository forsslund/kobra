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
/// \file H3DVolumeRendererNode.cpp
/// \brief CPP file for H3DVolumeRendererNode.
///
//
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/MedX3D/H3DVolumeRendererNode.h>
#include <fstream>

using namespace H3D;

H3DNodeDatabase H3DVolumeRendererNode::database( "H3DVolumeRendererNode", 
                                                 NULL, 
                                                 typeid( H3DVolumeRendererNode ),
                                                 &X3DNode::database );

namespace H3DVolumeRendererNodeInternals {
  
}

 // The glsl fragment shader
// "(MEDX3D_DIRECTORY)/src/shaders/StyleFunctions.glsl"
const string H3DVolumeRendererNode::style_function =
    "//////////////////////////////////////////////////////////////////////////////\n"
    "//    Copyright 2012-2019, SenseGraphics AB\n"
    "//\n"
    "//    This file is part of MedX3D.\n"
    "//\n"
    "//    MedX3D is free software; you can redistribute it and/or modify\n"
    "//    it under the terms of the GNU General Public License as published by\n"
    "//    the Free Software Foundation; either version 2 of the License, or\n"
    "//    (at your option) any later version.\n"
    "//\n"
    "//    MedX3D is distributed in the hope that it will be useful,\n"
    "//    but WITHOUT ANY WARRANTY; without even the implied warranty of\n"
    "//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the\n"
    "//    GNU General Public License for more details.\n"
    "//\n"
    "//    You should have received a copy of the GNU General Public License\n"
    "//    along with MedX3D; if not, write to the Free Software\n"
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
    "// compute t such that r0+t*dir is the exit point of the [0,1] box\n"
    "// t is returned\n"
    "float rayexitOrig(vec3 r0, vec3 dir) {\n"
    "  // cube side length is 1 in texture coordinates \n"
    "  // q is the number of ray steps in each component to\n"
    "  // go a full side. \n"
    "  vec3 q = 1.0/(dir+1.0e-6);\n"
    "\n"
    "  // r0 [0-1] \n"
    "  // Since the texture coordinates of the cube side is 0-1, r0 will be\n"
    "  // the fraction of the cube side\n"
    "  // t0 is the number of steps required for each component to reach\n"
    "  // 0 from r0  \n"
    "  vec3 t0 = -r0*q;\n"
    "\n"
    "  // if t0 is negative in a component, dir does not go towards 0\n"
    "  // in that case we want t in that component to be the direction\n"
    "  // towards 1 instead.\n"
    "  vec3 tmax = max(t0,q+t0);\n"
    " \n"
    "  // choose t to be the component that requires the least number of\n"
    "  // steps to reach the cube boundary.\n"
    "  return min(tmax.x,min(tmax.y,tmax.z));\n"
    "}\n"
    "\n"
    "//BEGIN MISC FUNCTIONS\n"
    "// compute t such that r0+t*dir is the exit point of the [0,1] box\n"
    "float rayexit(vec3 r0, vec3 dir) {\n"
    "  // this works just as rayexitOrig with the difference that we first\n"
    "  // transform the texture space with the texture matrix.\n"
    "  vec3 r0_local = (gl_TextureMatrixInverse[0] * vec4( r0, 1 )).xyz;\n"
    "  vec3 dir_local = (gl_TextureMatrixInverse[0] * vec4( dir, 0 )).xyz;\n"
    "\n"
    "  vec3 q = 1.0/(dir_local+1.0e-6);\n"
    "  //  q.x = 0.5 / (dir.x+1.0e-6);\n"
    "  vec3 t0 = -r0_local*q;\n"
    "  //  t0.x = -r0.x/0.5 *q.x;\n"
    "  vec3 tmax = max(t0,q+t0);\n"
    "  float t_local = min(tmax.x,min(tmax.y,tmax.z));\n"
    "\n"
    "  //  return length((textureMatrix * vec4( t_local, 0, 0, 0 )).xyz);\n"
    "  return t_local;\n"
    "}\n"
    "\n"
    "/// tex_coord - texture coordinate to evaluate color at. \n"
    "/// tex_source - the source texture \n"
    "/// tex_hg - filter offsets and weights \n"
    "/// e_x - texel size in x direction \n"
    "/// e_y - texel size in y direction \n"
    "/// e_z - texel size in z direction \n"
    "/// size_source - source texture size in pixels \n"
    "vec4 filterTriCubic( vec3 tex_coord, \n"
    "                     sampler3D tex_source, \n"
    "                     sampler2D tex_hg, \n"
    "                     vec3 e_x, \n"
    "                     vec3 e_y, \n"
    "                     vec3 e_z, \n"
    "                     vec3 size_source ) { \n"
    "  // calculate filter texture coordinates where [0,1] is a single texel. \n"
    "  vec3 coord_hg = tex_coord * size_source - vec3( 0.5, 0.5, 0.5 ); \n"
    " \n"
    "  // fetch offsets and weights from filter texture \n"
    "  vec3 hg_x = texture2D( tex_hg, vec2(coord_hg.s, 0.5) ).xyz; \n"
    "  vec3 hg_y = texture2D( tex_hg, vec2(coord_hg.t, 0.5) ).xyz; \n"
    "  vec3 hg_z = texture2D( tex_hg, vec2(coord_hg.r, 0.5) ).xyz; \n"
    " \n"
    "  // determine linear sampling coordinates \n"
    "  vec3 t000 = tex_coord - hg_x.y * e_x - hg_y.y * e_y - hg_z.y * e_z; \n"
    "  vec3 t100 = tex_coord + hg_x.x * e_x - hg_y.y * e_y - hg_z.y * e_z; \n"
    "  vec3 t010 = tex_coord - hg_x.y * e_x + hg_y.x * e_y - hg_z.y * e_z; \n"
    "  vec3 t110 = tex_coord + hg_x.x * e_x + hg_y.x * e_y - hg_z.y * e_z; \n"
    " \n"
    "  vec3 t001 = tex_coord - hg_x.y * e_x - hg_y.y * e_y + hg_z.x * e_z; \n"
    "  vec3 t101 = tex_coord + hg_x.x * e_x - hg_y.y * e_y + hg_z.x * e_z; \n"
    "  vec3 t011 = tex_coord - hg_x.y * e_x + hg_y.x * e_y + hg_z.x * e_z; \n"
    "  vec3 t111 = tex_coord + hg_x.x * e_x + hg_y.x * e_y + hg_z.x * e_z; \n"
    " \n"
    "  // fetch linearly interpolated inputs. \n"
    "  vec4 c000 = texture3D( tex_source, t000 ); \n"
    "  vec4 c100 = texture3D( tex_source, t100 ); \n"
    "  vec4 c010 = texture3D( tex_source, t010 ); \n"
    "  vec4 c110 = texture3D( tex_source, t110 ); \n"
    "  vec4 c001 = texture3D( tex_source, t001 ); \n"
    "  vec4 c101 = texture3D( tex_source, t101 ); \n"
    "  vec4 c011 = texture3D( tex_source, t011 ); \n"
    "  vec4 c111 = texture3D( tex_source, t111 ); \n"
    " \n"
    "  // weight along z direction \n"
    "  vec4 c00 = mix( c000, c001, hg_z.z ); \n"
    "  vec4 c10 = mix( c100, c101, hg_z.z ); \n"
    "  vec4 c01 = mix( c010, c011, hg_z.z ); \n"
    "  vec4 c11 = mix( c110, c111, hg_z.z ); \n"
    " \n"
    "  // weight along y direction \n"
    "  vec4 c0 = mix( c00, c01, hg_y.z ); \n"
    "  vec4 c1 = mix( c10, c11, hg_y.z ); \n"
    " \n"
    "  // weight along x direction \n"
    "  vec4 c = mix( c0, c1, hg_x.z ); \n"
    " \n"
    "  return c; \n"
    "} \n"
    "\n"
    "\n"
    "// Get the normal from a 3D texture where, for RGB,\n"
    "// the interval [0,0.5] means [-1.0,0] and [0.5,1.0] means [0.0,1.0].\n"
    "// The A component contain the normalized gradient magnitude.\n"
    "vec4 normalFromTexture(sampler3D normals, vec4 pos) {\n"
    "  vec4 n = texture3D(normals, pos.xyz);\n"
    "  n.xyz = 2.0*n.xyz-1.0;\n"
    "  return n;\n"
    "}\n"
    "vec4 normalizedNormalFromTexture(sampler3D normals, vec4 pos) {\n"
    "  vec4 n = texture3D(normals, pos.xyz);\n"
    "  n.xyz = 2.0*n.xyz-1.0;\n"
    "  if( length(n.xyz) > 0.001 )\n"
    "    n.xyz = normalize(n.xyz);\n"
    "  return n;\n"
    "}\n"
    "\n"
    "// convert from rgb to hsv \n"
    "vec4 RGBToHSV( vec4 rgba ) { \n"
    "  vec4 return_value; \n"
    "  float v, x, f, i; \n"
    "  float R = rgba.r, G = rgba.g, B = rgba.b; \n"
    "  x = min(R, min( G, B ) ); \n"
    "  v = max(R, max( G, B ) ); \n"
    "  if(v == x) \n"
    "    return_value = vec4(0, 0, v, rgba.a); \n"
    "  else { \n"
    "    f = (R == x) ? G - B : ((G == x) ? B - R : R - G); \n"
    "    i = (R == x) ? 3.0 : ((G == x) ? 5.0 : 1.0); \n"
    "    return_value = vec4(i - f /(v - x), (v - x)/v, v, rgba.a); \n"
    "  } \n"
    "  return return_value; \n"
    "}\n"
    "\n"
    "vec3 RGBToHSV( vec3 rgb ) {\n"
    "  return RGBToHSV( vec4(rgb,1.0) ).rgb;\n"
    "}\n"
    "\n"
    "// convert from hsv to rgb \n"
    "vec4 HSVToRGB( vec4 hsva ) { \n"
    "  vec4 return_value; \n"
    "  float h = hsva.x, s = hsva.y, v = hsva.z, m, n, f; \n"
    "  float i; \n"
    "  if( h == 0.0 ) \n"
    "    return_value = vec4(v, v, v, hsva.a); \n"
    "  else { \n"
    "    i = floor(h); \n"
    "    f = h - i; \n"
    "    float t = i / 2.0; \n"
    "    if( t - floor( t ) >  0.1 ) \n"
    "      f = 1.0 - f; // if i is even \n"
    "    m = v * (1.0 - s); \n"
    "    n = v * (1.0 - s * f); \n"
    "    if( i == 6.0 || i == 0.0 ) return_value = vec4(v, n, m, hsva.a); \n"
    "    else if( i == 1.0 ) return_value = vec4(n, v, m, hsva.a); \n"
    "    else if( i == 2.0 ) return_value = vec4(m, v, n, hsva.a); \n"
    "    else if( i == 3.0 ) return_value = vec4(m, n, v, hsva.a); \n"
    "    else if( i == 4.0 ) return_value = vec4(n, m, v, hsva.a); \n"
    "    else if( i == 5.0 ) return_value = vec4(v, m, n, hsva.a); \n"
    "    // should never happen \n"
    "    else return_value = vec4( 0, 0, 0, 1 ); \n"
    "  }\n"
    "  return return_value; \n"
    "}\n"
    "\n"
    "vec3 HSVToRGB( vec3 hsv ) {\n"
    "  return HSVToRGB( vec4(hsv,1.0) ).rgb;\n"
    "}\n"
    "//END MISC FUNCTIONS\n"
    "\n"
    "//BEGIN STYLE FUNCTIONS\n"
    "// OpacityMapVolumeStyle\n"
    "// Notes:\n"
    "void OpacityMapVolumeStyle2D( inout vec4 sample_color,\n"
    "                              vec4 orig_color,\n"
    "                              vec4 pos,\n"
    "                              bool enabled,\n"
    "                              int nr_components,\n"
    "                              sampler2D transferFunction) {\n"
    "  if( enabled ) {\n"
    "    vec2 tex_coord = vec2( 0.0, 0.0 );\n"
    "    if( nr_components == 1 ) {\n"
    "      tex_coord.s = orig_color.r;\n"
    "    } else if( nr_components == 2 ) {\n"
    "      tex_coord.s = orig_color.r;\n"
    "      tex_coord.t = orig_color.a;\n"
    "    } else {\n"
    "      tex_coord.s = orig_color.r;\n"
    "      tex_coord.t = orig_color.g;\n"
    "    }\n"
    "    sample_color = texture2D(transferFunction, tex_coord );\n"
    "  }\n"
    "}\n"
    "\n"
    "void OpacityMapVolumeStyle3D( inout vec4 sample_color,\n"
    "                              vec4 orig_color,\n"
    "                              vec4 pos,\n"
    "                              bool enabled,\n"
    "                              int nr_components,\n"
    "                              sampler3D transferFunction) {\n"
    "\n"
    "\n"
    "  if( enabled ) {\n"
    "    vec3 tex_coord = vec3( 0.0, 0.0, 0.0 );\n"
    "   \n"
    "    if( nr_components == 1 ) {\n"
    "      tex_coord.s = orig_color.r;\n"
    "    } else if( nr_components == 2 ) {\n"
    "      tex_coord.s = orig_color.r;\n"
    "      tex_coord.t = orig_color.a;\n"
    "    } else {\n"
    "      tex_coord.s = orig_color.r;\n"
    "      tex_coord.t = orig_color.g;\n"
    "      tex_coord.r = orig_color.b;\n"
    "    } \n"
    "\n"
    "    sample_color = texture3D(transferFunction, tex_coord);\n"
    "  }\n"
    "}\n"
    "\n"
    "void OpacityMapVolumeStylePreIntegrated(inout vec4 sample_color, \n"
    "                                        vec4 orig_color, \n"
    "                                        vec4 pos, \n"
    "                                        bool enabled, \n"
    "                                        vec4 last_step_orig_color, \n"
    "                                        sampler2D preintegrated_values ) { \n"
    "\n"
    "  if( enabled ) { \n"
    "    vec2 tex_coord = vec2( last_step_orig_color.r, orig_color.r );  \n"
    "    sample_color = texture2D(preintegrated_values, tex_coord); \n"
    "  }\n"
    "}\n"
    "\n"
    "\n"
    "// EdgeEnhancementVolumeStyle\n"
    "// Notes: \n"
    "void EdgeEnhancementVolumeStyle( inout vec4 current_color,\n"
    "                                 vec4 pos,\n"
    "                                 vec4 viewdir,\n"
    "                                 bool enabled,\n"
    "                                 vec3 edgeColor,\n"
    "                                 float gradientThreshold,\n"
    "                                 vec4 normal) {\n"
    "  if( enabled ) {\n"
    "    if( normal.a > 0.001 ) {\n"
    "      //float cosgt = 1.0-cos(gradientThreshold);\n"
    "      //float cosnv = 1.0-abs(dot(normal.xyz,viewdir.xyz));\n"
    "      //current_color += contribution*cosnv*step(cosgt,cosnv)*edgeColor;\n"
    "      \n"
    "      // according to updated spec.\n"
    "      // Note: In the spec it says that the gradientThreshold\n"
    "      // is the minimum angle, but the formula compares the dot product,\n"
    "      // i.e., cosine of the angle.\n"
    "      float nv = abs( dot( normal.xyz, viewdir.xyz ) );\n"
    "      if( nv < cos(gradientThreshold) ) {\n"
    "        current_color.rgb = mix(current_color.rgb, edgeColor, 1.0-nv);\n"
    "      }\n"
    "    }\n"
    "  }\n"
    "}\n"
    "\n"
    "// BoundaryEnhancementVolumeStyle\n"
    "void BoundaryEnhancementVolumeStyle( inout vec4 current_color,\n"
    "                                     vec4 pos,\n"
    "                                     bool enabled,\n"
    "                                     float retainedOpacity,\n"
    "                                     float boundaryOpacity,\n"
    "                                     float opacityFactor,\n"
    "                                     vec4 normal ) {\n"
    "  if( enabled ) {\n"
    "    current_color.a *= \n"
    "      (retainedOpacity + boundaryOpacity*pow(normal.a,opacityFactor));\n"
    "  }\n"
    " \n"
    "  \n"
    "}\n"
    "\n"
    "// SilhouetteEnhancementVolumeStyle\n"
    "// Notes: In the updated spec, the text is wrong. silhouetteFactor\n"
    "// is not a member of this style \n"
    "void SilhouetteEnhancementVolumeStyle( inout vec4 current_color,\n"
    "                                       vec4 pos,\n"
    "                                       vec4 viewdir,\n"
    "                                       bool enabled,\n"
    "                                       float silhouetteBoundaryOpacity,\n"
    "                                       float silhouetteRetainedOpacity,\n"
    "                                       float silhouetteSharpness,\n"
    "                                       vec4 normal ) {\n"
    "  if( enabled ) {\n"
    "    if( normal.a > 0.001 ) {\n"
    "      float a = 1.0-abs(dot(normal.xyz,viewdir.xyz));\n"
    "      float b = \n"
    "      silhouetteRetainedOpacity + \n"
    "      silhouetteBoundaryOpacity*pow(a,silhouetteSharpness);\n"
    "      current_color.a *= b;\n"
    "    } else {\n"
    "      current_color.a = 0.0;\n"
    "    }\n"
    "  }\n"
    "}\n"
    "\n"
    "// ToneMappedVolumeStyle\n"
    "void ToneMappedVolumeStyle( inout vec4 current_color,\n"
    "                            vec4 pos,\n"
    "                            mat4 view_to_tex,\n"
    "                            bool enabled,\n"
    "                            vec4 coolColor,\n"
    "                            vec4 warmColor,\n"
    "                            vec4 normal) {\n"
    "  if( enabled ) {\n"
    "    if( normal.a > 0.001 ) {\n"
    "      current_color.rgb = vec3( 0, 0, 0 );\n"
    "      for( int i = 0; i < nr_enabled_lights; ++i ) {\n"
    "        getEnabledLight( lightpos, light_diffuse,\n"
    "                         light_ambient, light_specular, i );\n"
    "        lightpos = view_to_tex * lightpos;\n"
    "        vec3 L;\n"
    "        if(lightpos.w == 0.0 ) L = normalize(lightpos.xyz);\n"
    "        else L =  normalize(lightpos.xyz-pos.xyz);\n"
    "\n"
    "        // according to X3D spec\n"
    "        float cc = (1.0+dot(L,normal.xyz))*0.5;\n"
    "        current_color.rgb += mix(coolColor.rgb, warmColor.rgb, cc); \n"
    "      }\n"
    "    } else {\n"
  "      current_color = mix(coolColor, warmColor, 0.5);\n"
    "      //current_color.rgb = mix(coolColor.rgb, warmColor.rgb, 0.5);\n"
  "      //current_color.a = 0.0;\n"
    "    }\n"
    "  }\n"
    "}\n"
    "\n"
    "// CartoonVolumeStyle\n"
    "// Notes:\n"
    "void CartoonVolumeStyle( inout vec4 current_color,\n"
    "                         vec4 pos,\n"
    "                         vec4 viewdir,\n"
    "                         bool enabled,\n"
    "                         vec4 parallelColorHSV,\n"
    "                         vec4 orthogonalColorHSV,\n"
    "                         int colorSteps,\n"
    "                         vec4 normal ) {\n"
    "  \n"
    "  if( enabled && colorSteps >= 1 ) {\n"
    "    if( normal.a > 0.001 ) {\n"
    "      float cos_a = dot( normal.xyz, viewdir.xyz );\n"
    "      if( cos_a < 0.0 ) {\n"
    "        current_color.rgb = vec3( 0.0, 0.0, 0.0 );\n"
    "      } else {\n"
    "        float step = 1.0 / float(colorSteps);\n"
    "        float interval = floor( cos_a / step );\n"
    "        if( interval >= float(colorSteps) )\n"
    "          interval = float(colorSteps) - 1.0;\n"
    "        float w = interval * step;\n"
    "        current_color.rgb = \n"
    "        HSVToRGB( orthogonalColorHSV * w + parallelColorHSV * (1.0-w) ).rgb;\n"
    "        // 2 -> 0-0.5-1           0.5\n"
    "        // 3 -> 0-0.33-0.66-1     0.33\n"
    "        // 4 -> 0-0.25-0.5-0.75-1 0.25\n"
    "      }\n"
    "    } else {\n"
    "      current_color.a = 0.0;\n"
    "    }\n"
    "  }\n"
    "  \n"
    "}\n"
    "\n"
#ifdef INCLUDE_DEPRECATED  
    "// MIPVolumeStyle\n"
    "// Notes:\n"
    "void MIPVolumeStyle( inout float maxvalue,\n"
    "                     inout vec4 maxpos,\n"
    "                     vec4 pos,\n"
    "                     bool enabled) {\n"
    "  \n"
    "  if( enabled ) {\n"
    "    vec4 col = texture3D(voxels, pos.xyz);\n"
    "    float val = max( col.r, max(col.g,col.b) );\n"
    "    if( val > maxvalue ) {\n"
    "      maxvalue = val;\n"
    "      maxpos = vec4(pos.xyz,1.0);\n"
    "    }\n"
    "  }\n"
    "  \n"
    "}\n"
    "void MIPVolumeStyle_POST( inout vec4 finalcolor,\n"
    "                          inout vec4 finalpos,\n"
    "                          float maxvalue,\n"
    "                          vec4 maxpos,\n"
    "                          bool enabled,\n"
    "                          sampler2D transferFunction) {\n"
    "  \n"
    "  if( enabled ) {\n"
    "    finalcolor = texture2D(transferFunction, vec2(maxvalue,0.5));\n"
    "    finalpos = maxpos;\n"
    "  }\n"
    "  \n"
    "}\n"
    "\n"
#endif
    "vec4 PhongLightingModel( vec3 normal,\n"
    "                         vec3 viewdir, \n"
    "                         vec4 material_diffuse, \n"
    "                         vec4 material_ambient, \n"
    "                         vec4 material_specular, \n"
    "                         vec4 material_emission, \n"
    "                         float material_shininess, \n"
    "                         vec3 lightdir,\n"
    "                         vec4 light_diffuse,\n"
    "                         vec4 light_ambient,\n"
    "                         vec4 light_specular ) {\n"
    "\n"
    "  // ambient\n"
    "  vec3 ambient = light_ambient.rgb * material_ambient.rgb;\n"
    "\n"
    "  float ndotl = dot(normal, lightdir);\n"
    "\n"
    "  // diffuse\n"
    "  vec3 diffuse = \n"
    "    light_diffuse.rgb * material_diffuse.rgb * max(ndotl,0.0);\n"
    "\n"
    "  vec3 specular = vec3(0,0,0);\n"
    "  // specular\n"
    "  if (ndotl > 0.0) {\n"
    "    vec3 reflv = normalize( reflect(-lightdir,normal) );\n"
    "    float rdotv = dot(reflv,viewdir);\n"
    "    if( rdotv > 0.0 ) {\n"
    "      specular = light_specular.rgb*material_specular.rgb * pow( rdotv, material_shininess);\n"
    "    }\n"
    "  }\n"
    "  \n"
    "\n"
    "  vec4 return_color;  \n"
    "  return_color.rgb = material_emission.rgb + ambient + diffuse + specular; \n"
    "  return_color.a = material_diffuse.a;\n"
    "\n"
    "  return return_color;\n"
    "}\n"
    "\n"
    "void ShadedVolumeStyle( inout vec4 current_color,\n"
    "                        vec4 emissive_color, \n"
    "                        vec4 diffuse_color, \n"
    "                        vec4 ambient_color, \n"
    "                        vec4 specular_color, \n"
    "                        float shininess, \n"
    "                        vec4 pos,\n"
    "                        vec4 viewdir,\n"
    "                        mat4 view_to_tex,\n"
    "                        bool enabled,\n"
    "                        bool lighting,\n"
    "                        vec4 normal ) {\n"
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
    "\n"
    "        }\n"
    "        current_color.a *= diffuse_color.a;\n"
    "      } else {\n"
    "      current_color = vec4( 0, 0, 0, 0 );\n"
    "      }\n"
    "    } else {\n"
    "      current_color.rgb = diffuse_color.rgb;\n"
    "      current_color.a *= diffuse_color.a;\n"
    "    }\n"
    "  }\n"
    "}\n"
    "\n"
#ifdef INCLUDE_DEPRACATED
    "// ISOSurfaceVolumeStyle\n"
    "// Notes:\n"
    "void ISOSurfaceVolumeStyle( inout vec4 current_color,\n"
    "                            vec4 dir,\n"
    "                            vec3 r,\n"
    "                            vec3 r1,\n"
    "                            float v,\n"
    "                            float v1,\n"
    "                            vec4 viewdir,\n"
    "                            vec4 lightpos,\n"
    "                            float isovalue,\n"
    "                            bool shadows,\n"
    "                            sampler3D surfaceNormals) {\n"
    "\n"
    "  // interpolate position\n"
    "  float a = clamp( (isovalue-v)/(v1-v), 0.0, 1.0 );\n"
    "  r = mix(r,r1,a);\n"
    "\n"
    "  // lookup transfer function\n"
    "  vec4 tfcol = vec4(0.5, 0.5, 0.5, 1 ); //texture2D(transferFunction,vec2(isovalue,0.5));\n"
    "\n"
    "  // set the opacity\n"
    "  current_color.a = tfcol.a;\n"
    "\n"
    "  // lighting\n"
    "  if( true /*lighting*/ ) {\n"
    "    vec4 ldir = vec4( normalize(lightpos.xyz-r.xyz), 1.0 );\n"
    "\n"
    "    // shadowing\n"
    "    bool shdw=false;\n"
    "    if( shadows ) {\n"
    "      float lexit = rayexit(r.xyz, ldir.xyz);\n"
    "      vec4 ldir2 = 3.0*dir.w*ldir;\n"
    "      vec4 lpos = vec4(r.xyz, lexit);\n"
    "      lpos += ldir2;\n"
    "\n"
    "      while( lpos.w>=0.0 && shdw==false ) {\n"
    "        float lv = texture3D(voxels, lpos.xyz).r;\n"
    "        float lv1 = texture3D(voxels, lpos.xyz+ldir2.xyz).r;\n"
    "        shdw = (min(lv,lv1)<=isovalue && isovalue<=max(lv,lv1));\n"
    "        lpos += ldir2;\n"
    "      }\n"
    "    }\n"
    "\n"
    "    if( !shdw ) {\n"
    "      vec4 normal = \n"
    "      normalizedNormalFromTexture(surfaceNormals,vec4(r,1.0));\n"
    "      // re-orient normal\n"
    "      if( dot(viewdir.xyz,normal.xyz)<0.0 )\n"
    "        normal.xyz = -normal.xyz;\n"
    "\n"
    "      vec3 L;\n"
    "      if(lightpos.w == 0.0 ) L = normalize(lightpos.xyz);\n"
    "      else L = ldir.xyz;\n"
    "\n"
    "      current_color = PhongLightingModel( normal.xyz,\n"
    "                                          viewdir.xyz,\n"
    "                                          gl_FrontMaterial.diffuse,\n"
    "                                          gl_FrontMaterial.ambient,\n"
    "                                          gl_FrontMaterial.specular,\n"
    "                                          gl_FrontMaterial.emission,\n"
    "                                          gl_FrontMaterial.shininess,\n"
    "                                          L,\n"
    "                                          gl_LightSource[0].diffuse,\n"
    "                                          gl_LightSource[0].ambient,\n"
    "                                          gl_LightSource[0].specular );\n"
    "    } // shdw\n"
    "\n"
    "  } // lighting \n"
    "\n"
    "}\n"
#endif
    "//END STYLE FUNCTIONS\n";



H3DVolumeRendererNode::H3DVolumeRendererNode( 
                                             Inst< SFNode>  _metadata ):
  X3DNode( _metadata ),
  paramsChanged( new Field )
{

  type_name = "H3DVolumeRendererNode";
  database.initFields( this );
  
  paramsChanged->setOwner( this );
  paramsChanged->setName( "paramsChanged" );
}


bool H3DVolumeRendererNode::
insertFragmentShaderCode( MFString *fragmentShaderString,
                          const string &add_before, 
                          const string &code_to_add) {
  if( code_to_add == "" ) return true;

  string fs = fragmentShaderString->getValue()[0];
  string::size_type pos = fs.find(code_to_add);
  if( pos != string::npos ) {
    //Console(3) << "Error in insertFragmentShaderCode, \"" 
    //           << code_to_add << "\" already added." << endl;
   // return false;
  }
  pos = fs.find(add_before);
  if( pos == string::npos ) {
    Console(3) << "Error in insertFragmentShaderCode, could not find \"" 
               << add_before << "\"" << endl;
    return false;
  }
  fs.insert(pos, code_to_add + "\n");
  fragmentShaderString->setValue( 0, fs );
  return true;
}

string H3DVolumeRendererNode::
readShaderFromFile(const string &filename) {
  ifstream is( filename.c_str() );
  string str;
  char c;
  while( is.get(c) ) 
    str += c;
  is.close();
    return str;
}

void H3DVolumeRendererNode::
writeShaderToFile(const string &shaderstring, 
                  const string &filename) {
  string s = shaderstring;
  
  // remove "glsl:"    
  string glsl_str = s.substr(0,5);
  if( glsl_str == "GLSL:" || glsl_str == "glsl:" ) {
    s = s.substr(5,s.size()-1);
    }
  
  ofstream os( filename.c_str() );
  for(int i=0; i<(int)s.size(); ++i) {
    os << s[i];
  }
  os.close();
}

string H3DVolumeRendererNode::
addUniformToFragmentShader( ComposedShader *shader,
          const string &name,
          const string &glsl_type,
          const Field::AccessType &access,
          Field *field,
          int array_size,
          bool delete_unadded_field ) {
    
  // if field successfully added, we add corresponding code 
  // to the fragment shader
  bool ok = shader->addField( name, access, field ); 
  if( ok ) {
    stringstream s;
    if( array_size == 0 ) s << "[1]";
    else if( array_size != -1 ) s << "[" << array_size << "]";
    string array_string = s.str();
    
    return "uniform " + glsl_type + " " + name + array_string + ";\n";
  } else {
    if( delete_unadded_field )
      delete field;
    return "";
  }
}


void H3DVolumeRendererNode::addVolume( X3DVolumeNode *volume ) {
  VolumeShaderMap::iterator i = volume_shaders.find( volume );
  if( i != volume_shaders.end() ) {
    delete (*i).second;
  }

  volume_shaders[ volume ] = new RendererShaderInfo;
  volume_shaders[ volume ]->shader->displayList->route( paramsChanged );
}

bool H3DVolumeRendererNode::removeVolume( X3DVolumeNode *volume ) {
  VolumeShaderMap::iterator i = volume_shaders.find( volume );
  if( i != volume_shaders.end() ) {
    delete (*i).second;
    volume_shaders.erase( i );
    return true;
  } else {
    return false;
  }
}


H3DVolumeRendererNode::RendererShaderInfo::RendererShaderInfo():
  shader( new ComposedShader ),
  fragmentShaderString( new MFString ) {

  fragmentShaderString->setName( "fragmentShaderString" );
  // set shader stuff
  shader->language->setValue("GLSL");
  shader->parts->resize( 2 );
#if H3DAPI_MAJOR_VERSION >= 2
#if H3DAPI_MINOR_VERSION > 1
  shader->suppressUniformWarnings->setValue( true );
#endif
#endif
  shader->initialize();
}
