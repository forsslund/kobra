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
/// \file Slices_FS_main.glsl
/// \brief This is the fragment shader program for volume rendering with glsl
/// shaders in the MedX3D package.
///
/// The different render styles add code to this shader program through the
/// functions addUniforms and addShaderCode. NOTE: This code can be used by
/// changing a few lines in X3DVolumeNode.cpp in order for faster development
/// cycles of how the shading part of MedX3D is done.
//
//////////////////////////////////////////////////////////////////////////////

varying vec3 viewDir;

// main function
void main() {

  mat4 view_to_tex = textureMatrix*gl_ModelViewMatrixInverse;
  mat4 tex_to_view = gl_ModelViewMatrix*textureMatrixInverse;
  vec4 viewdir_tex = vec4( -normalize(viewDir.xyz), 0.0 );

  //BEGIN PRE-LOOP
  //END PRE-LOOP
  

  vec4 r0 = gl_TexCoord[0];  
  
  //GET_ORIG_SAMPLE_COLOR

  //ORIG_SAMPLE_MANIP

  // color of this sample
  vec4 sample_color = orig_sample_color;  

  

  //BEGIN INSIDE-LOOP
  //END INSIDE-LOOP

  //  if( sample_color.a == 0.0 ) discard;
  
  // set color
  gl_FragColor = sample_color;
}
 
