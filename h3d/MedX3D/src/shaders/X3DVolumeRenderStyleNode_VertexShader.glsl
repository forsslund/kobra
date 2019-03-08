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
/// \file X3DVolumeRenderStyleNode_VertexShader.glsl
/// \brief shader file for X3DVolumeRenderStyleNode, not used by default.
///
//
//
//////////////////////////////////////////////////////////////////////////////
// UNIFORMS
uniform mat4 textureMatrix;
uniform mat4 textureMatrixInverse;

// VARYINGS
varying vec3 rayStart;
varying vec3 rayDir;

void main() {
  rayStart = (textureMatrix*gl_Vertex).xyz; //gl_MultiTexCoord0.xyz;
  rayDir = 
    rayStart-(textureMatrix*gl_ModelViewMatrixInverse*vec4(0,0,0,1)).xyz;
  
  gl_Position = gl_ModelViewProjectionMatrix*gl_Vertex;
}
