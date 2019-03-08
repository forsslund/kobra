
varying vec3 lightDir, normal, eyeVec;
  
void main() {
  normal = gl_NormalMatrix * gl_Normal;
  vec4 vertex_global = gl_ModelViewMatrix * gl_Vertex;
  eyeVec = -vec3(vertex_global);
  lightDir = vec3(gl_LightSource[0].position.xyz - vertex_global.xyz);

  gl_TexCoord[0] = gl_TextureMatrix[0] * gl_MultiTexCoord0;
  gl_Position = ftransform();
  gl_ClipVertex = vertex_global;
} 

