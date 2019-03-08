#version 150 compatibility
in vec3 translation;
in vec4 color;

out vec4 color1;

void main() {
  color1 = color;
  gl_Position = gl_ModelViewProjectionMatrix * ( gl_Vertex + vec4 ( translation, 0.0 ) );
}
