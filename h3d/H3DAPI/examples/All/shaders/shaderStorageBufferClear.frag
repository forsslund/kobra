#version 430 compatibility

layout( std430 ) buffer SSBO {
  
  vec3 data[];
  
} ssbo_instance;

uniform int width;

void main( void ) {
  int index = int(gl_FragCoord.x) * width + int(gl_FragCoord.y);
  ssbo_instance.data[index] = vec3(0);
  discard;
}