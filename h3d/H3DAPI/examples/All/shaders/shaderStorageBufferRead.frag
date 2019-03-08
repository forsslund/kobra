#version 430 compatibility

layout( std430 ) buffer SSBO {
  
  vec3 data[];
  
} ssbo_instance;

uniform int width;

void main( void ) {
  int index = int(gl_FragCoord.x) * width + int(gl_FragCoord.y);
  gl_FragColor = vec4( ssbo_instance.data[index], 1 );
}