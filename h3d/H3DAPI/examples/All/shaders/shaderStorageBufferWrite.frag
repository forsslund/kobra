#version 430 compatibility

layout( std430 ) buffer SSBO {
  
  vec3 data[];
  
} ssbo_instance;

uniform int width;

uniform sampler2D earth_day;

void main( void ) {
  vec3 color = texture2D( earth_day, gl_TexCoord[0].st ).rgb;
  int index = int(gl_FragCoord.x) * width + int(gl_FragCoord.y);
  ssbo_instance.data[index] = color;

  discard;
}