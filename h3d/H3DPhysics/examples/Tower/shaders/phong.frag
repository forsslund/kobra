uniform sampler2D texture;
varying vec3 lightDir,normal, eyeVec;

void main() {
  vec4 texel;

  vec3 N = normalize(normal);
  vec3 L = normalize(lightDir);
  float lambert_term = dot(N,L);
  float intensity = abs( lambert_term );

  texel = texture2D( texture, gl_TexCoord[0].xy ); //gl_FrontMaterial.diffuse;
  texel = vec4( texel.rgb * intensity, texel.a );

  gl_FragColor = texel;
}
