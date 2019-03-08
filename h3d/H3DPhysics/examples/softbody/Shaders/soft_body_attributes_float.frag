uniform sampler2D texture;
uniform int displayMode= 0;
uniform float forceScale= 100;
varying vec3 lightDir,normal, eyeVec;
varying float interactionForceVar;

void main() {
  vec4 texel;

  vec3 N = normalize(normal);
  vec3 L = normalize(lightDir);
  float lambert_term = dot(N,L);
  float intensity = abs( lambert_term );

  texel = texture2D( texture, gl_TexCoord[0].xy ); //gl_FrontMaterial.diffuse;
  texel = vec4( texel.rgb * intensity, texel.a );

  if ( displayMode == 0 ) {
    gl_FragColor = texel;
  } else {
    // Visualize interaction forces
    gl_FragColor = vec4 ( forceScale*interactionForceVar, 0.0, 0.0, 1.0 ) + texel;
  }
}
