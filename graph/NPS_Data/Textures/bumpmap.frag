//
// Fragment shader for bump mapping in surface local coordinates
//
// Author: Randi Rost
//
// Copyright (c) 2002-2004 3Dlabs Inc. Ltd.
//

uniform sampler2D normalMap;
uniform sampler2D baseTexture;
varying vec3 lightDir; 	  // interpolated surface local coordinate light direction 
varying vec3 viewDir;     // interpolated surface local coordinate view direction

const float diffuseFactor  = 0.7;
const float specularFactor = 0.7;
//vec3 basecolor = vec3 (0.8, 0.7, 0.3);
//uniform vec3 basecolor;
vec3 basecolor;
void main (void)
{
    vec3 norm;
    vec3 r;
    vec3 color;
    float intensity;
    float spec;
    float d;

    // Fetch normal from normal map
    norm = vec3(texture2D(normalMap, vec2 (gl_TexCoord[0])));
    norm = (norm - 0.5) * 2.0;
    norm.y = -norm.y;

    intensity = max(dot(lightDir, norm), 0.0) * diffuseFactor;

    // Compute specular reflection component

    d = 2.0 * dot(lightDir, norm);
    r = d * norm;
    r = lightDir - r;
    spec = pow(max(dot(r, viewDir), 0.0) , 6.0) * specularFactor;
    intensity += min (spec, 1.0);

     // Compute final color value
     // jas:
    basecolor = vec3(texture2D(baseTexture, vec3 (gl_TexCoord[0])));

    color = clamp(basecolor * intensity, 0.0, 1.0);
 
    // Write out final fragment color

    gl_FragColor = vec4 (color, 1.0);
    
}
