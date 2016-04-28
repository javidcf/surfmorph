#version 110

uniform mat4 projection;
uniform mat4 modelView;

varying vec3 fragPosition;
varying vec3 fragNormal;
varying vec3 fragDiffuse;

// const vec3 lightDirection = vec3(0.408248, -0.816497, 0.408248);
const vec3 eyeLightDirection = vec3(0, 0, -1);

const vec4 lightDiffuse = vec4(0.8, 0.8, 0.8, 0.0);
const vec4 lightAmbient = vec4(0.2, 0.2, 0.2, 1.0);
const vec4 lightSpecular = vec4(1.0, 1.0, 1.0, 1.0);

const float shininess = 20.0;
const float specular = 0.5;

void main()
{
    // vec3 eyeLightDirection = (modelView * vec4(lightDirection, 0.0)).xyz;

    vec3 normal = normalize(fragNormal);
    vec3 eye = normalize(fragPosition);
    vec3 reflection = reflect(eyeLightDirection, normal);

    vec4 diffuseFactor
        = max(-dot(normal, eyeLightDirection), 0.0) * lightDiffuse;
    vec4 ambientDiffuseFactor
        = diffuseFactor + lightAmbient;
    vec4 specularFactor
        = max(pow(-dot(reflection, eye), shininess), 0.0)
            * lightSpecular;
    
    gl_FragColor = specularFactor * specular
        + ambientDiffuseFactor * vec4(fragDiffuse, 0.0);
    // gl_FragColor = vec4(fragDiffuse, 1.0);
    // gl_FragColor = vec4(1.0, 1.0, 1.0, 1.0);
}
