#version 110

uniform mat4 projection;
uniform mat4 modelView;

attribute vec3 position;
attribute vec3 normal;
attribute vec3 color;

varying vec3 fragPosition;
varying vec3 fragNormal;
varying vec3 fragDiffuse;

void main()
{
    vec4 eye_position = modelView * vec4(position, 1.0);
    gl_Position = projection * eye_position;
    fragPosition = eye_position.xyz;
    fragNormal = (modelView * vec4(normal, 0.0)).xyz;
    fragDiffuse = color;
}
