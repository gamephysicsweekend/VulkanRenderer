#version 450
#extension GL_ARB_separate_shader_objects : enable

/*
==========================================
uniforms
==========================================
*/

layout( binding = 0 ) uniform uboCamera {
    mat4 view;
    mat4 proj;
} camera;

/*
==========================================
attributes
==========================================
*/

layout( location = 0 ) in vec3 inPosition;
layout( location = 1 ) in vec2 inTexCoord;
layout( location = 2 ) in vec4 inNormal;
layout( location = 3 ) in vec4 inTangent;
layout( location = 4 ) in vec4 inColor;

/*
==========================================
output
==========================================
*/

layout( location = 0 ) out vec4 modelPos;

out gl_PerVertex {
    vec4 gl_Position;
};

/*
==========================================
main
==========================================
*/
void main() {
    modelPos = vec4( inPosition, 1.0 );

    vec4 viewPos = camera.view * vec4( inPosition, 0.0 );

    // Project coordinate to screen
    gl_Position = camera.proj * vec4( viewPos.xyz, 1.0 );
}