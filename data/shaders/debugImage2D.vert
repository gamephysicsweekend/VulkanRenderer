#version 450
#extension GL_ARB_separate_shader_objects : enable

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

layout( location = 0 ) out vec2 fragTexCoord;

out gl_PerVertex {
    vec4 gl_Position;
};

/*
==========================================
main
==========================================
*/
void main() {
    // Project coordinate to screen
    gl_Position = vec4( inPosition, 1.0 );
    fragTexCoord = inTexCoord;
}