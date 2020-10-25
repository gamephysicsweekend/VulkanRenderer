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
layout( binding = 1 ) uniform uboModel {
    mat4 model;
} model;
layout( binding = 2 ) uniform uboShadow {
    mat4 view;
    mat4 proj;
} shadow;

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

layout( location = 0 ) out vec4 worldNormal;
layout( location = 1 ) out vec4 modelPos;
layout( location = 2 ) out vec3 modelNormal;
layout( location = 3 ) out vec4 shadowPos;

out gl_PerVertex {
    vec4 gl_Position;
};

/*
==========================================
main
==========================================
*/
void main() {
    vec3 normal = 2.0 * ( inNormal.xyz - vec3( 0.5 ) );
	modelNormal = normal;
	modelPos = vec4( inPosition, 1.0 );
   
    // Get the tangent space in world coordinates
    worldNormal = model.model * vec4( normal.xyz, 0.0 );
   
    // Project coordinate to screen
    gl_Position = camera.proj * camera.view * model.model * vec4( inPosition, 1.0 );

    // Project the world position into the shadow texture position
    shadowPos = shadow.proj * shadow.view * model.model * vec4( inPosition, 1.0 );
}