#version 450

/*
==========================================
uniforms
==========================================
*/

layout( binding = 0 ) uniform sampler2D texDiffuse;

/*
==========================================
input
==========================================
*/

layout( location = 0 ) in vec2 fragTexCoord;

/*
==========================================
output
==========================================
*/

layout( location = 0 ) out vec4 outColor;

/*
==========================================
main
==========================================
*/
void main() {
    vec3 diffuse = texture( texDiffuse, fragTexCoord ).rgb;
    
    vec4 finalColor;
    finalColor.r = diffuse.r;
    finalColor.g = diffuse.g;
    finalColor.b = diffuse.b;
    finalColor.a = 1.0;

    outColor = finalColor;
}