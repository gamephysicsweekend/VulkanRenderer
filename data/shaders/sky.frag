#version 450

/*
==========================================
input
==========================================
*/

layout( location = 0 ) in vec4 modelPos;

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
    vec3 sky = vec3( 0.5, 0.7, 1.0 );
    vec3 horizon = vec3( 1, 1, 1 );

    float t = abs( clamp( 0.5 + 0.5 * modelPos.z, 0.0, 1.0 ) );
    vec3 skycolor = mix( horizon, sky, t );

    outColor = vec4( skycolor.xyz, 1.0 );
}