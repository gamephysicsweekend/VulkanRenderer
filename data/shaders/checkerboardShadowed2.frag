#version 450

/*
==========================================
uniforms
==========================================
*/

layout( binding = 3 ) uniform sampler2D texShadow;

/*
==========================================
input
==========================================
*/

layout( location = 0 ) in vec4 worldNormal;
layout( location = 1 ) in vec4 modelPos;
layout( location = 2 ) in vec3 modelNormal;
layout( location = 3 ) in vec4 shadowPos;

/*
==========================================
output
==========================================
*/

layout( location = 0 ) out vec4 outColor;

/*
==========================================
GetColorFromPositionAndNormal
==========================================
*/
vec3 GetColorFromPositionAndNormal( in vec3 worldPosition, in vec3 normal ) {
    const float pi = 3.141519;

    vec3 scaledPos = worldPosition.xyz * pi * 2.0;
    vec3 scaledPos2 = worldPosition.xyz * pi * 2.0 / 10.0 + vec3( pi / 4.0 );
    float s = cos( scaledPos2.x ) * cos( scaledPos2.y ) * cos( scaledPos2.z );  // [-1,1] range
    float t = cos( scaledPos.x ) * cos( scaledPos.y ) * cos( scaledPos.z );     // [-1,1] range

    vec3 colorMultiplier = vec3( 0.5, 0.5, 1 );
    if ( abs( normal.x ) > abs( normal.y ) && abs( normal.x ) > abs( normal.z ) ) {
        colorMultiplier = vec3( 1, 0.5, 0.5 );
    } else if ( abs( normal.y ) > abs( normal.x ) && abs( normal.y ) > abs( normal.z ) ) {
        colorMultiplier = vec3( 0.5, 1, 0.5 );
    }

    t = ceil( t * 0.9 );
    s = ( ceil( s * 0.9 ) + 3.0 ) * 0.25;
    vec3 colorB = vec3( 0.85, 0.85, 0.85 );
    vec3 colorA = vec3( 1, 1, 1 );
    vec3 finalColor = mix( colorA, colorB, t ) * s;

    return colorMultiplier * finalColor;
}

// Generate a random angle based on the XY XOR hash
float randAngle() {
  uint x = uint( gl_FragCoord.x );
  uint y = uint( gl_FragCoord.y );
  return ( 30u * x ^ y + 10u * x * y );
}

/*
==========================================
main
==========================================
*/
void main() {
    vec3 dirToLight = normalize( vec3( 1, 1, 1 ) );

    // This is better than before, but it still has Moore patterns
    float dx = 0.25;
    float dy = 0.25;
    vec3 colorMultiplier = vec3( 0.0, 0.0, 0.0 );
    for ( float y = 0.0; y < 1.0; y += dy ) {
        for ( float x = 0.0; x < 1.0; x += dx ) {
            vec4 samplePos = modelPos + dFdx( modelPos ) * x + dFdy( modelPos ) * y;
            colorMultiplier += GetColorFromPositionAndNormal( samplePos.xyz, modelNormal.xyz ) * dx * dy;
        }
    }
    
	vec4 finalColor;
    finalColor.rgb = colorMultiplier.rgb;// * flux;
	finalColor.a = 1.0;

    //
    //  Shadow Mapping
    //
    float shadowDepth = shadowPos.z / shadowPos.w;
    vec2 shadowCoord = shadowPos.xy / shadowPos.w;
    shadowCoord *= 0.5;
    shadowCoord += vec2( 0.5 );
    vec3 shadowSample = texture( texShadow, shadowCoord.xy ).rgb;
    float depthDifference = shadowDepth - shadowSample.r;   // positive if in shadow
    depthDifference = max( 0.0, depthDifference ) * 1000.0;
   
    float shadowFactor = 0.0;
    float ds = 1.0 / 4096.0;
    float sampleCount = 0.0;
    
    vec2 sampleArray[ 9 ];
    sampleArray[ 0 ] = vec2( 0, 0 );
    sampleArray[ 1 ] = vec2( 1, 0.1 );
    sampleArray[ 2 ] = vec2( -0.2, 1 );
    sampleArray[ 3 ] = vec2( -1, 0.2 );
    sampleArray[ 4 ] = vec2( 0.3, -1 );

    sampleArray[ 5 ] = vec2( 1, 1 );
    sampleArray[ 6 ] = vec2( -1, 1 );
    sampleArray[ 7 ] = vec2( -1, -1 );
    sampleArray[ 8 ] = vec2( 1, -1 );

    float angle = randAngle();
    mat2 rotator;
    rotator[ 0 ][ 0 ] = cos( angle );
    rotator[ 0 ][ 1 ] = sin( angle );
    rotator[ 1 ][ 0 ] = -sin( angle );
    rotator[ 1 ][ 1 ] = cos( angle );
    for ( int i = 0; i < 9; i++ ) {
        vec2 uv = shadowCoord.xy + rotator * sampleArray[ i ] * ds;
        vec3 shadowSample = texture( texShadow, uv ).rgb;
        if ( shadowDepth > shadowSample.r ) {
            shadowFactor += 1.0;    // in shadow
        }
        sampleCount += 1.0;
    }
    
    shadowFactor /= sampleCount;
    shadowFactor = 1.0 - shadowFactor;

    float ambient = 0.5;
    float flux = clamp( dot( worldNormal.xyz, dirToLight.xyz ), 0.0, 1.0 - ambient ) * shadowFactor + ambient;
    finalColor.rgb = colorMultiplier.rgb * flux;

    outColor = finalColor;
}