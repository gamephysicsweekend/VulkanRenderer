//
//  ShapeConvex.cpp
//
#include "ShapeConvex.h"

/*
========================================================================================================

ShapeConvex

========================================================================================================
*/

void BuildConvexHull( const std::vector< Vec3 > & verts, std::vector< Vec3 > & hullPts, std::vector< tri_t > & hullTris ) {
	// TODO: Add code
}

/*
====================================================
ShapeConvex::Build
====================================================
*/
void ShapeConvex::Build( const Vec3 * pts, const int num ) {
	// TODO: Add code
}

/*
====================================================
ShapeConvex::Support
====================================================
*/
Vec3 ShapeConvex::Support( const Vec3 & dir, const Vec3 & pos, const Quat & orient, const float bias ) const {
	Vec3 supportPt;
	
	// TODO: Add code

	return supportPt;
}

/*
====================================================
ShapeConvex::GetBounds
====================================================
*/
Bounds ShapeConvex::GetBounds( const Vec3 & pos, const Quat & orient ) const {
	Bounds bounds;
	
	// TODO: Add code

	return bounds;
}

/*
====================================================
ShapeConvex::FastestLinearSpeed
====================================================
*/
float ShapeConvex::FastestLinearSpeed( const Vec3 & angularVelocity, const Vec3 & dir ) const {
	float maxSpeed = 0.0f;
	
	// TODO: Add code

	return maxSpeed;
}