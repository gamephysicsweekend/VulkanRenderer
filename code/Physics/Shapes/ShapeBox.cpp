//
//  Shapes.cpp
//
#include "ShapeBox.h"

/*
========================================================================================================

ShapeBox

========================================================================================================
*/

/*
====================================================
ShapeBox::Build
====================================================
*/
void ShapeBox::Build( const Vec3 * pts, const int num ) {
	// TODO: Add code
}

/*
====================================================
ShapeBox::Support
====================================================
*/
Vec3 ShapeBox::Support( const Vec3 & dir, const Vec3 & pos, const Quat & orient, const float bias ) const {
	Vec3 supportPt;
	
	// TODO: Add code

	return supportPt;
}

/*
====================================================
ShapeBox::InertiaTensor
====================================================
*/
Mat3 ShapeBox::InertiaTensor() const {
	Mat3 tensor;
	
	// TODO: Add code

	return tensor;
}

/*
====================================================
ShapeBox::GetBounds
====================================================
*/
Bounds ShapeBox::GetBounds( const Vec3 & pos, const Quat & orient ) const {
	Bounds bounds;
	
	// TODO: Add code

	return bounds;
}

/*
====================================================
ShapeBox::FastestLinearSpeed
====================================================
*/
float ShapeBox::FastestLinearSpeed( const Vec3 & angularVelocity, const Vec3 & dir ) const {
	float maxSpeed = 0.0f;
	
	// TODO: Add code

	return maxSpeed;
}