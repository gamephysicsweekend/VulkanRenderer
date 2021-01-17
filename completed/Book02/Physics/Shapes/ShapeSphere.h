//
//	ShapeSphere.h
//
#pragma once
#include "ShapeBase.h"

/*
====================================================
ShapeSphere
====================================================
*/
class ShapeSphere : public Shape {
public:
	explicit ShapeSphere( const float radius ) : m_radius( radius ) {
		m_centerOfMass.Zero();
	}

	Vec3 Support( const Vec3 & dir, const Vec3 & pos, const Quat & orient, const float bias ) const override;

	Mat3 InertiaTensor() const override;

	Bounds GetBounds( const Vec3 & pos, const Quat & orient ) const override;
	Bounds GetBounds() const override;

	shapeType_t GetType() const override { return SHAPE_SPHERE; }

public:
	float m_radius;
};