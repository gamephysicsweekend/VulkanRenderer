//
//	ShapeConvex.h
//
#pragma once
#include "ShapeBase.h"

struct tri_t {
	int a;
	int b;
	int c;
};

struct edge_t {
	int a;
	int b;

	bool operator == ( const edge_t & rhs ) const {
		return ( ( a == rhs.a && b == rhs.b ) || ( a == rhs.b && b == rhs.a ) );
	}
};

void BuildConvexHull( const std::vector< Vec3 > & verts, std::vector< Vec3 > & hullPts, std::vector< tri_t > & hullTris );

/*
====================================================
ShapeConvex
====================================================
*/
class ShapeConvex : public Shape {
public:
	explicit ShapeConvex( const Vec3 * pts, const int num ) {
		Build( pts, num );
	}
	void Build( const Vec3 * pts, const int num );

	Vec3 Support( const Vec3 & dir, const Vec3 & pos, const Quat & orient, const float bias ) const override;

	Mat3 InertiaTensor() const override { return m_inertiaTensor; }

	Bounds GetBounds( const Vec3 & pos, const Quat & orient ) const override;
	Bounds GetBounds() const override { return m_bounds; }

	float FastestLinearSpeed( const Vec3 & angularVelocity, const Vec3 & dir ) const override;

	shapeType_t GetType() const override { return SHAPE_CONVEX; }

public:
	std::vector< Vec3 > m_points;
	Bounds m_bounds;
	Mat3 m_inertiaTensor;
};