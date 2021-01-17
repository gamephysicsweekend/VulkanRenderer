//
//	Body.h
//
#pragma once
#include "../Math/Vector.h"
#include "../Math/Quat.h"
#include "../Math/Matrix.h"
#include "../Math/Bounds.h"
#include "Shapes.h"

#include "../Renderer/model.h"
#include "../Renderer/shader.h"

/*
====================================================
Body
====================================================
*/
class Body {
public:
	Body();

	Vec3		m_position;
	Quat		m_orientation;
	Vec3		m_linearVelocity;
	Vec3		m_angularVelocity;
	
	float		m_invMass;
	float		m_elasticity;
	float		m_friction;
	Shape *		m_shape;

	Vec3 GetCenterOfMassWorldSpace() const;
	Vec3 GetCenterOfMassModelSpace() const;

	Vec3 WorldSpaceToBodySpace( const Vec3 & pt ) const;
	Vec3 BodySpaceToWorldSpace( const Vec3 & pt ) const;

	Mat3 GetInverseInertiaTensorBodySpace() const;
	Mat3 GetInverseInertiaTensorWorldSpace() const;

	void ApplyImpulse( const Vec3 & impulsePoint, const Vec3 & impulse );
	void ApplyImpulseLinear( const Vec3 & impulse );
	void ApplyImpulseAngular( const Vec3 & impulse );

	void Update( const float dt_sec );
};