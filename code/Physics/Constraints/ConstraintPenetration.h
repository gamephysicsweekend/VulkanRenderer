//
//	ConstraintPenetration.h
//
#pragma once
#include "ConstraintBase.h"

/*
================================
ConstraintPenetration
================================
*/
class ConstraintPenetration : public Constraint {
public:
	ConstraintPenetration() : Constraint(), m_cachedLambda( 3 ), m_Jacobian( 3, 12 ) {
		m_cachedLambda.Zero();
		m_baumgarte = 0.0f;
		m_friction = 0.0f;
	}

	void PreSolve( const float dt_sec ) override;
	void Solve() override;

	VecN m_cachedLambda;
	Vec3 m_normal;		// in Body A's local space

	MatMN m_Jacobian;

	float m_baumgarte;
	float m_friction;
};