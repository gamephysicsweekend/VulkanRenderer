//
//	ConstraintDistance.h
//
#pragma once
#include "ConstraintBase.h"

/*
================================
ConstraintHingeQuat
================================
*/
class ConstraintDistance : public Constraint {
public:
	ConstraintDistance() : Constraint(),
		m_cachedLambda( 1 ),
		m_Jacobian( 1, 12 ) {
		m_cachedLambda.Zero();
		m_baumgarte = 0.0f;
	}

	void PreSolve( const float dt_sec ) override;
	void Solve() override;
	void PostSolve() override;

private:
	MatMN m_Jacobian;

	VecN m_cachedLambda;
	float m_baumgarte;
};