//
//	ConstraintConstantVelocity.h
//
#pragma once
#include "ConstraintBase.h"

class ConstraintConstantVelocity : public Constraint {
public:
	ConstraintConstantVelocity() : Constraint(), m_cachedLambda( 2 ), m_Jacobian( 2, 12 ) {
		m_cachedLambda.Zero();
		m_baumgarte = 0.0f;
	}
	void PreSolve( const float dt_sec ) override;
	void Solve() override;
	void PostSolve() override;

	Quat m_q0;	// The initial relative quaternion q1 * q2^-1

	VecN m_cachedLambda;
	MatMN m_Jacobian;

	float m_baumgarte;
};





class ConstraintConstantVelocityLimited : public Constraint {
public:
	ConstraintConstantVelocityLimited() : Constraint(), m_cachedLambda( 4 ), m_Jacobian( 4, 12 ) {
		m_cachedLambda.Zero();
		m_baumgarte = 0.0f;
		m_isAngleViolatedU = false;
		m_isAngleViolatedV = false;
		m_angleU = 0.0f;
		m_angleV = 0.0f;
	}
	void PreSolve( const float dt_sec ) override;
	void Solve() override;
	void PostSolve() override;

	Quat m_q0;	// The initial relative quaternion q1^-1 * q2

	VecN m_cachedLambda;
	MatMN m_Jacobian;

	float m_baumgarte;

	bool m_isAngleViolatedU;
	bool m_isAngleViolatedV;
	float m_angleU;
	float m_angleV;
};