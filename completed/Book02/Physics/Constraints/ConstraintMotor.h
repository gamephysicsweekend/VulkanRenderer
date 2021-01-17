//
//	ConstraintMotor.h
//
#pragma once
#include "ConstraintBase.h"


class ConstraintMotor : public Constraint {
public:
	ConstraintMotor() : Constraint(), m_Jacobian( 4, 12 ) {
		m_motorSpeed = 0.0f;
		m_motorAxis = Vec3( 0, 0, 1 );
		m_baumgarte = 0.0f;
	}

	void PreSolve( const float dt_sec ) override;
	void Solve() override;

	float m_motorSpeed;
	Vec3 m_motorAxis;	// Motor Axis in BodyA's local space
	Quat m_q0;		// The initial relative quaternion q1^-1 * q2

	MatMN m_Jacobian;

	Vec3 m_baumgarte;
};