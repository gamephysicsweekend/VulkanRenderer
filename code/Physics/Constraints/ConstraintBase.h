//
//	ConstraintBase.h
//
#pragma once
#include "../../Math/Vector.h"
#include "../../Math/Quat.h"
#include "../../Math/Matrix.h"
#include "../../Math/Bounds.h"
#include "../../Math/LCP.h"
#include "../Body.h"
#include <vector>

/*
====================================================
Constraint
====================================================
*/
class Constraint {
public:
	virtual void PreSolve( const float dt_sec ) {}
	virtual void Solve() {}
	virtual void PostSolve() {}

	static Mat4 Left( const Quat & q );
	static Mat4 Right( const Quat & q );

protected:
	MatMN GetInverseMassMatrix() const;
	VecN GetVelocities() const;
	void ApplyImpulses( const VecN & impulses );

public:
	Body * m_bodyA;
	Body * m_bodyB;

	Vec3 m_anchorA;		// The anchor location in bodyA's space
	Vec3 m_axisA;		// The axis direction in bodyA's space

	Vec3 m_anchorB;		// The anchor location in bodyB's space
	Vec3 m_axisB;		// The axis direction in bodyB's space
};

/*
====================================================
Constraint::GetInverseMassMatrix
====================================================
*/
inline MatMN Constraint::GetInverseMassMatrix() const {
	MatMN invMassMatrix( 12, 12 );
	
	// TODO: Add code

	return invMassMatrix;
}

/*
====================================================
Constraint::GetVelocities
====================================================
*/
inline VecN Constraint::GetVelocities() const {
	VecN q_dt( 12 );

	// TODO: Add code

	return q_dt;
}

/*
====================================================
Constraint::ApplyImpulses
====================================================
*/
inline void Constraint::ApplyImpulses( const VecN & impulses ) {
	// TODO: Add code
}

/*
====================================================
Constraint::Left
====================================================
*/
inline Mat4 Constraint::Left( const Quat & q ) {
	Mat4 L;
	
	// TODO: Add code

	return L.Transpose();
}

/*
====================================================
Constraint::Right
====================================================
*/
inline Mat4 Constraint::Right( const Quat & q ) {
	Mat4 R;
	
	// TODO: Add code

	return R.Transpose();
}