//
//  ConstraintDistance.cpp
//
#include "ConstraintDistance.h"

/*
================================
ConstraintDistance::PreSolve
================================
*/
void ConstraintDistance::PreSolve( const float dt_sec ) {
	// Get the world space position of the hinge from A's orientation
	const Vec3 worldAnchorA = m_bodyA->BodySpaceToWorldSpace( m_anchorA );

	// Get the world space position of the hinge from B's orientation
	const Vec3 worldAnchorB = m_bodyB->BodySpaceToWorldSpace( m_anchorB );

	const Vec3 r = worldAnchorB - worldAnchorA;
	const Vec3 ra = worldAnchorA - m_bodyA->GetCenterOfMassWorldSpace();
	const Vec3 rb = worldAnchorB - m_bodyB->GetCenterOfMassWorldSpace();
	const Vec3 a = worldAnchorA;
	const Vec3 b = worldAnchorB;

	m_Jacobian.Zero();

	Vec3 J1 = ( a - b ) * 2.0f;
	m_Jacobian.rows[ 0 ][ 0 ] = J1.x;
	m_Jacobian.rows[ 0 ][ 1 ] = J1.y;
	m_Jacobian.rows[ 0 ][ 2 ] = J1.z;

	Vec3 J2 = ra.Cross( ( a - b ) * 2.0f );
	m_Jacobian.rows[ 0 ][ 3 ] = J2.x;
	m_Jacobian.rows[ 0 ][ 4 ] = J2.y;
	m_Jacobian.rows[ 0 ][ 5 ] = J2.z;

	Vec3 J3 = ( b - a ) * 2.0f;
	m_Jacobian.rows[ 0 ][ 6 ] = J3.x;
	m_Jacobian.rows[ 0 ][ 7 ] = J3.y;
	m_Jacobian.rows[ 0 ][ 8 ] = J3.z;

	Vec3 J4 = rb.Cross( ( b - a ) * 2.0f );
	m_Jacobian.rows[ 0 ][ 9 ] = J4.x;
	m_Jacobian.rows[ 0 ][ 10] = J4.y;
	m_Jacobian.rows[ 0 ][ 11] = J4.z;

	//
	// Apply warm starting from last frame
	//
	const VecN impulses = m_Jacobian.Transpose() * m_cachedLambda;
	ApplyImpulses( impulses );

	//
	//	Calculate the baumgarte stabilization
	//
	float C = r.Dot( r );
	C = std::max( 0.0f, C - 0.01f );
	const float Beta = 0.05f;
	m_baumgarte = ( Beta / dt_sec ) * C;
}

/*
================================
ConstraintDistance::Solve
================================
*/
void ConstraintDistance::Solve() {
	const MatMN JacobianTranspose = m_Jacobian.Transpose();

	// Build the system of equations
	const VecN q_dt = GetVelocities();
	const MatMN invMassMatrix = GetInverseMassMatrix();
	const MatMN J_W_Jt = m_Jacobian * invMassMatrix * JacobianTranspose;
	VecN rhs = m_Jacobian * q_dt * -1.0f;
	rhs[ 0 ] -= m_baumgarte;
	
	// Solve for the Lagrange multipliers
	const VecN lambdaN = LCP_GaussSeidel( J_W_Jt, rhs );

	// Apply the impulses
	const VecN impulses = JacobianTranspose * lambdaN;
	ApplyImpulses( impulses );

	// Accumulate the impulses for warm starting
	m_cachedLambda += lambdaN;
}

/*
================================
ConstraintDistance::PostSolve
================================
*/
void ConstraintDistance::PostSolve() {
	// Limit the warm starting to reasonable limits
	if ( m_cachedLambda[ 0 ] * 0.0f != m_cachedLambda[ 0 ] * 0.0f ) {
		m_cachedLambda[ 0 ] = 0.0f;
	}
	const float limit = 1e5f;
	if ( m_cachedLambda[ 0 ] > limit ) {
		m_cachedLambda[ 0 ] = limit;
	}
	if ( m_cachedLambda[ 0 ] < -limit ) {
		m_cachedLambda[ 0 ] = -limit;
	}
}