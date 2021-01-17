//
//  ConstraintPenetration.cpp
//
#include "ConstraintPenetration.h"


void ConstraintPenetration::PreSolve( const float dt_sec ) {
	// Get the world space position of the hinge from A's orientation
	const Vec3 worldAnchorA = m_bodyA->BodySpaceToWorldSpace( m_anchorA );

	// Get the world space position of the hinge from B's orientation
	const Vec3 worldAnchorB = m_bodyB->BodySpaceToWorldSpace( m_anchorB );

	const Vec3 ra = worldAnchorA - m_bodyA->GetCenterOfMassWorldSpace();
	const Vec3 rb = worldAnchorB - m_bodyB->GetCenterOfMassWorldSpace();
	const Vec3 a = worldAnchorA;
	const Vec3 b = worldAnchorB;

	const float frictionA = m_bodyA->m_friction;
	const float frictionB = m_bodyB->m_friction;
	m_friction = frictionA * frictionB;

	Vec3 u;
	Vec3 v;
	m_normal.GetOrtho( u, v );

	// Convert tangent space from model space to world space
	Vec3 normal = m_bodyA->m_orientation.RotatePoint( m_normal );
	u = m_bodyA->m_orientation.RotatePoint( u );
	v = m_bodyA->m_orientation.RotatePoint( v );

	//
	//	Penetration Constraint
	//
	m_Jacobian.Zero();

	// First row is the primary distance constraint that holds the anchor points together
	Vec3 J1 = normal * -1.0f;
	m_Jacobian.rows[ 0 ][ 0 ] = J1.x;
	m_Jacobian.rows[ 0 ][ 1 ] = J1.y;
	m_Jacobian.rows[ 0 ][ 2 ] = J1.z;

	Vec3 J2 = ra.Cross( normal * -1.0f );
	m_Jacobian.rows[ 0 ][ 3 ] = J2.x;
	m_Jacobian.rows[ 0 ][ 4 ] = J2.y;
	m_Jacobian.rows[ 0 ][ 5 ] = J2.z;

	Vec3 J3 = normal * 1.0f;
	m_Jacobian.rows[ 0 ][ 6 ] = J3.x;
	m_Jacobian.rows[ 0 ][ 7 ] = J3.y;
	m_Jacobian.rows[ 0 ][ 8 ] = J3.z;

	Vec3 J4 = rb.Cross( normal * 1.0f );
	m_Jacobian.rows[ 0 ][ 9 ] = J4.x;
	m_Jacobian.rows[ 0 ][ 10] = J4.y;
	m_Jacobian.rows[ 0 ][ 11] = J4.z;

	//
	//	Friction Jacobians
	//
	if ( m_friction > 0.0f ) {
		Vec3 J1 = u * -1.0f;
		m_Jacobian.rows[ 1 ][ 0 ] = J1.x;
		m_Jacobian.rows[ 1 ][ 1 ] = J1.y;
		m_Jacobian.rows[ 1 ][ 2 ] = J1.z;

		Vec3 J2 = ra.Cross( u * -1.0f );
		m_Jacobian.rows[ 1 ][ 3 ] = J2.x;
		m_Jacobian.rows[ 1 ][ 4 ] = J2.y;
		m_Jacobian.rows[ 1 ][ 5 ] = J2.z;

		Vec3 J3 = u * 1.0f;
		m_Jacobian.rows[ 1 ][ 6 ] = J3.x;
		m_Jacobian.rows[ 1 ][ 7 ] = J3.y;
		m_Jacobian.rows[ 1 ][ 8 ] = J3.z;

		Vec3 J4 = rb.Cross( u * 1.0f );
		m_Jacobian.rows[ 1 ][ 9 ] = J4.x;
		m_Jacobian.rows[ 1 ][ 10] = J4.y;
		m_Jacobian.rows[ 1 ][ 11] = J4.z;
	}
	if ( m_friction > 0.0f ) {
		Vec3 J1 = v * -1.0f;
		m_Jacobian.rows[ 2 ][ 0 ] = J1.x;
		m_Jacobian.rows[ 2 ][ 1 ] = J1.y;
		m_Jacobian.rows[ 2 ][ 2 ] = J1.z;

		Vec3 J2 = ra.Cross( v * -1.0f );
		m_Jacobian.rows[ 2 ][ 3 ] = J2.x;
		m_Jacobian.rows[ 2 ][ 4 ] = J2.y;
		m_Jacobian.rows[ 2 ][ 5 ] = J2.z;

		Vec3 J3 = v * 1.0f;
		m_Jacobian.rows[ 2 ][ 6 ] = J3.x;
		m_Jacobian.rows[ 2 ][ 7 ] = J3.y;
		m_Jacobian.rows[ 2 ][ 8 ] = J3.z;

		Vec3 J4 = rb.Cross( v * 1.0f );
		m_Jacobian.rows[ 2 ][ 9 ] = J4.x;
		m_Jacobian.rows[ 2 ][ 10] = J4.y;
		m_Jacobian.rows[ 2 ][ 11] = J4.z;
	}

	//
	// Apply warm starting from last frame
	//
	const VecN impulses = m_Jacobian.Transpose() * m_cachedLambda;
	ApplyImpulses( impulses );

	//
	//	Calculate the baumgarte stabilization
	//
	float C = ( b - a ).Dot( normal );
	C = std::min( 0.0f, C + 0.02f );	// Add slop
	float Beta = 0.25f;
	m_baumgarte = Beta * C / dt_sec;
}

void ConstraintPenetration::Solve() {
	const MatMN JacobianTranspose = m_Jacobian.Transpose();

	// Build the system of equations
	const VecN q_dt = GetVelocities();
	const MatMN invMassMatrix = GetInverseMassMatrix();
	const MatMN J_W_Jt = m_Jacobian * invMassMatrix * JacobianTranspose;
	VecN rhs = m_Jacobian * q_dt * -1.0f;
	rhs[ 0 ] -= m_baumgarte;

	// Solve for the Lagrange multipliers
	VecN lambdaN = LCP_GaussSeidel( J_W_Jt, rhs );

	// Accumulate the impulses and clamp to within the constraint limits
	VecN oldLambda = m_cachedLambda;
	m_cachedLambda += lambdaN;
	const float lambdaLimit = 0.0f;
	if ( m_cachedLambda[ 0 ] < lambdaLimit ) {
		m_cachedLambda[ 0 ] = lambdaLimit;
	}
	if ( m_friction > 0.0f ) {
		const float umg = m_friction * 10.0f * 1.0f / ( m_bodyA->m_invMass + m_bodyB->m_invMass );
		const float normalForce = fabsf( lambdaN[ 0 ] * m_friction );
		const float maxForce = ( umg > normalForce ) ? umg : normalForce;

		if ( m_cachedLambda[ 1 ] > maxForce ) {
			m_cachedLambda[ 1 ] = maxForce;
		}
		if ( m_cachedLambda[ 1 ] < -maxForce ) {
			m_cachedLambda[ 1 ] = -maxForce;
		}

		if ( m_cachedLambda[ 2 ] > maxForce ) {
			m_cachedLambda[ 2 ] = maxForce;
		}
		if ( m_cachedLambda[ 2 ] < -maxForce ) {
			m_cachedLambda[ 2 ] = -maxForce;
		}
	}
	lambdaN = m_cachedLambda - oldLambda;

	// Apply the impulses
	const VecN impulses = JacobianTranspose * lambdaN;
	ApplyImpulses( impulses );
}