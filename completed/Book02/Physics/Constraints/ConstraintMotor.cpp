//
//  ConstraintMotor.cpp
//
#include "ConstraintMotor.h"


/*
================================
ConstraintMotor::PreSolve
================================
*/
void ConstraintMotor::PreSolve( const float dt_sec ) {
	// Get the world space position of the hinge from A's orientation
	const Vec3 worldAnchorA = m_bodyA->BodySpaceToWorldSpace( m_anchorA );

	// Get the world space position of the hinge from B's orientation
	const Vec3 worldAnchorB = m_bodyB->BodySpaceToWorldSpace( m_anchorB );

	const Vec3 r = worldAnchorB - worldAnchorA;
	const Vec3 ra = worldAnchorA - m_bodyA->GetCenterOfMassWorldSpace();
	const Vec3 rb = worldAnchorB - m_bodyB->GetCenterOfMassWorldSpace();
	const Vec3 a = worldAnchorA;
	const Vec3 b = worldAnchorB;

	// Get the orientation information of the bodies
	const Quat q1 = m_bodyA->m_orientation;
	const Quat q2 = m_bodyB->m_orientation;
	const Quat q0_inv = m_q0.Inverse();
	const Quat q1_inv = q1.Inverse();

	const Vec3 motorAxis = m_bodyA->m_orientation.RotatePoint( m_motorAxis );
	Vec3 motorU;
	Vec3 motorV;
	motorAxis.GetOrtho( motorU, motorV );

	const Vec3 u = motorU;
	const Vec3 v = motorV;
	const Vec3 w = motorAxis;

	Mat4 P;
	P.rows[ 0 ] = Vec4( 0, 0, 0, 0 );
	P.rows[ 1 ] = Vec4( 0, 1, 0, 0 );
	P.rows[ 2 ] = Vec4( 0, 0, 1, 0 );
	P.rows[ 3 ] = Vec4( 0, 0, 0, 1 );
	Mat4 P_T = P.Transpose();	// I know it's pointless to do this with our particular matrix implementations.  But I like its self commenting.

	const Mat4 MatA = P * Left( q1_inv ) * Right( q2 * q0_inv ) * P_T * -0.5f;
	const Mat4 MatB = P * Left( q1_inv ) * Right( q2 * q0_inv ) * P_T * 0.5f;

	//
	//	The distance constraint
	//

	m_Jacobian.Zero();

	// First row is the primary distance constraint that holds the anchor points together
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
	// The quaternion jacobians
	//
	const int idx = 1;

	Vec4 tmp;
	{
		J1.Zero();
		m_Jacobian.rows[ 1 ][ 0 ] = J1.x;
		m_Jacobian.rows[ 1 ][ 1 ] = J1.y;
		m_Jacobian.rows[ 1 ][ 2 ] = J1.z;

		tmp = MatA * Vec4( 0, u.x, u.y, u.z );
		J2 = Vec3( tmp[ idx + 0 ], tmp[ idx + 1 ], tmp[ idx + 2 ] );
		m_Jacobian.rows[ 1 ][ 3 ] = J2.x;
		m_Jacobian.rows[ 1 ][ 4 ] = J2.y;
		m_Jacobian.rows[ 1 ][ 5 ] = J2.z;

		J3.Zero();
		m_Jacobian.rows[ 1 ][ 6 ] = J3.x;
		m_Jacobian.rows[ 1 ][ 7 ] = J3.y;
		m_Jacobian.rows[ 1 ][ 8 ] = J3.z;

		tmp = MatB * Vec4( 0, u.x, u.y, u.z );
		J4 = Vec3( tmp[ idx + 0 ], tmp[ idx + 1 ], tmp[ idx + 2 ] );
		m_Jacobian.rows[ 1 ][ 9 ] = J4.x;
		m_Jacobian.rows[ 1 ][ 10] = J4.y;
		m_Jacobian.rows[ 1 ][ 11] = J4.z;
	}
	{
		J1.Zero();
		m_Jacobian.rows[ 2 ][ 0 ] = J1.x;
		m_Jacobian.rows[ 2 ][ 1 ] = J1.y;
		m_Jacobian.rows[ 2 ][ 2 ] = J1.z;

		tmp = MatA * Vec4( 0, v.x, v.y, v.z );
		J2 = Vec3( tmp[ idx + 0 ], tmp[ idx + 1 ], tmp[ idx + 2 ] );
		m_Jacobian.rows[ 2 ][ 3 ] = J2.x;
		m_Jacobian.rows[ 2 ][ 4 ] = J2.y;
		m_Jacobian.rows[ 2 ][ 5 ] = J2.z;

		J3.Zero();
		m_Jacobian.rows[ 2 ][ 6 ] = J3.x;
		m_Jacobian.rows[ 2 ][ 7 ] = J3.y;
		m_Jacobian.rows[ 2 ][ 8 ] = J3.z;

		tmp = MatB * Vec4( 0, v.x, v.y, v.z );
		J4 = Vec3( tmp[ idx + 0 ], tmp[ idx + 1 ], tmp[ idx + 2 ] );
		m_Jacobian.rows[ 2 ][ 9 ] = J4.x;
		m_Jacobian.rows[ 2 ][ 10] = J4.y;
		m_Jacobian.rows[ 2 ][ 11] = J4.z;
	}
	{
		J1.Zero();
		m_Jacobian.rows[ 3 ][ 0 ] = J1.x;
		m_Jacobian.rows[ 3 ][ 1 ] = J1.y;
		m_Jacobian.rows[ 3 ][ 2 ] = J1.z;

		tmp = MatA * Vec4( 0, w.x, w.y, w.z );
		J2 = Vec3( tmp[ idx + 0 ], tmp[ idx + 1 ], tmp[ idx + 2 ] );
		m_Jacobian.rows[ 3 ][ 3 ] = J2.x;
		m_Jacobian.rows[ 3 ][ 4 ] = J2.y;
		m_Jacobian.rows[ 3 ][ 5 ] = J2.z;

		J3.Zero();
		m_Jacobian.rows[ 3 ][ 6 ] = J3.x;
		m_Jacobian.rows[ 3 ][ 7 ] = J3.y;
		m_Jacobian.rows[ 3 ][ 8 ] = J3.z;

		tmp = MatB * Vec4( 0, w.x, w.y, w.z );
		J4 = Vec3( tmp[ idx + 0 ], tmp[ idx + 1 ], tmp[ idx + 2 ] );
		m_Jacobian.rows[ 3 ][ 9 ] = J4.x;
		m_Jacobian.rows[ 3 ][ 10] = J4.y;
		m_Jacobian.rows[ 3 ][ 11] = J4.z;
	}

	//
	//	Calculate the baumgarte stabilization
	//
	const float Beta = 0.05f;
	const float C = r.Dot( r );

	const Quat qr = m_bodyA->m_orientation.Inverse() * m_bodyB->m_orientation;
	const Quat qrA = qr * q0_inv;	// Relative orientation in BodyA's space

	// Get the world space axis for the relative rotation
	const Vec3 axisA = m_bodyA->m_orientation.RotatePoint( qrA.xyz() );

	m_baumgarte.Zero();
	m_baumgarte[ 0 ] = ( Beta / dt_sec ) * C;
	m_baumgarte[ 1 ] = motorU.Dot( axisA ) * ( Beta / dt_sec );
	m_baumgarte[ 2 ] = motorV.Dot( axisA ) * ( Beta / dt_sec );
}

/*
================================
ConstraintMotor::Solve
================================
*/
void ConstraintMotor::Solve() {
	const Vec3 motorAxis = m_bodyA->m_orientation.RotatePoint( m_motorAxis );

	VecN w_dt( 12 );
	w_dt.Zero();
	w_dt[ 3 ] = motorAxis[ 0 ] * -m_motorSpeed;
	w_dt[ 4 ] = motorAxis[ 1 ] * -m_motorSpeed;
	w_dt[ 5 ] = motorAxis[ 2 ] * -m_motorSpeed;
	w_dt[ 9 ] = motorAxis[ 0 ] * m_motorSpeed;
	w_dt[ 10 ] = motorAxis[ 1 ] * m_motorSpeed;
	w_dt[ 11 ] = motorAxis[ 2 ] * m_motorSpeed;

	const MatMN JacobianTranspose = m_Jacobian.Transpose();

	// Build the system of equations
	const VecN q_dt = GetVelocities() - w_dt;	// By subtracting by the desired velocity, the solver is tricked into applying the impulse to give us that velocity
	const MatMN invMassMatrix = GetInverseMassMatrix();
	const MatMN J_W_Jt = m_Jacobian * invMassMatrix * JacobianTranspose;
	VecN rhs = m_Jacobian * q_dt * -1.0f;
	for ( int i = 0; i < 3; i++ ) {
		rhs[ i ] -= m_baumgarte[ i ];
	}

	// Solve for the Lagrange multipliers
	VecN lambdaN = LCP_GaussSeidel( J_W_Jt, rhs );

	// Apply the impulses
	const VecN impulses = JacobianTranspose * lambdaN;
	ApplyImpulses( impulses );
}