//
//  Scene.cpp
//
#include "Scene.h"
#include "Physics/GJK.h"
#include "Physics/Contact.h"
#include "Physics/Broadphase.h"
#include "Physics/Intersections.h"

/*
========================================================================================================

Scene

========================================================================================================
*/

/*
====================================================
Scene::~Scene
====================================================
*/
Scene::~Scene() {
	for ( int i = 0; i < m_bodies.size(); i++ ) {
		delete m_bodies[ i ].m_shape;
	}
	m_bodies.clear();
}

/*
====================================================
Scene::Reset
====================================================
*/
void Scene::Reset() {
	for ( int i = 0; i < m_bodies.size(); i++ ) {
		delete m_bodies[ i ].m_shape;
	}
	m_bodies.clear();

	for ( int i = 0; i < m_constraints.size(); i++ ) {
		delete m_constraints[ i ];
	}
	m_constraints.clear();

	Initialize();
}

/*
====================================================
AddStandardSandBox
====================================================
*/
void AddStandardSandBox( std::vector< Body > & bodies ) {
	Body body;

	body.m_position = Vec3( 0, 0, 0 );
	body.m_orientation = Quat( 0, 0, 0, 1 );
	body.m_linearVelocity.Zero();
	body.m_angularVelocity.Zero();
	body.m_invMass = 0.0f;
	body.m_elasticity = 0.5f;
	body.m_friction = 0.5f;
	body.m_shape = new ShapeBox( g_boxGround, sizeof( g_boxGround ) / sizeof( Vec3 ) );
	bodies.push_back( body );

	body.m_position = Vec3( 50, 0, 0 );
	body.m_orientation = Quat( 0, 0, 0, 1 );
	body.m_linearVelocity.Zero();
	body.m_angularVelocity.Zero();
	body.m_invMass = 0.0f;
	body.m_elasticity = 0.5f;
	body.m_friction = 0.0f;
	body.m_shape = new ShapeBox( g_boxWall0, sizeof( g_boxWall0 ) / sizeof( Vec3 ) );
	bodies.push_back( body );

	body.m_position = Vec3(-50, 0, 0 );
	body.m_orientation = Quat( 0, 0, 0, 1 );
	body.m_linearVelocity.Zero();
	body.m_angularVelocity.Zero();
	body.m_invMass = 0.0f;
	body.m_elasticity = 0.5f;
	body.m_friction = 0.0f;
	body.m_shape = new ShapeBox( g_boxWall0, sizeof( g_boxWall0 ) / sizeof( Vec3 ) );
	bodies.push_back( body );

	body.m_position = Vec3( 0, 25, 0 );
	body.m_orientation = Quat( 0, 0, 0, 1 );
	body.m_linearVelocity.Zero();
	body.m_angularVelocity.Zero();
	body.m_invMass = 0.0f;
	body.m_elasticity = 0.5f;
	body.m_friction = 0.0f;
	body.m_shape = new ShapeBox( g_boxWall1, sizeof( g_boxWall1 ) / sizeof( Vec3 ) );
	bodies.push_back( body );

	body.m_position = Vec3( 0,-25, 0 );
	body.m_orientation = Quat( 0, 0, 0, 1 );
	body.m_linearVelocity.Zero();
	body.m_angularVelocity.Zero();
	body.m_invMass = 0.0f;
	body.m_elasticity = 0.5f;
	body.m_friction = 0.0f;
	body.m_shape = new ShapeBox( g_boxWall1, sizeof( g_boxWall1 ) / sizeof( Vec3 ) );
	bodies.push_back( body );
}

/*
====================================================
Scene::Initialize
====================================================
*/
void Scene::Initialize() {
	const float pi = acosf( -1.0f );
	Body body;

	//
	//	Build a ragdoll
	//
	{
		Vec3 offset = Vec3( -5, 0, 0 );
		
		// head
		body.m_position = Vec3( 0, 0, 5.5f ) + offset;
		body.m_orientation = Quat( 0, 0, 0, 1 );
		body.m_shape = new ShapeBox( g_boxSmall, sizeof( g_boxSmall ) / sizeof( Vec3 ) );
		body.m_invMass = 2.0f;
		body.m_elasticity = 1.0f;
		body.m_friction = 1.0f;
		m_bodies.push_back( body );

		// torso
		body.m_position = Vec3( 0, 0, 4 ) + offset;
		body.m_orientation = Quat( 0, 0, 0, 1 );
		body.m_shape = new ShapeBox( g_boxBody, sizeof( g_boxBody ) / sizeof( Vec3 ) );
		body.m_invMass = 0.5f;
		body.m_elasticity = 1.0f;
		body.m_friction = 1.0f;
		m_bodies.push_back( body );

		// left arm
		body.m_position = Vec3( 0.0f, 2.0f, 4.75f ) + offset;
		body.m_orientation = Quat( Vec3( 0, 0, 1 ), -3.1415f / 2.0f );
		body.m_shape = new ShapeBox( g_boxLimb, sizeof( g_boxLimb ) / sizeof( Vec3 ) );
		body.m_invMass = 1.0f;
		body.m_elasticity = 1.0f;
		body.m_friction = 1.0f;
		m_bodies.push_back( body );

		// right arm
		body.m_position = Vec3( 0.0f, -2.0f, 4.75f ) + offset;
		body.m_orientation = Quat( Vec3( 0, 0, 1 ), 3.1415f / 2.0f );
		body.m_shape = new ShapeBox( g_boxLimb, sizeof( g_boxLimb ) / sizeof( Vec3 ) );
		body.m_invMass = 1.0f;
		body.m_elasticity = 1.0f;
		body.m_friction = 1.0f;
		m_bodies.push_back( body );

		// left leg
		body.m_position = Vec3( 0.0f, 1.0f, 2.5f ) + offset;
		body.m_orientation = Quat( Vec3( 0, 1, 0 ), 3.1415f / 2.0f );
		body.m_shape = new ShapeBox( g_boxLimb, sizeof( g_boxLimb ) / sizeof( Vec3 ) );
		body.m_invMass = 1.0f;
		body.m_elasticity = 1.0f;
		body.m_friction = 1.0f;
		m_bodies.push_back( body );

		// right leg
		body.m_position = Vec3( 0.0f, -1.0f, 2.5f ) + offset;
		body.m_orientation = Quat( Vec3( 0, 1, 0 ), 3.1415f / 2.0f );
		body.m_shape = new ShapeBox( g_boxLimb, sizeof( g_boxLimb ) / sizeof( Vec3 ) );
		body.m_invMass = 1.0f;
		body.m_elasticity = 1.0f;
		body.m_friction = 1.0f;
		m_bodies.push_back( body );

		const int idxHead = 0;
		const int idxTorso = 1;
		const int idxArmLeft = 2;
		const int idxArmRight = 3;
		const int idxLegLeft = 4;
		const int idxLegRight = 5;

		// Neck
		{
			ConstraintHingeQuatLimited * joint = new ConstraintHingeQuatLimited();
			joint->m_bodyA = &m_bodies[ idxHead ];
			joint->m_bodyB = &m_bodies[ idxTorso ];

			const Vec3 jointWorldSpaceAnchor	= joint->m_bodyA->m_position + Vec3( 0, 0, -0.5f );
			joint->m_anchorA	= joint->m_bodyA->WorldSpaceToBodySpace( jointWorldSpaceAnchor );
			joint->m_anchorB	= joint->m_bodyB->WorldSpaceToBodySpace( jointWorldSpaceAnchor );

			joint->m_axisA = joint->m_bodyA->m_orientation.Inverse().RotatePoint( Vec3( 0, 1, 0 ) );

			// Set the initial relative orientation
			joint->m_q0 = joint->m_bodyA->m_orientation.Inverse() * joint->m_bodyB->m_orientation;

			m_constraints.push_back( joint );
		}

		// Shoulder Left
		{
			ConstraintConstantVelocityLimited * joint = new ConstraintConstantVelocityLimited();
			joint->m_bodyB = &m_bodies[ idxArmLeft ];
			joint->m_bodyA = &m_bodies[ idxTorso ];

			const Vec3 jointWorldSpaceAnchor	= joint->m_bodyB->m_position + Vec3( 0, -1.0f, 0.0f );
			joint->m_anchorA	= joint->m_bodyA->WorldSpaceToBodySpace( jointWorldSpaceAnchor );
			joint->m_anchorB	= joint->m_bodyB->WorldSpaceToBodySpace( jointWorldSpaceAnchor );

			joint->m_axisA = joint->m_bodyA->m_orientation.Inverse().RotatePoint( Vec3( 0, 1, 0 ) );

			// Set the initial relative orientation
			joint->m_q0 = joint->m_bodyA->m_orientation.Inverse() * joint->m_bodyB->m_orientation;

			m_constraints.push_back( joint );
		}

		// Shoulder Right
		{
			ConstraintConstantVelocityLimited * joint = new ConstraintConstantVelocityLimited();
			joint->m_bodyB = &m_bodies[ idxArmRight ];
			joint->m_bodyA = &m_bodies[ idxTorso ];

			const Vec3 jointWorldSpaceAnchor	= joint->m_bodyB->m_position + Vec3( 0, 1.0f, 0.0f );
			joint->m_anchorA	= joint->m_bodyA->WorldSpaceToBodySpace( jointWorldSpaceAnchor );
			joint->m_anchorB	= joint->m_bodyB->WorldSpaceToBodySpace( jointWorldSpaceAnchor );

			joint->m_axisA = joint->m_bodyA->m_orientation.Inverse().RotatePoint( Vec3( 0, -1, 0 ) );

			// Set the initial relative orientation
			joint->m_q0 = joint->m_bodyA->m_orientation.Inverse() * joint->m_bodyB->m_orientation;

			m_constraints.push_back( joint );
		}

		// Hip Left
		{
			ConstraintHingeQuatLimited * joint = new ConstraintHingeQuatLimited();
			joint->m_bodyB = &m_bodies[ idxLegLeft ];
			joint->m_bodyA = &m_bodies[ idxTorso ];

			const Vec3 jointWorldSpaceAnchor	= joint->m_bodyB->m_position + Vec3( 0, 0, 0.5f );
			joint->m_anchorA	= joint->m_bodyA->WorldSpaceToBodySpace( jointWorldSpaceAnchor );
			joint->m_anchorB	= joint->m_bodyB->WorldSpaceToBodySpace( jointWorldSpaceAnchor );

			joint->m_axisA = joint->m_bodyA->m_orientation.Inverse().RotatePoint( Vec3( 0, 1, 0 ) );

			// Set the initial relative orientation
			joint->m_q0 = joint->m_bodyA->m_orientation.Inverse() * joint->m_bodyB->m_orientation;

			m_constraints.push_back( joint );
		}

		// Hip Right
		{
			ConstraintHingeQuatLimited * joint = new ConstraintHingeQuatLimited();
			joint->m_bodyB = &m_bodies[ idxLegRight ];
			joint->m_bodyA = &m_bodies[ idxTorso ];

			const Vec3 jointWorldSpaceAnchor	= joint->m_bodyB->m_position + Vec3( 0, 0, 0.5f );
			joint->m_anchorA	= joint->m_bodyA->WorldSpaceToBodySpace( jointWorldSpaceAnchor );
			joint->m_anchorB	= joint->m_bodyB->WorldSpaceToBodySpace( jointWorldSpaceAnchor );

			joint->m_axisA = joint->m_bodyA->m_orientation.Inverse().RotatePoint( Vec3( 0, 1, 0 ) );

			// Set the initial relative orientation
			joint->m_q0 = joint->m_bodyA->m_orientation.Inverse() * joint->m_bodyB->m_orientation;

			m_constraints.push_back( joint );
		}
	}

	//
	// Build a chain for funsies
	//
	const int numJoints = 5;
	for ( int i = 0; i < numJoints; i++ ) {
		if ( i == 0 ) {
			body.m_position = Vec3( 0.0f, 5.0f, (float)numJoints + 3.0f );
			body.m_orientation = Quat( 0, 0, 0, 1 );
			body.m_shape = new ShapeBox( g_boxSmall, sizeof( g_boxSmall ) / sizeof( Vec3 ) );
			body.m_invMass = 0.0f;
			body.m_elasticity = 1.0f;
			m_bodies.push_back( body );
		} else {
			body.m_invMass = 1.0f;
		}

		body.m_linearVelocity = Vec3( 0, 0, 0 );

		Body * bodyA = &m_bodies[ m_bodies.size() - 1 ];

		const Vec3 jointWorldSpaceAnchor	= bodyA->m_position;
		const Vec3 jointWorldSpaceAxis		= Vec3( 0, 0, 1 ).Normalize();
		Mat3 jointWorldSpaceMatrix;
		jointWorldSpaceMatrix.rows[ 0 ] = jointWorldSpaceAxis;
		jointWorldSpaceMatrix.rows[ 0 ].GetOrtho( jointWorldSpaceMatrix.rows[ 1 ], jointWorldSpaceMatrix.rows[ 2 ] );
		jointWorldSpaceMatrix.rows[ 2 ] = jointWorldSpaceAxis;
		jointWorldSpaceMatrix.rows[ 2 ].GetOrtho( jointWorldSpaceMatrix.rows[ 0 ], jointWorldSpaceMatrix.rows[ 1 ] );
		Vec3 jointWorldSpaceAxisLimited = Vec3( 0, 1, -1 );
		jointWorldSpaceAxisLimited.Normalize();

		ConstraintDistance * joint = new ConstraintDistance();

		const float pi = acosf( -1.0f );

		joint->m_bodyA			= &m_bodies[ m_bodies.size() - 1 ];
		joint->m_anchorA		= joint->m_bodyA->WorldSpaceToBodySpace( jointWorldSpaceAnchor );
		joint->m_axisA			= joint->m_bodyA->m_orientation.Inverse().RotatePoint( jointWorldSpaceAxis );

		body.m_position = joint->m_bodyA->m_position - jointWorldSpaceAxis * 1.0f;
		body.m_position = joint->m_bodyA->m_position + Vec3( 1, 0, 0 );
		body.m_orientation = Quat( 0, 0, 0, 1 );
		body.m_shape = new ShapeBox( g_boxSmall, sizeof( g_boxSmall ) / sizeof( Vec3 ) );
		body.m_invMass = 1.0f;
		body.m_elasticity = 1.0f;
		m_bodies.push_back( body );

		joint->m_bodyB			= &m_bodies[ m_bodies.size() - 1 ];
		joint->m_anchorB		= joint->m_bodyB->WorldSpaceToBodySpace( jointWorldSpaceAnchor );
		joint->m_axisB			= joint->m_bodyB->m_orientation.Inverse().RotatePoint( jointWorldSpaceAxis );

		m_constraints.push_back( joint );
	}

	//
	//	Stack of Boxes
	//
	const int stackHeight = 5;
	for ( int x = 0; x < 1; x++ ) {
		for ( int y = 0; y < 1; y++ ) {
			for ( int z = 0; z < stackHeight; z++ ) {
				float offset = ( ( z & 1 ) == 0 ) ? 0.0f : 0.15f;
				float xx = (float)x + offset;
				float yy = (float)y + offset;
				float delta = 0.04f;
				float scaleHeight = 2.0f + delta;
				float deltaHeight = 1.0f + delta;
				body.m_position = Vec3( (float)xx * scaleHeight, (float)yy * scaleHeight, deltaHeight + (float)z * scaleHeight );
				body.m_orientation = Quat( 0, 0, 0, 1 );
				body.m_shape = new ShapeBox( g_boxUnit, sizeof( g_boxUnit ) / sizeof( Vec3 ) );
				body.m_invMass = 1.0f;
				body.m_elasticity = 0.5f;
				body.m_friction = 0.5f;
				m_bodies.push_back( body );
			}
		}
	}

	//
	//	Sphere and Convex Hull
	//
	body.m_position = Vec3( -10.0f, 0.0f, 5.0f );
	body.m_linearVelocity = Vec3( 0.0f, 0.0f, 0.0f );
	body.m_orientation = Quat( 0, 0, 0, 1 );
	body.m_shape = new ShapeSphere( 1.0f );
	body.m_invMass = 1.0f;
	body.m_elasticity = 0.9f;
	body.m_friction = 0.5f;
	m_bodies.push_back( body );

	body.m_position = Vec3( -10.0f, 0.0f, 10.0f );
	body.m_linearVelocity = Vec3( 0.0f, 0.0f, 0.0f );
	body.m_orientation = Quat( 0, 0, 0, 1 );
	body.m_shape = new ShapeConvex( g_diamond, sizeof( g_diamond ) / sizeof( Vec3 ) );
	body.m_invMass = 1.0f;
	body.m_elasticity = 1.0f;
	body.m_friction = 0.5f;
	m_bodies.push_back( body );

	//
	//	Motor
	//
	Vec3 motorPos = Vec3( 5, 0, 2 );
	Vec3 motorAxis = Vec3( 0, 0, 1 ).Normalize();
	Quat motorOrient = Quat( 1, 0, 0, 0 );

	body.m_position = motorPos;
	body.m_linearVelocity = Vec3( 0.0f, 0.0f, 0.0f );
	body.m_orientation = Quat( 0, 0, 0, 1 );
	body.m_shape = new ShapeBox( g_boxSmall, sizeof( g_boxSmall ) / sizeof( Vec3 ) );
	body.m_invMass = 0.0f;
	body.m_elasticity = 0.9f;
	body.m_friction = 0.5f;
	m_bodies.push_back( body );

	body.m_position = motorPos - motorAxis;
	body.m_linearVelocity = Vec3( 0.0f, 0.0f, 0.0f );
	body.m_orientation = motorOrient;
	body.m_shape = new ShapeBox( g_boxBeam, sizeof( g_boxBeam ) / sizeof( Vec3 ) );
	body.m_invMass = 0.01f;
	body.m_elasticity = 1.0f;
	body.m_friction = 0.5f;
	m_bodies.push_back( body );
	{
		ConstraintMotor * joint = new ConstraintMotor();
		joint->m_bodyA = &m_bodies[ m_bodies.size() - 2 ];
		joint->m_bodyB = &m_bodies[ m_bodies.size() - 1 ];

		const Vec3 jointWorldSpaceAnchor	= joint->m_bodyA->m_position;
		joint->m_anchorA	= joint->m_bodyA->WorldSpaceToBodySpace( jointWorldSpaceAnchor );
		joint->m_anchorB	= joint->m_bodyB->WorldSpaceToBodySpace( jointWorldSpaceAnchor );

		joint->m_motorSpeed = 2.0f;
		joint->m_motorAxis	= joint->m_bodyA->m_orientation.Inverse().RotatePoint( motorAxis );
	
		// Set the initial relative orientation (in bodyA's space)
		joint->m_q0 = joint->m_bodyA->m_orientation.Inverse() * joint->m_bodyB->m_orientation;
		
		m_constraints.push_back( joint );
	}

	//
	//	Mover
	//
	body.m_position = Vec3( 10, 0, 5 );
	body.m_linearVelocity = Vec3( 0, 0, 0 );
	body.m_orientation = Quat( 0, 0, 0, 1 );
	body.m_shape = new ShapeBox( g_boxPlatform, sizeof( g_boxPlatform ) / sizeof( Vec3 ) );
	body.m_invMass = 0.0f;
	body.m_elasticity = 0.1f;
	body.m_friction = 0.9f;
	m_bodies.push_back( body );
	{
		ConstraintMoverSimple * mover = new ConstraintMoverSimple();
		mover->m_bodyA = &m_bodies[ m_bodies.size() - 1 ];

		m_constraints.push_back( mover );
	}

	body.m_position = Vec3( 10, 0, 6.3f );
	body.m_linearVelocity = Vec3( 0, 0, 0 );
	body.m_orientation = Quat( 0, 0, 0, 1 );
	body.m_shape = new ShapeBox( g_boxUnit, sizeof( g_boxUnit ) / sizeof( Vec3 ) );
	body.m_invMass = 1.0f;
	body.m_elasticity = 0.1f;
	body.m_friction = 0.9f;
	m_bodies.push_back( body );

	//
	//	Hinge Constraint
	//
	body.m_position = Vec3( -2, -5, 6 );
	body.m_linearVelocity = Vec3( 0.0f, 0.0f, 0.0f );
	body.m_orientation = Quat( 0, 0, 0, 1 );
	body.m_orientation = Quat( Vec3( 1, 1, 1 ), pi * 0.25f );
	body.m_shape = new ShapeBox( g_boxSmall, sizeof( g_boxSmall ) / sizeof( Vec3 ) );
	body.m_invMass = 0.0f;
	body.m_elasticity = 0.9f;
	body.m_friction = 0.5f;
	m_bodies.push_back( body );

	body.m_position = Vec3( -2, -5, 5 );
	body.m_linearVelocity = Vec3( 0.0f, 0.0f, 0.0f );
	body.m_orientation = Quat( Vec3( 0, 1, 1 ), pi * 0.5f );
	body.m_shape = new ShapeBox( g_boxSmall, sizeof( g_boxSmall ) / sizeof( Vec3 ) );
	body.m_invMass = 1.0f;
	body.m_elasticity = 1.0f;
	body.m_friction = 0.5f;
	m_bodies.push_back( body );
	{
		ConstraintHingeQuatLimited * joint = new ConstraintHingeQuatLimited();
		joint->m_bodyA = &m_bodies[ m_bodies.size() - 2 ];
		joint->m_bodyB = &m_bodies[ m_bodies.size() - 1 ];

		const Vec3 jointWorldSpaceAnchor	= joint->m_bodyA->m_position;
		joint->m_anchorA	= joint->m_bodyA->WorldSpaceToBodySpace( jointWorldSpaceAnchor );
		joint->m_anchorB	= joint->m_bodyB->WorldSpaceToBodySpace( jointWorldSpaceAnchor );

		joint->m_axisA = joint->m_bodyA->m_orientation.Inverse().RotatePoint( Vec3( 1, 0, 0 ) );

		// Set the initial relative orientation
		joint->m_q0 = joint->m_bodyA->m_orientation.Inverse() * joint->m_bodyB->m_orientation;

		m_constraints.push_back( joint );
	}

	//
	//	Constant Velocity Constraint
	//
	body.m_position = Vec3( 2, -5, 6 );
	body.m_linearVelocity = Vec3( 0.0f, 0.0f, 0.0f );
	body.m_orientation = Quat( 0, 0, 0, 1 );
	body.m_orientation = Quat( Vec3( 1, 1, 1 ), pi * 0.5f );
	body.m_shape = new ShapeBox( g_boxSmall, sizeof( g_boxSmall ) / sizeof( Vec3 ) );
	body.m_invMass = 0.0f;
	body.m_elasticity = 0.9f;
	body.m_friction = 0.5f;
	m_bodies.push_back( body );

	body.m_position = Vec3( 2, -5, 5 );
	body.m_linearVelocity = Vec3( 0.0f, 0.0f, 0.0f );
	body.m_orientation = Quat( Vec3( 0, 1, 1 ), pi * 0.5f );
	body.m_shape = new ShapeBox( g_boxSmall, sizeof( g_boxSmall ) / sizeof( Vec3 ) );
	body.m_invMass = 1.0f;
	body.m_elasticity = 1.0f;
	body.m_friction = 0.5f;
	m_bodies.push_back( body );
	{
		ConstraintConstantVelocityLimited * joint = new ConstraintConstantVelocityLimited();
		joint->m_bodyA = &m_bodies[ m_bodies.size() - 2 ];
		joint->m_bodyB = &m_bodies[ m_bodies.size() - 1 ];

		const Vec3 jointWorldSpaceAnchor	= joint->m_bodyA->m_position;
		joint->m_anchorA	= joint->m_bodyA->WorldSpaceToBodySpace( jointWorldSpaceAnchor );
		joint->m_anchorB	= joint->m_bodyB->WorldSpaceToBodySpace( jointWorldSpaceAnchor );

		joint->m_axisA = joint->m_bodyA->m_orientation.Inverse().RotatePoint( Vec3( 0, 0, 1 ) );

		// Set the initial relative orientation
		joint->m_q0 = joint->m_bodyA->m_orientation.Inverse() * joint->m_bodyB->m_orientation;

		m_constraints.push_back( joint );
	}

	//
	//	Teleportation Bug Fix
	//
	body.m_position = Vec3( 10, -10, 3 );
	body.m_orientation = Quat( 0, 0, 0, 1 );
	body.m_linearVelocity = Vec3( -100, 0, 0 );
	body.m_angularVelocity = Vec3( 0.0f, 0.0f, 0.0f );
	body.m_invMass = 1.0f;
	body.m_elasticity = 0.5f;
	body.m_friction = 0.5f;
	body.m_shape = new ShapeSphere( 0.5f );
	m_bodies.push_back( body );

	body.m_position = Vec3( -10, -10, 3 );
	body.m_orientation = Quat( 0, 0, 0, 1 );
	body.m_linearVelocity = Vec3( 100, 0, 0 );
	body.m_angularVelocity = Vec3( 0, 10, 0 );
	body.m_invMass = 1.0f;
	body.m_elasticity = 0.5f;
	body.m_friction = 0.5f;
	body.m_shape = new ShapeConvex( g_diamond, sizeof( g_diamond ) / sizeof( Vec3 ) );
	m_bodies.push_back( body );

	//
	//	Orientation Constraint
	//
	body.m_position = Vec3( 5, 0, 5 );
	body.m_linearVelocity = Vec3( 0.0f, 0.0f, 0.0f );
	body.m_angularVelocity = Vec3( 0.0f, 0.0f, 0.0f );
	body.m_orientation = Quat( 0, 0, 0, 1 );
	body.m_shape = new ShapeBox( g_boxSmall, sizeof( g_boxSmall ) / sizeof( Vec3 ) );
	body.m_invMass = 0.0f;
	body.m_elasticity = 0.9f;
	body.m_friction = 0.5f;
	m_bodies.push_back( body );

	body.m_position = Vec3( 6, 0, 5 );
	body.m_linearVelocity = Vec3( 0.0f, 0.0f, 0.0f );
	body.m_angularVelocity = Vec3( 0.0f, 0.0f, 0.0f );
	body.m_orientation = Quat( 0, 0, 0, 1 );
	body.m_shape = new ShapeBox( g_boxSmall, sizeof( g_boxSmall ) / sizeof( Vec3 ) );
	body.m_invMass = 0.001f;
	body.m_elasticity = 1.0f;
	body.m_friction = 0.5f;
	m_bodies.push_back( body );
	{
		ConstraintOrientation * joint = new ConstraintOrientation();
		joint->m_bodyA = &m_bodies[ m_bodies.size() - 2 ];
		joint->m_bodyB = &m_bodies[ m_bodies.size() - 1 ];

		const Vec3 jointWorldSpaceAnchor	= joint->m_bodyA->m_position;
		joint->m_anchorA	= joint->m_bodyA->WorldSpaceToBodySpace( jointWorldSpaceAnchor );
		joint->m_anchorB	= joint->m_bodyB->WorldSpaceToBodySpace( jointWorldSpaceAnchor );

		// Set the initial relative orientation
		joint->m_q0 = joint->m_bodyA->m_orientation.Inverse() * joint->m_bodyB->m_orientation;

		m_constraints.push_back( joint );
	}

	//
	//	Standard floor and walls
	//
	AddStandardSandBox( m_bodies );
}



#if 0

struct psuedoBody_t {
	int id;
	float value;
	bool ismin;
};

int CompareSAP( const void * a, const void * b ) {
	const psuedoBody_t * ea = (const psuedoBody_t *)a;
	const psuedoBody_t * eb = (const psuedoBody_t *)b;

	if ( ea->value < eb->value ) {
		return -1;
	}
	return 1;
}

void SortBodiesBounds( const Body * bodies, const int num, psuedoBody_t * sortedArray, const float dt_sec ) {
	Vec3 axis = Vec3( 1, 1, 1 );
	axis.Normalize();

	for ( int i = 0; i < num; i++ ) {
		const Body & body = bodies[ i ];
		Bounds bounds = body.m_shape->GetBounds( body.m_position, body.m_orientation );

		// Expand the bounds by the linear velocity
		bounds.Expand( bounds.mins + body.m_linearVelocity * dt_sec );
		bounds.Expand( bounds.maxs + body.m_linearVelocity * dt_sec );

		const float epsilon = 0.01f;
		bounds.Expand( bounds.mins + Vec3(-1,-1,-1 ) * epsilon );
		bounds.Expand( bounds.maxs + Vec3( 1, 1, 1 ) * epsilon );

		sortedArray[ i * 2 + 0 ].id = i;
		sortedArray[ i * 2 + 0 ].value = axis.Dot( bounds.mins );
		sortedArray[ i * 2 + 0 ].ismin = true;

		sortedArray[ i * 2 + 1 ].id = i;
		sortedArray[ i * 2 + 1 ].value = axis.Dot( bounds.maxs );
		sortedArray[ i * 2 + 1 ].ismin = false;
	}

	qsort( sortedArray, num * 2, sizeof( psuedoBody_t ), CompareSAP );
}

struct collisionPair_t {
	int a;
	int b;

	bool operator == ( const collisionPair_t & rhs ) const {
		return ( ( ( a == rhs.a ) && ( b == rhs.b ) ) || ( ( a == rhs.b ) && ( b == rhs.a ) ) );
	}
	bool operator != ( const collisionPair_t & rhs ) const {
		return !( *this == rhs );
	}
};

void BuildPairs( std::vector< collisionPair_t > & collisionPairs, const psuedoBody_t * sortedBodies, const int num ) {
	collisionPairs.clear();

	// Now that the bodies are sorted, build the collision pairs
	for ( int i = 0; i < num * 2; i++ ) {
		const psuedoBody_t & a = sortedBodies[ i ];
		if ( !a.ismin ) {
			continue;
		}

		collisionPair_t pair;
		pair.a = a.id;		

		for ( int j = i + 1; j < num * 2; j++ ) {
			const psuedoBody_t & b = sortedBodies[ j ];
			// if we've hit the end of the a element, then we're done creating pairs with a
			if ( b.id == a.id ) {
				break;
			}

			if ( !b.ismin ) {
				continue;
			}

			pair.b = b.id;
			collisionPairs.push_back( pair );
		}
	}
}

void SweepAndPrune1D( const Body * bodies, const int num, std::vector< collisionPair_t > & finalPairs, const float dt_sec ) {
	psuedoBody_t * sortedBodies = (psuedoBody_t *)alloca( sizeof( psuedoBody_t ) * num * 2 );

	SortBodiesBounds( bodies, num, sortedBodies, dt_sec );
	BuildPairs( finalPairs, sortedBodies, num );
}

void BroadPhase( const Body * bodies, const int num, std::vector< collisionPair_t > & finalPairs, const float dt_sec ) {
	finalPairs.clear();

	SweepAndPrune1D( bodies, num, finalPairs, dt_sec );
}

bool RaySphere( const Vec3 & rayStart, const Vec3 & rayDir, const Vec3 & sphereCenter, const float sphereRadius, float & t1, float & t2 ) {
	const Vec3 m = sphereCenter - rayStart;
	const float a = rayDir.Dot( rayDir );
	const float b = m.Dot( rayDir );
	const float c = m.Dot( m ) - sphereRadius * sphereRadius;

	const float delta = b * b - a * c;
	const float invA = 1.0f / a;

	if ( delta < 0 ) {
		// no real solutions exist
		return false;
	}

	const float deltaRoot = sqrtf( delta );
	t1 = invA * ( b - deltaRoot );
	t2 = invA * ( b + deltaRoot );

	return true;
}

bool SphereSphereDynamic( const ShapeSphere * shapeA, const ShapeSphere * shapeB, const Vec3 & posA, const Vec3 & posB, const Vec3 & velA, const Vec3 & velB, const float dt, Vec3 & ptOnA, Vec3 & ptOnB, float & toi ) {
	const Vec3 relativeVelocity = velA - velB;

	const Vec3 startPtA = posA;
	const Vec3 endPtA = posA + relativeVelocity * dt;
	const Vec3 rayDir = endPtA - startPtA;

	float t0 = 0;
	float t1 = 0;
	if ( rayDir.GetLengthSqr() < 0.001f * 0.001f ) {
		// Ray is too short, just check if already intersecting
		Vec3 ab = posB - posA;
		float radius = shapeA->m_radius + shapeB->m_radius + 0.001f;
		if ( ab.GetLengthSqr() > radius * radius ) {
			return false;
		}
	} else if ( !RaySphere( posA, rayDir, posB, shapeA->m_radius + shapeB->m_radius, t0, t1 ) ) {
		return false;
	}

	// Change from [0,1] range to [0,dt] range
	t0 *= dt;
	t1 *= dt;

	// If the collision is only in the past, then there's not future collision this frame
	if ( t1 < 0.0f ) {
		return false;
	}

	// Get the earliest positive time of impact
	toi = ( t0 < 0.0f ) ? 0.0f : t0;

	// If the earliest collision is too far in the future, then there's no collision this frame
	if ( toi > dt ) {
		return false;
	}

	// Get the points on the respective points of collision and return true
	Vec3 newPosA = posA + velA * toi;
	Vec3 newPosB = posB + velB * toi;
	Vec3 ab = newPosB - newPosA;
	ab.Normalize();

	ptOnA = newPosA + ab * shapeA->m_radius;
	ptOnB = newPosB - ab * shapeB->m_radius;
	return true;
}




bool SphereSphereStatic( const ShapeSphere * sphereA, const ShapeSphere * sphereB, const Vec3 & posA, const Vec3 & posB, Vec3 & ptOnA, Vec3 & ptOnB ) {
	const Vec3 ab = posB - posA;
	Vec3 norm = ab;
	norm.Normalize();

	ptOnA = posA + norm * sphereA->m_radius;
	ptOnB = posB - norm * sphereB->m_radius;

	const float radiusAB = sphereA->m_radius + sphereB->m_radius;
	const float lengthSquare = ab.GetLengthSqr();
	if ( lengthSquare <= ( radiusAB * radiusAB ) ) {
		return true;
	}

	return false;
}

bool Intersect( Body * bodyA, Body * bodyB, contact_t & contact ) {
	contact.bodyA = bodyA;
	contact.bodyB = bodyB;
	contact.timeOfImpact = 0.0f;

	if ( bodyA->m_shape->GetType() == Shape::SHAPE_SPHERE && bodyB->m_shape->GetType() == Shape::SHAPE_SPHERE ) {
		const ShapeSphere * sphereA = (const ShapeSphere *)bodyA->m_shape;
		const ShapeSphere * sphereB = (const ShapeSphere *)bodyB->m_shape;

		Vec3 posA = bodyA->m_position;
		Vec3 posB = bodyB->m_position;

		if ( SphereSphereStatic( sphereA, sphereB, posA, posB, contact.ptOnA_WorldSpace, contact.ptOnB_WorldSpace ) ) {
			contact.normal = posA - posB;
			contact.normal.Normalize();

			contact.ptOnA_LocalSpace = bodyA->WorldSpaceToBodySpace( contact.ptOnA_WorldSpace );
			contact.ptOnB_LocalSpace = bodyB->WorldSpaceToBodySpace( contact.ptOnB_WorldSpace );

			Vec3 ab = bodyB->m_position - bodyA->m_position;
			float r = ab.GetMagnitude() - ( sphereA->m_radius + sphereB->m_radius );
			contact.separationDistance = r;
			return true;
		}
	} else {
		Vec3 ptOnA;
		Vec3 ptOnB;
		const float bias = 0.001f;
		if ( GJK_DoesIntersect( bodyA, bodyB, bias, ptOnA, ptOnB ) ) {
			// There was an intersection, so get the contact data
			Vec3 normal = ptOnB - ptOnA;
			normal.Normalize();

			ptOnA -= normal * bias;
			ptOnB += normal * bias;

			contact.normal = normal;

			contact.ptOnA_WorldSpace = ptOnA;
			contact.ptOnB_WorldSpace = ptOnB;

			contact.ptOnA_LocalSpace = bodyA->WorldSpaceToBodySpace( contact.ptOnA_WorldSpace );
			contact.ptOnB_LocalSpace = bodyB->WorldSpaceToBodySpace( contact.ptOnB_WorldSpace );

			Vec3 ab = bodyB->m_position - bodyA->m_position;
			float r = ( ptOnA - ptOnB ).GetMagnitude();
			contact.separationDistance = -r;
			return true;
		}

		// There was no collision, but we still want the contact data, so get it
		GJK_ClosestPoints( bodyA, bodyB, ptOnA, ptOnB );
		contact.ptOnA_WorldSpace = ptOnA;
		contact.ptOnB_WorldSpace = ptOnB;

		contact.ptOnA_LocalSpace = bodyA->WorldSpaceToBodySpace( contact.ptOnA_WorldSpace );
		contact.ptOnB_LocalSpace = bodyB->WorldSpaceToBodySpace( contact.ptOnB_WorldSpace );

		Vec3 ab = bodyB->m_position - bodyA->m_position;
		float r = ( ptOnA - ptOnB ).GetMagnitude();
		contact.separationDistance = r;
	}
	return false;
}

bool ConservativeAdvance( Body * bodyA, Body * bodyB, float dt, contact_t & contact ) {
	contact.bodyA = bodyA;
	contact.bodyB = bodyB;

	float toi = 0.0f;

	int numIters = 0;

	// Advance the positions of the bodies until they touch or there's not time left
	while ( dt > 0.0f ) {
		// Check for intersection
		bool didIntersect = Intersect( bodyA, bodyB, contact );
		if ( didIntersect ) {
			contact.timeOfImpact = toi;
			bodyA->Update( -toi );
			bodyB->Update( -toi );
			return true;
		}

		++numIters;
		if ( numIters > 10 ) {
			break;
		}

		// Get the vector from the closest point on A to the closest point on B
		Vec3 ab = contact.ptOnB_WorldSpace - contact.ptOnA_WorldSpace;
		ab.Normalize();

		// project the relative velocity onto the ray of shortest distance
		Vec3 relativeVelocity = bodyA->m_linearVelocity - bodyB->m_linearVelocity;
		float orthoSpeed = relativeVelocity.Dot( ab );

		// Add to the orthoSpeed the maximum angular speeds of the relative shapes
		float angularSpeedA = bodyA->m_shape->FastestLinearSpeed( bodyA->m_angularVelocity, ab );
		float angularSpeedB = bodyB->m_shape->FastestLinearSpeed( bodyB->m_angularVelocity, ab * -1.0f );
		orthoSpeed += angularSpeedA + angularSpeedB;
		if ( orthoSpeed <= 0.0f ) {
			break;
		}

		float timeToGo = contact.separationDistance / orthoSpeed;
		if ( timeToGo > dt ) {
			break;
		}

		dt -= timeToGo;
		toi += timeToGo;
		bodyA->Update( timeToGo );
		bodyB->Update( timeToGo );
	}

	// unwind the clock
	bodyA->Update( -toi );
	bodyB->Update( -toi );
	return false;
}

bool Intersect( Body * bodyA, Body * bodyB, const float dt, contact_t & contact ) {
	contact.bodyA = bodyA;
	contact.bodyB = bodyB;

	if ( bodyA->m_shape->GetType() == Shape::SHAPE_SPHERE && bodyB->m_shape->GetType() == Shape::SHAPE_SPHERE ) {
		const ShapeSphere * sphereA = (const ShapeSphere *)bodyA->m_shape;
		const ShapeSphere * sphereB = (const ShapeSphere *)bodyB->m_shape;

		Vec3 posA = bodyA->m_position;
		Vec3 posB = bodyB->m_position;

		Vec3 velA = bodyA->m_linearVelocity;
		Vec3 velB = bodyB->m_linearVelocity;

		if ( SphereSphereDynamic( sphereA, sphereB, posA, posB, velA, velB, dt, contact.ptOnA_WorldSpace, contact.ptOnB_WorldSpace, contact.timeOfImpact ) ) {
			// Step bodies forward to get local space collision points
			bodyA->Update( contact.timeOfImpact );
			bodyB->Update( contact.timeOfImpact );

			// Convert world space contacts to local space
			contact.ptOnA_LocalSpace = bodyA->WorldSpaceToBodySpace( contact.ptOnA_WorldSpace );
			contact.ptOnB_LocalSpace = bodyB->WorldSpaceToBodySpace( contact.ptOnB_WorldSpace );

			contact.normal = bodyA->m_position - bodyB->m_position;
			contact.normal.Normalize();

			// Unwind time step
			bodyA->Update( -contact.timeOfImpact );
			bodyB->Update( -contact.timeOfImpact );

			// Calculate the separation distance
			Vec3 ab = bodyB->m_position - bodyA->m_position;
			float r = ab.GetMagnitude() - ( sphereA->m_radius + sphereB->m_radius );
			contact.separationDistance = r;
			return true;
		}
	} else {
		// Use GJK to perform conservative advancement
		bool result = ConservativeAdvance( bodyA, bodyB, dt, contact );
		return result;
	}
	return false;
}

void ResolveContact( contact_t & contact ) {
	Body * bodyA = contact.bodyA;
	Body * bodyB = contact.bodyB;

	const Vec3 ptOnA = bodyA->BodySpaceToWorldSpace( contact.ptOnA_LocalSpace );
	const Vec3 ptOnB = bodyB->BodySpaceToWorldSpace( contact.ptOnB_LocalSpace );

	const float elasticityA = bodyA->m_elasticity;
	const float elasticityB = bodyB->m_elasticity;
	const float elasticity = elasticityA * elasticityB;

	const float invMassA = bodyA->m_invMass;
	const float invMassB = bodyB->m_invMass;

	const Mat3 invWorldInertiaA = bodyA->GetInverseInertiaTensorWorldSpace();
	const Mat3 invWorldInertiaB = bodyB->GetInverseInertiaTensorWorldSpace();

	const Vec3 n = contact.normal;

	const Vec3 ra = ptOnA - bodyA->GetCenterOfMassWorldSpace();
	const Vec3 rb = ptOnB - bodyB->GetCenterOfMassWorldSpace();

	const Vec3 angularJA = ( invWorldInertiaA * ra.Cross( n ) ).Cross( ra );
	const Vec3 angularJB = ( invWorldInertiaB * rb.Cross( n ) ).Cross( rb );
	const float angularFactor = ( angularJA + angularJB ).Dot( n );

	// Get the world space velocity of the motion and rotation
	const Vec3 velA = bodyA->m_linearVelocity + bodyA->m_angularVelocity.Cross( ra );
	const Vec3 velB = bodyB->m_linearVelocity + bodyB->m_angularVelocity.Cross( rb );

	// Calculate the collision impulse
	const Vec3 vab = velA - velB;
	const float ImpulseJ = ( 1.0f + elasticity ) * vab.Dot( n ) / ( invMassA + invMassB + angularFactor );
	const Vec3 vectorImpulseJ = n * ImpulseJ;

	bodyA->ApplyImpulse( ptOnA, vectorImpulseJ * -1.0f );
	bodyB->ApplyImpulse( ptOnB, vectorImpulseJ * 1.0f );

	//
	// Calculate the impulse caused by friction
	//

	const float frictionA = bodyA->m_friction;
	const float frictionB = bodyB->m_friction;
	const float friction = frictionA * frictionB;

	// Find the normal direction of the velocity with respect to the normal of the collision
	const Vec3 velNorm = n * n.Dot( vab );

	// Find the tangent direction of the velocity with respect to the normal of the collision
	const Vec3 velTang = vab - velNorm;

	// Get the tangential velocities relative to the other body
	Vec3 relativeVelTang = velTang;
	relativeVelTang.Normalize();

	const Vec3 inertiaA = ( invWorldInertiaA * ra.Cross( relativeVelTang ) ).Cross( ra );
	const Vec3 inertiaB = ( invWorldInertiaB * rb.Cross( relativeVelTang ) ).Cross( rb );
	const float invInertia = ( inertiaA + inertiaB ).Dot( relativeVelTang );

	// Calculate the tangential impulse for friction
	const float reducedMass = 1.0f / ( bodyA->m_invMass + bodyB->m_invMass + invInertia );
	const Vec3 impulseFriction = velTang * reducedMass * friction;

	// Apply kinetic friction
	bodyA->ApplyImpulse( ptOnA, impulseFriction * -1.0f );
	bodyB->ApplyImpulse( ptOnB, impulseFriction * 1.0f );

	//
	// Let's also move our colliding objects to just outside of each other (projection method)
	//
	if ( 0.0f == contact.timeOfImpact ) {
		const Vec3 ds = ptOnB - ptOnA;

		const float tA = invMassA / ( invMassA + invMassB );
		const float tB = invMassB / ( invMassA + invMassB );

		bodyA->m_position += ds * tA;
		bodyB->m_position -= ds * tB;
	}
}
#endif

/*
====================================================
CompareContacts
====================================================
*/
int CompareContacts( const void * p1, const void * p2 ) {
	contact_t a = *(contact_t*)p1;
	contact_t b = *(contact_t*)p2;

	if ( a.timeOfImpact < b.timeOfImpact ) {
		return -1;
	}

	if ( a.timeOfImpact == b.timeOfImpact ) {
		return 0;
	}

	return 1;
}

/*
====================================================
Scene::Update
====================================================
*/
void Scene::Update( const float dt_sec ) {
	m_manifolds.RemoveExpired();

	// Gravity impulse
	for ( int i = 0; i < m_bodies.size(); i++ ) {
		Body * body = &m_bodies[ i ];
		float mass = 1.0f / body->m_invMass;
		Vec3 impulseGravity = Vec3( 0, 0, -10 ) * mass * dt_sec;
		body->ApplyImpulseLinear( impulseGravity );
	}

	//
	// Broadphase (build potential collision pairs)
	//
	std::vector< collisionPair_t > collisionPairs;
	BroadPhase( m_bodies.data(), (int)m_bodies.size(), collisionPairs, dt_sec );

	//
	//	NarrowPhase (perform actual collision detection)
	//
	int numContacts = 0;
	contact_t * contacts = (contact_t *)alloca( sizeof( contact_t ) * collisionPairs.size() );
	for ( int i = 0; i < collisionPairs.size(); i++ ) {
		const collisionPair_t & pair = collisionPairs[ i ];
		Body * bodyA = &m_bodies[ pair.a ];
		Body * bodyB = &m_bodies[ pair.b ];

		// Skip body pairs with infinite mass
		if ( 0.0f == bodyA->m_invMass && 0.0f == bodyB->m_invMass ) {
			continue;
		}

		// Check for intersection
		contact_t contact;
		if ( Intersect( bodyA, bodyB, dt_sec, contact ) ) {
			if ( 0.0f == contact.timeOfImpact ) {
				// Static contact
				m_manifolds.AddContact( contact );
			} else {
				// Ballistic contact
				contacts[ numContacts ] = contact;
				numContacts++;
			}
		}
	}

	// Sort the times of impact from first to last
	if ( numContacts > 1 ) {
		qsort( contacts, numContacts, sizeof( contact_t ), CompareContacts );
	}

	//
	//	Solve Constraints
	//
	for ( int i = 0; i < m_constraints.size(); i++ ) {
		m_constraints[ i ]->PreSolve( dt_sec );
	}
	m_manifolds.PreSolve( dt_sec );

	const int maxIters = 5;
	for ( int iters = 0; iters < maxIters; iters++ ) {
		for ( int i = 0; i < m_constraints.size(); i++ ) {
			m_constraints[ i ]->Solve();
		}
		m_manifolds.Solve();
	}

	for ( int i = 0; i < m_constraints.size(); i++ ) {
		m_constraints[ i ]->PostSolve();
	}
	m_manifolds.PostSolve();


	//
	// Apply ballistic impulses
	//
	float accumulatedTime = 0.0f;
	for ( int i = 0; i < numContacts; i++ ) {
		contact_t & contact = contacts[ i ];
		const float dt = contact.timeOfImpact - accumulatedTime;

		// Position update
		for ( int j = 0; j < m_bodies.size(); j++ ) {
			m_bodies[ j ].Update( dt );
		}

		ResolveContact( contact );
		accumulatedTime += dt;
	}

	// Update the positions for the rest of this frame's time
	const float timeRemaining = dt_sec - accumulatedTime;
	if ( timeRemaining > 0.0f ) {
		for ( int i = 0; i < m_bodies.size(); i++ ) {
			m_bodies[ i ].Update( timeRemaining );
		}
	}
}