//
//  Intersections.cpp
//
#include "Intersections.h"

/*
====================================================
RaySphere
====================================================
*/
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

/*
====================================================
SphereSphereDynamic
====================================================
*/
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

/*
====================================================
Intersect
====================================================
*/
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
	}
	return false;
}
