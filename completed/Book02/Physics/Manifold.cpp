//
//  Manifold.cpp
//
#include "Manifold.h"


/*
================================================================================================

ManifoldCollector

================================================================================================
*/

/*
================================
ManifoldCollector::AddContact
================================
*/
void ManifoldCollector::AddContact( const contact_t & contact ) {
	// Try to find the previously existing manifold for contacts between these two bodies
	int foundIdx = -1;
	for ( int i = 0; i < m_manifolds.size(); i++ ) {
		const Manifold & manifold = m_manifolds[ i ];
		bool hasA = ( manifold.m_bodyA == contact.bodyA || manifold.m_bodyB == contact.bodyA );
		bool hasB = ( manifold.m_bodyA == contact.bodyB || manifold.m_bodyB == contact.bodyB );
		if ( hasA && hasB ) {
			foundIdx = i;
			break;
		}
	}

	// Add contact to manifolds
	if ( foundIdx >= 0 ) {
		m_manifolds[ foundIdx ].AddContact( contact );
	} else {
		Manifold manifold;
		manifold.m_bodyA = contact.bodyA;
		manifold.m_bodyB = contact.bodyB;

		manifold.AddContact( contact );
		m_manifolds.push_back( manifold );
	}
}

/*
================================
ManifoldCollector::RemoveExpired
================================
*/
void ManifoldCollector::RemoveExpired() {
	// Remove expired manifolds
	for ( int i = (int)m_manifolds.size() - 1; i >= 0; i-- ) {
		Manifold & manifold = m_manifolds[ i ];
		manifold.RemoveExpiredContacts();

		if ( 0 == manifold.m_numContacts ) {
			m_manifolds.erase( m_manifolds.begin() + i );
		}
	}
}

/*
================================
ManifoldCollector::PreSolve
================================
*/
void ManifoldCollector::PreSolve( const float dt_sec ) {
	for ( int i = 0; i < m_manifolds.size(); i++ ) {
		m_manifolds[ i ].PreSolve( dt_sec );
	}
}

/*
================================
ManifoldCollector::Solve
================================
*/
void ManifoldCollector::Solve() {
	for ( int i = 0; i < m_manifolds.size(); i++ ) {
		m_manifolds[ i ].Solve();
	}
}

/*
================================
Manifold::PostSolve
================================
*/
void ManifoldCollector::PostSolve() {
	for ( int i = 0; i < m_manifolds.size(); i++ ) {
		m_manifolds[ i ].PostSolve();
	}
}

/*
================================================================================================

Manifold

================================================================================================
*/

/*
================================
Manifold::RemoveExpiredContacts
================================
*/
void Manifold::RemoveExpiredContacts() {
	// remove any contacts that have drifted too far
	for ( int i = 0; i < m_numContacts; i++ ) {
		contact_t & contact = m_contacts[ i ];

		Body * bodyA = contact.bodyA;
		Body * bodyB = contact.bodyB;

		// Get the tangential distance of the point on A and the point on B
		const Vec3 a = bodyA->BodySpaceToWorldSpace( contact.ptOnA_LocalSpace );
		const Vec3 b = bodyB->BodySpaceToWorldSpace( contact.ptOnB_LocalSpace );

		Vec3 normal = m_constraints[ i ].m_normal;
		normal = bodyA->m_orientation.RotatePoint( normal );

		// Calculate the tangential separation and penetration depth
		const Vec3 ab = b - a;
		float penetrationDepth = normal.Dot( ab );
		Vec3 abNormal = normal * penetrationDepth;
		Vec3 abTangent = ab - abNormal;

		// If the tangential displacement is less than a specific threshold, it's okay to keep it
		const float distanceThreshold = 0.02f;
		if ( abTangent.GetLengthSqr() < distanceThreshold * distanceThreshold && penetrationDepth <= 0.0f ) {
			continue;
		}

		// This contact has moved beyond its threshold and should be removed
		for ( int j = i; j < MAX_CONTACTS - 1; j++ ) {
			m_constraints[ j ] = m_constraints[ j + 1 ];
			m_contacts[ j ] = m_contacts[ j + 1 ];
			if ( j >= m_numContacts ) {
				m_constraints[ j ].m_cachedLambda.Zero();
			}
		}
		m_numContacts--;
		i--;
	}
}

/*
================================
Manifold::AddContact
================================
*/
void Manifold::AddContact( const contact_t & contact_old ) {
	// Make sure the contact's BodyA and BodyB are of the correct order
	contact_t contact = contact_old;
	if ( contact_old.bodyA != m_bodyA || contact_old.bodyB != m_bodyB ) {
		contact.ptOnA_LocalSpace = contact_old.ptOnB_LocalSpace;
		contact.ptOnB_LocalSpace = contact_old.ptOnA_LocalSpace;
		contact.ptOnA_WorldSpace = contact_old.ptOnB_WorldSpace;
		contact.ptOnB_WorldSpace = contact_old.ptOnA_WorldSpace;

		contact.bodyA = m_bodyA;
		contact.bodyB = m_bodyB;
	}

	// If this contact is close to another contact, then keep the old contact
	for ( int i = 0; i < m_numContacts; i++ ) {
		const Body * bodyA = m_contacts[ i ].bodyA;
		const Body * bodyB = m_contacts[ i ].bodyB;

		const Vec3 oldA = bodyA->BodySpaceToWorldSpace( m_contacts[ i ].ptOnA_LocalSpace );
		const Vec3 oldB = bodyB->BodySpaceToWorldSpace( m_contacts[ i ].ptOnB_LocalSpace );

		const Vec3 newA = contact.bodyA->BodySpaceToWorldSpace( contact.ptOnA_LocalSpace );
		const Vec3 newB = contact.bodyB->BodySpaceToWorldSpace( contact.ptOnB_LocalSpace );

		const Vec3 aa = newA - oldA;
		const Vec3 bb = newB - oldB;

		const float distanceThreshold = 0.02f;
		if ( aa.GetLengthSqr() < distanceThreshold * distanceThreshold ) {
			return;
		}
		if ( bb.GetLengthSqr() < distanceThreshold * distanceThreshold ) {
			return;
		}
	}

	// If we're all full on contacts, then keep the contacts that are furthest away from each other
	int newSlot = m_numContacts;
	if ( newSlot >= MAX_CONTACTS ) {
		Vec3 avg = Vec3( 0, 0, 0 );
		avg += m_contacts[ 0 ].ptOnA_LocalSpace;
		avg += m_contacts[ 1 ].ptOnA_LocalSpace;
		avg += m_contacts[ 2 ].ptOnA_LocalSpace;
		avg += m_contacts[ 3 ].ptOnA_LocalSpace;
		avg += contact.ptOnA_LocalSpace;
		avg *= 0.2f;

		float minDist = ( avg - contact.ptOnA_LocalSpace ).GetLengthSqr();
		int newIdx = -1;
		for ( int i = 0; i < MAX_CONTACTS; i++ ) {
			float dist2 = ( avg - m_contacts[ i ].ptOnA_LocalSpace ).GetLengthSqr();

			if ( dist2 < minDist ) {
				minDist = dist2;
				newIdx = i;
			}
		}

		if ( -1 != newIdx ) {
			newSlot = newIdx;
		} else {
			return;
		}
	}

	m_contacts[ newSlot ] = contact;

	m_constraints[ newSlot ].m_bodyA = contact.bodyA;
	m_constraints[ newSlot ].m_bodyB = contact.bodyB;
	m_constraints[ newSlot ].m_anchorA = contact.ptOnA_LocalSpace;
	m_constraints[ newSlot ].m_anchorB = contact.ptOnB_LocalSpace;

	// Get the normal in BodyA's space
	Vec3 normal = m_bodyA->m_orientation.Inverse().RotatePoint( contact.normal * -1.0f );
	m_constraints[ newSlot ].m_normal = normal;
	m_constraints[ newSlot ].m_normal.Normalize();

	m_constraints[ newSlot ].m_cachedLambda.Zero();

	if ( newSlot == m_numContacts ) {
		m_numContacts++;
	}
}

/*
================================
Manifold::PreSolve
================================
*/
void Manifold::PreSolve( const float dt_sec ) {
	for ( int i = 0; i < m_numContacts; i++ ) {
		m_constraints[ i ].PreSolve( dt_sec );
	}
}

/*
================================
Manifold::Solve
================================
*/
void Manifold::Solve() {
	for ( int i = 0; i < m_numContacts; i++ ) {
		m_constraints[ i ].Solve();
	}
}

/*
================================
Manifold::PostSolve
================================
*/
void Manifold::PostSolve() {
	for ( int i = 0; i < m_numContacts; i++ ) {
		m_constraints[ i ].PostSolve();
	}
}