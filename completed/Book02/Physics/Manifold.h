//
//	Manifold.h
//
#pragma once
#include "Body.h"
#include "Constraints.h"
#include "Contact.h"

/*
================================
Manifold
================================
*/
class Manifold {
public:
	Manifold() : m_bodyA( NULL ), m_bodyB( NULL ), m_numContacts( 0 ) {}

	void AddContact( const contact_t & contact );
	void RemoveExpiredContacts();

	void PreSolve( const float dt_sec );
	void Solve();
	void PostSolve();

	contact_t GetContact( const int idx ) const { return m_contacts[ idx ]; }
	int GetNumContacts() const { return m_numContacts; }

private:
	static const int MAX_CONTACTS = 4;
	contact_t m_contacts[ MAX_CONTACTS ];

	int m_numContacts;

	Body * m_bodyA;
	Body * m_bodyB;

	ConstraintPenetration m_constraints[ MAX_CONTACTS ];

	friend class ManifoldCollector;
};

/*
================================
ManifoldCollector
================================
*/
class ManifoldCollector {
public:
	ManifoldCollector() {}

	void AddContact( const contact_t & contact );

	void PreSolve( const float dt_sec );
	void Solve();
	void PostSolve();

	void RemoveExpired();
	void Clear() { m_manifolds.clear(); }	// For resetting the demo

public:
	std::vector< Manifold > m_manifolds;
};