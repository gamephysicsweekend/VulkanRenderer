//
//  ConstraintMover.cpp
//
#include "ConstraintMover.h"

/*
====================================================
ConstraintMoverSimple::PreSolve
====================================================
*/
void ConstraintMoverSimple::PreSolve( const float dt_sec ) {
	m_time += dt_sec;
	m_bodyA->m_linearVelocity.y = cosf( m_time * 0.25f ) * 4.0f;
}