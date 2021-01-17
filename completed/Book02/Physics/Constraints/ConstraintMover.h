//
//	ConstraintMover.h
//
#pragma once
#include "ConstraintBase.h"

/*
====================================================
ConstraintMoverSimple
====================================================
*/
class ConstraintMoverSimple : public Constraint {
public:
	ConstraintMoverSimple() : Constraint(), m_time( 0 ) {}

	void PreSolve( const float dt_sec ) override;

	float m_time;
};