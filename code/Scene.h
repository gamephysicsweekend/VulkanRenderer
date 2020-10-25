//
//  Scene.h
//
#pragma once
#include <vector>

#include "Physics/Shapes.h"
#include "Physics/Body.h"

/*
====================================================
Scene
====================================================
*/
class Scene {
public:
	Scene() { m_bodies.reserve( 128 ); }
	~Scene();

	void Reset();
	void Initialize();
	void Update( const float dt_sec );	

	std::vector< Body > m_bodies;
};

