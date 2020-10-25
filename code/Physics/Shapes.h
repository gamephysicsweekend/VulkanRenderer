//
//	Shapes.h
//
#pragma once
#include "Shapes/ShapeSphere.h"
#include "Shapes/ShapeBox.h"
#include "Shapes/ShapeConvex.h"

extern Vec3 g_boxGround[ 8 ];
extern Vec3 g_boxWall0[ 8 ];
extern Vec3 g_boxWall1[ 8 ];
extern Vec3 g_boxUnit[ 8 ];
extern Vec3 g_boxSmall[ 8 ];
extern Vec3 g_boxBeam[ 8 ];
extern Vec3 g_boxPlatform[ 8 ];
extern Vec3 g_boxBody[ 8 ];
extern Vec3 g_boxLimb[ 8 ];
extern Vec3 g_boxHead[ 8 ];
extern Vec3 g_diamond[ 7 * 8 ];
void FillDiamond();