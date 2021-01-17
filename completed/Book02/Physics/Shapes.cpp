//
//  Shapes.cpp
//
#include "Shapes.h"

static const float w = 50;
static const float h = 25;

Vec3 g_boxGround[] = {
	Vec3(-w,-h, 0 ),
	Vec3( w,-h, 0 ),
	Vec3(-w, h, 0 ),
	Vec3( w, h, 0 ),

	Vec3(-w,-h,-1 ),
	Vec3( w,-h,-1 ),
	Vec3(-w, h,-1 ),
	Vec3( w, h,-1 ),
};

Vec3 g_boxWall0[] = {
	Vec3(-1,-h, 0 ),
	Vec3( 1,-h, 0 ),
	Vec3(-1, h, 0 ),
	Vec3( 1, h, 0 ),

	Vec3(-1,-h, 5 ),
	Vec3( 1,-h, 5 ),
	Vec3(-1, h, 5 ),
	Vec3( 1, h, 5 ),
};

Vec3 g_boxWall1[] = {
	Vec3(-w,-1, 0 ),
	Vec3( w,-1, 0 ),
	Vec3(-w, 1, 0 ),
	Vec3( w, 1, 0 ),

	Vec3(-w,-1, 5 ),
	Vec3( w,-1, 5 ),
	Vec3(-w, 1, 5 ),
	Vec3( w, 1, 5 ),
};

Vec3 g_boxUnit[] = {
	Vec3(-1,-1,-1 ),
	Vec3( 1,-1,-1 ),
	Vec3(-1, 1,-1 ),
	Vec3( 1, 1,-1 ),

	Vec3(-1,-1, 1 ),
	Vec3( 1,-1, 1 ),
	Vec3(-1, 1, 1 ),
	Vec3( 1, 1, 1 ),
};

static const float t = 0.25f;
Vec3 g_boxSmall[] = {
	Vec3(-t,-t,-t ),
	Vec3( t,-t,-t ),
	Vec3(-t, t,-t ),
	Vec3( t, t,-t ),

	Vec3(-t,-t, t ),
	Vec3( t,-t, t ),
	Vec3(-t, t, t ),
	Vec3( t, t, t ),
};

static const float l = 3.0f;
Vec3 g_boxBeam[] = {
	Vec3(-l,-t,-t ),
	Vec3( l,-t,-t ),
	Vec3(-l, t,-t ),
	Vec3( l, t,-t ),

	Vec3(-l,-t, t ),
	Vec3( l,-t, t ),
	Vec3(-l, t, t ),
	Vec3( l, t, t ),
};

Vec3 g_boxPlatform[] = {
	Vec3(-l,-l,-t ),
	Vec3( l,-l,-t ),
	Vec3(-l, l,-t ),
	Vec3( l, l,-t ),

	Vec3(-l,-l, t ),
	Vec3( l,-l, t ),
	Vec3(-l, l, t ),
	Vec3( l, l, t ),
};

static const float t2 = 0.25f;
static const float w2 = t2 * 2.0f;
static const float h3 = t2 * 4.0f;
Vec3 g_boxBody[] = {
	Vec3(-t2,-w2,-h3 ),
	Vec3( t2,-w2,-h3 ),
	Vec3(-t2, w2,-h3 ),
	Vec3( t2, w2,-h3 ),

	Vec3(-t2,-w2, h3 ),
	Vec3( t2,-w2, h3 ),
	Vec3(-t2, w2, h3 ),
	Vec3( t2, w2, h3 ),
};

static const float h2 = 0.25f;
Vec3 g_boxLimb[] = {
	Vec3(-h3,-h2,-h2 ),
	Vec3( h3,-h2,-h2 ),
	Vec3(-h3, h2,-h2 ),
	Vec3( h3, h2,-h2 ),

	Vec3(-h3,-h2, h2 ),
	Vec3( h3,-h2, h2 ),
	Vec3(-h3, h2, h2 ),
	Vec3( h3, h2, h2 ),
};

Vec3 g_boxHead[] = {
	Vec3(-h2,-h2,-h2 ),
	Vec3( h2,-h2,-h2 ),
	Vec3(-h2, h2,-h2 ),
	Vec3( h2, h2,-h2 ),

	Vec3(-h2,-h2, h2 ),
	Vec3( h2,-h2, h2 ),
	Vec3(-h2, h2, h2 ),
	Vec3( h2, h2, h2 ),
};

Vec3 g_diamond[ 7 * 8 ];
void FillDiamond() {
	Vec3 pts[ 4 + 4 ];
	pts[ 0 ] = Vec3( 0.1f, 0, -1 );
	pts[ 1 ] = Vec3( 1, 0, 0 );
	pts[ 2 ] = Vec3( 1, 0, 0.1f );
	pts[ 3 ] = Vec3( 0.4f, 0, 0.4f );

	const float pi = acosf( -1.0f );
	const Quat quatHalf( Vec3( 0, 0, 1 ), 2.0f * pi * 0.125f * 0.5f );
	pts[ 4 ] = Vec3( 0.8f, 0, 0.3f );
	pts[ 4 ] = quatHalf.RotatePoint( pts[ 4 ] );
	pts[ 5 ] = quatHalf.RotatePoint( pts[ 1 ] );
	pts[ 6 ] = quatHalf.RotatePoint( pts[ 2 ] );

	const Quat quat( Vec3( 0, 0, 1 ), 2.0f * pi * 0.125f );
	int idx = 0;
	for ( int i = 0; i < 7; i++ ) {
		g_diamond[ idx ] = pts[ i ];
		idx++;
	}

	Quat quatAccumulator;
	for ( int i = 1; i < 8; i++ ) {
		quatAccumulator = quatAccumulator * quat;
		for ( int pt = 0; pt < 7; pt++ ) {
			g_diamond[ idx ] = quatAccumulator.RotatePoint( pts[ pt ] );
			idx++;
		}
	}
}