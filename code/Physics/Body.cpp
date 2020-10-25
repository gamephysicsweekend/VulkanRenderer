//
//  Body.cpp
//
#include "Body.h"

/*
====================================================
Body::Body
====================================================
*/
Body::Body() :
m_position( 0.0f ),
m_orientation( 0.0f, 0.0f, 0.0f, 1.0f ),
m_shape( NULL ) {
}