//
//	Vector.h
//
#pragma once
#include <math.h>
#include <assert.h>
#include <stdio.h>

/*
 ================================
 Vec2
 ================================
 */
class Vec2 {
public:
	Vec2();
	Vec2( const float value );
	Vec2( const Vec2 & rhs );
	Vec2( float X, float Y );
	Vec2( const float * xy );
	Vec2 & operator = ( const Vec2 & rhs );
	
	bool			operator == ( const Vec2 & rhs ) const;
	bool			operator != ( const Vec2 & rhs ) const;
	Vec2			operator + ( const Vec2 & rhs ) const;
	const Vec2 &	operator += ( const Vec2 & rhs );
	const Vec2 &	operator -= ( const Vec2 & rhs );
	Vec2			operator - ( const Vec2 & rhs ) const;
	Vec2			operator * ( const float rhs ) const;
    const Vec2 &	operator *= ( const float rhs );
    const Vec2 &	operator /= ( const float rhs );
	float			operator [] ( const int idx ) const;
	float &			operator [] ( const int idx );
	
	const Vec2 & Normalize();
	float GetMagnitude() const;
	bool IsValid() const;
	float Dot( const Vec2 & rhs ) const { return x * rhs.x + y * rhs.y; }
	
	const float * ToPtr() const { return &x; }
	
public:
	float x;
	float y;
};

inline Vec2::Vec2() : 
x( 0 ), 
y( 0 ) {
}

inline Vec2::Vec2( const float value ) :
x( value ),
y( value ) {
}

inline Vec2::Vec2( const Vec2 & rhs ) :
x( rhs.x ),
y( rhs.y ) {
}

inline Vec2::Vec2( float X, float Y ) :
x( X ),
y( Y ) {
}

inline Vec2::Vec2( const float * xy ) :
x( xy[ 0 ] ),
y( xy[ 1 ] ) {
}

inline Vec2& Vec2::operator=( const Vec2 & rhs ) {
	x = rhs.x;
	y = rhs.y;
	return *this;
}

inline bool Vec2::operator==( const Vec2 & rhs ) const {
	if ( x != rhs.x ) {
		return false;
	}
	if ( y != rhs.y ) {
		return false;
	}
	
	return true;
}

inline bool Vec2::operator != ( const Vec2 & rhs ) const {
	if ( *this == rhs ) {
		return false;
	}
	
	return true;
}

inline Vec2 Vec2::operator + ( const Vec2 & rhs ) const {
	Vec2 temp;
	temp.x = x + rhs.x;
	temp.y = y + rhs.y;
	return temp;
}

inline const Vec2 & Vec2::operator += ( const Vec2 & rhs ) {
	x += rhs.x;
	y += rhs.y;
	return *this;
}

inline const Vec2 & Vec2::operator -= ( const Vec2 & rhs ) {
	x -= rhs.x;
	y -= rhs.y;
	return *this;
}

inline Vec2 Vec2::operator - ( const Vec2 & rhs ) const {
	Vec2 temp;
	temp.x = x - rhs.x;
	temp.y = y - rhs.y;
	return temp;
}

inline Vec2 Vec2::operator * ( const float rhs ) const {
	Vec2 temp;
	temp.x = x * rhs;
	temp.y = y * rhs;
	return temp;
}

inline const Vec2 & Vec2::operator *= ( const float rhs ) {
	x *= rhs;
	y *= rhs;
    return *this;
}

inline const Vec2 & Vec2::operator /= ( const float rhs ) {
	x /= rhs;
	y /= rhs;
    return *this;
}

inline float Vec2::operator [] ( const int idx ) const {
	assert( idx >= 0 && idx < 2 );
	return ( &x )[ idx ];
}

inline float & Vec2::operator [] ( const int idx ) {
	assert( idx >= 0 && idx < 2 );
	return ( &x )[ idx ];
}

inline const Vec2 & Vec2::Normalize() {
	float mag = GetMagnitude();
	float invMag = 1.0f / mag;
	if ( 0.0f * invMag == 0.0f * invMag ) {
		x = x * invMag;
		y = y * invMag;
	}
    
    return *this;
}

inline float Vec2::GetMagnitude() const {
	float mag;
	
	mag = x * x + y * y;
	mag = sqrtf( mag );
	
	return mag;
}

inline bool Vec2::IsValid() const {
	if ( x * 0.0f != x * 0.0f ) {
		// x is NaN or Inf
		return false;
	}
	
	if ( y * 0.0f != y * 0.0f ) {
		// y is NaN or Inf
		return false;
	}
	
	return true;
}

/*
 ================================
 Vec3
 ================================
 */
class Vec3 {
public:
	Vec3();
	Vec3( float value );
	Vec3( const Vec3 & rhs );
	Vec3( float X, float Y, float Z );
	Vec3( const float * xyz );
	Vec3 & operator = ( const Vec3 & rhs );
	Vec3 & operator = ( const float * rhs );
    
	bool			operator == ( const Vec3 & rhs ) const;
	bool			operator != ( const Vec3 & rhs ) const;
	Vec3			operator + ( const Vec3 & rhs ) const;
	const Vec3 &	operator += ( const Vec3 & rhs );
	const Vec3 &	operator -= ( const Vec3 & rhs );
	Vec3			operator - ( const Vec3 & rhs ) const;
	Vec3			operator * ( const float rhs ) const;
    Vec3			operator / ( const float rhs ) const;
	const Vec3 &	operator *= ( const float rhs );
    const Vec3 &	operator /= ( const float rhs );
	float			operator [] ( const int idx ) const;
	float &			operator [] ( const int idx );
	
	void Zero() { x = 0.0f; y = 0.0f; z = 0.0f; }

	Vec3 Cross( const Vec3 & rhs ) const;
	float Dot( const Vec3 & rhs ) const;
	
	const Vec3 & Normalize();
	float GetMagnitude() const;
	float GetLengthSqr() const { return Dot( *this ); }
	bool IsValid() const;
	void GetOrtho( Vec3 & u, Vec3 & v ) const;
	
	const float * ToPtr() const { return &x; }

public:
	float x;
	float y;
	float z;
};

inline Vec3::Vec3() :
x( 0 ),
y( 0 ),
z( 0 ) {
}

inline Vec3::Vec3( float value ) :
x( value ),
y( value ),
z( value ) {
}

inline Vec3::Vec3( const Vec3 &rhs ) :
x( rhs.x ),
y( rhs.y ),
z( rhs.z ) {
}

inline Vec3::Vec3( float X, float Y, float Z ) :
x( X ),
y( Y ),
z( Z ) {
}

inline Vec3::Vec3( const float * xyz ) :
x( xyz[ 0 ] ),
y( xyz[ 1 ] ),
z( xyz[ 2 ] ) {
}

inline Vec3 & Vec3::operator = ( const Vec3 & rhs ) {
	x = rhs.x;
	y = rhs.y;
	z = rhs.z;
	return *this;
}

inline Vec3& Vec3::operator=( const float * rhs ) {
	x = rhs[ 0 ];
	y = rhs[ 1 ];
	z = rhs[ 2 ];
	return *this;
}

inline bool Vec3::operator == ( const Vec3 & rhs ) const {
	if ( x != rhs.x ) {
		return false;
	}
	
	if ( y != rhs.y ) {
		return false;
	}
	
	if ( z != rhs.z ) {
		return false;
	}
	
	return true;
}

inline bool Vec3::operator != ( const Vec3 & rhs ) const {
	if ( *this == rhs ) {
		return false;
	}
	
	return true;
}

inline Vec3 Vec3::operator + ( const Vec3 & rhs ) const {
	Vec3 temp;
	temp.x = x + rhs.x;
	temp.y = y + rhs.y;
	temp.z = z + rhs.z;
	return temp;
}

inline const Vec3 & Vec3::operator += ( const Vec3 & rhs ) {
	x += rhs.x;
	y += rhs.y;
	z += rhs.z;
	return *this;
}

inline const Vec3 & Vec3::operator -= ( const Vec3 & rhs ) {
	x -= rhs.x;
	y -= rhs.y;
	z -= rhs.z;
	return *this;
}

inline Vec3 Vec3::operator - ( const Vec3 & rhs ) const {
	Vec3 temp;
	temp.x = x - rhs.x;
	temp.y = y - rhs.y;
	temp.z = z - rhs.z;
	return temp;
}

inline Vec3 Vec3::operator * ( const float rhs ) const {
	Vec3 temp;
	temp.x = x * rhs;
	temp.y = y * rhs;
	temp.z = z * rhs;
	return temp;
}

inline Vec3 Vec3::operator / ( const float rhs ) const {
	Vec3 temp;
	temp.x = x / rhs;
	temp.y = y / rhs;
	temp.z = z / rhs;
	return temp;
}

inline const Vec3 & Vec3::operator *= ( const float rhs ) {
	x *= rhs;
	y *= rhs;
	z *= rhs;
	return *this;
}

inline const Vec3 & Vec3::operator /= ( const float rhs ) {
	x /= rhs;
	y /= rhs;
	z /= rhs;
	return *this;
}

inline float Vec3::operator [] ( const int idx ) const {
	assert( idx >= 0 && idx < 3 );
	return ( &x )[ idx ];
}

inline float & Vec3::operator [] ( const int idx ) {
	assert( idx >= 0 && idx < 3 );
	return ( &x )[ idx ];
}

inline Vec3 Vec3::Cross( const Vec3 & rhs ) const {
	// This cross product is A x B, where this is A and rhs is B
	Vec3 temp;
	temp.x = ( y * rhs.z ) - ( rhs.y * z );
	temp.y = ( rhs.x * z ) - ( x * rhs.z );
	temp.z = ( x * rhs.y ) - ( rhs.x * y );
	return temp;
}

inline float Vec3::Dot( const Vec3 & rhs ) const {
	float temp = ( x * rhs.x ) + ( y * rhs.y ) + ( z * rhs.z );
	return temp;
}

inline const Vec3 & Vec3::Normalize() {
	float mag = GetMagnitude();
	float invMag = 1.0f / mag;
	if ( 0.0f * invMag == 0.0f * invMag ) {
		x *= invMag;
		y *= invMag;
		z *= invMag;
	}
    return *this;
}

inline float Vec3::GetMagnitude() const {
	float mag;
	
	mag = x * x + y * y + z * z;
	mag = sqrtf( mag );
	
	return mag;
}

inline bool Vec3::IsValid() const {
	if ( x * 0.0f != x * 0.0f ) {
		return false;
	}
	
	if ( y * 0.0f != y * 0.0f ) {
		return false;
	}
	
	if ( z * 0.0f != z * 0.0f ) {
		return false;
	}
	
	return true;
}

inline void Vec3::GetOrtho( Vec3 & u, Vec3 & v ) const {
	Vec3 n = *this;
	n.Normalize();

	const Vec3 w = ( n.z * n.z > 0.9f * 0.9f ) ? Vec3( 1, 0, 0 ) : Vec3( 0, 0, 1 );
	u = w.Cross( n );
	u.Normalize();

	v = n.Cross( u );
	v.Normalize();
	u = v.Cross( n );
	u.Normalize();
}

/*
 ================================
 Vec4
 ================================
 */
class Vec4 {
public:
	Vec4();
	Vec4( const float value );
	Vec4( const Vec4 & rhs );
	Vec4( float X, float Y, float Z, float W );
	Vec4( const float * rhs );
	Vec4 & operator = ( const Vec4 & rhs );
	
	bool			operator == ( const Vec4 & rhs ) const;
	bool			operator != ( const Vec4 & rhs ) const;
	Vec4			operator + ( const Vec4 & rhs ) const;
	const Vec4 &	operator += ( const Vec4 & rhs );
	const Vec4 &	operator -= ( const Vec4 & rhs );
    const Vec4 &	operator *= ( const Vec4 & rhs );
	const Vec4 &	operator /= ( const Vec4 & rhs );
	Vec4			operator - ( const Vec4 & rhs ) const;
	Vec4			operator * ( const float rhs ) const;
	float			operator [] ( const int idx ) const;
	float &			operator [] ( const int idx );
	
	float Dot( const Vec4 & rhs ) const;
	const Vec4 & Normalize();
	float GetMagnitude() const;
	bool IsValid() const;
	void Zero() { x = 0; y = 0; z = 0; w = 0; }
	
    const float *   ToPtr() const   { return &x; }
	float *         ToPtr()         { return &x; }
	
public:
	float x;
	float y;
	float z;
	float w;
};

inline Vec4::Vec4() :
x( 0 ),
y( 0 ),
z( 0 ),
w( 0 ) {
}

inline Vec4::Vec4( const float value ) :
x( value ),
y( value ),
z( value ),
w( value ) {
}

inline Vec4::Vec4( const Vec4 & rhs ) :
x( rhs.x ),
y( rhs.y ),
z( rhs.z ),
w( rhs.w ) {
}

inline Vec4::Vec4( float X, float Y, float Z, float W ) :
x( X ),
y( Y ),
z( Z ),
w( W ) {
}

inline Vec4::Vec4( const float * rhs ) {
	x = rhs[ 0 ];
	y = rhs[ 1 ];
	z = rhs[ 2 ];
	w = rhs[ 3 ];
}

inline Vec4 & Vec4::operator = ( const Vec4 & rhs ) {
	x = rhs.x;
	y = rhs.y;
	z = rhs.z;
	w = rhs.w;
	return *this;
}

inline bool Vec4::operator == ( const Vec4 & rhs ) const {
	if ( x != rhs.x ) {
		return false;
	}
	
	if ( y != rhs.y ) {
		return false;
	}
	
	if ( z != rhs.z ) {
		return false;
	}
	
	if ( w != rhs.w ) {
		return false;
	}
	
	return true;
}

inline bool Vec4::operator != ( const Vec4 & rhs ) const {
	if ( *this == rhs ) {
		return false;
	}
	
	return true;
}

inline Vec4 Vec4::operator + ( const Vec4 & rhs ) const {
	Vec4 temp;
	temp.x = x + rhs.x;
	temp.y = y + rhs.y;
	temp.z = z + rhs.z;
	temp.w = w + rhs.w;
	return temp;
}

inline const Vec4 & Vec4::operator += ( const Vec4 & rhs ) {
	x += rhs.x;
	y += rhs.y;
	z += rhs.z;
	w += rhs.w;
	return *this;
}

inline const Vec4 & Vec4::operator -= ( const Vec4 & rhs ) {
	x -= rhs.x;
	y -= rhs.y;
	z -= rhs.z;
	w -= rhs.w;
	return *this;
}

inline const Vec4 & Vec4::operator *= ( const Vec4 & rhs ) {
	x *= rhs.x;
	y *= rhs.y;
	z *= rhs.z;
	w *= rhs.w;
	return *this;
}

inline const Vec4 & Vec4::operator /= ( const Vec4 & rhs ) {
	x /= rhs.x;
	y /= rhs.y;
	z /= rhs.z;
	w /= rhs.w;
	return *this;
}

inline Vec4 Vec4::operator - ( const Vec4 & rhs ) const {
	Vec4 temp;
	temp.x = x - rhs.x;
	temp.y = y - rhs.y;
	temp.z = z - rhs.z;
	temp.w = w - rhs.w;
	return temp;
}

inline Vec4 Vec4::operator * ( const float rhs ) const {
	Vec4 temp;
	temp.x = x * rhs;
	temp.y = y * rhs;
	temp.z = z * rhs;
	temp.w = w * rhs;
	return temp;
}

inline float Vec4::operator [] ( const int idx ) const {
	assert( idx >= 0 && idx < 4 );
	return ( &x )[ idx ];
}

inline float& Vec4::operator [] ( const int idx ) {
	assert( idx >= 0 && idx < 4 );
	return ( &x )[ idx ];
}

inline float Vec4::Dot( const Vec4 & rhs ) const {
	float xx = x * rhs.x;
	float yy = y * rhs.y;
	float zz = z * rhs.z;
	float ww = w * rhs.w;
	return ( xx + yy + zz + ww );
}

inline const Vec4 & Vec4::Normalize() {
	float mag = GetMagnitude();
	float invMag = 1.0f / mag;
	if ( 0.0f * invMag == 0.0f * invMag ) {
		x *= invMag;
		y *= invMag;
		z *= invMag;
		w *= invMag;
	}
    
    return *this;
}

inline float Vec4::GetMagnitude() const {
	float mag;
	
	mag = x * x + y * y + z * z + w * w;
	mag = sqrtf( mag );
	
	return mag;
}

inline bool Vec4::IsValid() const {
	if ( x * 0.0f != x * 0.0f ) {
		return false;
	}
	
	if ( y * 0.0f != y * 0.0f ) {
		return false;
	}
	
	if ( z * 0.0f != z * 0.0f ) {
		return false;
	}
	
	if ( w * 0.0f != w * 0.0f ) {
		return false;
	}
	
	return true;
}

/*
 ================================
 VecN
 ================================
 */
class VecN {
public:
	VecN() : N( 0 ), data( NULL ) {}
	VecN( int _N );
	VecN( const VecN & rhs );
	VecN & operator = ( const VecN & rhs );
	~VecN() { delete[] data; }

	float			operator[] ( const int idx ) const { return data[ idx ]; }
	float &			operator[] ( const int idx ) { return data[ idx ]; }
	const VecN &	operator *= ( float rhs );
	VecN			operator * ( float rhs ) const;
	VecN			operator + ( const VecN & rhs ) const;
	VecN			operator - ( const VecN & rhs ) const;
	const VecN &	operator += ( const VecN & rhs );
	const VecN &	operator -= ( const VecN & rhs );

	float Dot( const VecN & rhs ) const;
	void Zero();
	
public:
	int		N;
	float *	data;
};

inline VecN::VecN( int _N ) {
	N = _N;
	data = new float[ _N ];
}

inline VecN::VecN( const VecN & rhs ) {
	N = rhs.N;
	data = new float[ N ];
	for ( int i = 0; i < N; i++ ) {
		data[ i ] = rhs.data[ i ];
	}
}

inline VecN & VecN::operator = ( const VecN & rhs ) {
	delete[] data;

	N = rhs.N;
	data = new float[ N ];
	for ( int i = 0; i < N; i++ ) {
		data[ i ] = rhs.data[ i ];
	}
	return *this;
}

inline const VecN & VecN::operator *= ( float rhs ) {
	for ( int i = 0; i < N; i++ ) {
		data[ i ] *= rhs;
	}
	return *this;
}

inline VecN VecN::operator * ( float rhs ) const {
	VecN tmp = *this;
	tmp *= rhs;
	return tmp;
}

inline VecN VecN::operator + ( const VecN & rhs ) const {
	VecN tmp = *this;
	for ( int i = 0; i < N; i++ ) {
		tmp.data[ i ] += rhs.data[ i ];
	}
	return tmp;
}

inline VecN VecN::operator - ( const VecN & rhs ) const {
	VecN tmp = *this;
	for ( int i = 0; i < N; i++ ) {
		tmp.data[ i ] -= rhs.data[ i ];
	}
	return tmp;
}

inline const VecN & VecN::operator += ( const VecN & rhs ) {
	for ( int i = 0; i < N; i++ ) {
		data[ i ] += rhs.data[ i ];
	}
	return *this;
}

inline const VecN & VecN::operator -= ( const VecN & rhs ) {
	for ( int i = 0; i < N; i++ ) {
		data[ i ] -= rhs.data[ i ];
	}
	return *this;
}

inline float VecN::Dot( const VecN & rhs ) const {
	float sum = 0;
	for ( int i = 0; i < N; i++ ) {
		sum += data[ i ] * rhs.data[ i ];
	}
	return sum;
}

inline void VecN::Zero() {
	for ( int i = 0; i < N; i++ ) {
		data[ i ] = 0.0f;
	}
}