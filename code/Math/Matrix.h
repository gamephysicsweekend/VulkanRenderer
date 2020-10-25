//
//	Matrix.h
//
#pragma once
#include "Vector.h"

/*
====================================================
Mat2
====================================================
*/
class Mat2 {
public:
	Mat2() {}
	Mat2( const Mat2 & rhs );
	Mat2( const float * mat );
	Mat2( const Vec2 & row0, const Vec2 & row1 );
	Mat2 & operator = ( const Mat2 & rhs );

	const Mat2 & operator *= ( const float rhs );
	const Mat2 & operator += ( const Mat2 & rhs );

	float Determinant() const { return rows[ 0 ].x * rows[ 1 ].y - rows[ 0 ].y * rows[ 1 ].x; }

public:
	Vec2 rows[ 2 ];
};

inline Mat2::Mat2( const Mat2 & rhs ) {
	rows[ 0 ] = rhs.rows[ 0 ];
	rows[ 1 ] = rhs.rows[ 1 ];
}

inline Mat2::Mat2( const float * mat ) {
	rows[ 0 ] = mat + 0;
	rows[ 1 ] = mat + 2;
}

inline Mat2::Mat2( const Vec2 & row0, const Vec2 & row1 ) {
	rows[ 0 ] = row0;
	rows[ 1 ] = row1;
}

inline Mat2 & Mat2::operator = ( const Mat2 & rhs ) {
	rows[ 0 ] = rhs.rows[ 0 ];
	rows[ 1 ] = rhs.rows[ 1 ];
	return *this;
}

inline const Mat2 & Mat2::operator *= ( const float rhs ) {
	rows[ 0 ] *= rhs;
	rows[ 1 ] *= rhs;
	return *this;
}

inline const Mat2 & Mat2::operator += ( const Mat2 & rhs ) {
	rows[ 0 ] += rhs.rows[ 0 ];
	rows[ 1 ] += rhs.rows[ 1 ];
	return *this;
}

/*
====================================================
Mat3
====================================================
*/
class Mat3 {
public:
	Mat3() {}
	Mat3( const Mat3 & rhs );
	Mat3( const float * mat );
	Mat3( const Vec3 & row0, const Vec3 & row1, const Vec3 & row2 );
	Mat3 & operator = ( const Mat3 & rhs );

	void Zero();
	void Identity();

	float Trace() const;
	float Determinant() const;
	Mat3 Transpose() const;
	Mat3 Inverse() const;
	Mat2 Minor( const int i, const int j ) const;
	float Cofactor( const int i, const int j ) const;

	Vec3 operator * ( const Vec3 & rhs ) const;
	Mat3 operator * ( const float rhs ) const;
	Mat3 operator * ( const Mat3 & rhs ) const;
	Mat3 operator + ( const Mat3 & rhs ) const;
	const Mat3 & operator *= ( const float rhs );
	const Mat3 & operator += ( const Mat3 & rhs );

public:
	Vec3 rows[ 3 ];
};

inline Mat3::Mat3( const Mat3 & rhs ) {
	rows[ 0 ] = rhs.rows[ 0 ];
	rows[ 1 ] = rhs.rows[ 1 ];
	rows[ 2 ] = rhs.rows[ 2 ];
}

inline Mat3::Mat3( const float * mat ) {
	rows[ 0 ] = mat + 0;
	rows[ 1 ] = mat + 3;
	rows[ 2 ] = mat + 6;
}

inline Mat3::Mat3( const Vec3 & row0, const Vec3 & row1, const Vec3 & row2 ) {
	rows[ 0 ] = row0;
	rows[ 1 ] = row1;
	rows[ 2 ] = row2;
}

inline Mat3 & Mat3::operator = ( const Mat3 & rhs ) {
	rows[ 0 ] = rhs.rows[ 0 ];
	rows[ 1 ] = rhs.rows[ 1 ];
	rows[ 2 ] = rhs.rows[ 2 ];
	return *this;
}

inline const Mat3 & Mat3::operator *= ( const float rhs ) {
	rows[ 0 ] *= rhs;
	rows[ 1 ] *= rhs;
	rows[ 2 ] *= rhs;
	return *this;
}

inline const Mat3 & Mat3::operator += ( const Mat3 & rhs ) {
	rows[ 0 ] += rhs.rows[ 0 ];
	rows[ 1 ] += rhs.rows[ 1 ];
	rows[ 2 ] += rhs.rows[ 2 ];
	return *this;
}

inline void Mat3::Zero() {
	rows[ 0 ].Zero();
	rows[ 1 ].Zero();
	rows[ 2 ].Zero();
}

inline void Mat3::Identity() {
	rows[ 0 ] = Vec3( 1, 0, 0 );
	rows[ 1 ] = Vec3( 0, 1, 0 );
	rows[ 2 ] = Vec3( 0, 0, 1 );
}

inline float Mat3::Trace() const {
	const float xx = rows[ 0 ][ 0 ] * rows[ 0 ][ 0 ];
	const float yy = rows[ 1 ][ 1 ] * rows[ 1 ][ 1 ];
	const float zz = rows[ 2 ][ 2 ] * rows[ 2 ][ 2 ];
	return ( xx + yy + zz );
}

inline float Mat3::Determinant() const {
	const float i = rows[ 0 ][ 0 ] * ( rows[ 1 ][ 1 ] * rows[ 2 ][ 2 ] - rows[ 1 ][ 2 ] * rows[ 2 ][ 1 ] );
	const float j = rows[ 0 ][ 1 ] * ( rows[ 1 ][ 0 ] * rows[ 2 ][ 2 ] - rows[ 1 ][ 2 ] * rows[ 2 ][ 0 ] );
	const float k = rows[ 0 ][ 2 ] * ( rows[ 1 ][ 0 ] * rows[ 2 ][ 1 ] - rows[ 1 ][ 1 ] * rows[ 2 ][ 0 ] );
	return ( i - j + k );
}

inline Mat3 Mat3::Transpose() const {
	Mat3 transpose;
	for ( int i = 0; i < 3; i++ ) {
		for ( int j = 0; j < 3; j++ ) {
			transpose.rows[ i ][ j ] = rows[ j ][ i ];
		}
	}
	return transpose;
}

inline Mat3 Mat3::Inverse() const {
	Mat3 inv;
	for ( int i = 0; i < 3; i++ ) {
		for ( int j = 0; j < 3; j++ ) {
			inv.rows[ j ][ i ] = Cofactor( i, j );	// Perform the transpose while calculating the cofactors
		}
	}
	float det = Determinant();
	float invDet = 1.0f / det;
	inv *= invDet;
	return inv;
}

inline Mat2 Mat3::Minor( const int i, const int j ) const {
	Mat2 minor;

	int yy = 0;
	for ( int y = 0; y < 3; y++ ) {
		if ( y == j ) {
			continue;
		}

		int xx = 0;
		for ( int x = 0; x < 3; x++ ) {
			if ( x == i ) {
				continue;
			}

			minor.rows[ xx ][ yy ] = rows[ x ][ y ];
			xx++;
		}

		yy++;
	}
	return minor;
}

inline float Mat3::Cofactor( const int i, const int j ) const {
	const Mat2 minor = Minor( i, j );
	const float C = float( pow( -1, i + 1 + j + 1 ) ) * minor.Determinant();
	return C;
}

inline Vec3 Mat3::operator * ( const Vec3 & rhs ) const {
	Vec3 tmp;
	tmp[ 0 ] = rows[ 0 ].Dot( rhs );
	tmp[ 1 ] = rows[ 1 ].Dot( rhs );
	tmp[ 2 ] = rows[ 2 ].Dot( rhs );
	return tmp;
}

inline Mat3 Mat3::operator * ( const float rhs ) const {
	Mat3 tmp;
	tmp.rows[ 0 ] = rows[ 0 ] * rhs;
	tmp.rows[ 1 ] = rows[ 1 ] * rhs;
	tmp.rows[ 2 ] = rows[ 2 ] * rhs;
	return tmp;
}

inline Mat3 Mat3::operator * ( const Mat3 & rhs ) const {
	Mat3 tmp;
	for ( int i = 0; i < 3; i++ ) {
		tmp.rows[ i ].x = rows[ i ].x * rhs.rows[ 0 ].x + rows[ i ].y * rhs.rows[ 1 ].x + rows[ i ].z * rhs.rows[ 2 ].x;
		tmp.rows[ i ].y = rows[ i ].x * rhs.rows[ 0 ].y + rows[ i ].y * rhs.rows[ 1 ].y + rows[ i ].z * rhs.rows[ 2 ].y;
		tmp.rows[ i ].z = rows[ i ].x * rhs.rows[ 0 ].z + rows[ i ].y * rhs.rows[ 1 ].z + rows[ i ].z * rhs.rows[ 2 ].z;
	}
	return tmp;
}

inline Mat3 Mat3::operator + ( const Mat3 & rhs ) const {
	Mat3 tmp;
	for ( int i = 0; i < 3; i++ ) {
		tmp.rows[ i ] = rows[ i ] + rhs.rows[ i ];
	}
	return tmp;
}

/*
====================================================
Mat4
====================================================
*/
class Mat4 {
public:
	Mat4() {}
	Mat4( const Mat4 & rhs );
	Mat4( const float * mat );
	Mat4( const Vec4 & row0, const Vec4 & row1, const Vec4 & row2, const Vec4 & row3 );
	Mat4 & operator = ( const Mat4 & rhs );
	~Mat4() {}

	void Zero();
	void Identity();

	float Trace() const;
	float Determinant() const;
	Mat4 Transpose() const;
	Mat4 Inverse() const;
	Mat3 Minor( const int i, const int j ) const;
	float Cofactor( const int i, const int j ) const;

	void Orient( Vec3 pos, Vec3 fwd, Vec3 up );
	void LookAt( Vec3 pos, Vec3 lookAt, Vec3 up );
	void PerspectiveOpenGL( float fovy, float aspect_ratio, float near, float far );
	void PerspectiveVulkan( float fovy, float aspect_ratio, float near, float far );
	void OrthoOpenGL( float xmin, float xmax, float ymin, float ymax, float znear, float zfar );
	void OrthoVulkan( float xmin, float xmax, float ymin, float ymax, float znear, float zfar );

	const float * ToPtr() const { return rows[ 0 ].ToPtr(); }
	float * ToPtr() { return rows[ 0 ].ToPtr(); }

	Vec4 operator * ( const Vec4 & rhs ) const;
	Mat4 operator * ( const float rhs ) const;
	Mat4 operator * ( const Mat4 & rhs ) const;
	const Mat4 & operator *= ( const float rhs );

public:
	Vec4 rows[ 4 ];
};

inline Mat4::Mat4( const Mat4 & rhs ) {
	rows[ 0 ] = rhs.rows[ 0 ];
	rows[ 1 ] = rhs.rows[ 1 ];
	rows[ 2 ] = rhs.rows[ 2 ];
	rows[ 3 ] = rhs.rows[ 3 ];
}

inline Mat4::Mat4( const float * mat ) {
	rows[ 0 ] = mat + 0;
	rows[ 1 ] = mat + 4;
	rows[ 2 ] = mat + 8;
	rows[ 3 ] = mat + 12;
}

inline Mat4::Mat4( const Vec4 & row0, const Vec4 & row1, const Vec4 & row2, const Vec4 & row3 ) {
	rows[ 0 ] = row0;
	rows[ 1 ] = row1;
	rows[ 2 ] = row2;
	rows[ 3 ] = row3;
}

inline Mat4 & Mat4::operator = ( const Mat4 & rhs ) {
	rows[ 0 ] = rhs.rows[ 0 ];
	rows[ 1 ] = rhs.rows[ 1 ];
	rows[ 2 ] = rhs.rows[ 2 ];
	rows[ 3 ] = rhs.rows[ 3 ];
	return *this;
}

inline const Mat4 & Mat4::operator *= ( const float rhs ) {
	rows[ 0 ] *= rhs;
	rows[ 1 ] *= rhs;
	rows[ 2 ] *= rhs;
	rows[ 3 ] *= rhs;
	return *this;
}

inline void Mat4::Zero() {
	rows[ 0 ].Zero();
	rows[ 1 ].Zero();
	rows[ 2 ].Zero();
	rows[ 3 ].Zero();
}

inline void Mat4::Identity() {
	rows[ 0 ] = Vec4( 1, 0, 0, 0 );
	rows[ 1 ] = Vec4( 0, 1, 0, 0 );
	rows[ 2 ] = Vec4( 0, 0, 1, 0 );
	rows[ 3 ] = Vec4( 0, 0, 0, 1 );
}

inline float Mat4::Trace() const {
	const float xx = rows[ 0 ][ 0 ] * rows[ 0 ][ 0 ];
	const float yy = rows[ 1 ][ 1 ] * rows[ 1 ][ 1 ];
	const float zz = rows[ 2 ][ 2 ] * rows[ 2 ][ 2 ];
	const float ww = rows[ 3 ][ 3 ] * rows[ 3 ][ 3 ];
	return ( xx + yy + zz + ww );
}

inline float Mat4::Determinant() const {
	float det = 0.0f;
	float sign = 1.0f;
	for ( int j = 0; j < 4; j++ ) {
		Mat3 minor = Minor( 0, j );

		det += rows[ 0 ][ j ] * minor.Determinant() * sign;
		sign = sign * -1.0f;
	}
	return det;
}

inline Mat4 Mat4::Transpose() const {
	Mat4 transpose;
	for ( int i = 0; i < 4; i++ ) {
		for ( int j = 0; j < 4; j++ ) {
			transpose.rows[ i ][ j ] = rows[ j ][ i ];
		}
	}
	return transpose;
}

inline Mat4 Mat4::Inverse() const {
	Mat4 inv;
	for ( int i = 0; i < 4; i++ ) {
		for ( int j = 0; j < 4; j++ ) {
			inv.rows[ j ][ i ] = Cofactor( i, j );	// Perform the transpose while calculating the cofactors
		}
	}
	float det = Determinant();
	float invDet = 1.0f / det;
	inv *= invDet;
	return inv;
}

inline Mat3 Mat4::Minor( const int i, const int j ) const {
	Mat3 minor;

	int yy = 0;
	for ( int y = 0; y < 4; y++ ) {
		if ( y == j ) {
			continue;
		}

		int xx = 0;
		for ( int x = 0; x < 4; x++ ) {
			if ( x == i ) {
				continue;
			}

			minor.rows[ xx ][ yy ] = rows[ x ][ y ];
			xx++;
		}

		yy++;
	}
	return minor;
}

inline float Mat4::Cofactor( const int i, const int j ) const {
	const Mat3 minor = Minor( i, j );
	const float C = float( pow( -1, i + 1 + j + 1 ) ) * minor.Determinant();
	return C;
}

inline void Mat4::Orient( Vec3 pos, Vec3 fwd, Vec3 up ) {
	Vec3 left = up.Cross( fwd );

	// For our coordinate system where:
	// +x-axis = fwd
	// +y-axis = left
	// +z-axis = up
	rows[ 0 ] = Vec4( fwd.x, left.x, up.x, pos.x );
	rows[ 1 ] = Vec4( fwd.y, left.y, up.y, pos.y );
	rows[ 2 ] = Vec4( fwd.z, left.z, up.z, pos.z );
	rows[ 3 ] = Vec4( 0, 0, 0, 1 );
}

inline void Mat4::LookAt( Vec3 pos, Vec3 lookAt, Vec3 up ) {
	Vec3 fwd = pos - lookAt;
	fwd.Normalize();

	Vec3 right = up.Cross( fwd );
	right.Normalize();

	up = fwd.Cross( right );
	up.Normalize();

	// For NDC coordinate system where:
	// +x-axis = right
	// +y-axis = up
	// +z-axis = fwd
	rows[ 0 ] = Vec4( right.x, right.y, right.z, -pos.Dot( right ) );
	rows[ 1 ] = Vec4( up.x, up.y, up.z, -pos.Dot( up ) );
	rows[ 2 ] = Vec4( fwd.x, fwd.y, fwd.z, -pos.Dot( fwd ) );
	rows[ 3 ] = Vec4( 0, 0, 0, 1 );
}

inline void Mat4::PerspectiveOpenGL( float fovy, float aspect_ratio, float near, float far ) {
	const float pi = acosf( -1.0f );
	const float fovy_radians = fovy * pi / 180.0f;
	const float f = 1.0f / tanf( fovy_radians * 0.5f );
	const float xscale = f;
	const float yscale = f / aspect_ratio;

	rows[ 0 ] = Vec4( xscale, 0, 0, 0 );
	rows[ 1 ] = Vec4( 0, yscale, 0, 0 );
	rows[ 2 ] = Vec4( 0, 0, ( far + near ) / ( near - far ), ( 2.0f * far * near ) / ( near - far ) );
	rows[ 3 ] = Vec4( 0, 0, -1, 0 );
}

inline void Mat4::PerspectiveVulkan( float fovy, float aspect_ratio, float near, float far ) {
	// Vulkan changed its NDC.  It switch from a left handed coordinate system to a right handed one.
	// +x points to the right, +z points into the screen, +y points down (it used to point up, in opengl).
	// It also changed the range from [-1,1] to [0,1] for the z.
	// Clip space remains [-1,1] for x and y.
	// Check section 23 of the specification.
	Mat4 matVulkan;
	matVulkan.rows[ 0 ] = Vec4( 1, 0, 0, 0 );
	matVulkan.rows[ 1 ] = Vec4( 0, -1, 0, 0 );
	matVulkan.rows[ 2 ] = Vec4( 0, 0, 0.5f, 0.5f );
	matVulkan.rows[ 3 ] = Vec4( 0, 0, 0, 1 );

	Mat4 matOpenGL;
	matOpenGL.PerspectiveOpenGL( fovy, aspect_ratio, near, far );

	*this = matVulkan * matOpenGL;
}

inline void Mat4::OrthoOpenGL( float xmin, float xmax, float ymin, float ymax, float znear, float zfar ) {
	const float width	= xmax - xmin;
	const float height	= ymax - ymin;
	const float depth	= zfar - znear;

	const float tx = -( xmax + xmin ) / width;
	const float ty = -( ymax + ymin ) / height;
	const float tz = -( zfar + znear ) / depth;

	rows[ 0 ] = Vec4( 2.0f / width, 0, 0, tx );
	rows[ 1 ] = Vec4( 0, 2.0f / height, 0, ty );
	rows[ 2 ] = Vec4( 0, 0, -2.0f / depth, tz );
	rows[ 3 ] = Vec4( 0, 0, 0, 1 );
}

inline void Mat4::OrthoVulkan( float xmin, float xmax, float ymin, float ymax, float znear, float zfar ) {
	// Vulkan changed its NDC.  It switch from a left handed coordinate system to a right handed one.
	// +x points to the right, +z points into the screen, +y points down (it used to point up, in opengl).
	// It also changed the range from [-1,1] to [0,1] for the z.
	// Clip space remains [-1,1] for x and y.
	// Check section 23 of the specification.
	Mat4 matVulkan;
	matVulkan.rows[ 0 ] = Vec4( 1, 0, 0, 0 );
	matVulkan.rows[ 1 ] = Vec4( 0, -1, 0, 0 );
	matVulkan.rows[ 2 ] = Vec4( 0, 0, 0.5f, 0.5f );
	matVulkan.rows[ 3 ] = Vec4( 0, 0, 0, 1 );

	Mat4 matOpenGL;
	matOpenGL.OrthoOpenGL( xmin, xmax, ymin, ymax, znear, zfar );

	*this = matVulkan * matOpenGL;
}

inline Vec4 Mat4::operator * ( const Vec4 & rhs ) const {
	Vec4 tmp;
	tmp[ 0 ] = rows[ 0 ].Dot( rhs );
	tmp[ 1 ] = rows[ 1 ].Dot( rhs );
	tmp[ 2 ] = rows[ 2 ].Dot( rhs );
	tmp[ 3 ] = rows[ 3 ].Dot( rhs );
	return tmp;
}

inline Mat4 Mat4::operator * ( const float rhs ) const {
	Mat4 tmp;
	tmp.rows[ 0 ] = rows[ 0 ] * rhs;
	tmp.rows[ 1 ] = rows[ 1 ] * rhs;
	tmp.rows[ 2 ] = rows[ 2 ] * rhs;
	tmp.rows[ 3 ] = rows[ 3 ] * rhs;
	return tmp;
}

inline Mat4 Mat4::operator * ( const Mat4 & rhs ) const {
	Mat4 tmp;
	for ( int i = 0; i < 4; i++ ) {
		tmp.rows[ i ].x = rows[ i ].x * rhs.rows[ 0 ].x + rows[ i ].y * rhs.rows[ 1 ].x + rows[ i ].z * rhs.rows[ 2 ].x + rows[ i ].w * rhs.rows[ 3 ].x;
		tmp.rows[ i ].y = rows[ i ].x * rhs.rows[ 0 ].y + rows[ i ].y * rhs.rows[ 1 ].y + rows[ i ].z * rhs.rows[ 2 ].y + rows[ i ].w * rhs.rows[ 3 ].y;
		tmp.rows[ i ].z = rows[ i ].x * rhs.rows[ 0 ].z + rows[ i ].y * rhs.rows[ 1 ].z + rows[ i ].z * rhs.rows[ 2 ].z + rows[ i ].w * rhs.rows[ 3 ].z;
		tmp.rows[ i ].w = rows[ i ].x * rhs.rows[ 0 ].w + rows[ i ].y * rhs.rows[ 1 ].w + rows[ i ].z * rhs.rows[ 2 ].w + rows[ i ].w * rhs.rows[ 3 ].w;
	}
	return tmp;
}

/*
====================================================
MatMN
====================================================
*/
class MatMN {
public:
	MatMN() : M( 0 ), N( 0 ) {}
	MatMN( int M, int N );
	MatMN( const MatMN & rhs ) {
		*this = rhs;
	}
	~MatMN() { delete[] rows; }

	const MatMN & operator = ( const MatMN & rhs );
	const MatMN & operator *= ( float rhs );
	VecN operator * ( const VecN & rhs ) const;
	MatMN operator * ( const MatMN & rhs ) const;
	MatMN operator * ( const float rhs ) const;

	void Zero();
	MatMN Transpose() const;

public:
	int		M;	// M rows
	int		N;	// N columns
	VecN *	rows;
};

inline MatMN::MatMN( int _M, int _N ) {
	M = _M;
	N = _N;
	rows = new VecN[ M ];
	for ( int m = 0; m < M; m++ ) {
		rows[ m ] = VecN( N );
	}
}

inline const MatMN & MatMN::operator = ( const MatMN & rhs ) {
	M = rhs.M;
	N = rhs.N;
	rows = new VecN[ M ];
	for ( int m = 0; m < M; m++ ) {
		rows[ m ] = rhs.rows[ m ];
	}
	return *this;
}

inline const MatMN & MatMN::operator *= ( float rhs ) {
	for ( int m = 0; m < M; m++ ) {
		rows[ m ] *= rhs;
	}
	return *this;
}

inline VecN MatMN::operator * ( const VecN & rhs ) const {
	// Check that the incoming vector is of the correct dimension
	if ( rhs.N != N ) {
		return rhs;
	}

	VecN tmp( M );
	for ( int m = 0; m < M; m++ ) {
		tmp[ m ] = rhs.Dot( rows[ m ] );
	}
	return tmp;
}

inline MatMN MatMN::operator * ( const MatMN & rhs ) const {
	// Check that the incoming matrix of the correct dimension
	if ( rhs.M != N && rhs.N != M ) {
		return rhs;
	}

	MatMN tranposedRHS = rhs.Transpose();

	MatMN tmp( M, rhs.N );
	for ( int m = 0; m < M; m++ ) {
		for ( int n = 0; n < rhs.N; n++ ) {
			tmp.rows[ m ][ n ] = rows[ m ].Dot( tranposedRHS.rows[ n ] );
		}
	}
	return tmp;
}

inline MatMN MatMN::operator * ( const float rhs ) const {
	MatMN tmp = *this;
	for ( int m = 0; m < M; m++ ) {
		for ( int n = 0; n < N; n++ ) {
			tmp.rows[ m ][ n ] *= rhs;
		}
	}
	return tmp;
}

inline void MatMN::Zero() {
	for ( int m = 0; m < M; m++ ) {
		rows[ m ].Zero();
	}
}

inline MatMN MatMN::Transpose() const {
	MatMN tmp( N, M );
	for ( int m = 0; m < M; m++ ) {
		for ( int n = 0; n < N; n++ ) {
			tmp.rows[ n ][ m ] = rows[ m ][ n ];
		}		
	}
	return tmp;
}

/*
====================================================
MatN
====================================================
*/
class MatN {
public:
	MatN() : numDimensions( 0 ) {}
	MatN( int N );
	MatN( const MatN & rhs ) {
		*this = rhs;
	}
	MatN( const MatMN & rhs ) {
		*this = rhs;
	}
	~MatN() { delete[] rows; }

	const MatN & operator = ( const MatN & rhs );
	const MatN & operator = ( const MatMN & rhs );

	void Identity();
	void Zero();
	void Transpose();

	void operator *= ( float rhs );
	VecN operator * ( const VecN & rhs );
	MatN operator * ( const MatN & rhs );

public:
	int		numDimensions;
	VecN *	rows;
};

inline MatN::MatN( int N ) {
	numDimensions = N;
	rows = new VecN[ N ];
	for ( int i = 0; i < N; i++ ) {
		rows[ i ] = VecN( N );
	}
}

inline const MatN & MatN::operator = ( const MatN & rhs ) {
	numDimensions = rhs.numDimensions;
	rows = new VecN[ numDimensions ];
	for ( int i = 0; i < numDimensions; i++ ) {
		rows[ i ] = rhs.rows[ i ];
	}
	return *this;
}

inline const MatN & MatN::operator = ( const MatMN & rhs ) {
	if ( rhs.M != rhs.N ) {
		return *this;
	}

	numDimensions = rhs.N;
	rows = new VecN[ numDimensions ];
	for ( int i = 0; i < numDimensions; i++ ) {
		rows[ i ] = rhs.rows[ i ];
	}
	return *this;
}

inline void MatN::Zero() {
	for ( int i = 0; i < numDimensions; i++ ) {
		rows[ i ].Zero();
	}
}

inline void MatN::Identity() {
	for ( int i = 0; i < numDimensions; i++ ) {
		rows[ i ].Zero();
		rows[ i ][ i ] = 1.0f;
	}
}

inline void MatN::Transpose() {
	MatN tmp( numDimensions );

	for ( int i = 0; i < numDimensions; i++ ) {
		for ( int j = 0; j < numDimensions; j++ ) {
			tmp.rows[ i ][ j ] = rows[ j ][ i ];
		}
	}

	*this = tmp;
}

inline void MatN::operator *= ( float rhs ) {
	for ( int i = 0; i < numDimensions; i++ ) {
		rows[ i ] *= rhs;
	}
}

inline VecN MatN::operator * ( const VecN & rhs ) {
	VecN tmp( numDimensions );

	for ( int i = 0; i < numDimensions; i++ ) {
		tmp[ i ] = rows[ i ].Dot( rhs );
	}

	return tmp;
}

inline MatN MatN::operator * ( const MatN & rhs ) {
	MatN tmp( numDimensions );
	tmp.Zero();

	for ( int i = 0; i < numDimensions; i++ ) {
		for ( int j = 0; j < numDimensions; j++ ) {
			tmp.rows[ i ][ j ] += rows[ i ][ j ] * rhs.rows[ j ][ i ];
		}
	}

	return tmp;
}