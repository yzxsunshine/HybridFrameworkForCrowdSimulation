#ifndef _COMMOM_DEF
#define _COMMOM_DEF
#include <vcg/complex/complex.h>
#include "HybridAlgorithm\Vector2.h"

#ifndef PI
	#define PI 3.14159265359
#endif

#define WALKABLE_FACTOR 0.5 // WARNING this macro can be updated

#define MAX_SPEED 1.2f

#ifndef EPSILON
#define EPSILON 0.000001
#endif

#ifndef EDGE_EPS
#define EDGE_EPS 0.01
#endif

#ifndef LARGE_EPS
#define LARGE_EPS 0.01
#endif

enum POINT_POSITION
{
	LEFT_SIDE,
	ON_LINE,
	RIGHT_SIDE
};

enum INTERSECT_STATUS
{
	COLLINEAR,
	PARALELL,
	SEGMENTS_INTERSECT,
	A_BISECTS_B,
	B_BISECTS_A,
	LINES_INTERSECT
};

struct SimulationParams {
	float m_maxNeighborDist;
	float m_maxNumNeighbors;
	float m_planHorizon;
	float m_agentRadius;
	float m_preferSpeed;
	SimulationParams() {
		m_maxNeighborDist = 15.0f;
		m_maxNumNeighbors = 10.0f;
		m_planHorizon = 10.0f;
		m_agentRadius = 0.8f;
		m_preferSpeed = 1.4f;
	}
};

bool IsEqual(vcg::Point3f a, vcg::Point3f b);

float HelenArea(const float a, const float b, const float c);

float getDistance(const vcg::Point3f& p0, const vcg::Point3f& p1);

float ComputeTriArea(vcg::Point3f a, vcg::Point3f b, vcg::Point3f c);

int GetPointSide(vcg::Point2f ptA, vcg::Point2f ptB, vcg::Point2f wayPt);
int GetIntersection(vcg::Point2f startPt, vcg::Point2f endPt
				  , vcg::Point2f lineA, vcg::Point2f lineB, vcg::Point2f* intersectPt=NULL);

bool ClosestHeightPointTriangle(vcg::Point3f p, vcg::Point3f a, vcg::Point3f b, vcg::Point3f c, float& h);

// flb, function value on left bottom
// frb, function value on right bottom
// frt, function value on right top
// flt, function vlaue on left top
// leftBottom, position of left bottom corner
// rightTop, position of right top corner
// curPos, the target point to be interpolated
// return the value of interpolation
float bilateralInterpolation(float flb, float frb, float frt, float flt
							, Vector2 leftBottom, Vector2 rightTop, Vector2 curPos);

// vlb, average velocity on left bottom
// vrb, average velocity on right bottom
// vrt, average velocity on right top
// vlt, average velocity on left top
// leftBottom, position of left bottom corner
// rightTop, position of right top corner
// curPos, the target point to be interpolated
// return the value of interpolation
Vector2 bilateralInterpolation(Vector2 vlb, Vector2 vrb, Vector2 vrt, Vector2 vlt
							, Vector2 leftBottom, Vector2 rightTop, Vector2 curPos);


/// Clamps the value to the specified range.
///  @param[in]		v	The value to clamp.
///  @param[in]		mn	The minimum permitted return value.
///  @param[in]		mx	The maximum permitted return value.
///  @return The value, clamped to the specified range.
template<class T> inline T clamp(T v, T mn, T mx) { return v < mn ? mn : (v > mx ? mx : v); }

/// @}
/// @name Vector helper functions.
/// @{

/// Derives the cross product of two vectors. (@p v1 x @p v2)
///  @param[out]	dest	The cross product. [(x, y, z)]
///  @param[in]		v1		A Vector [(x, y, z)]
///  @param[in]		v2		A vector [(x, y, z)]
inline void vecCross(float* dest, const float* v1, const float* v2)
{
	dest[0] = v1[1] * v2[2] - v1[2] * v2[1];
	dest[1] = v1[2] * v2[0] - v1[0] * v2[2];
	dest[2] = v1[0] * v2[1] - v1[1] * v2[0];
}

/// Derives the dot product of two vectors. (@p v1 . @p v2)
///  @param[in]		v1	A Vector [(x, y, z)]
///  @param[in]		v2	A vector [(x, y, z)]
/// @return The dot product.
inline float vecDot(const float* v1, const float* v2)
{
	return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

/// Performs a linear interpolation between two vectors. (@p v1 toward @p v2)
///  @param[out]	dest	The result vector. [(x, y, x)]
///  @param[in]		v1		The starting vector.
///  @param[in]		v2		The destination vector.
///	 @param[in]		t		The interpolation factor. [Limits: 0 <= value <= 1.0]
inline void vecLerp(float* dest, const float* v1, const float* v2, const float t)
{
	dest[0] = v1[0] + (v2[0] - v1[0])*t;
	dest[1] = v1[1] + (v2[1] - v1[1])*t;
	dest[2] = v1[2] + (v2[2] - v1[2])*t;
}

/// Performs a vector addition. (@p v1 + @p v2)
///  @param[out]	dest	The result vector. [(x, y, z)]
///  @param[in]		v1		The base vector. [(x, y, z)]
///  @param[in]		v2		The vector to add to @p v1. [(x, y, z)]
inline void vecAdd(float* dest, const float* v1, const float* v2)
{
	dest[0] = v1[0] + v2[0];
	dest[1] = v1[1] + v2[1];
	dest[2] = v1[2] + v2[2];
}

/// Performs a vector subtraction. (@p v1 - @p v2)
///  @param[out]	dest	The result vector. [(x, y, z)]
///  @param[in]		v1		The base vector. [(x, y, z)]
///  @param[in]		v2		The vector to subtract from @p v1. [(x, y, z)]
inline void vecSub(float* dest, const float* v1, const float* v2)
{
	dest[0] = v1[0] - v2[0];
	dest[1] = v1[1] - v2[1];
	dest[2] = v1[2] - v2[2];
}

/// Scales the vector by the specified value. (@p v * @p t)
///  @param[out]	dest	The result vector. [(x, y, z)]
///  @param[in]		v		The vector to scale. [(x, y, z)]
///  @param[in]		t		The scaling factor.
inline void vecScale(float* dest, const float* v, const float t)
{
	dest[0] = v[0] * t;
	dest[1] = v[1] * t;
	dest[2] = v[2] * t;
}

/// Selects the minimum value of each element from the specified vectors.
///  @param[in,out]	mn	A vector.  (Will be updated with the result.) [(x, y, z)]
///  @param[in]	v	A vector. [(x, y, z)]
inline void vecMin(float* mn, const float* v)
{
	mn[0] = std::min(mn[0], v[0]);
	mn[1] = std::min(mn[1], v[1]);
	mn[2] = std::min(mn[2], v[2]);
}

/// Selects the maximum value of each element from the specified vectors.
///  @param[in,out]	mx	A vector.  (Will be updated with the result.) [(x, y, z)]
///  @param[in]		v	A vector. [(x, y, z)]
inline void vecMax(float* mx, const float* v)
{
	mx[0] = std::max(mx[0], v[0]);
	mx[1] = std::max(mx[1], v[1]);
	mx[2] = std::max(mx[2], v[2]);
}

/// Sets the vector elements to the specified values.
///  @param[out]	dest	The result vector. [(x, y, z)]
///  @param[in]		x		The x-value of the vector.
///  @param[in]		y		The y-value of the vector.
///  @param[in]		z		The z-value of the vector.
inline void vecSet(float* dest, const float x, const float y, const float z)
{
	dest[0] = x; dest[1] = y; dest[2] = z;
}

/// Performs a vector copy.
///  @param[out]	dest	The result. [(x, y, z)]
///  @param[in]		a		The vector to copy. [(x, y, z)]
inline void vecCopy(float* dest, const float* a)
{
	dest[0] = a[0];
	dest[1] = a[1];
	dest[2] = a[2];
}

/// Derives the scalar length of the vector.
///  @param[in]		v The vector. [(x, y, z)]
/// @return The scalar length of the vector.
inline float vecLen(const float* v)
{
	return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

/// Derives the square of the scalar length of the vector. (len * len)
///  @param[in]		v The vector. [(x, y, z)]
/// @return The square of the scalar length of the vector.
inline float vecLenSqr(const float* v)
{
	return v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
}

/// Returns the distance between two points.
///  @param[in]		v1	A point. [(x, y, z)]
///  @param[in]		v2	A point. [(x, y, z)]
/// @return The distance between the two points.
inline float vecDist(const float* v1, const float* v2)
{
	const float dx = v2[0] - v1[0];
	const float dy = v2[1] - v1[1];
	const float dz = v2[2] - v1[2];
	return std::sqrt(dx*dx + dy*dy + dz*dz);
}

/// Returns the square of the distance between two points.
///  @param[in]		v1	A point. [(x, y, z)]
///  @param[in]		v2	A point. [(x, y, z)]
/// @return The square of the distance between the two points.
inline float vecDistSqr(const float* v1, const float* v2)
{
	const float dx = v2[0] - v1[0];
	const float dy = v2[1] - v1[1];
	const float dz = v2[2] - v1[2];
	return dx*dx + dy*dy + dz*dz;
}

/// Derives the distance between the specified points on the xz-plane.
///  @param[in]		v1	A point. [(x, y, z)]
///  @param[in]		v2	A point. [(x, y, z)]
/// @return The distance between the point on the xz-plane.
///
/// The vectors are projected onto the xz-plane, so the y-values are ignored.
inline float vecDist2D(const float* v1, const float* v2)
{
	const float dx = v2[0] - v1[0];
	const float dz = v2[2] - v1[2];
	return std::sqrt(dx*dx + dz*dz);
}

/// Derives the square of the distance between the specified points on the xz-plane.
///  @param[in]		v1	A point. [(x, y, z)]
///  @param[in]		v2	A point. [(x, y, z)]
/// @return The square of the distance between the point on the xz-plane.
inline float vecDist2DSqr(const float* v1, const float* v2)
{
	const float dx = v2[0] - v1[0];
	const float dz = v2[2] - v1[2];
	return dx*dx + dz*dz;
}

/// Normalizes the vector.
///  @param[in,out]	v	The vector to normalize. [(x, y, z)]
inline void vecNormalize(float* v)
{
	float d = 1.0f / sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
	v[0] *= d;
	v[1] *= d;
	v[2] *= d;
}

/// Performs a 'sloppy' colocation check of the specified points.
///  @param[in]		p0	A point. [(x, y, z)]
///  @param[in]		p1	A point. [(x, y, z)]
/// @return True if the points are considered to be at the same location.
///
/// Basically, this function will return true if the specified points are 
/// close enough to eachother to be considered colocated.
inline bool vecEqual(const float* p0, const float* p1)
{
	static const float thr = (1.0f / 16384.0f) * (1.0f / 16384.0f);
	const float d = vecDistSqr(p0, p1);
	return d < thr;
}

/// Derives the dot product of two vectors on the xz-plane. (@p u . @p v)
///  @param[in]		u		A vector [(x, y, z)]
///  @param[in]		v		A vector [(x, y, z)]
/// @return The dot product on the xz-plane.
///
/// The vectors are projected onto the xz-plane, so the y-values are ignored.
inline float cecDot2D(const float* u, const float* v)
{
	return u[0] * v[0] + u[2] * v[2];
}

/// Derives the xz-plane 2D perp product of the two vectors. (uz*vx - ux*vz)
///  @param[in]		u		The LHV vector [(x, y, z)]
///  @param[in]		v		The RHV vector [(x, y, z)]
/// @return The dot product on the xz-plane.
///
/// The vectors are projected onto the xz-plane, so the y-values are ignored.
inline float vecPerp2D(const float* u, const float* v)
{
	return u[2] * v[0] - u[0] * v[2];
}

/// @}
/// @name Computational geometry helper functions.
/// @{

/// Derives the signed xz-plane area of the triangle ABC, or the relationship of line AB to point C.
///  @param[in]		a		Vertex A. [(x, y, z)]
///  @param[in]		b		Vertex B. [(x, y, z)]
///  @param[in]		c		Vertex C. [(x, y, z)]
/// @return The signed xz-plane area of the triangle.
inline float triArea2D(const float* a, const float* b, const float* c)
{
	const float abx = b[0] - a[0];
	const float abz = b[2] - a[2];
	const float acx = c[0] - a[0];
	const float acz = c[2] - a[2];
	return acx*abz - abx*acz;
}

/// Performs a scaled vector addition. (@p v1 + (@p v2 * @p s))
///  @param[out]	dest	The result vector. [(x, y, z)]
///  @param[in]		v1		The base vector. [(x, y, z)]
///  @param[in]		v2		The vector to scale and add to @p v1. [(x, y, z)]
///  @param[in]		s		The amount to scale @p v2 by before adding to @p v1.
inline void vecMad(float* dest, const float* v1, const float* v2, const float s)
{
	dest[0] = v1[0] + v2[0] * s;
	dest[1] = v1[1] + v2[1] * s;
	dest[2] = v1[2] + v2[2] * s;
}

#else
#endif