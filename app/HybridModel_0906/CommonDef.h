#ifndef _COMMOM_DEF
#define _COMMOM_DEF
#include <vcg/complex/complex.h>
#include "HybridAlgorithm\Vector2.h"

#ifndef PI
	#define PI 3.14159265359
#endif

#define WALKABLE_FACTOR 0.5 // WARNING this macro can be updated

#define MAX_SPEED 2.0f

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
		m_maxNeighborDist = 15;
		m_maxNumNeighbors = 10;
		m_planHorizon = 10;
		m_agentRadius = 0.8;
		m_preferSpeed = 1.4;
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

#else
#endif