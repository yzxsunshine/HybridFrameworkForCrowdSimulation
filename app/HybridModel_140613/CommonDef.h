#include <vcg/complex/complex.h>

#ifndef PI
	#define PI 3.14159265359
#endif

#define WALKABLE_FACTOR 0.5 // WARNING this macro can be updated

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

bool IsEqual(vcg::Point3f a, vcg::Point3f b);

float HelenArea(const float a, const float b, const float c);

float getDistance(const vcg::Point3f& p0, const vcg::Point3f& p1);

float ComputeTriArea(vcg::Point3f a, vcg::Point3f b, vcg::Point3f c);

int GetPointSide(vcg::Point2f ptA, vcg::Point2f ptB, vcg::Point2f wayPt);
int GetIntersection(vcg::Point2f startPt, vcg::Point2f endPt
				  , vcg::Point2f lineA, vcg::Point2f lineB, vcg::Point2f* intersectPt=NULL);

bool ClosestHeightPointTriangle(vcg::Point3f p, vcg::Point3f a, vcg::Point3f b, vcg::Point3f c, float& h);