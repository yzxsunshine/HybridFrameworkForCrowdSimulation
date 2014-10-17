#include "CommonDef.h"

bool IsEqual(vcg::Point3f a, vcg::Point3f b)
{
	vcg::Point3f c = a - b;
	return c.dot(c) < EPSILON;
}

float HelenArea(const float a, const float b, const float c)
{
	float p = (a+b+c)/2;
	float S = sqrtf(p*(p-a)*(p-b)*(p-c));
	return S;
}

float getDistance(const vcg::Point3f& p0, const vcg::Point3f& p1)
{
	return sqrtf((p0-p1).dot(p0-p1));
}


float ComputeTriArea(vcg::Point3f a, vcg::Point3f b, vcg::Point3f c)
{
	float edgeA = getDistance(a, b);
	float edgeB = getDistance(b, c);
	float edgeC = getDistance(c, a);
	return HelenArea(edgeA, edgeB, edgeC);
}

int GetPointSide(vcg::Point2f ptA, vcg::Point2f ptB, vcg::Point2f wayPt)
{
	vcg::Point2f normal = ptB - ptA;
	normal.Normalize();
	float oldVal = normal.Y();
	normal.Y() = -normal.X();
	normal.X() = oldVal;

	vcg::Point2f testVec = wayPt - ptA;
	float dotVal = testVec.dot(normal);
	if(dotVal > EDGE_EPS)	// right
	{
		return RIGHT_SIDE;		
	}
	else if(dotVal < -EDGE_EPS)	// left
	{
		return LEFT_SIDE;			
	}
	else
	{
		return ON_LINE;
	}
}

int GetIntersection(vcg::Point2f startPt, vcg::Point2f endPt
							   , vcg::Point2f lineA, vcg::Point2f lineB, vcg::Point2f* intersectPt)
{
	float Ay_minus_Cy = startPt.Y() - lineA.Y();	
	float Dx_minus_Cx = lineB.X() - lineA.X();	
	float Ax_minus_Cx = startPt.X() - lineA.X();	
	float Dy_minus_Cy = lineB.Y() - lineA.Y();	
	float Bx_minus_Ax = endPt.X() - startPt.X();	
	float By_minus_Ay = endPt.Y() - startPt.Y();	
	float Numerator = (Ay_minus_Cy * Dx_minus_Cx) - (Ax_minus_Cx * Dy_minus_Cy);
	float Denominator = (Bx_minus_Ax * Dy_minus_Cy) - (By_minus_Ay * Dx_minus_Cx);
	// if lines do not intersect, return now
	if (!Denominator)
	{
		if (!Numerator)
		{
			return COLLINEAR;
		}
		return PARALELL;
	}
	float FactorAB = Numerator / Denominator;
	float FactorCD = ((Ay_minus_Cy * Bx_minus_Ax) - (Ax_minus_Cx * By_minus_Ay)) / Denominator;
	// if an interection point was provided, fill it in now
	if (intersectPt)
	{
		intersectPt->X() = (startPt.X() + (FactorAB * Bx_minus_Ax));
		intersectPt->Y() = (startPt.Y() + (FactorAB * By_minus_Ay));
	}


	// now determine the type of intersection
	if ((FactorAB >= 0.0f) && (FactorAB <= 1.0f) && (FactorCD >= 0.0f) && (FactorCD <= 1.0f))
	{
		return SEGMENTS_INTERSECT;
	}
	else if ((FactorCD >= 0.0f) && (FactorCD <= 1.0f))
	{
		return (A_BISECTS_B);
	}
	else if ((FactorAB >= 0.0f) && (FactorAB <= 1.0f))
	{
		return (B_BISECTS_A);
	}
	return LINES_INTERSECT;
}

bool ClosestHeightPointTriangle(vcg::Point3f p, vcg::Point3f a, vcg::Point3f b, vcg::Point3f c, float& h)
{
	vcg::Point3f v0, v1, v2;
	v0 = c-a;
	v1 = b-a;
	v2 = p-a;

	const float dot00 = v0.dot(v0);
	const float dot01 = v0.dot(v1);
	const float dot02 = v0.dot(v2);
	const float dot11 = v1.dot(v1);
	const float dot12 = v1.dot(v2);
	
	// Compute barycentric coordinates
	const float invDenom = 1.0f / (dot00 * dot11 - dot01 * dot01);
	const float u = (dot11 * dot02 - dot01 * dot12) * invDenom;
	const float v = (dot00 * dot12 - dot01 * dot02) * invDenom;

	// The (sloppy) epsilon is needed to allow to get height of points which
	// are interpolated along the edges of the triangles.
	static const float EPS = 1e-4f;
	
	// If point lies inside the triangle, return interpolated ycoord.
	if (u >= -EPS && v >= -EPS && (u+v) <= 1+EPS)
	{
		h = a[1] + v0[1]*u + v1[1]*v;
		return true;
	}
	
	return false;
}
