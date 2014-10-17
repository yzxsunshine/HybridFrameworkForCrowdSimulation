#include <vcg/complex/complex.h>

#ifndef PI
	#define PI 3.14159265359
#endif

#define WALKABLE_FACTOR 0.5 // WARNING this macro can be updated

#define EPSILON 0.000001

inline bool IsEqual(vcg::Point3f a, vcg::Point3f b)
{
	vcg::Point3f c = a - b;
	return c.dot(c) < EPSILON;
}

inline float TriArea2D(vcg::Point3f a, vcg::Point3f b, vcg::Point3f c)
{
	const float abx = b.X() - a.X();
	const float abz = b.Z() - a.Z();
	const float acx = c.X() - a.X();
	const float acz = c.Z() - a.Z();
	return acx*abz - abx*acz;
}