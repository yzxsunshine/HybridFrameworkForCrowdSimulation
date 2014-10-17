#include "NavMeshQuery.h"

NavMeshQuery::NavMeshQuery(void){
}

NavMeshQuery::NavMeshQuery(NavMesh* nav) : m_nav(nav)
{
}


NavMeshQuery::~NavMeshQuery(void)
{
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

void NavMeshQuery::GetNearestFace(const float* pos, int& faceID, float* nearestPos)
{
	vcg::Point2f a, b, p;
	p.X() = pos[0];
	p.Y() = pos[2];
	faceID = -1;
	for(int i=0; i<m_nav->fn; i++)
	{
		int sideCount = 0;
		for(int j=0; j<3; j++)
		{
			int aid = j;
			int bid = (j+1)%3;
			a.X() = m_nav->face[i].V(aid)->P().X();
			a.Y() = m_nav->face[i].V(aid)->P().Z();
			b.X() = m_nav->face[i].V(bid)->P().X();
			b.Y() = m_nav->face[i].V(bid)->P().Z();
			if(node.GetPointSide(a, b, p) == RIGHT_SIDE)
			{
				sideCount++;
			}
		}
		if(sideCount==3)
		{
			faceID = i;
			float height;
			vcg::Point3f posVec(pos[0], pos[1], pos[2]);
			if(ClosestHeightPointTriangle(posVec, m_nav->face[i].V(0)->P(), m_nav->face[i].V(1)->P(), m_nav->face[i].V(2)->P(), height))
			{
				nearestPos[0] = pos[0];
				nearestPos[1] = height;
				nearestPos[2] = pos[2];
			}
			break;
		}
	}
}

void NavMeshQuery::FindPath(int startFaceID, int targetFaceID
						  , vcg::Point3f startPos, vcg::Point3f targetPos
						  , std::vector<int>& pathFaces
						  , std::vector<vcg::Point3f>& pathPts)
{
	std::vector<int> path;
	std::vector<vcg::Point3f> wayPoints;
	m_midPtCtrl->ResetMPList();
	node.AStarSearch(m_nav->face[startFaceID], m_nav->face[targetFaceID]
				   , startPos, targetPos
				   , m_midPtCtrl->m_midPointList, path, wayPoints);
	node.AStarPathSmooth(*m_nav, path, wayPoints, pathPts, pathFaces);	
}