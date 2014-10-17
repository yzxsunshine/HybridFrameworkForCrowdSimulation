#include "NavMeshQuery.h"
#include "CommonDef.h"

NavMeshQuery::NavMeshQuery(void){
}

NavMeshQuery::NavMeshQuery(NavMesh* nav) : m_nav(nav)
{
}


NavMeshQuery::~NavMeshQuery(void)
{
}

bool NavMeshQuery::GetNearestFace(const float* pos, int& faceID, float* nearestPos)
{
	vcg::Point2f a, b, p;
	p.X() = pos[0];
	p.Y() = pos[2];
	faceID = -1;
	bool foundFace = false;
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
			if(GetPointSide(a, b, p) == RIGHT_SIDE)
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
				foundFace = true;
			}	
			break;
		}
	}
	return foundFace;
}

bool NavMeshQuery::GetNearestFace(vcg::Point3f& pos, int& tileID, int& gridID, int& faceID, vcg::Point3f& nearestPos)
{
	Tile* tile = m_tileCtrl->GetTile(pos);
	tileID = tile->m_tileID;
	faceID = -1;
	return tile->GetNearestFace(pos, gridID, faceID, m_nav, m_tileCtrl->m_grids, nearestPos);
}

bool NavMeshQuery::GetNearestFace(const float* pos, int& tileID, int& gridID, int& faceID, float* nearestPos)
{
	vcg::Point3f p(pos[0], pos[1], pos[2]);
	vcg::Point3f n;
	bool res = GetNearestFace(p, tileID, gridID, faceID, n);
	nearestPos[0] = n.X();	nearestPos[1] = n.Y();	nearestPos[2] = n.Z();
	return res;
}

void NavMeshQuery::FindPath(vcg::Point3f startPos, vcg::Point3f targetPos
				, std::vector<int>& pathFaces
				, std::vector<vcg::Point3f>& pathPts)
{
	int startTile, startGrid, startFace, endTile, endGrid, endFace;
	vcg::Point3f startPt, endPt;
	GetNearestFace(startPos, startTile, startGrid, startFace, startPt);
	GetNearestFace(targetPos, endTile, endGrid, endFace, endPt);
	std::vector<int> path;
	std::vector<vcg::Point3f> wayPts;
	m_tileCtrl->ResetGridList();
	m_nodeGrid.AStarSearch(startGrid, endGrid, startPt, endPt, m_tileCtrl->m_grids, path, wayPts);
	
	// smooth grid path
	std::vector<int> smoothGridIds;
	std::vector<vcg::Point3f> smoothGridPts;
	m_nodeGrid.AStarPathSmooth(m_tileCtrl->m_grids, m_tileCtrl, m_nav, path, wayPts, smoothGridPts, smoothGridIds);
	// generate faces
	//pathFaces.push_back(startFace);
	//pathPts.push_back(startPt);
	for(int i=1; i<smoothGridIds.size() - 1; i++)
	{
		int gid = smoothGridIds[i];
		if(m_tileCtrl->m_grids[gid].m_obstacleStatus == NO_OBSTACLE)
		{	// directly push the center of the grid inside
			pathFaces.push_back(m_tileCtrl->m_grids[gid].centerFace);
			pathPts.push_back(m_tileCtrl->m_grids[gid].center);
		}
		else if(m_tileCtrl->m_grids[gid].m_obstacleStatus == HAS_OBSTACLE)
		{	// do detailed path finding, first find entry and exit point
			startPt = smoothGridPts[i-1];
			endPt = smoothGridPts[i+1];
			vcg::Point3f entryPt, exitPt;
			int entryFace, exitFace;
			m_tileCtrl->m_grids[gid].GetEdgePoint(startPt, endPt, entryPt, exitPt, entryFace, exitFace, m_nav, m_tileCtrl->m_grids);
			pathFaces.push_back(entryFace);
			pathPts.push_back(entryPt);
			pathFaces.push_back(exitFace);
			pathPts.push_back(exitPt);
		}
		else
		{
			printf("[NavMeshQuery - FindPath] A Grid full of obstacles shouldn't be here!!\n");
		}
	}
	pathFaces.push_back(endFace);
	pathPts.push_back(endPt);
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
	if(wayPoints.size() > 2)
	{
		node.AStarPathSmooth(*m_nav, path, wayPoints, pathPts, pathFaces);	
	}
}