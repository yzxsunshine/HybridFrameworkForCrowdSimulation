#pragma once
#include "NavMesh.h"
#include "AStarSearch.h"
#include "MidPointCtrl.h"

class NavMeshQuery
{
public:
	NavMeshQuery(void);
	NavMeshQuery(NavMesh* nav);
	~NavMeshQuery(void);
	void SetNavMesh(NavMesh* nav) { m_nav = nav; }
	void SetMidPointCtrl(MidPointCtrl* mpc) { m_midPtCtrl = mpc; }
	void FindPath(int startFaceID, int targetFaceID
				, vcg::Point3f startPos, vcg::Point3f targetPos
				, std::vector<int>& pathFaces
				, std::vector<vcg::Point3f>& pathPts);
	void GetNearestFace(const float* pos, int& faceID, float* nearestPos);
protected:
	NavMesh* m_nav;
	AStarMPNode node;
	MidPointCtrl* m_midPtCtrl;
};

