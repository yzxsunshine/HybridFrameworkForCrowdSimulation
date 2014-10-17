#pragma once
#include "NavMesh.h"
#include "AStarSearch.h"
#include "MidPointCtrl.h"
#include "TileCtrl.h"

class NavMeshQuery
{
public:
	NavMeshQuery(void);
	NavMeshQuery(NavMesh* nav);
	~NavMeshQuery(void);
	void SetNavMesh(NavMesh* nav) { m_nav = nav; }
	NavMesh* GetNavMesh(void)	{ return m_nav; }
	void SetMidPointCtrl(MidPointCtrl* mpc) { m_midPtCtrl = mpc; }
	MidPointCtrl* GetMidPointCtrl(void) { return m_midPtCtrl; }
	void SetTileCtrl(TileCtrl* tc) { m_tileCtrl = tc; }
	TileCtrl* GetTileCtrl(void) { return m_tileCtrl; }
	void FindPath(int startFaceID, int targetFaceID
				, vcg::Point3f startPos, vcg::Point3f targetPos
				, std::vector<int>& pathFaces
				, std::vector<vcg::Point3f>& pathPts);
	void FindPath(vcg::Point3f startPos, vcg::Point3f targetPos
				, std::vector<int>& pathFaces
				, std::vector<vcg::Point3f>& pathPts);
	bool GetNearestFace(const float* pos, int& faceID, float* nearestPos);
	bool GetNearestFace(vcg::Point3f& pos, int& tileID, int& gridID, int& faceID, vcg::Point3f& nearestPos);
	bool GetNearestFace(const float* pos, int& tileID, int& gridID, int& faceID, float* nearestPos);
protected:
	NavMesh* m_nav;
	AStarMPNode node;
	AStarGridNode m_nodeGrid;
	MidPointCtrl* m_midPtCtrl;
	TileCtrl* m_tileCtrl;
};

