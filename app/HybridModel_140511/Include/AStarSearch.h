#pragma once
#include "NavMesh.h"
#include "MidPointCtrl.h"

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

// Mid Point Tree Node
class AStarMPNode
{
public:
	AStarMPNode()
	{
		cost = 0;
		hcost = 0;
		m_childNodes[0] = NULL;
		m_childNodes[1] = NULL;
		m_childNodes[2] = NULL;
		m_childNodes[3] = NULL;
		m_parentNode = NULL;
	}
	
	~AStarMPNode()
	{
		
	}

	void AStarSearch(NavMeshFace& startFace, NavMeshFace& endFace
				   , vcg::Point3f startPt, vcg::Point3f endPt, std::vector<MidPoint>& m_midPointList
				   , std::vector<int>& path, std::vector<vcg::Point3f>& wayPoints);
	int AStarPathSmooth(NavMesh& navMesh, std::vector<int>& path, std::vector<vcg::Point3f>& wayPoints
					  , std::vector<vcg::Point3f>& smoothPath, std::vector<int>& smoothPathIds);
	//void print(AStarMPNode* Node, std::vector<MidPoint>& m_midPointList);
	int GetPointSide(vcg::Point2f ptA, vcg::Point2f ptB, vcg::Point2f wayPt);
	int GetIntersection(vcg::Point2f startPt, vcg::Point2f endPt
					  , vcg::Point2f lineA, vcg::Point2f lineB, vcg::Point2f* intersectPt=NULL);

	int AddVertexToPath(vcg::Point3f pt, int faceID, std::vector<int>& smoothPathIds, std::vector<vcg::Point3f>& smoothPathPts);
	int AddPortalToPath();
public:
	int m_mpID;//midPoint ID
	AStarMPNode* m_childNodes[4];
	AStarMPNode* m_parentNode;
	float cost;
	float hcost;
};

class OpenQueue
{
private:
	std::vector<AStarMPNode*> heap;
	std::vector<MidPoint>* mpList;
public:
	OpenQueue(std::vector<MidPoint>* mp, int initSize = 10);
	~OpenQueue();
	//open±íµÄ²Ù×÷
	AStarMPNode*& GetNodeAt(int id);
	AStarMPNode* PopQueue();
	void PushQueue(AStarMPNode* node);
	void UpdateQueue(AStarMPNode* node);
	bool IsQueueEmpty();
	int GetHeapSize() {return heap.size(); }
	void PreAllocateMemory(int extendSize);
};

class NodeHCostGreater
{
public:
	bool operator() (AStarMPNode* first, AStarMPNode* second)
	{
		return (first->hcost > second->hcost);
	}
};