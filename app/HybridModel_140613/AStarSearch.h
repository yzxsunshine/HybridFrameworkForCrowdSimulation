#pragma once
#include "NavMesh.h"
#include "MidPointCtrl.h"
#include "TileCtrl.h"

#define DIVIDE_NUM 5

int AddVertexToPath(vcg::Point3f pt, int faceID, std::vector<int>& smoothPathIds, std::vector<vcg::Point3f>& smoothPathPts);

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
public:
	int m_mpID;//midPoint ID
	AStarMPNode* m_childNodes[4];
	AStarMPNode* m_parentNode;
	float cost;
	float hcost;
};

class OpenMPQueue
{
private:
	std::vector<AStarMPNode*> heap;
	std::vector<MidPoint>* mpList;
public:
	OpenMPQueue(std::vector<MidPoint>* mp, int initSize = 10);
	~OpenMPQueue();
	//open表的操作
	AStarMPNode*& GetNodeAt(int id);
	AStarMPNode* PopQueue();
	void PushQueue(AStarMPNode* node);
	void UpdateQueue(AStarMPNode* node);
	bool IsQueueEmpty();
	int GetHeapSize() {return heap.size(); }
	void PreAllocateMemory(int extendSize);
};

class MPNodeHCostGreater
{
public:
	bool operator() (AStarMPNode* first, AStarMPNode* second)
	{
		return (first->hcost > second->hcost);
	}
};


// for grids
class AStarGridNode
{
public:
	AStarGridNode()
	{
		cost = 0;
		hcost = 0;
		m_childNodes[0] = NULL;
		m_childNodes[1] = NULL;
		m_childNodes[2] = NULL;
		m_childNodes[3] = NULL;
		m_parentNode = NULL;
	}
	
	~AStarGridNode()
	{
		
	}

	void AStarSearch(int startGrid, int endGrid
				   , vcg::Point3f startPt, vcg::Point3f endPt, std::vector<Grid>& gridList
				   , std::vector<int>& path, std::vector<vcg::Point3f>& wayPoints);
	int AStarPathSmooth(std::vector<Grid>& grids, TileCtrl* tileCtrl, NavMesh* nav
					  , std::vector<int>& path, std::vector<vcg::Point3f>& wayPoints
					  , std::vector<vcg::Point3f>& smoothPath, std::vector<int>& smoothPathIds);
	//void print(AStarMPNode* Node, std::vector<MidPoint>& m_midPointList);
public:
	int m_gridID;//midPoint ID
	AStarGridNode* m_childNodes[4];
	AStarGridNode* m_parentNode;
	float cost;
	float hcost;
};

class OpenGridQueue
{
private:
	std::vector<AStarGridNode*> heap;
	std::vector<Grid>* gridList;
public:
	OpenGridQueue(std::vector<Grid>* grids, int initSize = 10);
	~OpenGridQueue();
	//open表的操作
	AStarGridNode*& GetNodeAt(int id);
	AStarGridNode* PopQueue();
	void PushQueue(AStarGridNode* node);
	void UpdateQueue(AStarGridNode* node);
	bool IsQueueEmpty();
	int GetHeapSize() { return heap.size(); }
	void PreAllocateMemory(int extendSize);
};

class GridNodeHCostGreater
{
public:
	bool operator() (AStarGridNode* first, AStarGridNode* second)
	{
		return (first->hcost > second->hcost);
	}
};