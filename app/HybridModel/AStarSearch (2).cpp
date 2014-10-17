#include "AStarSearch.h"
#include "CommonDef.h"


int AddVertexToPath(vcg::Point3f pt, int faceID, std::vector<int>& smoothPathIds, std::vector<vcg::Point3f>& smoothPathPts)
{
	int smoothPathSize = smoothPathIds.size();
	if(smoothPathSize > 0 && IsEqual(smoothPathPts[smoothPathSize-1], pt) )
	{	// point already exist, just update current position
		smoothPathIds[smoothPathSize-1] = faceID;
	}
	else
	{
		smoothPathIds.push_back(faceID);
		smoothPathPts.push_back(pt);
	}
	return 0;
}

// for mid points
OpenMPQueue::OpenMPQueue(std::vector<MidPoint>* mp, int initSize)
{
	heap.reserve(initSize);
	mpList = mp;
}

OpenMPQueue::~OpenMPQueue()
{
	for(int i=0; i<heap.size(); i++)
	{
		delete heap[i];
		heap[i] = NULL;
	}
	heap.clear();
}

void OpenMPQueue::PreAllocateMemory(int extendSize)
{
	int prevSize = heap.capacity();
	heap.reserve(prevSize + extendSize);
}

bool OpenMPQueue::IsQueueEmpty()
{
	return (heap.empty());
}

AStarMPNode* OpenMPQueue::PopQueue()
{
	AStarMPNode* node = heap.front();
	std::pop_heap(heap.begin(), heap.end(), MPNodeHCostGreater());
	heap.pop_back();
	return node;
}

void OpenMPQueue::PushQueue(AStarMPNode* node)
{
	heap.push_back(node);
	std::push_heap(heap.begin(), heap.end(), MPNodeHCostGreater());
	
	for(int i=0; i<heap.size(); i++)
	{
		(*mpList)[heap[i]->m_mpID].openID = i;
	}
}

void OpenMPQueue::UpdateQueue(AStarMPNode* node)
{
	std::vector<AStarMPNode*>::iterator iter;
	for(iter = heap.begin(); iter!=heap.end(); iter++)
	{
		if((*iter)->m_mpID == node->m_mpID)
		{
			std::push_heap(heap.begin(), iter+1, MPNodeHCostGreater());	
			return;
		}
	}
}

AStarMPNode*& OpenMPQueue::GetNodeAt(int id)
{
	return heap[id];
}

void AStarMPNode::AStarSearch(NavMeshFace& startFace, NavMeshFace& endFace
			   , vcg::Point3f startPt, vcg::Point3f endPt, std::vector<MidPoint>& m_midPointList
			   , std::vector<int>& path, std::vector<vcg::Point3f>& wayPoints)
{
	//std::vector<int> path;	// store faces lie on the path
	//std::vector<vcg::Point3f> wayPoints;	// store middle points lie on the path
	if(startFace.CCID != endFace.CCID)
	{
		return;
	}
	int N = m_midPointList.size();
	OpenMPQueue open(&m_midPointList);
	for(int i = 0; i < 3; i++)
	{
		AStarMPNode* node = new AStarMPNode();
		node->m_mpID = startFace.midPointIds[i];
		if(node->m_mpID < 0)
		{
			delete node;
			continue;
		}
		node->cost = (startPt - m_midPointList[node->m_mpID].position).Norm();
		m_midPointList[node->m_mpID].openID = i;
		open.PushQueue(node);
	}
	int flag = 2;
	while(!open.IsQueueEmpty())
	{
		AStarMPNode* curNode = open.PopQueue();
		if(m_midPointList[curNode->m_mpID].faceIds[0] == endFace.faceID
		|| m_midPointList[curNode->m_mpID].faceIds[1] == endFace.faceID)
		{
			wayPoints.push_back(endPt);
			path.push_back(endFace.faceID);
			AStarMPNode* L = curNode->m_parentNode;
			while(L != NULL && L->m_parentNode != NULL)	// remove redundant point
			{
				wayPoints.push_back(m_midPointList[L->m_mpID].position);
				//printf("point id: %d\n", L->m_mpID);
				if(L->m_parentNode != NULL)
				{
					for(int i=0; i<2; i++)	
						for(int j=0; j<2; j++)
						{
							if(m_midPointList[L->m_mpID].faceIds[i] == m_midPointList[L->m_parentNode->m_mpID].faceIds[j])
							{
								path.push_back(m_midPointList[L->m_mpID].faceIds[i]);
							}
						}
				}
				L = L->m_parentNode;
			}
			path.push_back(startFace.faceID);
			wayPoints.push_back(startPt);
			std::reverse(path.begin(), path.end());
			std::reverse(wayPoints.begin(), wayPoints.end());
			break;
		}
		// WARNING we need to find the bug here, first list all the points on the page
		// mid points: 145 141 117 116 92 91 67 79 53 49 26 27
		// tomorrow, find faces of those mid points and find which one is not continuous.
		int neighborNum = 4;
		if(m_midPointList[curNode->m_mpID].isBorder)
		{
			neighborNum = 2;
		}
		for(int j = 0; j < neighborNum; j++)
		{
			int nid = m_midPointList[curNode->m_mpID].neighbor[j];
			if(nid < 0)
				continue;
			if(m_midPointList[nid].isClose)
			{
				continue;
			}
			float gCost = curNode->cost + m_midPointList[curNode->m_mpID].cost[j];
			float hCost = gCost + (endPt - m_midPointList[nid].position).Norm();
			int openID = -1;
			for(int k=0; k<open.GetHeapSize(); k++)
			{
				if(open.GetNodeAt(k)->m_mpID == nid)
				{
					openID = k;
					break;
				}
			}
			//if(m_midPointList[nid].openID >= 0)
			if(openID >= 0)
			{
				//if(m_midPointList[nid].openID < open.GetHeapSize() && open.GetNodeAt(m_midPointList[nid].openID)->hcost > hCost)
				if(open.GetNodeAt(openID)->hcost > hCost)
				{
					//open.GetNodeAt(m_midPointList[nid].openID)->m_parentNode = curNode;
					//open.GetNodeAt(m_midPointList[nid].openID)->cost = gCost;
					//open.GetNodeAt(m_midPointList[nid].openID)->hcost = hCost;
					//open.UpdateQueue(open.GetNodeAt(m_midPointList[nid].openID));
					open.GetNodeAt(openID)->m_parentNode = curNode;
					open.GetNodeAt(openID)->cost = gCost;
					open.GetNodeAt(openID)->hcost = hCost;
					open.UpdateQueue(open.GetNodeAt(openID));
				}
				continue;
			}
			curNode->m_childNodes[j] = new AStarMPNode();
			curNode->m_childNodes[j]->m_parentNode = curNode;
			curNode->m_childNodes[j]->m_mpID = nid;
			curNode->m_childNodes[j]->cost = gCost;
			curNode->m_childNodes[j]->hcost = hCost;
			open.PushQueue(curNode->m_childNodes[j]);
		}
		m_midPointList[curNode->m_mpID].isClose = true;
		
		for(int i=0; i<flag; i++)
		{
			m_midPointList[open.GetNodeAt(i)->m_mpID].openID = i;
		}
	}
}


int AStarMPNode::AStarPathSmooth(NavMesh& navMesh, std::vector<int>& path, std::vector<vcg::Point3f>& wayPoints
							   , std::vector<vcg::Point3f>& smoothPath, std::vector<int>& smoothPathIds)
{
	int wayPtNum = wayPoints.size();
	int pathSize = path.size();
	smoothPathIds.reserve(wayPtNum);
	smoothPath.reserve(wayPtNum);
//	vcg::Point3f startPt = wayPoints[0];
//	vcg::Point3f endPt = wayPoints[wayPtNum-1];
	bool foundEndPt = false;
	int startId = 0;
	int res = AddVertexToPath(wayPoints[0], path[0], smoothPathIds, smoothPath);

	for(int i=startId + 1; i<wayPtNum; i++)	// traverse middle points to extend line of sight
	{
		vcg::Point2f startPt, endPt;
		startPt.X() = wayPoints[startId].X();
		startPt.Y() = wayPoints[startId].Z();
		endPt.X() = wayPoints[i].X();
		endPt.Y() = wayPoints[i].Z();
		for(int j=startId + 1; j<i; j++)	// traverse faces lie on current line of sight
		{
			int leftFace = path[j-1];
			int rightFace = path[j];
			bool foundEdge = false;
			int edgeID = 0;
			for(int k=0; k<3&&!foundEdge; k++)
			{
				for(int l=0; l<3&&!foundEdge; l++)
				{
					if(navMesh.face[leftFace].edgeIDs[k] == navMesh.face[rightFace].edgeIDs[l])
					{
						edgeID = navMesh.face[leftFace].edgeIDs[k];
						foundEdge = true;
						break;
					}
				}
			}
			if(!foundEdge)
			{
				printf("[AStarPathSmooth] Not avaliable path.\n");
				return -1;	// not avaliable path
			}
			vcg::Point2f ptA, ptB;
			int ptAId = navMesh.edges[edgeID].vertIDs[0];
			int ptBId = navMesh.edges[edgeID].vertIDs[1];
			ptA.X() = navMesh.vert[ptAId].P().X();
			ptA.Y() = navMesh.vert[ptAId].P().Z();
			ptB.X() = navMesh.vert[ptBId].P().X();
			ptB.Y() = navMesh.vert[ptBId].P().Z();
			vcg::Point2f intersectPt;
			int status = GetIntersection(startPt, endPt, ptA, ptB, &intersectPt);
			if(status == SEGMENTS_INTERSECT)
			{
				continue;	// current way point have inter section with the face, continue to next face
			}
			else
			{
				startId = i;
				res = AddVertexToPath(wayPoints[i - 1], path[i - 1], smoothPathIds, smoothPath);
				break;
			}
		}
	}

	res = AddVertexToPath(wayPoints[wayPtNum - 1], path[pathSize - 1], smoothPathIds, smoothPath);

	return 0;
}

// Grid based A Star

OpenGridQueue::OpenGridQueue(std::vector<Grid>* grid, int initSize)
{
	heap.reserve(initSize);
	gridList = grid;
}

OpenGridQueue::~OpenGridQueue()
{
	for(int i=0; i<heap.size(); i++)
	{
		delete heap[i];
		heap[i] = NULL;
	}
	heap.clear();
}

void OpenGridQueue::PreAllocateMemory(int extendSize)
{
	int prevSize = heap.capacity();
	heap.reserve(prevSize + extendSize);
}

bool OpenGridQueue::IsQueueEmpty()
{
	return (heap.empty());
}

AStarGridNode* OpenGridQueue::PopQueue()
{
	AStarGridNode* node = heap.front();
	std::pop_heap(heap.begin(), heap.end(), GridNodeHCostGreater());
	heap.pop_back();
	return node;
}

void OpenGridQueue::PushQueue(AStarGridNode* node)
{
	heap.push_back(node);
	std::push_heap(heap.begin(), heap.end(), GridNodeHCostGreater());
	
	for(int i=0; i<heap.size(); i++)
	{
		(*gridList)[heap[i]->m_gridID].openID = i;
	}
}

void OpenGridQueue::UpdateQueue(AStarGridNode* node)
{
	std::vector<AStarGridNode*>::iterator iter;
	for(iter = heap.begin(); iter!=heap.end(); iter++)
	{
		if((*iter)->m_gridID == node->m_gridID)
		{
			std::push_heap(heap.begin(), iter+1, GridNodeHCostGreater());	
			return;
		}
	}
}

AStarGridNode*& OpenGridQueue::GetNodeAt(int id)
{
	return heap[id];
}

void AStarGridNode::AStarSearch(int startGrid, int endGrid
							  , vcg::Point3f startPt, vcg::Point3f endPt, std::vector<Grid>& gridList
							  , std::vector<int>& path, std::vector<vcg::Point3f>& wayPoints)
{
	int N = gridList.size();
	OpenGridQueue open(&gridList);
	AStarGridNode* node = new AStarGridNode();
	node->m_gridID = startGrid;
	node->cost = 0.0f;
	gridList[startGrid].openID = 0;
	open.PushQueue(node);

	int flag = 2;
	while(!open.IsQueueEmpty())
	{
		AStarGridNode* curNode = open.PopQueue();
		if(curNode->m_gridID == endGrid)
		{
			wayPoints.push_back(endPt);
			path.push_back(endGrid);
			AStarGridNode* L = curNode->m_parentNode;	// remove redundant center
			while(L != NULL && L->m_parentNode != NULL)	// remove redundant center
			{
				wayPoints.push_back(gridList[L->m_gridID].center);
				path.push_back(L->m_gridID);
				//printf("point id: %d\n", L->m_gridID);
				L = L->m_parentNode;
			}
			path.push_back(startGrid);
			wayPoints.push_back(startPt);
			std::reverse(path.begin(), path.end());
			std::reverse(wayPoints.begin(), wayPoints.end());
			break;
		}
		// WARNING we need to find the bug here, first list all the points on the page
		// mid points: 145 141 117 116 92 91 67 79 53 49 26 27
		// tomorrow, find faces of those mid points and find which one is not continuous.
		int neighborNum = 4;
		for(int j = 0; j < neighborNum; j++)
		{
			int nid = gridList[curNode->m_gridID].m_neighbor[j];
			if(nid < 0)
				continue;
			if(gridList[nid].isClose)
			{
				continue;
			}
			float gCost = curNode->cost + (gridList[curNode->m_gridID].center - gridList[nid].center).Norm();
			float hCost = gCost + (endPt - gridList[nid].center).Norm();
			int openID = -1;
			for(int k=0; k<open.GetHeapSize(); k++)
			{
				if(open.GetNodeAt(k)->m_gridID == nid)
				{
					openID = k;
					break;
				}
			}
			//if(m_midPointList[nid].openID >= 0)
			if(openID >= 0)
			{
				//if(m_midPointList[nid].openID < open.GetHeapSize() && open.GetNodeAt(m_midPointList[nid].openID)->hcost > hCost)
				if(open.GetNodeAt(openID)->hcost > hCost)
				{
					//open.GetNodeAt(m_midPointList[nid].openID)->m_parentNode = curNode;
					//open.GetNodeAt(m_midPointList[nid].openID)->cost = gCost;
					//open.GetNodeAt(m_midPointList[nid].openID)->hcost = hCost;
					//open.UpdateQueue(open.GetNodeAt(m_midPointList[nid].openID));
					open.GetNodeAt(openID)->m_parentNode = curNode;
					open.GetNodeAt(openID)->cost = gCost;
					open.GetNodeAt(openID)->hcost = hCost;
					open.UpdateQueue(open.GetNodeAt(openID));
				}
				continue;
			}
			curNode->m_childNodes[j] = new AStarGridNode();
			curNode->m_childNodes[j]->m_parentNode = curNode;
			curNode->m_childNodes[j]->m_gridID = nid;
			curNode->m_childNodes[j]->cost = gCost;
			curNode->m_childNodes[j]->hcost = hCost;
			open.PushQueue(curNode->m_childNodes[j]);
		}
		gridList[curNode->m_gridID].isClose = true;
		
		for(int i=0; i<flag; i++)
		{
			gridList[open.GetNodeAt(i)->m_gridID].openID = i;
		}
	}
}

int AStarGridNode::AStarPathSmooth(std::vector<Grid>& grids,  TileCtrl* tileCtrl, NavMesh* nav
								 , std::vector<int>& path, std::vector<vcg::Point3f>& wayPoints
								 , std::vector<vcg::Point3f>& smoothPath, std::vector<int>& smoothPathIds)
{
	// line of sight algorithm, divide the line to several points to check if some points are lied in obstacle tiles
	float tileSize = tileCtrl->m_tileSize;
	int wayPtNum = wayPoints.size();
	int pathSize = path.size();
	smoothPathIds.reserve(wayPtNum);
	smoothPath.reserve(wayPtNum);
//	vcg::Point3f startPt = wayPoints[0];
//	vcg::Point3f endPt = wayPoints[wayPtNum-1];
	bool foundEndPt = false;
	int startId = 0;
	int res = AddVertexToPath(wayPoints[0], path[0], smoothPathIds, smoothPath);

	float segLen = tileSize / DIVIDE_NUM;

	for(int i=startId + 1; i<wayPtNum; i++)	// traverse middle points to extend line of sight
	{
		vcg::Point3f startPt, endPt;
		startPt = wayPoints[startId];
		endPt = wayPoints[i];
		float dist = (endPt - startPt).Norm();
		vcg::Point3f dir = (endPt - startPt).Normalize();
		int segNum = dist / segLen;	// divide points
		for(int j=0; j<=segNum; j++)
		{	//traverse every point
			vcg::Point3f pt = startPt + dir*(j*segLen);
			vcg::Point3f nearest;
			bool foundFace = false;
			int gridId, faceId;
			Tile* tile = tileCtrl->GetTile(pt);
			if(tile != NULL)
			{
				foundFace = tile->GetNearestFace(pt, gridId, faceId, nav, grids, nearest);		// if the point is in a valid position
			}
			if(tile == NULL ||!foundFace || grids[gridId].m_obstacleStatus == FULL_OBSTACLE || !nav->face[faceId].walkable)
			{
				startId = i;
				res = AddVertexToPath(wayPoints[i - 1], path[i - 1], smoothPathIds, smoothPath);
				break;
			}
		}
	}

	res = AddVertexToPath(wayPoints[wayPtNum - 1], path[pathSize - 1], smoothPathIds, smoothPath);

	return 0;
}