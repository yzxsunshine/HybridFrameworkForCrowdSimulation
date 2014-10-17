#include "AStarSearch.h"
#include "CommonDef.h"
OpenQueue::OpenQueue(std::vector<MidPoint>* mp, int initSize)
{
	heap.reserve(initSize);
	mpList = mp;
}

OpenQueue::~OpenQueue()
{
	for(int i=0; i<heap.size(); i++)
	{
		delete heap[i];
		heap[i] = NULL;
	}
	heap.clear();
}

void OpenQueue::PreAllocateMemory(int extendSize)
{
	int prevSize = heap.capacity();
	heap.reserve(prevSize + extendSize);
}

bool OpenQueue::IsQueueEmpty()
{
	return (heap.empty());
}

AStarMPNode* OpenQueue::PopQueue()
{
	AStarMPNode* node = heap.front();
	std::pop_heap(heap.begin(), heap.end(), NodeHCostGreater());
	heap.pop_back();
	return node;
}

void OpenQueue::PushQueue(AStarMPNode* node)
{
	heap.push_back(node);
	std::push_heap(heap.begin(), heap.end(), NodeHCostGreater());
	
	for(int i=0; i<heap.size(); i++)
	{
		(*mpList)[heap[i]->m_mpID].openID = i;
	}
}

void OpenQueue::UpdateQueue(AStarMPNode* node)
{
	std::vector<AStarMPNode*>::iterator iter;
	for(iter = heap.begin(); iter!=heap.end(); iter++)
	{
		if((*iter)->m_mpID == node->m_mpID)
		{
			std::push_heap(heap.begin(), iter+1, NodeHCostGreater());	
			return;
		}
	}
}

AStarMPNode*& OpenQueue::GetNodeAt(int id)
{
	return heap[id];
}

void AStarMPNode::AStarSearch(NavMeshFace& startFace, NavMeshFace& endFace
			   , vcg::Point3f startPt, vcg::Point3f endPt, std::vector<MidPoint>& m_midPointList
			   , std::vector<int>& path, std::vector<vcg::Point3f>& wayPoints)
{
	//std::vector<int> path;	// store faces lie on the path
	//std::vector<vcg::Point3f> wayPoints;	// store middle points lie on the path
	int N = m_midPointList.size();
	OpenQueue open(&m_midPointList);
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
			AStarMPNode* L = curNode;
			while(L != NULL)
			{
				wayPoints.push_back(m_midPointList[L->m_mpID].position);
				printf("point id: %d\n", L->m_mpID);
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

int AStarMPNode::AddVertexToPath(vcg::Point3f pt, int faceID, std::vector<int>& smoothPathIds, std::vector<vcg::Point3f>& smoothPathPts)
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

int AStarMPNode::GetPointSide(vcg::Point2f ptA, vcg::Point2f ptB, vcg::Point2f wayPt)
{
	vcg::Point2f normal = ptB - ptA;
	normal.Normalize();
	float oldVal = normal.Y();
	normal.Y() = -normal.X();
	normal.X() = oldVal;

	vcg::Point2f testVec = wayPt - ptA;
	float dotVal = testVec.dot(normal);
	if(dotVal > EPSILON)	// right
	{
		return RIGHT_SIDE;		
	}
	else if(dotVal < -EPSILON)	// left
	{
		return LEFT_SIDE;			
	}
	else
	{
		return ON_LINE;
	}
}

int AStarMPNode::GetIntersection(vcg::Point2f startPt, vcg::Point2f endPt
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

/*void AStarMPNode::print(AStarMPNode* Node, std::vector<MidPoint>& m_midPointList)
{
	AStarMPNode* L = Node;
	if(Node != NULL)
	{
		L = Node->m_parentNode;
		print(L, m_midPointList);
		if(Node->m_parentNode != NULL)
		{
			for(int i=0; i<2; i++)
				for(int j=0; j<2; j++)
				{
					if(m_midPointList[Node->m_mpID].faceIds[i] == m_midPointList[Node->m_parentNode->m_mpID].faceIds[j])
					{
						printf("%d\n",m_midPointList[Node->m_mpID].faceIds[i]);
					}
				}
		}
	}
	
}
*/