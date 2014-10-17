#include "InputMesh.h"
#include "CommonDef.h"
#include "AStarSearch.h"
#include <wrap/io_trimesh/import.h>
#include "Recast.h"

InputMesh::InputMesh(void)
{

}


InputMesh::~InputMesh(void)
{
}

bool InputMesh::ReadMap(char* filepath)
{
	if(vcg::tri::io::Importer<NavMesh>::Open(m_navMesh, filepath)>0)
	{
		printf("Error reading file  %s\n",filepath);
		return false;
	}
	
	int vertNum = m_navMesh.vert.size();
	if(vertNum<=0)
	{
		printf("Error reading file  %s\n",filepath);
		return false;
	}
	m_minBox[0] = m_navMesh.vert[0].P()[0];
	m_minBox[1] = m_navMesh.vert[0].P()[1];
	m_minBox[2] = m_navMesh.vert[0].P()[2];
	m_maxBox[0] = m_navMesh.vert[0].P()[0];
	m_maxBox[1] = m_navMesh.vert[0].P()[1];
	m_maxBox[2] = m_navMesh.vert[0].P()[2];
	m_navMesh.vert[0].vertID= 0;
	for(int i = 1; i< vertNum; i++)
	{
		if(m_navMesh.vert[i].P()[0]<m_minBox[0])
			m_minBox[0] = m_navMesh.vert[i].P()[0];
		if(m_navMesh.vert[i].P()[1]<m_minBox[1])
			m_minBox[1] = m_navMesh.vert[i].P()[1];
		if(m_navMesh.vert[i].P()[2]<m_minBox[2])
			m_minBox[2] = m_navMesh.vert[i].P()[2];

		if(m_navMesh.vert[i].P()[0]>m_maxBox[0])
			m_maxBox[0] = m_navMesh.vert[i].P()[0];
		if(m_navMesh.vert[i].P()[1]>m_maxBox[1])
			m_maxBox[1] = m_navMesh.vert[i].P()[1];
		if(m_navMesh.vert[i].P()[2]>m_maxBox[2])
			m_maxBox[2] = m_navMesh.vert[i].P()[2];
		m_navMesh.vert[i].vertID = i;
	}

	vcg::tri::UpdateTopology<NavMesh>::VertexFace(m_navMesh);
	int edgeNum = m_navMesh.edge.size();

	for(int i = 0; i< vertNum; i++)
	{
		NavMeshFace* face = m_navMesh.vert[i].VFp();
		int id = m_navMesh.vert[i].VFi();
		while(face != NULL || id < 0)
		{
			face->vertexIDs[id] = i;
			int tmp_id = face->VFi(id);
			// 到下一个链表节点，链表存储的是邻面
			face = face->VFp(id);
			id = tmp_id;
		}
	}
}

void InputMesh::FindWalkableArea(float walkableSlopeAngle, float agentRadius)
{
	// 更新三角面元的邻接关系
	vcg::tri::UpdateTopology<NavMesh>::FaceFace(this->m_navMesh);
	vcg::tri::UpdateBounding<NavMesh>::Box(this->m_navMesh);
	const float walkableThr = cosf(walkableSlopeAngle/180.0f*PI);
	float MinWalkableSize = agentRadius * agentRadius * PI * WALKABLE_FACTOR;
	
	for(int i = 0; i< m_navMesh.vn; i++)
	{
		m_navMesh.vert[i].edgeIDs.clear();
	}
	
	for(int i=0; i< m_navMesh.face.size(); i++)
	{ 
		// calculate the normal of each face in order to see which face is walkable
		vcg::Point3f norm = m_navMesh.face[i].N();

		if (norm[1] > walkableThr)
			m_navMesh.face[i].walkable = true;
		else
			m_navMesh.face[i].walkable = false;
		m_navMesh.face[i].CCID = -1;
		m_navMesh.face[i].faceID = i;
		m_navMesh.face[i].interactionType = 0;
		//memset(m_navMesh.face[i].midPointIds, -1, 3*sizeof(int));
		//memset(m_navMesh.face[i].vertexIDs, -1, 3*sizeof(int));
		
	}
	int edgeNum = 0;
	m_navMesh.edges.resize(m_navMesh.fn*3);
	std::vector<int> faceEdgeNum;
	faceEdgeNum.resize(m_navMesh.fn, 0);
	int borderIdx = -1;
	for(int i=0; i<m_navMesh.fn; i++)	// traverse all faces
	{
		for(int j=0; j<3; j++) // traverse neighbor faces
		{
			int fid = m_navMesh.face[i].faceID;
			int nfid = m_navMesh.face[i].FFp(j)->faceID;
			if(fid > nfid)
				continue;
			if(fid == nfid)
			{
				borderIdx = j;
				continue;
			}
			m_navMesh.edges[edgeNum].edgeID = edgeNum;
			m_navMesh.edges[edgeNum].faceIDs[0] = fid;
			m_navMesh.edges[edgeNum].faceIDs[1] = nfid;
			m_navMesh.edges[edgeNum].isBorder = false;
			m_navMesh.face[fid].edgeIDs[faceEdgeNum[fid]] = edgeNum;
			faceEdgeNum[fid]++;
			m_navMesh.face[nfid].edgeIDs[faceEdgeNum[nfid]] = edgeNum;
			faceEdgeNum[nfid]++;
			int edgeVertNum = 0;
			int vertID1 = -1;
			int vertID2 = -1;
			for(int k=0; k<3; k++)	// traverse vertexes of current face
			{
				for(int l=0; l<3; l++)	//traverse vertexes of neighbor face
				{
					if(m_navMesh.face[fid].vertexIDs[k] == m_navMesh.face[nfid].vertexIDs[l])
					{
						if(edgeVertNum == 0)
						{
							vertID1 = k;
							edgeVertNum++;
						}
						else
						{
							vertID2 = k;
							break;
						}
					}
				}
			}
			if(vertID1 >= 0 && vertID2 >= 0)
			{
				if((vertID1+1)%3 != vertID2)	// reversed faces
				{
					if((vertID2+1)%3 == vertID1)
					{	// swap faces
						int tmp = vertID1;
						vertID1 = vertID2;
						vertID2 = tmp;
					}
					else
					{
						printf("[InputMesh/FindWalkableArea] Not avaliable edge, the vertexes of the edge are invalid.\n");
					}
				}
				m_navMesh.edges[edgeNum].vertIDs[0] = m_navMesh.face[fid].vertexIDs[vertID1];
				m_navMesh.edges[edgeNum].vertRelIDs[0] = vertID1;
				m_navMesh.vert[m_navMesh.face[fid].vertexIDs[vertID1]].edgeIDs.push_back(edgeNum);
				
				m_navMesh.edges[edgeNum].vertIDs[1] = m_navMesh.face[fid].vertexIDs[vertID2];
				m_navMesh.edges[edgeNum].vertRelIDs[1] = vertID2;
				m_navMesh.vert[m_navMesh.face[fid].vertexIDs[vertID2]].edgeIDs.push_back(edgeNum);
			}
			edgeNum++;
		}
		if(borderIdx >= 0)
		{
			int fid = m_navMesh.face[i].faceID;
			int vertIDs[3];
			int vertRefNum[3];
			for(int j=0; j<3; j++)
			{
				vertIDs[j] = m_navMesh.face[fid].vertexIDs[j];
				vertRefNum[j] = 0;
			}
			for(int j=0; j<faceEdgeNum[fid]; j++)	// traverse vertexes of current face
			{
				int eid = m_navMesh.face[fid].edgeIDs[j];
				int vrid0 = m_navMesh.edges[eid].vertRelIDs[0];
				int vrid1 = m_navMesh.edges[eid].vertRelIDs[1];
				vertRefNum[vrid0]++;
				vertRefNum[vrid1]++;
			}
			for(int j=faceEdgeNum[fid]; j<3; j++)	// traverse edges not processed
			{
				m_navMesh.edges[edgeNum].edgeID = edgeNum;
				m_navMesh.edges[edgeNum].faceIDs[0] = fid;
				m_navMesh.edges[edgeNum].isBorder = true;
				m_navMesh.face[fid].edgeIDs[faceEdgeNum[fid]] = edgeNum;
				
				int min1=0, min2=0; 
				int minVal1 = vertRefNum[0];
				int minVal2 = vertRefNum[0];
				
				for(int k=1; k<3; k++)	// traverse 3 vertexes to find the most minimum one
				{
					if(minVal1 < vertRefNum[k])
					{
						minVal1 = vertRefNum[k];
						min1 = k;
					}
				}
				for(int k=1; k<3; k++)	// traverse 3 vertexes to find the second minimum one
				{
					if(minVal2 < vertRefNum[k] && k != min1)
					{
						minVal2 = vertRefNum[k];
						min2 = k;
					}
				}
				m_navMesh.edges[edgeNum].vertIDs[0] = vertIDs[min1];
				m_navMesh.edges[edgeNum].vertRelIDs[0] = min1;
				m_navMesh.vert[vertIDs[min1]].edgeIDs.push_back(edgeNum);
				
				m_navMesh.edges[edgeNum].vertIDs[1] = vertIDs[min2];
				m_navMesh.edges[edgeNum].vertRelIDs[1] = min2;
				m_navMesh.vert[vertIDs[min2]].edgeIDs.push_back(edgeNum);

				vertRefNum[min1]++;
				vertRefNum[min2]++;
				faceEdgeNum[fid]++;
				edgeNum++;
			}
		}
	}

	m_navMesh.edge.resize(edgeNum);

	//mark the connected component on the mesh
	NavMesh::FaceIterator beginfi = m_navMesh.face.begin();
	int counterCCID = 0;
	for(int i=0; i<m_navMesh.face.size(); i++)
	{
		if(m_navMesh.face[i].CCID < 0)
		{
			m_navMesh.face[i].CCID = counterCCID++;
			SetConnectComponentRecursively(beginfi, i);
		}
	}
	std::vector<std::vector<int>> CCtoFace;
	std::vector<float> CCArea;
	CCArea.resize(counterCCID, 0.0f);
	CCtoFace.resize(counterCCID);
	for(int i=0; i<m_navMesh.face.size(); i++)
	{
		int ccid = m_navMesh.face[i].CCID;
		CCtoFace[ccid].push_back(i);

		CCArea[ccid] += HelenArea(getDistance(m_navMesh.face[i].P(0), m_navMesh.face[i].P(1))
			                    , getDistance(m_navMesh.face[i].P(1), m_navMesh.face[i].P(2))
								, getDistance(m_navMesh.face[i].P(2), m_navMesh.face[i].P(0))
								);
	}
	std::vector<std::vector<int>>::iterator iter;
	int i;
	for(iter =CCtoFace.begin(), i=0; iter != CCtoFace.end(); i++)	// WARNING it may be useful to create a data structure for Connect Component to store the area and index.
	{
		if(CCArea[i] < MinWalkableSize)
		{
			for(int j=0; j<iter->size(); j++)
			{
				int idx = (*iter)[j];
				m_navMesh.face[idx].CCID = -1;
				m_navMesh.face[idx].walkable = false;
			}
			iter = CCtoFace.erase(iter);	// WARNING may be slow
		}
		else
		{
			iter++;
		}
	}
}
//recursive function
int InputMesh::SetConnectComponentRecursively(NavMesh::FaceIterator beginfi, int offset)
{
	NavMesh::FaceIterator fi = beginfi + offset;
	for(int i = 0; i < 3; i++)
	{
		if(vcg::face::IsBorder(*fi,i))
			continue;
		else 
		{
			NavMeshFace* tmpFace = (NavMeshFace*)fi->FFp(i);
			if(tmpFace->CCID < 0)
			{
				tmpFace->CCID = fi->CCID;
				SetConnectComponentRecursively(beginfi, fi->FFp(i)->faceID);
			}
		}
	}
	return 0;
}

void InputMesh::BuildNavMesh()
{
	//m_midPtCtrl.BuildMidPointList(m_navMesh);
	/*
	AStarMPNode node;
	std::vector<int> path;
	std::vector<vcg::Point3f> wayPoints;
	vcg::Point3f startPos(61.901825, 10.100003, 67.083229);
	vcg::Point3f endPos(16.574692, 10.100000, -49.587627);
	node.AStarSearch(m_navMesh.face[140], m_navMesh.face[232]
				   , m_midPtCtrl.m_midPointList[18].position, m_midPtCtrl.m_midPointList[10].position
				   , m_midPtCtrl.m_midPointList, path, wayPoints);
	std::vector<vcg::Point3f> smoothPath;
	std::vector<int> smoothPathIds;
	node.AStarPathSmooth(m_navMesh, path, wayPoints, smoothPath, smoothPathIds);
	*/
}

static bool intersectSegmentTriangle(vcg::Point3f sp, vcg::Point3f sq,
									 vcg::Point3f a, vcg::Point3f b, vcg::Point3f c,
									 float &t)
{
	float v, w;
	vcg::Point3f ab, ac, qp, ap, norm, e;
	ab = b - a;
	ac = c - a;
	qp = sp - sq;
	
	// Compute triangle normal. Can be precalculated or cached if
	// intersecting multiple segments against the same triangle
	norm.X() = ab.Y()*ac.Z() - ab.Z()*ac.Y();
	norm.Y() = ab.Z()*ac.X() - ab.X()*ac.Z();
	norm.Z() = ab.X()*ac.Y() - ab.Y()*ac.X();
	//rcVcross(norm, ab, ac);
	
	// Compute denominator d. If d <= 0, segment is parallel to or points
	// away from triangle, so exit early
	float d = qp.dot(norm);//rcVdot(qp, norm);
	if (d <= 0.0f) return false;
	
	// Compute intersection t value of pq with plane of triangle. A ray
	// intersects iff 0 <= t. Segment intersects iff 0 <= t <= 1. Delay
	// dividing by d until intersection has been found to pierce triangle
	//rcVsub(ap, sp, a);
	ap = sp - a;
	//t = rcVdot(ap, norm);
	t = ap.dot(norm);
	if (t < 0.0f) return false;
	if (t > d) return false; // For segment; exclude this code line for a ray test
	
	// Compute barycentric coordinate components and test if within bounds
	e.X() = qp.Y()*ap.Z() - qp.Z()*ap.Y();
	e.Y() = qp.Z()*ap.X() - qp.X()*ap.Z();
	e.Z() = qp.X()*ap.Y() - qp.Y()*ap.X();
	
	//rcVcross(e, qp, ap);
	v = ac.dot(e);//rcVdot(ac, e);
	if (v < 0.0f || v > d) return false;
	w = -ab.dot(e);//rcVdot(ab, e);
	if (w < 0.0f || v + w > d) return false;
	
	// Segment/ray intersects triangle. Perform delayed division
	t /= d;
	
	return true;
}

static bool isectSegAABB(const float* sp, const float* sq,
						 const float* amin, const float* amax,
						 float& tmin, float& tmax)
{
	static const float EPS = 1e-6f;
	
	float d[3];
	d[0] = sq[0] - sp[0];
	d[1] = sq[1] - sp[1];
	d[2] = sq[2] - sp[2];
	tmin = 0.0;
	tmax = 1.0f;
	
	for (int i = 0; i < 3; i++)
	{
		if (fabsf(d[i]) < EPS)
		{
			if (sp[i] < amin[i] || sp[i] > amax[i])
				return false;
		}
		else
		{
			const float ood = 1.0f / d[i];
			float t1 = (amin[i] - sp[i]) * ood;
			float t2 = (amax[i] - sp[i]) * ood;
			if (t1 > t2) { float tmp = t1; t1 = t2; t2 = tmp; }
			if (t1 > tmin) tmin = t1;
			if (t2 < tmax) tmax = t2;
			if (tmin > tmax) return false;
		}
	}
	
	return true;
}

bool InputMesh::raycastMesh(float* src, float* dst, float& tmin)
{
	float dir[3];
	rcVsub(dir, dst, src);

	// Prune hit ray.
	float btmin, btmax;
	if (!isectSegAABB(src, dst, m_minBox, m_maxBox, btmin, btmax))
		return false;
	float p[2], q[2];
	p[0] = src[0] + (dst[0]-src[0])*btmin;
	p[1] = src[2] + (dst[2]-src[2])*btmin;
	q[0] = src[0] + (dst[0]-src[0])*btmax;
	q[1] = src[2] + (dst[2]-src[2])*btmax;
	
	tmin = 1.0f;
	bool hit = false;
	/*
	int cid[512];
	const int ncid = rcGetChunksOverlappingSegment(m_chunkyMesh, p, q, cid, 512);
	if (!ncid)
		return false;
	
	//const float* verts = m_mesh->getVerts();
	
	for (int i = 0; i < ncid; ++i)
	{
		const rcChunkyTriMeshNode& node = m_chunkyMesh->nodes[cid[i]];
		const int* tris = &m_chunkyMesh->tris[node.i*3];
		const int ntris = node.n;

		for (int j = 0; j < ntris*3; j += 3)
		{
			float t = 1;
			if (intersectSegmentTriangle(src, dst,
										 &verts[tris[j]*3],
										 &verts[tris[j+1]*3],
										 &verts[tris[j+2]*3], t))
			{
				if (t < tmin)
					tmin = t;
				hit = true;
			}
		}
	}
	*/
	vcg::Point3f srcVec, dstVec;
	srcVec.X() = src[0];	srcVec.Y() = src[1];	srcVec.Z() = src[2];
	dstVec.X() = dst[0];	dstVec.Y() = dst[1];	dstVec.Z() = dst[2];
	for(int i=0; i<m_navMesh.fn; i++)
	{
		float t = 1;
		if (intersectSegmentTriangle(srcVec, dstVec,
										m_navMesh.face[i].V(0)->P(),
										m_navMesh.face[i].V(1)->P(),
										m_navMesh.face[i].V(2)->P(), t) )
		{
			if (t < tmin)
				tmin = t;
			hit = true;
		}
	}
	return hit;
}