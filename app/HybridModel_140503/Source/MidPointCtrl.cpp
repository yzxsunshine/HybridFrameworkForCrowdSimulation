#include "MidPointCtrl.h"
#include "CommonDef.h"
// MidPoint
MidPoint::MidPoint(void) : isBorder(true), openID(-1), isClose(false)
{
}


MidPoint::~MidPoint(void)
{
}

// MidPointCtrl
MidPointCtrl::MidPointCtrl(void)
{
}


MidPointCtrl::~MidPointCtrl(void)
{
}

void MidPointCtrl::BuildMidPointList(NavMesh& navMesh)
{
	// build mid point list
	int mpCount = 0;
	int faceNum = navMesh.face.size();
	m_midPointList.clear();
	m_midPointList.reserve(faceNum * 3);
	int* flag = new int[faceNum];
	memset(flag, 0, faceNum*sizeof(int));
	for(int fi=0; fi<faceNum; fi++)
	{
		if( !navMesh.face[fi].walkable || flag[navMesh.face[fi].faceID] >= 3 )
				continue;
		for(int i=0; i<3; i++)//遍历相邻三角面
		{
			if( !navMesh.face[fi].FFp(i)->walkable || flag[navMesh.face[fi].FFp(i)->faceID] >= 3 )
				continue;
			vcg::Point3f tmpPoints[2];
			int ptsCount = 0;
			bool findEnoughPts = false;
			MidPoint mp;
			for(int j=0; j<3 && !findEnoughPts; j++)
			{
				for(int k=0; k<3 && !findEnoughPts; k++)
				{
					if(navMesh.face[fi].edgeIDs[j] == navMesh.face[fi].FFp(i)->edgeIDs[k])
					{
						int eid = navMesh.face[fi].edgeIDs[j];
						int vid0 = navMesh.edges[eid].vertIDs[0];
						int vid1 = navMesh.edges[eid].vertIDs[1];
						mp.position = navMesh.vert[vid0].P() + navMesh.vert[vid1].P();
					}
				}
			}
			mp.faceIds[0] = fi;
			mp.faceIds[1] = navMesh.face[fi].FFp(i)->faceID;
			int midPointID = m_midPointList.size();
			navMesh.face[mp.faceIds[0]].midPointIds[flag[mp.faceIds[0]]] = midPointID;
			navMesh.face[mp.faceIds[1]].midPointIds[flag[mp.faceIds[1]]] = midPointID;
			if(mp.faceIds[0] != mp.faceIds[1])
			{
				flag[mp.faceIds[0]]++;
				flag[mp.faceIds[1]]++;
				mp.isBorder = false;
			}
			else
			{
				flag[mp.faceIds[0]]++;
				mp.isBorder = true;
			}
			m_midPointList.push_back(mp);
			mpCount++;
		}
	}
	m_midPointList.resize(mpCount);
	delete[] flag;

	// build mid point neiborhood
	for(int i=0; i<mpCount; i++)
	{
		for(int j=0; j<2; j++)
		{
			int faceId = m_midPointList[i].faceIds[j];
			int count = 0;
			for(int k=0; k<3; k++)
			{
				if(navMesh.face[faceId].midPointIds[k] != i)
				{
					int nid = navMesh.face[faceId].midPointIds[k];	//neighbor id
					if(nid < 0)
						continue;
					m_midPointList[i].neighbor[j*2 + count] = nid;
					m_midPointList[i].cost[j*2 + count] = (m_midPointList[i].position - m_midPointList[nid].position).Norm();
					count++;
				}
			}
		}
	}
}