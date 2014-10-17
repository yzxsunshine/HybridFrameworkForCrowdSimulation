#include "Tile.h"
#include "CommonDef.h"


Tile::Tile(void)
{
	m_neighbor[0] = NULL;
	m_neighbor[1] = NULL;
	m_neighbor[2] = NULL;
	m_neighbor[3] = NULL;
}


Tile::~Tile(void)
{
}

void Tile::Init(vcg::Point2f leftTop, int tileID, float size)
{
	m_box = CRectangle<float>(leftTop.X(), leftTop.Y(), size, size);
	m_tileID = tileID;
}

void Tile::BuildGrids(NavMesh* nav, std::vector<Grid>& grids)
{
	int faceNum = m_faces.size();
	if(faceNum <= 0)
		return;
	std::map<int, int> faceIdDict;
	std::map<int, int> sameMark;
	std::vector<int> faceCRIds;	// connect region ID
	faceCRIds.resize(faceNum);
	for(int i=0; i<faceNum; i++)
	{
		int fid = m_faces[i];
		//  the faceIdDict is used to find faces from it's id in navmesh
		faceIdDict.insert(std::pair<int, int>(fid, i));
		// we assume each face as a connect region at first. the connect region id is themselves
		faceCRIds[i] = i;
	}

	std::map<int, int>::iterator iter;
	for(int i=0; i<faceNum; i++)
	{
		int fid = m_faces[i];
		for(int j=0; j<3; j++)
		{
			int nfgid = nav->face[fid].FFp(j)->faceID;	// neighbor face global id
			iter = faceIdDict.find(nfgid);
			if(iter == faceIdDict.end())
			{
				continue;
			}
			int nfid = iter->second;		// neighbor face local id
			if(faceCRIds[nfid] >= faceCRIds[i])
			{
				faceCRIds[nfid] = faceCRIds[i];
			}
			else
			{
				std::map<int, int>::iterator iterPair = sameMark.find(faceCRIds[nfid]);
				int id = faceCRIds[nfid];
				while(iterPair != sameMark.end())	// maybe we don't need to have a loop, one level is enough
				{
					id = iterPair->second;
					iterPair = sameMark.find(iterPair->second);
				}
				sameMark.insert(std::pair<int, int>(faceCRIds[i], id));
				faceCRIds[i] = id;
			}
		}
	}

	int crNum = 0;	//number of connect region
	int startID = grids.size();
	std::vector<int> gridCRId;
	// initialize the first grid
	Grid grid;
	grid.gridID = startID;
	grid.m_faces.push_back(m_faces[0]);
	grid.tileID = m_tileID;
	grid.m_box = m_box;
	grid.m_area = m_faceArea[0];
	grids.push_back(grid);
	gridCRId.push_back(faceCRIds[0]);
	m_gridIds.push_back(grid.gridID);

	for(int i=1; i<faceNum; i++)
	{
		bool isNewGroup = true;
		for(int j=0; j<gridCRId.size(); j++)
		{
			if(faceCRIds[i] == gridCRId[j])
			{
				grids[startID + j].m_faces.push_back(m_faces[i]);
				grids[startID + j].m_area += m_faceArea[i];
				isNewGroup = false;
				break;
			}
			else
			{
				std::map<int, int>::iterator iterPair = sameMark.find(faceCRIds[i]);
				if(iterPair != sameMark.end() && iterPair->second == gridCRId[j])
				{
					grids[startID + j].m_faces.push_back(m_faces[i]);
					grids[startID + j].m_area += m_faceArea[i];
					isNewGroup = false;
					break;
				}
			}
		}
		if(isNewGroup==true)
		{
			Grid grid;
			grid.gridID = grids.size();
			grid.m_faces.push_back(m_faces[i]);
			grid.tileID = m_tileID;
			grid.m_box = m_box;
			grid.m_area = m_faceArea[i];
			grids.push_back(grid);
			gridCRId.push_back(faceCRIds[i]);
			m_gridIds.push_back(grid.gridID);
		}
	}
	// compute centers of each grids
	int gridNum = m_gridIds.size();
	for(int i=0; i<gridNum; i++)
	{
		vcg::Point3f center(0, 0, 0);
		int faceNum = grids[m_gridIds[i]].m_faces.size();
		int obstacleNum = 0;
		for(int j=0; j<faceNum; j++)
		{
			int fid = grids[m_gridIds[i]].m_faces[j];
			if(!nav->face[fid].walkable)
			{
				grids[m_gridIds[i]].m_obstacleStatus = HAS_OBSTACLE;
				obstacleNum++;
				continue;
			}
			center += nav->face[fid].V(0)->P();
			center += nav->face[fid].V(1)->P();
			center += nav->face[fid].V(2)->P();
		}
		if(obstacleNum == faceNum)
		{
			grids[m_gridIds[i]].m_obstacleStatus = FULL_OBSTACLE;
		}
		center = center / 3 / (faceNum-obstacleNum);
		float pos[3], nearest[3];
		pos[0] = center.X();	pos[1] = center.Y();	pos[2] = center.Z();
		grids[m_gridIds[i]].GetNearestFace(pos, grids[m_gridIds[i]].centerFace, nav, grids, nearest);
		grids[m_gridIds[i]].center = vcg::Point3f(nearest[0], nearest[1], nearest[2]);
	}
}

void Tile::BuildGridNeighbor(NavMesh* nav, std::vector<Grid>& grids)
{
	std::vector<std::map<int, int>> gridFaceNeighborIdDict;
	int gridNum = m_gridIds.size();
	gridFaceNeighborIdDict.resize(gridNum);
	for(int i=0; i<gridNum; i++)	// find all the face ids of each grids, this will be very useful as a dictory to find whether a face is inside the grid 
	{
		int gid = m_gridIds[i];
		for(int j=0; j<grids[gid].m_faces.size(); j++)
		{
			for(int k=0; k<3; k++)	// traverse all neighbors
			{
				int nfid = nav->face[grids[gid].m_faces[j]].FFp(k)->faceID;
				gridFaceNeighborIdDict[i].insert(std::pair<int, int>(nfid, grids[gid].m_faces[j]));	//then we can find whether a face is the face of current grid, including two grids share a common face
			}
		}
	}

	// it's a quite heavy computation, and may be very low efficiency, but right now, we implement it this way
	std::map<int, int>::iterator iter;
	for(int l=0; l<gridNum; l++)
	{
		int gid = m_gridIds[l];
		for(int i=0; i<4; i++)	// traverse all neighbor of the tile
		{
			grids[gid].m_neighbor[i] = -1;

			if(m_neighbor[i]!=NULL)
			{
				bool foundNeighbor = false;
				for(int j=0; j<m_neighbor[i]->m_gridIds.size() && !foundNeighbor; j++)	// traverse all grids of the neighbors
				{
					int ngid = m_neighbor[i]->m_gridIds[j];
					for(int k=0; k<grids[ngid].m_faces.size() && !foundNeighbor; k++)	// traverse all faces of the grid to find if any faces are adjacent to current grid
					{
						int fid = grids[ngid].m_faces[k];
						iter = gridFaceNeighborIdDict[l].find(fid);
						if(iter != gridFaceNeighborIdDict[l].end())
						{	// gotha, we have a neighbor grid here, then we should figure out how to build the neighbor relationship
							grids[gid].m_neighbor[i] = ngid;
							foundNeighbor = true;
						}
					}
				}
			}
		}
	}
}

bool Tile::GetNearestFace(vcg::Point3f& pos, int& gridID, int& faceID, NavMesh* nav, std::vector<Grid>& grids, vcg::Point3f& nearestPos)
{
	float p[3], nearest[3];
	p[0] = pos.X();	p[1] = pos.Y();	p[2] = pos.Z();
	bool res = GetNearestFace(p, gridID, faceID, nav, grids, nearest);
	nearestPos = vcg::Point3f(nearest[0], nearest[1], nearest[2]);
	return res;
}

bool Tile::GetNearestFace(const float* pos, int& gridID, int& faceID, NavMesh* nav, std::vector<Grid>& grids, float* nearestPos)
{
	bool foundFace = false;
	for(int i=0; i<m_gridIds.size(); i++)
	{
		int gid = m_gridIds[i];
		foundFace = grids[gid].GetNearestFace(pos, faceID, nav, grids, nearestPos);
		if(foundFace)
		{
			gridID = gid;
			break;
		}
	}
	return foundFace;
}