#include "TileCtrl.h"
#include "CommonDef.h"

TileCtrl::TileCtrl(void)
{
	m_tiles = NULL;
}


TileCtrl::~TileCtrl(void)
{
	ClearTiles();
}

void TileCtrl::ClearTiles()
{
	for(int r=0; r<m_rows; r++)
	{
		if(m_tiles[r] != NULL)
		{
			delete[] m_tiles[r];
			m_tiles[r] = NULL;
		}
	}
	if(m_tiles != NULL)
	{
		delete[] m_tiles;
		m_tiles = NULL;
	}
}

int TileCtrl::GetTileID(int r, int c)
{
	return r*m_cols + c;
}

Tile* TileCtrl::GetTile(vcg::Point3f pt)
{
	if(pt.Z() < m_minZ - EDGE_THRESHOLD || pt.X() < m_minX - EDGE_THRESHOLD 
	|| pt.Z() > m_maxZ + EDGE_THRESHOLD || pt.X() > m_maxX + EDGE_THRESHOLD)
	{
		return NULL;
	}
	int r = (pt.Z() - m_minZ) / m_tileSize;
	int c = (pt.X() - m_minX) / m_tileSize;
	if(r < 0)	r = 0;
	if(c < 0)	c = 0;
	if(r > m_rows - 1)	r = m_rows - 1;
	if(c > m_cols - 1)	c = m_cols - 1;
	return &m_tiles[r][c];
}

int TileCtrl::BuildTiles(NavMesh* nav, float tileSize)
{
	m_tileSize = tileSize;
	m_minX = nav->bbox.min.X();
	m_minZ = nav->bbox.min.Z();
	m_maxX = nav->bbox.max.X();
	m_maxZ = nav->bbox.max.Z();
	m_cols = ceil((m_maxX - m_minX)/tileSize);	// x is columm
	m_rows = ceil((m_maxZ - m_minZ)/tileSize);	// z is row
	float width = m_cols * tileSize;
	float height = m_rows * tileSize;
	if(m_tiles != NULL)
	{
		ClearTiles();
	}
	m_tiles = new Tile*[m_rows];//(Tile**)malloc(m_rows*sizeof(Tile*));
	
	for(int r=0; r<m_rows; r++)
	{
		m_tiles[r] = new Tile[m_cols]; //(Tile*)malloc(m_cols*sizeof(Tile));
		for(int c=0; c<m_cols; c++)
		{
			int tileID = GetTileID(r, c);
			m_tiles[r][c].m_tileID = tileID;
			m_tiles[r][c].m_row = r;
			m_tiles[r][c].m_col = c;
			//m_tiles[r][c].m_box.set(m_minZ + r*tileSize, m_minX + c*tileSize, tileSize, tileSize);
			m_tiles[r][c].m_box.set(m_minX + c*tileSize, m_minZ + r*tileSize, tileSize, tileSize);
		}
	}

	for(int r=0; r<m_rows; r++)
	{
		for(int c=0; c<m_cols; c++)
		{
			if(r > 0)
				m_tiles[r][c].m_neighbor[0] = &m_tiles[r-1][c];	//up
			if(r < m_rows - 1)
				m_tiles[r][c].m_neighbor[1] = &m_tiles[r+1][c];	//down
			if(c > 0)
				m_tiles[r][c].m_neighbor[2] = &m_tiles[r][c-1];	//left
			if(c < m_cols - 1)
				m_tiles[r][c].m_neighbor[3] = &m_tiles[r][c+1];	//right
		}
	}

	// 遍历面片填入网格
	for(int i=0; i<nav->fn; i++)
	{
		// WARNING currently we only assume that each face should only belong to one tile 
		//if(!nav->face[i].walkable)
		//{
		//	continue;
		//}
		int minr, minc, maxr, maxc;
		printf("[No %d]*************************************\n", i);
		for(int j=0; j<3; j++)
		{
			vcg::Point3f pos = nav->face[i].V(j)->P();
			int r = (pos.Z() - m_minZ) / tileSize;
			int c = (pos.X() - m_minX) / tileSize;
			printf("points: %f %f %f, at %d %d.\n", pos.X(), pos.Y(), pos.Z(), r, c);
			if(j == 0)
			{
				minr = r;
				minc = c;
				maxr = r;
				maxc = c;
				continue;
			}
			if(minr > r)
				minr = r;
			if(minc > c)
				minc = c;
			if(maxr < r)
				maxr = r;
			if(maxc < c)
				maxc = c;
		}
		// find potential tiles for current face
		minr = std::max(minr - 1, 0);
		maxr = std::min(maxr + 1, m_rows);
		minc = std::max(minc - 1, 0);
		maxc = std::min(maxc + 1, m_cols);

		// traverse tiles to find where the face belongs to
		POINT_TILE_STATUS status[3];
		int statusSum = 0;
		for(int r = minr; r < maxr; r++)
		{
			for(int c = minc; c < maxc; c++)
			{
				statusSum = 0;
				for(int j=0; j<3; j++)
				{
					//status[j] = m_tiles[r][c].m_box.GetStatus(nav->face[i].V(j)->P().Z(), nav->face[i].V(j)->P().X());
					status[j] = m_tiles[r][c].m_box.GetStatus(nav->face[i].V(j)->P().X(), nav->face[i].V(j)->P().Z());
					statusSum += status[j];
				}

				if(statusSum == 3000)
				{
					continue; // Totally outside
				}
				else if(statusSum > 1000 && statusSum <= 2100)
				{
					// some parts of the triangle may be in the tile, should calculate the intersect points to make sure
					// right now, we don't implement it as it's quite complicated and not necessary.
					continue;	
				}
				else if(statusSum >= 6 && statusSum <= 300)
				{
					// all parts of the triangle is in the tile
					m_tiles[r][c].m_faces.push_back(i);
					float area = 0.0f;
					// here we have a heck, we should only count the projection of the faces
					vcg::Point3f ea, eb, ec;
					ea = nav->face[i].V(0)->P();	ea.Y() = eb.Y();
					eb = nav->face[i].V(1)->P();
					ec = nav->face[i].V(2)->P();	ec.Y() = eb.Y();
					area = ComputeTriArea(ea, eb, ec);
					m_tiles[r][c].m_faceArea.push_back(area);
					continue;
				}
			}
		}
	}
	return 0;
}

int TileCtrl::BuildGrids(NavMesh* nav)
{
	for(int r=0; r<m_rows; r++)
	{
		for(int c=0; c<m_cols; c++)
		{
			m_tiles[r][c].BuildGrids(nav, m_grids);
		}
	}

	for(int r=0; r<m_rows; r++)
	{
		for(int c=0; c<m_cols; c++)
		{
			m_tiles[r][c].BuildGridNeighbor(nav, m_grids);
		}
	}
	return 0;
}

void TileCtrl::ResetGridList()
{
	for(unsigned int i=0; i<m_grids.size(); i++)
	{
		m_grids[i].isClose = false;
		m_grids[i].openID = -1;
	}
}