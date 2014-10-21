#include "TileCtrl.h"
#include "CommonDef.h"

#include <kbool\booleng.h>

void ArmBoolEng(Bool_Engine* booleng)
{
	// set some global vals to arm the boolean engine
	double DGRID = 1000;  // round coordinate X or Y value in calculations to this
	double MARGE = 0.001;   // snap with in this range points to lines in the intersection routines
	// should always be > DGRID  a  MARGE >= 10*DGRID is oke
	// this is also used to remove small segments and to decide when
	// two segments are in line.
	double CORRECTIONFACTOR = 0.0;  // correct the polygons by this number
	double CORRECTIONABER = 1.0;    // the accuracy for the rounded shapes used in correction
	double ROUNDFACTOR = 0.015;    // when will we round the correction shape to a circle
	double SMOOTHABER = 0.1;   // accuracy when smoothing a polygon
	double MAXLINEMERGE = 1000.0; // leave as is, segments of this length in smoothen


	// DGRID is only meant to make fractional parts of input data which
	// are doubles, part of the integers used in vertexes within the boolean algorithm.
	// Within the algorithm all input data is multiplied with DGRID

	// space for extra intersection inside the boolean algorithms
	// only change this if there are problems
	int GRID = 10000;

	booleng->SetMarge(MARGE);
	booleng->SetGrid(GRID);
	booleng->SetDGrid(DGRID);
	booleng->SetCorrectionFactor(CORRECTIONFACTOR);
	booleng->SetCorrectionAber(CORRECTIONABER);
	booleng->SetSmoothAber(SMOOTHABER);
	booleng->SetMaxlinemerge(MAXLINEMERGE);
	booleng->SetRoundfactor(ROUNDFACTOR);

}

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

	// traverse all the faces
	for(int i=0; i<nav->fn; i++)
	{
		// WARNING currently we only assume that each face should only belong to one tile 
		//if(!nav->face[i].walkable)
		//{
		//	continue;
		//}
		// get the boundary of the three vertices to find the outer bounding box, and so we can find potentially intersect tiles
		int minr, minc, maxr, maxc;
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
				Bool_Engine* booleng = new Bool_Engine();
				ArmBoolEng(booleng);
				booleng->StartPolygonAdd(GROUP_A);	//counter-clock wise
				{
					booleng->AddPoint(m_tiles[r][c].m_box.x(), m_tiles[r][c].m_box.y());
					booleng->AddPoint(m_tiles[r][c].m_box.x(), m_tiles[r][c].m_box.b());
					booleng->AddPoint(m_tiles[r][c].m_box.r(), m_tiles[r][c].m_box.b());
					booleng->AddPoint(m_tiles[r][c].m_box.r(), m_tiles[r][c].m_box.y());
				}
				booleng->EndPolygonAdd();

				booleng->StartPolygonAdd(GROUP_B);
				{
					booleng->AddPoint(nav->face[i].V(0)->P().X(), nav->face[i].V(0)->P().Z());
					booleng->AddPoint(nav->face[i].V(2)->P().X(), nav->face[i].V(2)->P().Z());
					booleng->AddPoint(nav->face[i].V(1)->P().X(), nav->face[i].V(1)->P().Z());
				}
				booleng->EndPolygonAdd();
				booleng->Do_Operation(BOOL_AND);
				float firstPt[2], prePt[2], curPt[2];
				memset(firstPt, 0, 2 * sizeof(float));
				memset(prePt, 0, 2 * sizeof(float));
				memset(curPt, 0, 2 * sizeof(float));
				int ptCount = 0;
				float sumArea = 0;
				while (booleng->StartPolygonGet())
				{
					// foreach point in the polygon
					while (booleng->PolygonHasMorePoints())
					{
						if (ptCount == 0)
						{
							firstPt[0] = booleng->GetPolygonXPoint();
							firstPt[1] = booleng->GetPolygonYPoint();
							prePt[0] = firstPt[0];
							prePt[1] = firstPt[1];
							ptCount++;
						}
						else
						{
							curPt[0] = booleng->GetPolygonXPoint();
							curPt[1] = booleng->GetPolygonYPoint();
							sumArea += prePt[0] * curPt[1] - curPt[0] * prePt[1];
							prePt[0] = curPt[0];
							prePt[1] = curPt[1];
						}
					}
					booleng->EndPolygonGet();
				}
				sumArea += curPt[0] * firstPt[1] - firstPt[0] * curPt[1];
				sumArea = abs(sumArea / 2);
				if (sumArea > EPSILON) {
					m_tiles[r][c].m_faces.push_back(i);
					m_tiles[r][c].m_faceArea.push_back(sumArea);
				}
				delete booleng;
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