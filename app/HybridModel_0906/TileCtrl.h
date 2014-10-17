#pragma once
#include "Tile.h"
#include "NavMesh.h"

/*
	用来管理和存储tiles和grids的类
*/

class TileCtrl
{
public:
	TileCtrl(void);
	~TileCtrl(void);
	int BuildTiles(NavMesh* nav, float tileSize);
	int BuildGrids(NavMesh* nav);
	int GetTileID(int r, int c);
	Tile* GetTile(vcg::Point3f pt);
	void ClearTiles();
	float GetTileSize() { return m_tileSize; }
	void ResetGridList();
public:
	Tile** m_tiles;
	std::vector<Grid> m_grids;
	int m_rows;
	int m_cols;
	float m_minX;
	float m_minZ;
	float m_maxX;
	float m_maxZ;
	float m_tileSize;
};

