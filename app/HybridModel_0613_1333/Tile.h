#pragma once
#include "Rectangle.h"
#include "NavMesh.h"
#include "Grid.h"

/*
	tile������������ά�����ĸ��ӣ�tile��grid�󡣡���
*/

class Tile
{
public:
	Tile(void);
	~Tile(void);
	void Init(vcg::Point2f leftTop, int tileID, float size);
	void BuildGrids(NavMesh* nav, std::vector<Grid>& grids);
	void BuildGridNeighbor(NavMesh* nav, std::vector<Grid>& grids);
	bool GetNearestFace(vcg::Point3f& pos, int& gridID, int& faceID, NavMesh* nav, std::vector<Grid>& grids, vcg::Point3f& nearestPos);
	bool GetNearestFace(const float* pos, int& gridID, int& faceID, NavMesh* nav, std::vector<Grid>& grids, float* nearestPos);
public:
	CRectangle<float> m_box;	// 2D bounding box of the grid
	std::vector<int> m_faces;
	std::vector<float> m_faceArea;
	std::vector<int>	m_gridIds;
	int m_tileID;
	int m_row;
	int m_col;
	Tile* m_neighbor[4];	//�ĸ�������ھӽڵ� 0,1,2,3 ��,��,��,��
};

