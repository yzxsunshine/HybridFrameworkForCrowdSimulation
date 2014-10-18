#ifndef GRID_H
#define GRID_H
#include "Vector2.h"
#include "Rectangle.h"
#include "../NavMesh.h"
#include "Group.h"
#include <vector> 
using namespace std;

enum GRID_OBSTACLE_STATUS
{
	NO_OBSTACLE,
	HAS_OBSTACLE,
	FULL_OBSTACLE
};



//class HybridModelCrowd;
//class QuadTree;
class Grid
{
public:
	Grid(void);
	~Grid(void);
public:
	int gridID;
	int tileID;

	Group m_group;
	float m_size;	//size of each edge
	float m_area;	// avaliable area of the gird
	GRID_OBSTACLE_STATUS m_obstacleStatus;
	GRID_DENSITY_STATUS m_densityStatus;
	std::vector<int> m_faces;
	
	int openID;
	vcg::Point3f center;
	int centerFace;
	bool isClose;
public:
	bool GetNearestFace(vcg::Point3f& pos, int& faceID, NavMesh* nav, std::vector<Grid>& grids, vcg::Point3f& nearestPos);
	bool GetNearestFace(const float* pos, int& faceID, NavMesh* nav, std::vector<Grid>& grids, float* nearestPos);
	bool GetEdgePoint(vcg::Point3f startPt, vcg::Point3f endPt
					, vcg::Point3f& entryPt, vcg::Point3f& exitPt
					, int& entryFace, int& exitFace
					, NavMesh* nav, std::vector<Grid>& grids);
};

#endif