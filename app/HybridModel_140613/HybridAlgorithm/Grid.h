#ifndef GRID_H
#define GRID_H
#include "Vector2.h"
#include "Rectangle.h"
#include "../NavMesh.h"
#include <vector> 
using namespace std;

enum GRID_OBSTACLE_STATUS
{
	NO_OBSTACLE,
	HAS_OBSTACLE,
	FULL_OBSTACLE
};

enum GRID_DENSITY_STATUS  
{
	WHITE,
	BLACK,
	GRAY
};//格子状态定义，白黑灰

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

	vector<int> agentsInGrid;
	double nowDensity;
	double formerDensity;
	
	int x,y;
	//HybridModelCrowd* hmcrowd;
	Vector2 avgVelocity;
	bool isContour;

	CRectangle<float> m_box;	// 2D bounding box of the grid
	float m_size;	//size of each edge
	float m_area;	// avaliable area of the gird
	GRID_OBSTACLE_STATUS m_obstacleStatus;
	GRID_DENSITY_STATUS m_densityStatus;
	std::vector<int> m_faces;
	int m_neighbor[4];	//四个方向的邻居节点 0,1,2,3 上,下,左,右

	int openID;
	vcg::Point3f center;
	int centerFace;
	bool isClose;
public:
	static long int QuadTreeGridID;
public:
	bool GetNearestFace(vcg::Point3f& pos, int& faceID, NavMesh* nav, std::vector<Grid>& grids, vcg::Point3f& nearestPos);
	bool GetNearestFace(const float* pos, int& faceID, NavMesh* nav, std::vector<Grid>& grids, float* nearestPos);
	bool GetEdgePoint(vcg::Point3f startPt, vcg::Point3f endPt
					, vcg::Point3f& entryPt, vcg::Point3f& exitPt
					, int& entryFace, int& exitFace
					, NavMesh* nav, std::vector<Grid>& grids);
	int GetAgentNum(void) { return static_cast<int>(agentsInGrid.size());}
	void UpdateDensity(float threshold, float agentSize);
};

#endif