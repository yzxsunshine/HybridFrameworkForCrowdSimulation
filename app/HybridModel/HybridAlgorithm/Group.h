#pragma once
#include "Vector2.h"
#include "Rectangle.h"
#include <vector> 
using namespace std;

enum GRID_DENSITY_STATUS
{
	WHITE,
	BLACK,
	GRAY
};//格子状态定义，白黑灰

class Group
{
public:
	Group();
	~Group();
public:
	vector<int> agentsInGroup;
	double nowDensity;
	double formerDensity;

	int x, y;
	//HybridModelCrowd* hmcrowd;
	Vector2 avgVelocity;

	// Personality
	float PC1;
	float PC2;

	int groupID;
	bool isContour;
	float m_area;	// avaliable area of the gird
	CRectangle<float> m_box;	// 2D bounding box of the grid
	int m_neighbor[4];	//up down left right 0,1,2,3 
	GRID_DENSITY_STATUS m_densityStatus;
public:
	void init(int gid, float area) {
		groupID = gid;
		m_area = area;
	}
	int GetAgentNum(void) { return static_cast<int>(agentsInGroup.size()); }
	void UpdateDensity(float threshold, float agentSize);
	bool GetInterpolationNeighbors(Vector2 pos, std::vector<Group>* grids, std::vector<int>& nids
		, Vector2& leftBottom, Vector2& rightTop);
};

