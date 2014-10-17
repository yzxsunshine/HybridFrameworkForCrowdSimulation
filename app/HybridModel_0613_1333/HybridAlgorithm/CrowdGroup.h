#ifndef CROWDGROUP_H
#define CROWDGROUP_H

#include "Grid.h"
#include "HmAgent.h"
#include <vector>

struct elem_grid {	//这两个结构用来在重新排布人群的时候排序
	int id;
	//int r,c;
	Vector2 center;
	float score;
	HmAgent* agent;
};



class CrowdGroup
{
public:
	std::vector<int> crowdGrid;//建立vector，存储每个流体区域的所有格子
	long int RegionID;
	std::vector<int> contourAgent;
	std::vector<Vector2> contourPts;
	std::vector<std::vector<Vector2>> contours;
	Vector2 avgVel;	//average velocity
	Vector2 groupVel;
public:
	CrowdGroup(void);
	~CrowdGroup(void);
	void AddGrid(int gridId);
	void TrackContour(std::vector<Grid>* grids, HmAgent** agents);
	void GetGroupAgent(Vector2& pos, Vector2& vel, float& radius, std::vector<Grid>* grids, HmAgent** agents);	// used to add group as a agent to simulate interaction between groups
};

#endif
