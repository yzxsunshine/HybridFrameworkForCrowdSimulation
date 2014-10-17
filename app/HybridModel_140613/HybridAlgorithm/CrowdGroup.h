#ifndef CROWDGROUP_H
#define CROWDGROUP_H

#include "Grid.h"
#include "HmAgent.h"
#include <vector>

struct elem_grid {	//�������ṹ�����������Ų���Ⱥ��ʱ������
	int id;
	//int r,c;
	Vector2 center;
	float score;
	HmAgent* agent;
};



class CrowdGroup
{
public:
	std::vector<int> crowdGrid;//����vector���洢ÿ��������������и���
	long int RegionID;
	std::vector<int> contourAgent;
	std::vector<Vector2> contourPts;
	std::vector<std::vector<Vector2>> contours;
	Vector2 avgVel;	//average velocity
public:
	CrowdGroup(void);
	~CrowdGroup(void);
	void AddGrid(int gridId);
	void TrackContour(std::vector<Grid>* grids, HmAgent** agents);
};

#endif
