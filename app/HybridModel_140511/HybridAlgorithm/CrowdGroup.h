#ifndef CROWDGROUP_H
#define CROWDGROUP_H

#include "QuadTree.h"
#include "Grid.h"
#include <vector>
using namespace std;

class Grid;
class QuadTree;



class CrowdGroup
{
public:
	vector<Grid*> crowd_grid;//建立vector，存储每个流体区域的所有格子
	long int RegionID;
	vector<Grid*> contour_grid;
public:
	CrowdGroup(void);
	~CrowdGroup(void);
	void AddGrid(QuadTree* p);
	void TrackContour();
};

#endif
