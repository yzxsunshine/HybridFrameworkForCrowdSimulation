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
	vector<Grid*> crowd_grid;//����vector���洢ÿ��������������и���
	long int RegionID;
	vector<Grid*> contour_grid;
public:
	CrowdGroup(void);
	~CrowdGroup(void);
	void AddGrid(QuadTree* p);
	void TrackContour();
};

#endif
