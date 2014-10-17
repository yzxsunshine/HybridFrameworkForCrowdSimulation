#ifndef GRID_H
#define GRID_H
#include "Vector2.h"
#include "QuadTree.h"
#include "Rectangle.h"
#include "HybridModelCrowd.h"
#include <vector> 
using namespace std;

class HybridModelCrowd;
class QuadTree;
class Grid
{
public:
	Grid(void);
	~Grid(void);
public:
	int grid_id;
	vector<int> people_in_grid;
	double now_density;
	double former_density;
	QuadTree *tpnode;//格子指向四叉树的指针
	float avaliable_region_rate;
	int x,y;
	CRectangle<double> box;
	HybridModelCrowd* hmcrowd;
	Vector2 avgVelocity;
public:
	static long int QuadTreeGridID;
	static double LowDensity;	//阈值，低密度人群
	static double HighDensity;	//阈值，高密度人群
public:
	int GetGridId(double i,double j);
	int GetGridId(const Vector2& pos);
};

#endif