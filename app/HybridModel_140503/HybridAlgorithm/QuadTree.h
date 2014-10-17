#ifndef QUADTREE_H
#define QUADTREE_H

#include "Rectangle.h"
#include "HybridModelCrowd.h"
#include <vector>
using namespace std;

class HybridModelCrowd;
class Grid;
class CrowdGroup;
enum QuadTreeState  
{
	WHITE,
	BLACK,
	GRAY
};//格子状态定义，白黑灰

/* a quadrant defined below:
 
　　　　　　　　　 
 
　　　　　　　 NW(0)　　 |　　　 NE(1)
 
　　　　　　　 -----------|-----------
 
　　　　　　　 SW(3)　　 |　　　 SE(2)
 
*/
 

class QuadTree
{
public:
	QuadTree(void);
	~QuadTree(void);
public:
	QuadTree *node[4]; //子节点
	QuadTree *neighbor[4];//四个方向的邻居节点 0,1,2,3 上,下,左,右
	int layernum;
	QuadTreeState leaf_state;//节点的状态，QuadTreeState
	QuadTreeState former_leaf_state;
    int leaf_direction;//节点的方向，即使父节点的哪个子节点
    long int gridid;//对应格子的id，如果不是叶子节点则值为-1
	Grid* grid;
	QuadTree *parent;
	CRectangle<double> rect;
	long int RegionMark;	//判断区域连通的标记量
	HybridModelCrowd* hmcrowd;
public:
	QuadTree(HybridModelCrowd* hmc, int l, int ld, QuadTree *p, CRectangle<double> r);
	void MergeQuadTree();
	void InitNeighbor();
	QuadTreeState UpdateQuadTree();
	

public:
	static int maxlayer;
	
};

#endif