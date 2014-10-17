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
};//����״̬���壬�׺ڻ�

/* a quadrant defined below:
 
������������������ 
 
�������������� NW(0)���� |������ NE(1)
 
�������������� -----------|-----------
 
�������������� SW(3)���� |������ SE(2)
 
*/
 

class QuadTree
{
public:
	QuadTree(void);
	~QuadTree(void);
public:
	QuadTree *node[4]; //�ӽڵ�
	QuadTree *neighbor[4];//�ĸ�������ھӽڵ� 0,1,2,3 ��,��,��,��
	int layernum;
	QuadTreeState leaf_state;//�ڵ��״̬��QuadTreeState
	QuadTreeState former_leaf_state;
    int leaf_direction;//�ڵ�ķ��򣬼�ʹ���ڵ���ĸ��ӽڵ�
    long int gridid;//��Ӧ���ӵ�id���������Ҷ�ӽڵ���ֵΪ-1
	Grid* grid;
	QuadTree *parent;
	CRectangle<double> rect;
	long int RegionMark;	//�ж�������ͨ�ı����
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