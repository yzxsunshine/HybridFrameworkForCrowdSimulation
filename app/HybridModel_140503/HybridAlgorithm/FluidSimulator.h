#pragma once
#include "QuadTree.h"
#include "Vector2.h"
#include "HybridModelCrowd.h"
class HybridModelCrowd;

struct elem_grid {	//这两个结构用来在重新排布人群的时候排序
	int id;
	//int r,c;
	Vector2 center;
	float score;
	HmAgent* agent;
};
/*
正弦函数	f(x) = a*cos(b*x+c)+d
peak为波峰高度，即y轴缩放比例；cycle为x轴缩放比例，即周期；xtrans为x轴平移；ytrans为y轴平移
*/
struct crowdTemplate
{
	float peak;
	float cycle;
	float xtrans;
	float ytrans;
	int area;
	Grid* grid;
	int type;//0为正弦，1为钟形  
	std::vector<elem_grid> elem_grids;
};

/*struct agent_grid {
	int id;
	Vector2 center;
	float score;
};*/

class FluidSimulator
{
public:
	FluidSimulator(void);
	~FluidSimulator(void);
	void init(HybridModelCrowd* hmc);
	void doStep();
	void rearrange();
	void generateTemplate();
	Vector2 GetGridVelocity(QuadTree* node);
	Vector2 GetGridVelocity(int gridid);
public:
	std::vector<crowdTemplate> m_templates;
protected:
	HybridModelCrowd* m_hmcrowd;
};
