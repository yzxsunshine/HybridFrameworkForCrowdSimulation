#pragma once
#include "QuadTree.h"
#include "Vector2.h"
#include "HybridModelCrowd.h"
class HybridModelCrowd;

struct elem_grid {	//�������ṹ�����������Ų���Ⱥ��ʱ������
	int id;
	//int r,c;
	Vector2 center;
	float score;
	HmAgent* agent;
};
/*
���Һ���	f(x) = a*cos(b*x+c)+d
peakΪ����߶ȣ���y�����ű�����cycleΪx�����ű����������ڣ�xtransΪx��ƽ�ƣ�ytransΪy��ƽ��
*/
struct crowdTemplate
{
	float peak;
	float cycle;
	float xtrans;
	float ytrans;
	int area;
	Grid* grid;
	int type;//0Ϊ���ң�1Ϊ����  
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
