#ifndef HMAGENT_H
#define HMAGENT_H

#include "Vector2.h"
#include "RVO/Agent.h"
#include "RVO/RVOSimulator.h"
#include "DetourCrowd.h"
#include <vector>
#include "../NavCorridor.h"
#include "../CommonDef.h"

using namespace std;
class HmAgent:public RVO::Agent, public dtCrowdAgent
{
public:
	HmAgent();
	HmAgent(RVO::RVOSimulator* sim);
	HmAgent(RVO::RVOSimulator* sim,int p_id,int g_former_id,int g_now_id,double pos_x,double pos_y);
	~HmAgent(void);
	void setHmAgent(float pc1, float pc2);
	void setHmAgent(double neighborDist, size_t maxNeighbors, double timeHorizon, double timeHorizonObst, double radius, double maxSpeed);
	void setHmAgent(const HmAgent& HmAgent);
	void setHmAgent(const SimulationParams& simParams);
	void setHmAgent(const float* position, const float* vel );
public:
	int HmAgent_id;
	int grid_former_id;//人以前在的格子
	int grid_now_id;//人现在在的格子
	int simulate_type;	//0 fluid; 1 rvo
	int color_id;
	bool is_contour;
	bool is_selected;
	int targetFace;
	int curFace;
	int curGrid;
	int curTile;
	NavCorridor corridor;
	float density;

	float m_pc1;
	float m_pc2;

public:
	static float adjMat[2][5];
};

#endif
