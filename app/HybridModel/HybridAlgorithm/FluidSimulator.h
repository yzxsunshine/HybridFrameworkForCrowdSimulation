#pragma once
#include "Vector2.h"
#include "HmAgent.h"
#include "CrowdGroup.h"
#include "Group.h"


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
	void init(std::vector<Group>* groups, HmAgent** agents, std::vector<CrowdGroup>* fluid);
	void doStep();

	void setXLabelNum(int ln) { m_xLabelNum = ln; }
	int getXLabelNum(void) { return m_xLabelNum; };

	void setYLabelNum(int ln) { m_yLabelNum = ln; }
	int getYLabelNum(void) { return m_yLabelNum; };

	void setXYLabelNum(int xl, int yl) { m_xLabelNum = xl; m_xLabelNum = yl; }

	void setDataCostScale(int dcs) { m_dataCostScale = dcs; }
	int getDataCostScale(void) { return m_dataCostScale; }
	void setSmoothCostScale(int scs) { m_smoothCostScale = scs; }
	int getSmoothCostScale(void) { return m_smoothCostScale; }

	float* getXLabels(void) { return m_xLabels; }
	const float& getXLabel(int l) { return m_xLabels[l]; }
	float* getYLabels(void) { return m_yLabels; }
	const float& getYLabel(int l) { return m_yLabels[l]; }
	
	std::vector<Group>* getGroups(void) { return m_groups; }
	Group& getGroup(int gridID) { return (*m_groups)[gridID]; }
	const Group& getGroupConst(int gridID) { return (*m_groups)[gridID]; }

	std::vector<CrowdGroup>* getCrowdGroups(void) { return m_fluidGroups; }
	CrowdGroup& getCrowdGroups(int groupID) { return (*m_fluidGroups)[groupID]; }
	const CrowdGroup& getCrowdGroup(int groupID) { return (*m_fluidGroups)[groupID]; }

	int getMaxCost(void) {return m_maxCost;}
	void setMaxCost(int cost) { m_maxCost = cost; }

	int getCurGroupID(void) { return m_curGroup; }
	//void rearrange();
	Vector2 GetGroupVelocity(int gridid);
	void GetGroupTraits(int gridid, float& pc1, float& pc2);

	void SetTraitCoeff(float alpha, float beta);
	float GetAlpha() { return m_alpha; }
	float GetBeta() { return m_beta; }
protected:
	std::vector<Group>* m_groups;
	HmAgent** m_agents;
	std::vector<CrowdGroup>* m_fluidGroups;
	
	int m_xLabelNum;
	int m_yLabelNum;
	float* m_xLabels;
	float* m_yLabels;

	int m_curGroup;
	int m_dataCostScale;
	int m_smoothCostScale;
	int m_lastCallEM;

	int m_maxCost;

	// converting gorup personality pc1 and pc2 to simulation parameter by linear equation pi = alpha*pc1 + beta*pc2
	static float m_alpha;	
	static float m_beta;
};
