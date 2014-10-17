#pragma once
#include "Vector2.h"
#include "HmAgent.h"
#include "CrowdGroup.h"
#include "Grid.h"


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
	void init(std::vector<Grid>* grids, HmAgent** agents, std::vector<CrowdGroup>* fluid);
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
	
	std::vector<Grid>* getGrids(void) { return m_grids; }
	Grid& getGrid(int gridID) { return (*m_grids)[gridID]; }
	const Grid& getGridConst(int gridID) { return (*m_grids)[gridID]; }

	std::vector<CrowdGroup>* getGroups(void) { return m_fluidGroups; }
	CrowdGroup& getGroup(int groupID) { return (*m_fluidGroups)[groupID]; }
	const CrowdGroup& getGroupConst(int groupID) { return (*m_fluidGroups)[groupID]; }

	int getMaxCost(void) {return m_maxCost;}
	void setMaxCost(int cost) { m_maxCost = cost; }

	int getCurGroupID(void) { return m_curGroup; }
	//void rearrange();
	Vector2 GetGridVelocity(int gridid);
protected:
	std::vector<Grid>* m_grids;
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
};
