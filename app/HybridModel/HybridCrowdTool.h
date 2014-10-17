#pragma once
#include "HybridModelCrowd.h"
#include "Sample.h"
#include "CommonDef.h"



struct HybridCrowdToolParams
{
	bool m_expandSelectedDebugDraw;
	bool m_expandAgentColor;
	bool m_expandSimulationParameters;
	int m_selectedAgentColor;
	bool m_showCorners;
	bool m_showCollisionSegments;
	bool m_showPath;
	bool m_showVO;
	bool m_showOpt;
	bool m_showNeis;
	bool m_showContour;
	bool m_showGridLabels;
	bool m_expandDebugDraw;
	bool m_showLabels;
	bool m_showNodes;
	bool m_showPerfGraph;
	bool m_showDetailAll;
	
	bool m_expandOptions;
	bool m_anticipateTurns;
	bool m_optimizeVis;
	bool m_optimizeTopo;
	bool m_obstacleAvoidance;
	float m_obstacleAvoidanceType;
	bool m_separation;
	float m_separationWeight;
	bool m_showVelocityArrow;
	//HybridAlgorithmParams
	int m_gridSize;	//格子大小
	SimulationParams m_simParams;

};


class HybridCrowdToolState : public SampleToolState
{
	Sample* m_sample;
	NavMesh* m_nav;
	HybridModelCrowd* m_crowd;
	static const int AGENT_MAX_TRAIL = 64;
	static const int MAX_AGENTS = 20000;	//最多人数
	struct AgentTrail
	{
		float trail[AGENT_MAX_TRAIL*3];
		int htrail;
	};
	float m_targetPos[3];
	int m_targetFace;
	//dtPolyRef m_targetRef;
	AgentTrail m_trails[MAX_AGENTS];
	

	HybridCrowdToolParams m_toolParams;
	
	bool m_run;
public:
	HybridCrowdToolState();
	virtual ~HybridCrowdToolState();
	
	virtual void init(class Sample* sample);
	virtual void reset();
	virtual void handleRender();
	virtual void handleRenderOverlay(double* proj, double* model, int* view);
	virtual void handleUpdate(const float dt);

	void addAgent(const float* pos, int color = 0, SimulationParams simParams = SimulationParams());
	void removeAgent(const int idx);
	void hilightAgent(const int idx);
	void updateAgentParams();
	int hitTestAgents(const float* s, const float* p);
	void setMoveTarget(const float* p, bool adjust);
	void updateTick(const float dt);

	inline bool isRunning() const { return m_run; }
	inline void setRunning(const bool s) { m_run = s; }

	inline HybridCrowdToolParams* getToolParams() { return &m_toolParams; }
};


class HybridCrowdTool : public SampleTool
{
public:
	Sample* m_sample;
	HybridCrowdToolState* m_state;
	
	enum ToolMode
	{
		TOOLMODE_CREATE,
		TOOLMODE_MOVE_TARGET,
		TOOLMODE_SELECT,
		TOOLMODE_TOGGLE_POLYS,
	};
	ToolMode m_mode;
	float m_randomCrowdNumber;
	float m_densityThreshold;
	void updateAgentParams();
	void updateTick(const float dt);
public:
	HybridCrowdTool(void);
	virtual ~HybridCrowdTool(void);

	virtual int type() { return TOOL_HYBRIDCROWD; }
	virtual void init(Sample* sample);
	virtual void reset();
	virtual void handleMenu();
	virtual void handleClick(const float* s, const float* p, bool shift);
	virtual void handleDrag(const float* tl, const float* tr,const float* bl, const float* br);
	virtual void handleToggle();
	virtual void handleStep();
	virtual void handleUpdate(const float dt);
	virtual void handleRender();
	virtual void handleRenderOverlay(double* proj, double* model, int* view);
};
