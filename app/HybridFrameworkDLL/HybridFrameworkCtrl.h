#pragma once

#include "HmAgent.h"
#include "Group.h"
#include "CrowdGroup.h"
#include <vector>
#include "TileCtrl.h"
#include "RVO/RVOSimulator.h"
#include "FluidSimulator.h"

class HybridFrameworkCtrl
{
public:
	HybridFrameworkCtrl();
	~HybridFrameworkCtrl();

public:
	HmAgent* m_agents;	//all the agents
	HmAgent** m_activeAgents;	// pointer to agents
	std::vector<HmAgent*> m_selected;	// selected agents
	std::vector<CrowdGroup> m_densegroups;	//dense groups
	float m_groupsize;
	int m_cellsize;
	float m_min[2];
	float m_max[2];
	int m_numofgridinrow;//
	int agent_num;

	float m_maxAgentRadius;
	int m_maxAgents;
	int m_maxGrids;
	float m_densityThreshold;	//格子稠密的阈值
	std::vector<Group>* m_groups;
	std::vector<int> m_actGridIds;
	TileCtrl* m_tileCtrl;

	RVO::RVOSimulator* m_rvosim;	// agent agent rvo
	RVO::RVOSimulator* m_obstacleRVOSim;	// group group rvo
	RVO::RVOSimulator* m_groupRVOSim;	// agent group rvo
	FluidSimulator* m_fluidsim;

public:
	/// Initializes the crowd.  
	///  @param[in]		maxAgents		The maximum number of agents the crowd can manage. [Limit: >= 1]
	///  @param[in]		maxAgentRadius	The maximum radius of any agent that will be added to the crowd. [Limit: > 0]
	///  @param[in]		nav				The navigation mesh to use for planning.
	/// @return True if the initialization succeeded.
	bool init(const int maxAgents, const float maxAgentRadius, const float grid_size, const float ox, const float oy, const float ex, const float ey);

	void purge();
	/// Gets the specified agent from the pool.
	///	 @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
	/// @return The requested agent.
	HmAgent* getAgent(const int idx);
	int getActiveAgents(HmAgent** agents, const int maxAgents);
	/// The maximum number of agents that can be managed by the object.
	/// @return The maximum number of agents.
	const int getAgentCount() const;

	/// Adds a new agent to the crowd.
	///  @param[in]		pos		The requested position of the agent. [(x, y, z)]
	///  @param[in]		params	The configutation of the agent.
	/// @return The index of the agent in the agent pool. Or -1 if the agent could not be added.
	int addAgent(const float* pos, const dtCrowdAgentParams* params, SimulationParams& simParams, float* target = NULL, float* vel = NULL, int color = 0);

	/// Updates the specified agent's configuration.
	///  @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
	///  @param[in]		params	The new agent configuration.
	void updateAgentParameters(const int idx, const dtCrowdAgentParams* params);

	/// Removes the agent from the crowd.
	///  @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
	void removeAgent(const int idx);

	/// Updates the steering and positions of all agents.
	///  @param[in]		dt		The time, in seconds, to update the simulation. [Limit: > 0]
	///  @param[out]	debug	A debug object to load with debug information. [Opt]
	void update(const float dt);

	void save(FILE* fp);

	void MergeGrid(void);
	void Rearrange(void);

	void SetDensityThreshold(float thresh) {
		m_densityThreshold = thresh;
		RVO::Agent::SetDensity(thresh, 1.0f);
	}
};

