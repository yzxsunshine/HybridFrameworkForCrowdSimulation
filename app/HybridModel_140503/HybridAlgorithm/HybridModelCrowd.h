#ifndef HYBRIDMODELCROWD_H
#define HYBRIDMODELCROWD_H

#include "HmAgent.h"
#include "QuadTree.h"
#include "Grid.h"
#include "CrowdGroup.h"
#include <vector>

#include "DetourCrowd.h"
#include "DetourNavMeshQuery.h"
#include "DetourLocalBoundary.h"
#include "DetourPathCorridor.h"
#include "DetourPathQueue.h"
#include "RVO/RVOSimulator.h"
#include "FluidSimulator.h"
#include "../Include/NavMesh.h"

class QuadTree;
class Grid;
class CrowdGroup;
class FluidSimulator;
class HybridModelCrowd
{
public:
	HmAgent* m_agents;	//用数组存储操作效率高一些
	HmAgent** m_activeAgents;
	std::vector<RVO::Agent*> m_rvoagents;
	std::vector<HmAgent*> m_selected;
	Grid* m_grids;	//fluid grid
	Grid* m_defaultGrid;
	Grid** m_activeGrids;	//fluid grid
	QuadTree* m_root;
	std::vector<CrowdGroup> m_fluidgroups;	//唯一一个大小不确定的变量，用vector表示
	float m_gridsize;
	int m_cellsize;
	float m_min[2];
	float m_max[2];
	int m_numofgridinrow;//每行的格子数量
	int agent_num;
	//dtPolyRef* m_pathResult;
	int m_maxPathResult;
	
	float m_ext[3];
	//dtQueryFilter m_filter;
	
	float m_maxAgentRadius;
	int m_maxAgents;
	int m_maxGrids;
	int m_velocitySampleCount;
	float m_densityThreshold;	//格子稠密的阈值
	//dtNavMeshQuery* m_navquery;

	//dtPathQueue m_pathq;

	//dtProximityGrid* m_grid;

	RVO::RVOSimulator* m_rvosim;
	FluidSimulator* m_fluidsim;
public:
	HybridModelCrowd();
	~HybridModelCrowd();
	/// Initializes the crowd.  
	///  @param[in]		maxAgents		The maximum number of agents the crowd can manage. [Limit: >= 1]
	///  @param[in]		maxAgentRadius	The maximum radius of any agent that will be added to the crowd. [Limit: > 0]
	///  @param[in]		nav				The navigation mesh to use for planning.
	/// @return True if the initialization succeeded.
	bool init(const int maxAgents, const float maxAgentRadius, NavMesh* nav, const float grid_size, const float ox, const float oy,const float ex, const float ey);
	
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
	int addAgent(const float* pos, const dtCrowdAgentParams* params);

	/// Updates the specified agent's configuration.
	///  @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
	///  @param[in]		params	The new agent configuration.
	void updateAgentParameters(const int idx, const dtCrowdAgentParams* params);

	/// Removes the agent from the crowd.
	///  @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
	void removeAgent(const int idx);
	
	/// Submits a new move request for the specified agent.
	///  @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
	///  @param[in]		ref		The position's polygon reference.
	///  @param[in]		pos		The position within the polygon. [(x, y, z)]
	/// @return True if the request was successfully submitted.
	bool requestMoveTarget(const int idx, dtPolyRef ref, const float* pos);

	/// Submits a new move request for the specified agent.
	///  @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
	///  @param[in]		vel		The movement velocity. [(x, y, z)]
	/// @return True if the request was successfully submitted.
	bool requestMoveVelocity(const int idx, const float* vel);

	/// Resets any request for the specified agent.
	///  @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
	/// @return True if the request was successfully reseted.
	bool resetMoveTarget(const int idx);

	/// Gets the active agents int the agent pool.
	///  @param[out]	agents		An array of agent pointers. [(#dtCrowdAgent *) * maxAgents]
	///  @param[in]		maxAgents	The size of the crowd agent array.
	/// @return The number of agents returned in @p agents.
	int getActiveAgents(dtCrowdAgent** agents, const int maxAgents);

	/// Updates the steering and positions of all agents.
	///  @param[in]		dt		The time, in seconds, to update the simulation. [Limit: > 0]
	///  @param[out]	debug	A debug object to load with debug information. [Opt]
	void update(const float dt);
	
	/// Gets the filter used by the crowd.
	/// @return The filter used by the crowd.
//	const dtQueryFilter* getFilter() const { return &m_filter; }

	/// Gets the filter used by the crowd.
	/// @return The filter used by the crowd.
//	dtQueryFilter* getEditableFilter() { return &m_filter; }

	/// Gets the search extents [(x, y, z)] used by the crowd for query operations. 
	/// @return The search extents used by the crowd. [(x, y, z)]
	const float* getQueryExtents() const { return m_ext; }
	
	/// Gets the velocity sample count.
	/// @return The velocity sample count.
	inline int getVelocitySampleCount() const { return m_velocitySampleCount; }
	

	/// Gets the crowd's path request queue.
	/// @return The crowd's path request queue.
//	const dtPathQueue* getPathQueue() const { return &m_pathq; }

	/// Gets the query object used by the crowd.
//	const dtNavMeshQuery* getNavMeshQuery() const { return m_navquery; }

	void updateTopologyOptimization(dtCrowdAgent** agents, const int nagents, const float dt);
	void updateMoveRequest(const float dt);
	
};

#endif