#include "HybridFrameworkAPI.h"
#include "HybridFrameworkCtrl.h"

HybridFrameworkCtrl ctrl;
bool init(int maxAgents, float renderAgentRadius, float gridSize, int vn, float* verts, int fn, int* inds)
{
	return ctrl.init(maxAgents, renderAgentRadius, gridSize, vn, verts, fn, inds);
}

int addAgent(float* pos
	, float maxNeighborDist, int maxNeighborNum, float planHorizon, float radius, float maxSpeed	// simulation parameters
	, float* target, float* vel, int color)
{
	return ctrl.addAgent(pos, maxNeighborDist, maxNeighborNum, planHorizon, radius, maxSpeed, target, vel, color);
}

/// Updates the specified agent's configuration.
///  @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
///  @param[in]		params	The new agent configuration.
void updateAgentParameters(int idx, float maxNeighborDist, int maxNeighborNum, float planHorizon, float radius, float maxSpeed)
{
	return ctrl.updateAgentParameters(idx, maxNeighborDist, maxNeighborNum, planHorizon, radius, maxSpeed);
}

/// Removes the agent from the crowd.
///  @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
void removeAgent(int idx)
{
	return ctrl.removeAgent(idx);
}

/// Updates the steering and positions of all agents.
///  @param[in]		dt		The time, in seconds, to update the simulation. [Limit: > 0]
///  @param[out]	debug	A debug object to load with debug information. [Opt]
void update(float dt, int agentNum, int* agentIds, float* positions, float* velocities, float* rvoTime, float* GCTime, float* totalTime)
{
	return ctrl.update(dt, agentNum, agentIds, positions, velocities, rvoTime, GCTime, totalTime);
}

void clear() 
{
	ctrl.purge();
}

void setDensityThreshold(float thresh) {
	ctrl.SetDensityThreshold(thresh);
}

void setAgentCorridor(int aid, int cornerNum, float* corners)
{
	ctrl.SetAgentCorridor(aid, cornerNum, corners);
}