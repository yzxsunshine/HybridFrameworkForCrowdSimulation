#pragma once

bool init(int maxAgents, float renderAgentRadius, float gridSize, int vn, float* verts, int fn, int* inds);

int addAgent(float* pos
	, float maxNeighborDist, int maxNeighborNum, float planHorizon, float radius, float maxSpeed	// simulation parameters
	, float* target = 0, float* vel = 0, int color = 0);

/// Updates the specified agent's configuration.
///  @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
///  @param[in]		params	The new agent configuration.
void updateAgentParameters(int idx, float maxNeighborDist, int maxNeighborNum, float planHorizon, float radius, float maxSpeed);

/// Removes the agent from the crowd.
///  @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
void removeAgent(int idx);

/// Updates the steering and positions of all agents.
///  @param[in]		dt		The time, in seconds, to update the simulation. [Limit: > 0]
///  @param[out]	debug	A debug object to load with debug information. [Opt]
void update(float dt, int agentNum, int* agentIds, float* positions, float* velocities);

void clear();