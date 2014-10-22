#include "HybridFrameworkAPI.h"

#ifdef _WINDLL
#if _MSC_VER // this is defined when compiling with Visual Studio
#define EXPORT_API __declspec(dllexport) // Visual Studio needs annotating exported functions with this
#else
#define EXPORT_API // XCode does not need annotating exported functions, so define is empty
#endif

extern "C"
{
	extern EXPORT_API bool Init(int maxAgents, float renderAgentRadius, float gridSize, int vn, float* verts, int fn, int* inds);

	extern EXPORT_API int AddAgent(float* pos
		, float maxNeighborDist, int maxNeighborNum, float planHorizon, float radius, float maxSpeed	// simulation parameters
		, float* target, float* vel, int color);

	/// Updates the specified agent's configuration.
	///  @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
	///  @param[in]		params	The new agent configuration.
	extern EXPORT_API void UpdateAgentParameters(int idx, float maxNeighborDist, int maxNeighborNum, float planHorizon, float radius, float maxSpeed);

	/// Removes the agent from the crowd.
	///  @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
	extern EXPORT_API void RemoveAgent(int idx);

	/// Updates the steering and positions of all agents.
	///  @param[in]		dt		The time, in seconds, to update the simulation. [Limit: > 0]
	///  @param[out]	debug	A debug object to load with debug information. [Opt]
	extern EXPORT_API void Update(float dt, int agentNum, int* agentIds, float* positions, float* velocities, float* rvoTime, float* GCTime, float* totalTime);

	extern EXPORT_API void Clear();

	extern EXPORT_API void SetDensityThreshold(float thresh);

	extern EXPORT_API void SetAgentCorridor(int aid, int cornerNum, float* corners);

	bool Init(int maxAgents, float renderAgentRadius, float gridSize, int vn, float* verts, int fn, int* inds)
	{
		return init(maxAgents, renderAgentRadius, gridSize, vn, verts, fn, inds);
	}

	int AddAgent(float* pos
		, float maxNeighborDist, int maxNeighborNum, float planHorizon, float radius, float maxSpeed	// simulation parameters
		, float* target, float* vel, int color)
	{
		return addAgent(pos, maxNeighborDist, maxNeighborNum, planHorizon, radius, maxSpeed, target, vel, color);
	}

	/// Updates the specified agent's configuration.
	///  @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
	///  @param[in]		params	The new agent configuration.
	void UpdateAgentParameters(int idx, float maxNeighborDist, int maxNeighborNum, float planHorizon, float radius, float maxSpeed)
	{
		return updateAgentParameters(idx, maxNeighborDist, maxNeighborNum, planHorizon, radius, maxSpeed);
	}

	/// Removes the agent from the crowd.
	///  @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
	void RemoveAgent(int idx)
	{
		return removeAgent(idx);
	}

	/// Updates the steering and positions of all agents.
	///  @param[in]		dt		The time, in seconds, to update the simulation. [Limit: > 0]
	///  @param[out]	debug	A debug object to load with debug information. [Opt]
	void Update(float dt, int agentNum, int* agentIds, float* positions, float* velocities, float* rvoTime, float* GCTime, float* totalTime)
	{
		return update(dt, agentNum, agentIds, positions, velocities, rvoTime, GCTime, totalTime);
	}

	void Clear()
	{
		clear();
	}

	void SetDensityThreshold(float thresh)
	{
		setDensityThreshold(thresh);
	}

	void SetAgentCorridor(int aid, int cornerNum, float* corners)
	{
		setAgentCorridor(aid, cornerNum, corners);
	}
}
#endif