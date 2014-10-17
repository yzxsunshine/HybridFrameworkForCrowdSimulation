#ifndef HMAGENT_H
#define HMAGENT_H

#include "Vector2.h"
#include "RVO/Agent.h"
#include "RVO/RVOSimulator.h"
#include <vector>
#include "../NavCorridor.h"
#include "../CommonDef.h"

using namespace std;

static const int DT_CROWDAGENT_MAX_CORNERS = 4;

enum CrowdAgentState
{
	DT_CROWDAGENT_STATE_INVALID,		///< The agent is not in a valid state.
	DT_CROWDAGENT_STATE_WALKING,		///< The agent is traversing a normal navigation mesh polygon.
	DT_CROWDAGENT_STATE_OFFMESH,		///< The agent is traversing an off-mesh connection.
};

/// Configuration parameters for a crowd agent.
/// @ingroup crowd
struct dtCrowdAgentParams
{
	float radius;						///< Agent radius. [Limit: >= 0]
	float height;						///< Agent height. [Limit: > 0]
	float maxAcceleration;				///< Maximum allowed acceleration. [Limit: >= 0]
	float maxSpeed;						///< Maximum allowed speed. [Limit: >= 0]

	/// Defines how close a collision element must be before it is considered for steering behaviors. [Limits: > 0]
	float collisionQueryRange;

	float pathOptimizationRange;		///< The path visibility optimization range. [Limit: > 0]

	/// How aggresive the agent manager should be at avoiding collisions with this agent. [Limit: >= 0]
	float separationWeight;

	/// Flags that impact steering behavior. (See: #UpdateFlags)
	unsigned char updateFlags;

	/// The index of the avoidance configuration to use for the agent. 
	/// [Limits: 0 <= value <= #DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS]
	unsigned char obstacleAvoidanceType;

	/// User defined data attached to the agent.
	void* userData;
};

enum MoveRequestState
{
	DT_CROWDAGENT_TARGET_NONE = 0,
	DT_CROWDAGENT_TARGET_FAILED,
	DT_CROWDAGENT_TARGET_VALID,
	DT_CROWDAGENT_TARGET_REQUESTING,
	DT_CROWDAGENT_TARGET_WAITING_FOR_QUEUE,
	DT_CROWDAGENT_TARGET_WAITING_FOR_PATH,
	DT_CROWDAGENT_TARGET_VELOCITY,
};

class HmAgent:public RVO::Agent
{
public:
	HmAgent();
	HmAgent(RVO::RVOSimulator* sim);
	HmAgent(RVO::RVOSimulator* sim,int p_id,int g_former_id,int g_now_id,double pos_x,double pos_y);
	~HmAgent(void);
	
	void setHmAgent(double neighborDist, size_t maxNeighbors, double timeHorizon, double timeHorizonObst, double radius, double maxSpeed);
	void setHmAgent(const HmAgent& HmAgent);
	void setHmAgent(const SimulationParams& simParams);
	void setHmAgent(const float* position, const float* vel );
public:
	int HmAgent_id;
	int grid_former_id; //人以前在的格子
	int grid_now_id; //人现在在的格子
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

	/// 1 if the agent is active, or 0 if the agent is in an unused slot in the agent pool.
	unsigned char active;

	/// The type of mesh polygon the agent is traversing. (See: #CrowdAgentState)
	unsigned char state;

	/// The desired speed.
	float desiredSpeed;

	float npos[3];		///< The current agent position. [(x, y, z)]
	float disp[3];
	float dvel[3];		///< The desired velocity of the agent. [(x, y, z)]
	float nvel[3];
	float vel[3];		///< The actual velocity of the agent. [(x, y, z)]

	/// The agent's configuration parameters.
	dtCrowdAgentParams params;

	/// The local path corridor corners for the agent. (Staight path.) [(x, y, z) * #ncorners]
	float cornerVerts[DT_CROWDAGENT_MAX_CORNERS * 3];

	/// The local path corridor corner flags. (See: #dtStraightPathFlags) [(flags) * #ncorners]
	unsigned char cornerFlags[DT_CROWDAGENT_MAX_CORNERS];

	/// The number of corners.
	int ncorners;

	unsigned char targetState;			///< State of the movement request.
	float targetPos[3];					///< Target position of the movement request (or velocity in case of DT_CROWDAGENT_TARGET_VELOCITY).

};

#endif
