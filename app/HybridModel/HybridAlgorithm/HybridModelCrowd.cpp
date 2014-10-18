#include "HybridModelCrowd.h"
#include "DetourCommon.h"
#include "DetourAssert.h"
#include "../PerfTimer.h"
#include "../CommonDef.h"
#include <list>

static const int MAX_ITERS_PER_UPDATE = 100;

static const int MAX_PATHQUEUE_NODES = 4096;
static const int MAX_COMMON_NODES = 512;
static const int ITER_REARRANGE = 10;

static void integrate(HmAgent* ag, const float dt)
{
	// Fake dynamic constraint.
	const float maxDelta = ag->params.maxAcceleration * dt;
	float dv[3];
	dtVsub(dv, ag->nvel, ag->vel);
	float ds = dtVlen(dv);
	if (ds > maxDelta)
		dtVscale(dv, dv, maxDelta/ds);
	dtVadd(ag->vel, ag->vel, dv);
	
	// Integrate
	if (dtVlen(ag->vel) > 0.0001f)
		dtVmad(ag->npos, ag->npos, ag->vel, dt);
	else
		dtVset(ag->vel,0,0,0);
}

static float getDistanceToGoal(const HmAgent* ag, const float range)
{
	if (!ag->ncorners)
		return range;
	
	const bool endOfPath = (ag->cornerFlags[ag->ncorners-1]) ? true : false;
	if (endOfPath)
		return dtMin(dtVdist2D(ag->npos, &ag->cornerVerts[(ag->ncorners-1)*3]), range);
	
	return range;
}

static void calcSmoothSteerDirection(const HmAgent* ag, float* dir)
{
	if (!ag->ncorners)
	{
		dtVset(dir, 0,0,0);
		return;
	}
	
	const int ip0 = 0;
	const int ip1 = dtMin(1, ag->ncorners-1);
	const float* p0 = &ag->cornerVerts[ip0*3];
	const float* p1 = &ag->cornerVerts[ip1*3];
	
	float dir0[3], dir1[3];
	dtVsub(dir0, p0, ag->npos);
	dtVsub(dir1, p1, ag->npos);
	dir0[1] = 0;
	dir1[1] = 0;
	
	float len0 = dtVlen(dir0);
	float len1 = dtVlen(dir1);
	if (len1 > 0.001f)
		dtVscale(dir1,dir1,1.0f/len1);
	
	dir[0] = dir0[0] - dir1[0]*len0*0.5f;
	dir[1] = 0;
	dir[2] = dir0[2] - dir1[2]*len0*0.5f;
	
	dtVnormalize(dir);
}

static void calcStraightSteerDirection(const HmAgent* ag, float* dir)
{
	if (!ag->ncorners)
	{
		dtVset(dir, 0,0,0);
		return;
	}
	dtVsub(dir, &ag->cornerVerts[0], ag->npos);
	dir[1] = 0;
	dtVnormalize(dir);
}

int getNearestLog(int& num)
{
	int tmp = num;
	int n = 0;
	while(tmp>2)
	{
		int tmp_2 = tmp>>1;
		tmp_2 = tmp_2<<1;
		if(tmp-tmp_2!=0)
		{
			tmp++;
			num += 1<<n;
		}
		tmp = tmp>>1;	
		n++;
	}
	return ++n;
}

HybridModelCrowd::HybridModelCrowd():m_agents(0), m_maxAgents(0), m_activeAgents(0)
												, m_navquery(0), m_rvosim(0), m_fluidsim(0), m_maxGrids(0)
												, m_groupRVOSim(0), m_obstacleRVOSim(0), m_groups(0)
{

}

HybridModelCrowd::~HybridModelCrowd()
{
	purge();
}


void HybridModelCrowd::purge()
{
	if(m_agents!=0)
		delete[] m_agents;
	m_agents = 0;
	m_maxAgents = 0;

	if(m_activeAgents!=0)
		free(m_activeAgents);
	m_activeAgents = 0;

	if(m_fluidsim!=0)
		delete m_fluidsim;
	m_fluidsim = 0;

	if(m_rvosim!=0)
		delete m_rvosim;
	if(m_groupRVOSim!=0)
		delete m_groupRVOSim;
	if(m_obstacleRVOSim!=0)
		delete m_obstacleRVOSim;

	if (m_groups != 0)
		delete m_groups;

	m_groupRVOSim = 0;
	m_obstacleRVOSim = 0;
	m_rvosim = 0;
	m_maxGrids = 0;
	
}

bool HybridModelCrowd::init(const int maxAgents, const float maxAgentRadius, NavMesh* nav, NavMeshQuery* nmq, const float grid_size, const float ox, const float oy, const float ex, const float ey)
{
	purge();
	
	m_maxAgents = maxAgents;
	m_maxAgentRadius = maxAgentRadius;
	m_groupsize = grid_size;	//真实比例
	m_cellsize = (int)(grid_size/maxAgentRadius/2);	//h
	m_min[0] = ox;	m_min[1] = oy;	m_max[0] = ex;	m_max[1] = ey;

	m_navquery = nmq;
	m_tileCtrl = nmq->GetTileCtrl();
	int groupNum = m_tileCtrl->m_grids.size();
	m_groups = new std::vector < Group > ;
	(*m_groups).resize(groupNum);
	for (int i = 0; i < groupNum; i++)
	{
		(*m_groups)[i] = m_tileCtrl->m_grids[i].m_group;
	}
	m_rvosim = new RVO::RVOSimulator();
	m_groupRVOSim = new RVO::RVOSimulator();
	m_obstacleRVOSim = new RVO::RVOSimulator();
	agent_num = 0;
	// (pi*r^2/2) / maxAgentRadius;
	m_rvosim->setAgentDefaults(grid_size, PI*grid_size*grid_size / 8 / maxAgentRadius, 5.0f, grid_size*10.0f, maxAgentRadius, MAX_SPEED/*max speed*/);
	m_groupRVOSim->setAgentDefaults(grid_size, PI*grid_size*grid_size/8/maxAgentRadius, 15.0f, 15.0f, maxAgentRadius, MAX_SPEED/*max speed*/);
	//m_obstacleRVOSim->setAgentDefaults(grid_size*10.0f, 0, 5.0f, grid_size*10.0f, maxAgentRadius*8.0f, MAX_SPEED/*max speed*/);
	// Allocate temp buffer for merging paths.
	m_maxPathResult = 256;
	
	//Initialize Agents
	m_agents = new HmAgent[m_maxAgents];
	m_activeAgents = (HmAgent**)malloc(m_maxAgents*sizeof(HmAgent*));
	if (!m_agents)
		return false; 
	dtVset(m_ext, m_maxAgentRadius*2.0f,m_maxAgentRadius*1.5f,m_maxAgentRadius*2.0f);	

	for (int i = 0; i < m_maxAgents; ++i)
	{
		m_agents[i].sim_ = m_rvosim;
		m_agents[i].HmAgent_id = i;
		m_agents[i].active = 0;
		m_agents[i].corridor.ClearPath();
	}

	//use navigation map to generate obstacles and grids
	float m_width = ex-ox; //这里必须是能够被2^n整除的数
	float m_height = ey-oy;
	float mapsize = (m_width>m_height)?m_width:m_height;

	m_numofgridinrow = float(ceil(mapsize / grid_size));
	mapsize = m_numofgridinrow*grid_size;
	for(unsigned int i=0; i<m_groups->size(); i++)
	{
		m_actGridIds.push_back(i);
	}

	m_fluidsim = new FluidSimulator();
	m_fluidsim->init(m_groups, m_activeAgents, &m_fluidgroups);
	

	return true;
}

const int HybridModelCrowd::getAgentCount() const
{
	return m_maxAgents;
}

/// @par
/// 
/// Agents in the pool may not be in use.  Check #dtCrowdAgent.active before using the returned object.
HmAgent* HybridModelCrowd::getAgent(const int idx)
{
	return &m_agents[idx];
}

void HybridModelCrowd::updateAgentParameters(const int idx, const dtCrowdAgentParams* params)
{
	if (idx < 0 || idx > m_maxAgents)
		return;
	memcpy(&m_agents[idx].params, params, sizeof(dtCrowdAgentParams));
}

/// @par
///
/// The agent's position will be constrained to the surface of the navigation mesh.
int HybridModelCrowd::addAgent(const float* pos, const dtCrowdAgentParams* params, SimulationParams& simParams, float* target, float* vel, int color)
{
	// Find empty slot.
	int idx = -1;
	for (int i = 0; i < m_maxAgents; ++i)
	{
		if (!m_agents[i].active)
		{
			idx = i;
			agent_num++;
			break;
		}
	}
	if (idx == -1)
		return -1;
	
	HmAgent* ag = &m_agents[idx];

	// Find nearest position on navmesh and place the agent there.
	float nearest[3];
	int tileID = -1;
	int faceID = -1;
	int gridID = -1;
	bool res = m_navquery->GetNearestFace(pos, tileID, gridID, faceID, nearest);
	ag->curGrid = gridID;
	ag->curFace = faceID;
	ag->curTile = tileID;
	ag->color_id = color;
	updateAgentParameters(idx, params);
	ag->setHmAgent(simParams);
	if(vel != NULL)
	{
		dtVset(ag->dvel, vel[0],vel[1],vel[2]);
		dtVset(ag->nvel, vel[0],vel[1],vel[2]);
		dtVset(ag->vel, vel[0],vel[1],vel[2]);
	}
	else
	{
		dtVset(ag->dvel, 0,0,0);
		dtVset(ag->nvel, 0,0,0);
		dtVset(ag->vel, 0,0,0);
	}
	dtVcopy(ag->npos, nearest);
	
	ag->desiredSpeed = 0;

	if (res)
		ag->state = DT_CROWDAGENT_STATE_WALKING;
	else
		ag->state = DT_CROWDAGENT_STATE_INVALID;
	
	ag->targetState = DT_CROWDAGENT_TARGET_NONE;
	if(target!=NULL)
	{
		vcg::Point3f posPt(pos[0], pos[1], pos[2]);
		vcg::Point3f targetPt(target[0], target[1], target[2]);
		std::vector<int> pathIds;
		std::vector<vcg::Point3f> pathPts;
		m_navquery->FindPath(posPt, targetPt, pathIds, pathPts);
		ag->corridor.SetCorridor(pathPts);
		ag->targetState = DT_CROWDAGENT_TARGET_VALID;
	}

	ag->active = 1;
	ag->HmAgent_id = idx;
	//Add Agent to Grid
	if(gridID<0||gridID>m_numofgridinrow*m_numofgridinrow){
		ag->active = 0;
		return -1;
	}
	ag->grid_now_id = gridID;
	(*m_groups)[gridID].agentsInGroup.push_back(idx);
	(*m_groups)[gridID].nowDensity = (*m_groups)[gridID].agentsInGroup.size()*1.0/(m_cellsize*m_cellsize);
	(*m_groups)[gridID].formerDensity = (*m_groups)[gridID].nowDensity;
	
	return idx;
}

/// @par
///
/// The agent is deactivated and will no longer be processed.  Its #dtCrowdAgent object
/// is not removed from the pool.  It is marked as inactive so that it is available for reuse.
void HybridModelCrowd::removeAgent(const int idx)
{
	if (idx >= 0 && idx < m_maxAgents)
	{
		m_agents[idx].active = 0;
		agent_num--;
	}
}

/// @par
/// 
/// This method is used when a new target is set.
/// 
/// The position will be constrained to the surface of the navigation mesh.
///
/// The request will be processed during the next #update().
bool HybridModelCrowd::requestMoveTarget(const int idx, int faceId, const float* pos)
{
	if (idx < 0 || idx > m_maxAgents)
		return false;
	if (faceId < 0)
		return false;

	HmAgent* ag = &m_agents[idx];
	
	// Initialize request.
	ag->targetFace = faceId;
	dtVcopy(ag->targetPos, pos);
	if (ag->targetFace >= 0)
		ag->targetState = DT_CROWDAGENT_TARGET_REQUESTING;
	else
		ag->targetState = DT_CROWDAGENT_TARGET_FAILED;

	return true;
}

bool HybridModelCrowd::requestMoveVelocity(const int idx, const float* vel)
{
	if (idx < 0 || idx > m_maxAgents)
		return false;
	
	HmAgent* ag = &m_agents[idx];
	
	// Initialize request.
	dtVcopy(ag->targetPos, vel);
	ag->targetState = DT_CROWDAGENT_TARGET_VELOCITY;
	
	return true;
}

bool HybridModelCrowd::resetMoveTarget(const int idx)
{
	if (idx < 0 || idx > m_maxAgents)
		return false;
	
	HmAgent* ag = &m_agents[idx];
	
	// Initialize request.
	dtVset(ag->targetPos, 0,0,0);
	ag->targetState = DT_CROWDAGENT_TARGET_NONE;
	
	return true;
}

int HybridModelCrowd::getActiveAgents(HmAgent** agents, const int maxAgents)
{
	int n = 0;
	for (int i = 0; i < m_maxAgents; ++i)
	{
		if (!m_agents[i].active) continue;
		if (n < maxAgents){
			agents[n++] = &m_agents[i];
			m_agents[i].active = 1;
		}
	}
	return n;
}

void HybridModelCrowd::updateMoveRequest(const float /*dt*/)
{
	const int PATH_MAX_AGENTS = 8;
	int nqueue = 0;
	
	// Fire off new requests.
	for (int i = 0; i < m_maxAgents; ++i)
	{
		HmAgent* ag = &m_agents[i];
		if (!ag->active)
			continue;
		if (ag->state == DT_CROWDAGENT_STATE_INVALID)
			continue;
		if (ag->targetState == DT_CROWDAGENT_TARGET_NONE || ag->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
			continue;

		if (ag->targetState == DT_CROWDAGENT_TARGET_REQUESTING)
		{
			//const dtPolyRef* path = ag->corridor.getPath();
			//const int npath = ag->corridor.getPathCount();
			//dtAssert(npath);

			static const int MAX_RES = 32;
			//float reqPos[3];
			//dtPolyRef reqPath[MAX_RES];	// The path to the request location
			int reqPathCount = 0;

			// Quick seach towards the goal.
			static const int MAX_ITER = 20;
			float npos[3];
			std::vector<vcg::Point3f> pathPts;
			std::vector<int> pathFaces;
			ag->corridor.ClearPath();
			m_navquery->GetNearestFace(ag->npos, ag->curTile, ag->curGrid, ag->curFace, npos);
			//m_navquery->FindPath(ag->curFace, ag->targetFace, ag->npos, ag->targetPos, pathFaces, pathPts);
			m_navquery->FindPath(ag->npos, ag->targetPos, pathFaces, pathPts);
			reqPathCount = pathFaces.size();
			//m_navquery->initSlicedFindPath(path[0], ag->targetRef, ag->npos, ag->targetPos, &m_filter);
			//m_navquery->updateSlicedFindPath(MAX_ITER, 0);
			//dtStatus status = 0;

			if (reqPathCount > 0)
			{
				// In progress or succeed.
				if (pathFaces[reqPathCount-1] != ag->targetFace)
				{
					// Partial path, constrain target position inside the last polygon.
//					status = m_navquery->closestPointOnPoly(reqPath[reqPathCount-1], ag->targetPos, reqPos);
					//if (dtStatusFailed(status))
						reqPathCount = 0;
				}
				else
				{
					//dtVcopy(reqPos, ag->targetPos);
				}
			}
			else
			{
				reqPathCount = 0;
			}
				
			if (!reqPathCount)
			{
				// Could not find path, start the request from current location.
				//dtVcopy(reqPos, ag->npos);
				//reqPath[0] = path[0];
				reqPathCount = 1;
			}

			ag->corridor.SetCorridor(pathPts);
			//ag->corridor.setCorridor(reqPos, reqPath, reqPathCount);
			//ag->boundary.reset();
			
			if (pathFaces[reqPathCount-1] == ag->targetFace)
			{
				ag->targetState = DT_CROWDAGENT_TARGET_VALID;
			}
			else
			{
				// The path is longer or potentially unreachable, full plan.
				ag->targetState = DT_CROWDAGENT_TARGET_WAITING_FOR_QUEUE;
			}
		}
	}

		
}

void HybridModelCrowd::update(const float dt)
{
	TimeVal startTime = getPerfTime();
	m_velocitySampleCount = 0;
	m_rvosim->agents_.clear();
	m_rvosim->obstacles_.clear();
	m_obstacleRVOSim->agents_.clear();
	m_obstacleRVOSim->obstacles_.clear();

	m_groupRVOSim->agents_.clear();
	HmAgent** agents = m_activeAgents;
	int nagents = getActiveAgents(agents, m_maxAgents);
	float agentSize = m_maxAgentRadius*m_maxAgentRadius*4;
	// Check that all agents still have valid paths.
	//checkPathValidity(agents, nagents, dt);
	
	// Update async move request and path finder.
	updateMoveRequest(dt);
	m_maxGrids = m_actGridIds.size();
	
	for(int i=0; i<m_maxGrids;i++)
	{
		(*m_groups)[m_actGridIds[i]].agentsInGroup.clear();
	}
	// Calculate steering.
	//printf("========================AGENT_INFO========================\n");
#pragma omp parallel for
	for (int i = 0; i < nagents; ++i)
	{
		HmAgent* ag = agents[i];

		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;
		if (ag->targetState == DT_CROWDAGENT_TARGET_NONE)
			continue;
		ag->is_contour = false;

		float dvel[3] = {0,0,0};

		if (ag->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
		{
			dtVcopy(dvel, ag->targetPos);
			ag->desiredSpeed = dtVlen(ag->targetPos);
		}
		else
		{
			ag->corridor.GetSteerDir(ag->npos, dvel);

			// Calculate speed scale, which tells the agent to slowdown at the end of the path.
			const float slowDownRadius = ag->params.radius*2;	// TODO: make less hacky.
			const float speedScale = ag->corridor.GetDistanceToGoal(ag->npos, slowDownRadius) / slowDownRadius;
				
			ag->desiredSpeed = ag->params.maxSpeed;
			dtVscale(dvel, dvel, ag->desiredSpeed * speedScale);
		}
		// Set the desired velocity.
		dtVcopy(ag->dvel, dvel);
		ag->setHmAgent(ag->npos,ag->dvel);
		(*m_groups)[ag->curGrid].agentsInGroup.push_back(ag->HmAgent_id);
		//printf("%f %f %f %f %f %f %d\n", ag->npos[0], ag->npos[1], ag->npos[2], ag->dvel[0], ag->dvel[1], ag->dvel[2], ag->color_id);
		//m_rvosim->agents_.push_back(ag);
	}
	//m_densityThreshold = EPSILON;
	for(int i=0; i<m_maxGrids;i++)
	{
		(*m_groups)[m_actGridIds[i]].UpdateDensity(m_densityThreshold, agentSize);
		(*m_groups)[m_actGridIds[i]].avgVelocity.reset();
	}
	//
	TimeVal endTime = getPerfTime();
	printf("NavMesh Time: %f\n",getPerfDeltaTimeUsec(startTime, endTime) / 1000.0f);
	startTime = getPerfTime();
	MergeGrid();
	endTime = getPerfTime();
	printf("Merge Time: %f\n",getPerfDeltaTimeUsec(startTime, endTime) / 1000.0f);
	startTime = getPerfTime();
	/*int groupNum = m_fluidgroups.size();
	if(groupNum > 1)
	{
		for(int i=0; i<groupNum; i++)
		{
			Vector2 pos, vel;
			float radius;
			m_fluidgroups[i].GetGroupAgent(pos, vel, radius,  m_groups, m_activeAgents);
			float groupSpeed = sqrt(vel.dot(vel));
			//m_groupRVOSim->addAgent(pos, radius*50, 8, radius*50.0f, radius*50.0f, radius*8, groupSpeed, vel);
		}
		//m_groupRVOSim->doStep();
		for(int i=0; i<groupNum; i++)
		{
			//m_fluidgroups[i].groupVel = m_groupRVOSim->agents_[i]->velocity_;
		}
	}*/
	m_fluidsim->doStep();
	for (int i = 0; i < ITER_REARRANGE; i++)
	{
		Rearrange();
	}
	endTime = getPerfTime();
	printf("Fluid Time: %f\n",getPerfDeltaTimeUsec(startTime, endTime) / 1000.0f);
	startTime = getPerfTime();
	for(unsigned int i=0; i<m_fluidgroups.size(); i++)
	{
		
		for (unsigned int j = 0; j < m_fluidgroups[i].crowdGroup.size(); j++)
		{
			int gid = m_fluidgroups[i].crowdGroup[j];
			std::vector<Vector2> obstacle = (*m_groups)[gid].m_box.GetCorners();
			for (unsigned int k = 0; k<obstacle.size(); k++)
				obstacle[k] = obstacle[k] + (*m_groups)[gid].avgVelocity * dt;
			m_rvosim->addObstacle(obstacle, (*m_groups)[gid].nowDensity, (*m_groups)[gid].avgVelocity);
		}
		/*
		for (unsigned int j = 0; j < m_fluidgroups[i].contours.size(); j++)
		{
			std::vector<Vector2> obstacle = m_fluidgroups[i].contours[j];
			for (unsigned int k = 0; k<obstacle.size(); k++)
				obstacle[k] = obstacle[k] + m_fluidgroups[i].avgVel * dt;
			m_rvosim->addObstacle(obstacle, m_fluidgroups[i].avgDensity, m_fluidgroups[i].avgVel);
		}*/
	}
	
	m_rvosim->processObstacles();
	m_rvosim->doStep();
	endTime = getPerfTime();
	printf("RVO Time: %f\n",getPerfDeltaTimeUsec(startTime, endTime) / 1000.0f);
	startTime = getPerfTime();
	
	// Integrate. and get new position
#pragma omp parallel for
	for (int i = 0; i < nagents; ++i)
	{
		HmAgent* ag = agents[i];
		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;
		ag->nvel[0] = float(ag->velocity_.x());
		ag->nvel[1] = float(ag->dvel[1]);
		ag->nvel[2] = float(ag->velocity_.y());
		
		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;
		integrate(ag, dt);
		ag->corridor.UpdateDir(ag->npos, float(ag->radius_));
		float nearest[3];
		int tileID = -1;
		int faceID = -1;
		int gridID = -1;
		bool res = m_navquery->GetNearestFace(ag->npos, tileID, gridID, faceID, nearest);
		ag->curGrid = gridID;
		ag->curFace = faceID;
		ag->curTile = tileID;
		dtVcopy(ag->npos, nearest);
	}
	endTime = getPerfTime();
	printf("Ajust Position Time: %f\n",getPerfDeltaTimeUsec(startTime, endTime) / 1000.0f);
}

void HybridModelCrowd::MergeGrid()
{
	map<int, int> sameMark;
	int markid = 0;
	m_fluidgroups.clear();	//清空群体数组
	std::vector<int> regionMark;
	std::vector<int> denseGridIds;
	int gridNum = static_cast<int>(m_groups->size());
	regionMark.resize(gridNum);
	denseGridIds.resize(gridNum);

	for(int i=0; i<gridNum; i++)
	{
		if((*m_groups)[i].m_densityStatus==WHITE)
			continue;
		bool hasDense = false;
		for(vector<int>::iterator it =(*m_groups)[i].agentsInGroup.begin();it!=(*m_groups)[i].agentsInGroup.end();it++)
		{
			// We add a bilinear interpolation to solve the continuity problem on the border of dense crowd
			std::vector<int> neighborIds;
			Vector2 leftBottom, rightTop;
			bool hasInterp = (*m_groups)[i].GetInterpolationNeighbors(m_agents[*it].position_, m_groups, neighborIds, leftBottom, rightTop);
			if(hasInterp)
			{
				float densities[4];
				memset(densities, 0.0f, 4*sizeof(float));
				for(int di = 0; di < 4; di++)
				{
					if(neighborIds[di] >= 0)
					{
						densities[di] = (float)((*m_groups)[neighborIds[di]].nowDensity);
					}
				}
				float density = bilateralInterpolation(densities[0], densities[1], densities[2], densities[3], 
														leftBottom, rightTop,
														m_agents[*it].position_);
				m_agents[*it].density = density;
				if(density >= m_densityThreshold)
				{
					hasDense = true;
				}
				else
				{
					m_agents[*it].active = 2;
					m_rvosim->agents_.push_back(&m_agents[*it]);
					m_obstacleRVOSim->agents_.push_back(&m_agents[*it]);
				}
			}
			else
			{
				m_agents[*it].density = 0;
				m_agents[*it].active = 2;
				m_rvosim->agents_.push_back(&m_agents[*it]);
				m_obstacleRVOSim->agents_.push_back(&m_agents[*it]);
			}
		}
		if(hasDense)
		{
			denseGridIds[markid] = i;
			regionMark[i] = markid++;	// initialize a mark for each grid
		}
	}
	int denseGridNum = markid;
	if(denseGridNum <= 0)
		return;
	for(int i=0; i<denseGridNum; i++)
	{
		int gid = denseGridIds[i];
		for(int j=0;j<4;j++)//遍历四个邻居
		{
			int ngid = (*m_groups)[gid].m_neighbor[j];
			if(ngid <= 0)
				continue;
			if((*m_groups)[ngid].m_densityStatus == BLACK)
			{
				if(regionMark[ngid] >= regionMark[gid])
				{
					regionMark[ngid] = regionMark[gid];
				}
				else
				{
					std::map<int, int>::iterator iterPair = sameMark.find(regionMark[ngid]);
					int id = regionMark[ngid];
					while(iterPair != sameMark.end())	// maybe we don't need to have a loop, one level is enough
					{
						id = iterPair->second;
						iterPair = sameMark.find(iterPair->second);
					}
					sameMark.insert(std::pair<int, int>(regionMark[gid], id));
					regionMark[ngid] = id;
				}
			}
		}
	}

	CrowdGroup tempgroup;
	tempgroup.RegionID = regionMark[denseGridIds[0]];
	tempgroup.AddGrid(denseGridIds[0]);
	m_fluidgroups.push_back(tempgroup);
			

	//处理同义区域，将其合并到同一个crowdgroup的vector里
	for(int i=1; i<denseGridNum; i++)
	{
		int gid = denseGridIds[i];
		bool isNewGroup = true;
		for(unsigned int j=0;j<m_fluidgroups.size();j++)
		{
			if(regionMark[gid]==m_fluidgroups[j].RegionID)
			{
				m_fluidgroups[j].AddGrid(gid);
				isNewGroup = false;
				break;
			}
			std::map<int, int>::iterator iterPair = sameMark.find(regionMark[gid]);
			if(iterPair != sameMark.end() && iterPair->second==m_fluidgroups[j].RegionID)
			{
				regionMark[gid] = m_fluidgroups[j].RegionID;
				m_fluidgroups[j].AddGrid(gid);
				isNewGroup = false;
				break;
			}
		}
		if(isNewGroup==true)
		{
			CrowdGroup tempgroup;
			tempgroup.RegionID = regionMark[gid];
			tempgroup.AddGrid(gid);
			m_fluidgroups.push_back(tempgroup);
		}
	}
}

int compare(const void* a, const void* b)
{
	return ( ((elem_grid*)b)->score - ((elem_grid*)a)->score  );
}

void HybridModelCrowd::Rearrange()
{
	float elem_size = m_maxAgentRadius*2;
	int grid_area = m_cellsize*m_cellsize;
	elem_grid* elem_groups = (elem_grid*)malloc(grid_area*sizeof(elem_grid));
#pragma omp parallel for
	for (unsigned int crowdi = 0; crowdi<m_fluidgroups.size(); crowdi++)
	{
		for (unsigned int gridi = 0; gridi < m_fluidgroups[crowdi].crowdGroup.size(); gridi++)
		{
			int gid = m_fluidgroups[crowdi].crowdGroup[gridi];
			int people_num = (*m_groups)[gid].agentsInGroup.size();
			int firstGroupNum = people_num;
			// put neighbor groups agents in
			for (int i = 0; i < 4; i++) 
			{
				int nid = (*m_groups)[gid].m_neighbor[i];
				if (nid < 0)
					continue; 
				people_num += (*m_groups)[nid].agentsInGroup.size();
			}
			
			elem_grid* agent_grids = (elem_grid*)malloc(people_num*sizeof(elem_grid));
			int agentInRow = this->m_groupsize/m_maxAgentRadius/2;
			float elem_grid_size = this->m_groupsize/(1.0*agentInRow);
			// Assign parameters from each agent to the agent list used for sorting.
			Vector2 mean_center(0,0);
			int agentCount = 0;
			for (int i = 0; i<firstGroupNum; i++)
			{
				agent_grids[i].id = i;
				int agent_id = (*m_groups)[gid].agentsInGroup[i];		
				agent_grids[i].center = Vector2(m_agents[agent_id].npos[0] , m_agents[agent_id].npos[2]);
				agent_grids[i].agent = &m_agents[agent_id];
				agent_grids[i].radius = m_agents[agent_id].radius_;
				mean_center += agent_grids[i].center;
				agentCount++;
			}
			mean_center /= firstGroupNum;
			
			for (int i = 0; i < 4; i++)
			{
				int nid = (*m_groups)[gid].m_neighbor[i];
				if (nid < 0)
					continue;
				for (int j = 0; j<(*m_groups)[nid].agentsInGroup.size(); j++)
				{
					agent_grids[agentCount].id = agentCount;
					int agent_id = (*m_groups)[nid].agentsInGroup[j];
					agent_grids[agentCount].center = Vector2(m_agents[agent_id].npos[0], m_agents[agent_id].npos[2]);
					agent_grids[agentCount].agent = &m_agents[agent_id];
					agent_grids[agentCount].radius = m_agents[agent_id].radius_;
					agentCount++;
				}
			}
			
			for(int i=0;i<people_num;i++)
			{
				agent_grids[i].score = float((agent_grids[i].center-mean_center).dot((*m_groups)[gid].avgVelocity));	// projection on velocity
			}

			//sort agents by projection
			qsort(agent_grids,people_num,sizeof(elem_grid), compare);

			//insert agent into buckets
			std::vector<std::list<elem_grid>> agent_bucket;
			Vector2 vertVelocity((*m_groups)[gid].avgVelocity.y(),-(*m_groups)[gid].avgVelocity.x());//顺时针旋转90度
			int count = -1;
			for(int i=0;i<people_num;i++)
			{
				if(i==0||agent_grids[0].score-agent_grids[i].score>(count+1)*elem_grid_size)	// create buckets when first guy in the bucket appears
				{
					std::list<elem_grid> bucket;
					bucket.push_back(agent_grids[i]);
					count++;
					agent_bucket.push_back(bucket);
				}
				else
				{
					std::list<elem_grid>::iterator iter;
					for(iter=agent_bucket[count].begin();iter!=agent_bucket[count].end();iter++)
					{
						if(iter->center.dot(vertVelocity)>agent_grids[i].center.dot(vertVelocity))	// if the agent is on the left hand side of the tail
						{
							agent_bucket[count].insert(iter,agent_grids[i]);
							break;
						}
					}
					if(iter==agent_bucket[count].end())
						agent_bucket[count].push_back(agent_grids[i]);// if the agent is on the most right hand side
				}
				//agent_grids[i]

			}
			// rearrange
			for (unsigned int i = 0; i<agent_bucket.size(); i++)
			{
				
				std::list<elem_grid>::iterator riter, nextRiter;
				std::list<elem_grid>::reverse_iterator liter, nextLiter;
				int bucketSize = agent_bucket[i].size();
				liter = agent_bucket[i].rbegin();
				riter = agent_bucket[i].begin();
				int midId = bucketSize / 2;
				// set the pointers at middle point
				for (int i = 0; i<midId; i++)
				{
					riter++;
				}
				for (int i = midId + 1; i < bucketSize; i++)
				{
					liter++;
				}
				// liter is from the middle to the beginning, riter is from the middle to the end
				for (; liter != agent_bucket[i].rend() ; liter++)
				{
					nextLiter=liter;
					nextLiter++;
					for (; nextLiter != agent_bucket[i].rend(); nextLiter++)
					{
						if (liter->center.getDistance(nextLiter->center)<elem_grid_size)
						{
							Vector2 dir = normalize(nextLiter->center - liter->center);
							nextLiter->center = liter->center + dir*elem_grid_size;
							nextLiter->agent->npos[0] = float(nextLiter->center.x());
							nextLiter->agent->npos[2] = float(nextLiter->center.y());

						}
					}
					if(i+1<agent_bucket.size())
					{
						for (nextLiter = agent_bucket[i + 1].rbegin(); nextLiter != agent_bucket[i + 1].rend(); nextLiter++)
						{
							if (liter->center.getDistance(nextLiter->center)<elem_grid_size)
							{
								Vector2 dir = normalize(nextLiter->center - liter->center);
								nextLiter->center = liter->center + dir*elem_grid_size;
								nextLiter->agent->npos[0] = float(nextLiter->center.x());
								nextLiter->agent->npos[2] = float(nextLiter->center.y());
							}	
						}
					}
				}

				// liter is from the middle to the beginning, riter is from the middle to the end
				for (; riter != agent_bucket[i].end(); riter++)
				{
					nextRiter = riter;
					nextRiter++;
					for (; nextRiter != agent_bucket[i].end(); nextRiter++)
					{
						if (riter->center.getDistance(nextRiter->center)<elem_grid_size)
						{
							Vector2 dir = normalize(nextRiter->center - riter->center);
							nextRiter->center = riter->center + dir*elem_grid_size;
							nextRiter->agent->npos[0] = float(nextRiter->center.x());
							nextRiter->agent->npos[2] = float(nextRiter->center.y());

						}
					}
					if (i + 1<agent_bucket.size())
					{
						for (nextRiter = agent_bucket[i + 1].begin(); nextRiter != agent_bucket[i + 1].end(); nextRiter++)
						{
							if (riter->center.getDistance(nextRiter->center)<elem_grid_size)
							{
								Vector2 dir = normalize(nextRiter->center - riter->center);
								nextRiter->center = riter->center + dir*elem_grid_size;
								nextRiter->agent->npos[0] = float(nextRiter->center.x());
								nextRiter->agent->npos[2] = float(nextRiter->center.y());
							}
						}
					}
				}
			}
			free(agent_grids);
		}
	}
	free(elem_groups);
}

void HybridModelCrowd::save(FILE* fp)
{
	HmAgent** agents = m_activeAgents;
	int nagents = getActiveAgents(agents, m_maxAgents);
	for (int i = 0; i < nagents; ++i)
	{
		HmAgent* ag = agents[i];
		fprintf(fp, "%f %f %f %f %f %f %d ", ag->npos[0], ag->npos[1], ag->npos[2], ag->targetPos[0], ag->targetPos[1], ag->targetPos[2], ag->color_id);
		fprintf(fp, "%f %f %f %f %f\n", ag->neighborDist_, ag->maxNeighbors_, ag->timeHorizon_, ag->radius_, ag->prefVelocity_);
	}
}
