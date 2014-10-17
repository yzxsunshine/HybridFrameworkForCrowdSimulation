#include "HybridModelCrowd.h"
#include "DetourCommon.h"
#include "DetourAssert.h"
#include "../Include/PerfTimer.h"
static const int MAX_ITERS_PER_UPDATE = 100;

static const int MAX_PATHQUEUE_NODES = 4096;
static const int MAX_COMMON_NODES = 512;
static const float PI = 3.14159265f;

static void integrate(dtCrowdAgent* ag, const float dt)
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

static bool overOffmeshConnection(const dtCrowdAgent* ag, const float radius)
{
	if (!ag->ncorners)
		return false;
	
	const bool offMeshConnection = (ag->cornerFlags[ag->ncorners-1] & DT_STRAIGHTPATH_OFFMESH_CONNECTION) ? true : false;
	if (offMeshConnection)
	{
		const float distSq = dtVdist2DSqr(ag->npos, &ag->cornerVerts[(ag->ncorners-1)*3]);
		if (distSq < radius*radius)
			return true;
	}
	
	return false;
}

static float getDistanceToGoal(const dtCrowdAgent* ag, const float range)
{
	if (!ag->ncorners)
		return range;
	
	const bool endOfPath = (ag->cornerFlags[ag->ncorners-1] & DT_STRAIGHTPATH_END) ? true : false;
	if (endOfPath)
		return dtMin(dtVdist2D(ag->npos, &ag->cornerVerts[(ag->ncorners-1)*3]), range);
	
	return range;
}

static void calcSmoothSteerDirection(const dtCrowdAgent* ag, float* dir)
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

static void calcStraightSteerDirection(const dtCrowdAgent* ag, float* dir)
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

HybridModelCrowd::HybridModelCrowd():m_agents(0),m_maxAgents(0),m_activeAgents(0),m_grids(0), m_navquery(0),/*m_pathResult(0),m_grid(0),*/m_root(0),m_rvosim(0), m_fluidsim(0),m_activeGrids(0),m_maxGrids(0)
{
	m_defaultGrid = new Grid();
	m_defaultGrid->hmcrowd = this;
}

HybridModelCrowd::~HybridModelCrowd()
{
	purge();
	delete m_defaultGrid;
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
	
/*	if(m_pathResult!=0)
		dtFree(m_pathResult);
	m_pathResult = 0;
*/	
//	if(m_navquery!=0)
//		delete m_navquery;
//	m_navquery = 0;

	if(m_grids!=0)
		delete[] m_grids;
	m_grids = 0;

	if(m_root!=0)
		delete m_root;
	m_root = 0;
	
/*	if(m_grid!=0)
		dtFreeProximityGrid(m_grid);
	m_grid = 0;
*/
	if(m_fluidsim!=0)
		delete m_fluidsim;
	m_fluidsim = 0;

	if(m_rvosim!=0)
		delete m_rvosim;
	m_rvosim = 0;

	if(m_activeGrids!=0)
		free(m_activeGrids);
	m_activeGrids = 0;
	m_maxGrids = 0;
	
}

bool HybridModelCrowd::init(const int maxAgents, const float maxAgentRadius, NavMesh* nav, NavMeshQuery* nmq, const float grid_size, const float ox, const float oy, const float ex, const float ey)
{
	purge();
	
	m_maxAgents = maxAgents;
	m_maxAgentRadius = maxAgentRadius;
	m_gridsize = grid_size;	//真实比例
	m_cellsize = grid_size/maxAgentRadius/2;	//h
	m_min[0] = ox;	m_min[1] = oy;	m_max[0] = ex;	m_max[1] = ey;

	m_navquery = nmq;

	m_rvosim = new RVO::RVOSimulator();
	agent_num = 0;
	// (pi*r^2/2) / maxAgentRadius;
	m_rvosim->setAgentDefaults(grid_size/2, PI*grid_size*grid_size/8/maxAgentRadius, 5.0f, 5.0f, maxAgentRadius, 2.0f/*max speed*/);
	m_fluidsim = new FluidSimulator();
	m_fluidsim->init(this);
	// Allocate temp buffer for merging paths.
	m_maxPathResult = 256;
	//m_pathResult = (dtPolyRef*)dtAlloc(sizeof(dtPolyRef)*m_maxPathResult, DT_ALLOC_PERM);
	//if (!m_pathResult)
	//	return false;
	
	//if (!m_pathq.init(m_maxPathResult, MAX_PATHQUEUE_NODES, nav))
	//	return false;
	
	//Initialize Agents

	m_agents = new HmAgent[m_maxAgents];
	m_activeAgents = (HmAgent**)malloc(m_maxAgents*sizeof(HmAgent*));
	if (!m_agents)
		return false; 
	dtVset(m_ext, m_maxAgentRadius*2.0f,m_maxAgentRadius*1.5f,m_maxAgentRadius*2.0f);

	//m_grid = dtAllocProximityGrid();
	//if (!m_grid)
	//	return false;
	//if (!m_grid->init(m_maxAgents*4, maxAgentRadius*3))
	//	return false;
	

	for (int i = 0; i < m_maxAgents; ++i)
	{
		m_agents[i].sim_ = m_rvosim;
		m_agents[i].HmAgent_id = i;
		m_agents[i].active = 0;
		//if (!m_agents[i].corridor.init(m_maxPathResult))
		//	return false;
	}

	// The navquery is mostly used for local searches, no need for large node pool.
	//m_navquery = dtAllocNavMeshQuery();
	//if (!m_navquery)
	//	return false;
	//if (dtStatusFailed(m_navquery->init(nav, MAX_COMMON_NODES)))
	//	return false;
		
	//use navigation map to generate obstacles and grids
	float m_width = ex-ox; //这里必须是能够被2^n整除的数
	float m_height = ey-oy;
	float mapsize = (m_width>m_height)?m_width:m_height;

	m_numofgridinrow = ceil(mapsize/grid_size);
	QuadTree::maxlayer = getNearestLog(m_numofgridinrow);
	mapsize = m_numofgridinrow*grid_size;
	int grids_num = m_numofgridinrow * m_numofgridinrow;
	m_grids = new Grid[grids_num];
	m_activeGrids = (Grid**)malloc(grids_num*sizeof(Grid*));
	int grid_count = 0;
	for(int row = 0; row < m_numofgridinrow; row++)
	{
		for(int col = 0; col < m_numofgridinrow; col++)
		{
			m_grids[grid_count].grid_id = grid_count;
			m_grids[grid_count].x = col;
			m_grids[grid_count].y = row;
			m_grids[grid_count].box = CRectangle<double>(ox+col*grid_size,oy+row*grid_size,grid_size,grid_size);
			m_grids[grid_count].hmcrowd = this;
			//const dtMeshTile* tile = nav->getTileAt(col,row,0);
			//if(tile==NULL)
			//{
				//障碍物	
			//}
			//else
			{
				m_activeGrids[m_maxGrids++] = &m_grids[grid_count];
			/*	dtPolyRef base = nav->getPolyRefBase(tile);
				for (int j = 0; j < tile->header->polyCount; ++j)
				{
					const dtPoly* p = &tile->polys[j];
					const dtPoly* poly = 0;
					dtPolyRef ref = base|(dtPolyRef)j;
					if (dtStatusFailed(nav->getTileAndPolyByRef(ref, &tile, &poly)))
						continue;
					const unsigned int ip = (unsigned int)(poly - tile->polys);
					const dtPolyDetail* pd = &tile->detailMeshes[ip];
					for (int i = 0; i < pd->triCount; ++i)
					{
						const unsigned char* t = &tile->detailTris[(pd->triBase+i)*4];
						for (int j = 0; j < 3; ++j)
						{
							if (t[j] < poly->vertCount)
								tile->verts[poly->verts[t[j]]*3];
							else
								tile->detailVerts[(pd->vertBase+t[j]-poly->vertCount)*3];
						}
					}
				}*/
			}
			
			//nav->getPolyRefBase(nav->getTileAt(col,row,0));	//这里需要类型转换，并且需要检查是否为空，不为空则取其中的poly然后把剩下的当作障碍物，否则整个为障碍物
			grid_count++;
		}
	}
	
	CRectangle<double> rect(ox,oy,mapsize,mapsize);
	m_root = new QuadTree(this,0,0,NULL,rect);
	m_root->InitNeighbor();//初始化邻居
	//m_fluidsim->generateTemplate();
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
int HybridModelCrowd::addAgent(const float* pos, const dtCrowdAgentParams* params)
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
	dtPolyRef ref;
	//m_navquery->findNearestPoly(pos, m_ext, &m_filter, &ref, nearest);
	
	//ag->corridor.reset(ref, nearest);
	ag->boundary.reset();

	updateAgentParameters(idx, params);
	
	dtVset(ag->dvel, 0,0,0);
	dtVset(ag->nvel, 0,0,0);
	dtVset(ag->vel, 0,0,0);
	dtVcopy(ag->npos, pos);//nearest);
	
	ag->desiredSpeed = 0;

	//if (ref)
		ag->state = DT_CROWDAGENT_STATE_WALKING;
	//else
	//	ag->state = DT_CROWDAGENT_STATE_INVALID;
	
	ag->targetState = DT_CROWDAGENT_TARGET_NONE;
	
	ag->active = 1;
	ag->HmAgent_id = idx;
	//Add Agent to Grid
	int grid_id = m_defaultGrid->GetGridId(ag->npos[0],ag->npos[2]);
	if(grid_id<0||grid_id>m_numofgridinrow*m_numofgridinrow){
		ag->active = 0;
		return -1;
	}
	ag->grid_now_id = grid_id;
	m_grids[grid_id].people_in_grid.push_back(idx);
	m_grids[grid_id].now_density = m_grids[grid_id].people_in_grid.size()*1.0/(m_cellsize*m_cellsize);
	m_grids[grid_id].former_density = m_grids[grid_id].now_density;
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
bool HybridModelCrowd::requestMoveTarget(const int idx, dtPolyRef ref, const float* pos)
{
	if (idx < 0 || idx > m_maxAgents)
		return false;
	if (!ref)
		return false;

	HmAgent* ag = &m_agents[idx];
	
	// Initialize request.
	ag->targetRef = ref;
	dtVcopy(ag->targetPos, pos);
	ag->targetPathqRef = DT_PATHQ_INVALID;
	ag->targetReplan = false;
	if (ag->targetRef)
		ag->targetState = DT_CROWDAGENT_TARGET_REQUESTING;
	else
		ag->targetState = DT_CROWDAGENT_TARGET_FAILED;

	return true;
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
	ag->targetPathqRef = DT_PATHQ_INVALID;
	ag->targetReplan = false;
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
	ag->targetRef = 0;
	dtVcopy(ag->targetPos, vel);
	ag->targetPathqRef = DT_PATHQ_INVALID;
	ag->targetReplan = false;
	ag->targetState = DT_CROWDAGENT_TARGET_VELOCITY;
	
	return true;
}

bool HybridModelCrowd::resetMoveTarget(const int idx)
{
	if (idx < 0 || idx > m_maxAgents)
		return false;
	
	HmAgent* ag = &m_agents[idx];
	
	// Initialize request.
	ag->targetRef = 0;
	dtVset(ag->targetPos, 0,0,0);
	ag->targetPathqRef = DT_PATHQ_INVALID;
	ag->targetReplan = false;
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
	dtCrowdAgent* queue[PATH_MAX_AGENTS];
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
			m_navquery->GetNearestFace(ag->npos, ag->curFace, npos);
			m_navquery->FindPath(ag->curFace, ag->targetFace, ag->npos, ag->targetPos, pathFaces, pathPts);
			reqPathCount = pathFaces.size();
			//m_navquery->initSlicedFindPath(path[0], ag->targetRef, ag->npos, ag->targetPos, &m_filter);
			//m_navquery->updateSlicedFindPath(MAX_ITER, 0);
			//dtStatus status = 0;
			if (ag->targetReplan) // && npath > 10)
			{
				// Try to use existing steady path during replan if possible.
//				status = m_navquery->finalizeSlicedFindPathPartial(path, npath, reqPath, &reqPathCount, MAX_RES);
			}
			else
			{
				// Try to move towards target when goal changes.
//				status = m_navquery->finalizeSlicedFindPath(reqPath, &reqPathCount, MAX_RES);
			}

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
				ag->targetReplanTime = 0.0;
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
	HmAgent** agents = m_activeAgents;
	int nagents = getActiveAgents(agents, m_maxAgents);
	
	// Check that all agents still have valid paths.
	//checkPathValidity(agents, nagents, dt);
	
	// Update async move request and path finder.
	updateMoveRequest(dt);

	// Optimize path topology.
//	updateTopologyOptimization(agents, nagents, dt);
	
	// Register agents to proximity grid.
//	m_grid->clear();	
	// Get nearby navmesh segments and agents to collide with.
	for (int i = 0; i < nagents; ++i)
	{
		HmAgent* ag = agents[i];
		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;
		ag->is_contour = false;
		// Update the collision boundary after certain distance has been passed or
		// if it has become invalid.
		const float updateThr = ag->params.collisionQueryRange*0.25f;
//		if (dtVdist2DSqr(ag->npos, ag->boundary.getCenter()) > dtSqr(updateThr) ||
//			!ag->boundary.isValid(m_navquery, &m_filter))
		{
//			ag->boundary.update(ag->corridor.getFirstPoly(), ag->npos, ag->params.collisionQueryRange,
//								m_navquery, &m_filter);
		}
	}
	
	// Find next corner to steer to.
	for (int i = 0; i < nagents; ++i)
	{
		HmAgent* ag = agents[i];
		
		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;
		if (ag->targetState == DT_CROWDAGENT_TARGET_NONE || ag->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
			continue;
		
		// Find corners for steering
		//ag->ncorners = ag->corridor.findCorners(ag->cornerVerts, ag->cornerFlags, ag->cornerPolys,
//												DT_CROWDAGENT_MAX_CORNERS, m_navquery, &m_filter);
		
		// Check to see if the corner after the next corner is directly visible,
		// and short cut to there.
		//if ((ag->params.updateFlags & DT_CROWD_OPTIMIZE_VIS) && ag->ncorners > 0)
		{
			//const float* target = &ag->cornerVerts[dtMin(1,ag->ncorners-1)*3];
//			ag->corridor.optimizePathVisibility(target, ag->params.pathOptimizationRange, m_navquery, &m_filter);
		}
	}
	for(int i=0; i<m_maxGrids;i++)
	{
		m_activeGrids[i]->people_in_grid.clear();
	}
	// Calculate steering.
	for (int i = 0; i < nagents; ++i)
	{
		HmAgent* ag = agents[i];

		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;
		if (ag->targetState == DT_CROWDAGENT_TARGET_NONE)
			continue;
		
		float dvel[3] = {0,0,0};

		if (ag->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
		{
			dtVcopy(dvel, ag->targetPos);
			ag->desiredSpeed = dtVlen(ag->targetPos);
		}
		else
		{
			// Calculate steering direction.
			//if (ag->params.updateFlags & DT_CROWD_ANTICIPATE_TURNS)
			//	calcSmoothSteerDirection(ag, dvel);
			//else
			//	calcStraightSteerDirection(ag, dvel);
			
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
		int grid_id = m_defaultGrid->GetGridId(ag->npos[0],ag->npos[2]);
		m_grids[grid_id].people_in_grid.push_back(ag->HmAgent_id);
		//m_rvosim->agents_.push_back(ag);
		//m_rvosim->addAgent(ag->npos,ag->dvel);
	}
	/*
	for(int i=0; i<m_maxGrids;i++)
	{
		m_activeGrids[i]->now_density = m_activeGrids[i]->people_in_grid.size()*1.0/(m_cellsize*m_cellsize);	//grid size is not correct Deal with it later
		m_activeGrids[i]->former_density = m_activeGrids[i]->now_density;
		if(m_activeGrids[i]->now_density >= this->m_densityThreshold)
			m_activeGrids[i]->tpnode->leaf_state = BLACK;
		else if(m_activeGrids[i]->now_density >0)
			m_activeGrids[i]->tpnode->leaf_state = GRAY;
		else
			m_activeGrids[i]->tpnode->leaf_state = WHITE;
		m_activeGrids[i]->avgVelocity.reset();
	}
	//
	TimeVal endTime = getPerfTime();
	printf("NavMesh Time: %f\n",getPerfDeltaTimeUsec(startTime, endTime) / 1000.0f);
	startTime = getPerfTime();
	m_root->UpdateQuadTree();
	m_root->MergeQuadTree();
	endTime = getPerfTime();
	printf("Divide Time: %f\n",getPerfDeltaTimeUsec(startTime, endTime) / 1000.0f);
	startTime = getPerfTime();
	m_rvosim->doStep();
	endTime = getPerfTime();
	printf("RVO Time: %f\n",getPerfDeltaTimeUsec(startTime, endTime) / 1000.0f);
	startTime = getPerfTime();
	m_fluidsim->doStep();
	endTime = getPerfTime();
	printf("Fluid Time: %f\n",getPerfDeltaTimeUsec(startTime, endTime) / 1000.0f);
	startTime = getPerfTime();
	*/
	// Integrate. and get new position
	for (int i = 0; i < nagents; ++i)
	{
		HmAgent* ag = agents[i];
		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;
		ag->nvel[0] = ag->dvel[0];//ag->velocity_.x();
		ag->nvel[1] = ag->dvel[1];
		ag->nvel[2] = ag->dvel[2];//ag->velocity_.y();
		
		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;
		integrate(ag, dt);
	}

	TimeVal startTime1 = getPerfTime();
	for (int i = 0; i < nagents; ++i)
	{
		// Move along navmesh.
		HmAgent* ag = agents[i];
		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;
		ag->corridor.UpdateDir(ag->npos, ag->radius_);
//		ag->corridor.movePosition(ag->npos, m_navquery, &m_filter);
		
		// Get valid constrained position back.
		//dtVcopy(ag->npos, ag->corridor.getPos());

		// If not using path, truncate the corridor to just one poly.
		/*if (ag->targetState == DT_CROWDAGENT_TARGET_NONE || ag->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
		{
			ag->corridor.reset(ag->corridor.getFirstPoly(), ag->npos);
		}*/

	}
	//TimeVal endTime1 = getPerfTime();
	//printf("Move Position Time: %f\n",getPerfDeltaTimeUsec(startTime1, endTime1) / 1000.0f);
	//endTime = getPerfTime();
	//printf("Ajust Position Time: %f\n",getPerfDeltaTimeUsec(startTime, endTime) / 1000.0f);
}


