#include "HybridFrameworkCtrl.h"
#include "PerfTimer.h"

#include "HybridFrameworkCtrl.h"
#include "CommonDef.h"
#include<vcg/complex/complex.h>
#include<vcg/complex/algorithms/update/topology.h>
#include<vcg/complex/algorithms/update/bounding.h>
#include <list>
#include <map>


static const int MAX_ITERS_PER_UPDATE = 100;

static const int MAX_PATHQUEUE_NODES = 4096;
static const int MAX_COMMON_NODES = 512;
static const int ITER_REARRANGE = 10;

static void integrate(HmAgent* ag, const float dt)
{
	// Fake dynamic constraint.
	const float maxDelta = ag->maxSpeed_;
	float dv[3];
	vecSub(dv, ag->nvel, ag->vel);
	float ds = vecLen(dv);
	if (ds > maxDelta)
		vecScale(dv, dv, maxDelta / ds);
	vecAdd(ag->vel, ag->vel, dv);

	// Integrate
	if (vecLen(ag->vel) > 0.0001f)
		vecMad(ag->npos, ag->npos, ag->vel, dt);
	else
		vecSet(ag->vel, 0, 0, 0);
}

static float getDistanceToGoal(const HmAgent* ag, const float range)
{
	if (!ag->ncorners)
		return range;

	const bool endOfPath = (ag->cornerFlags[ag->ncorners - 1]) ? true : false;
	if (endOfPath)
		return std::min(vecDist2D(ag->npos, &ag->cornerVerts[(ag->ncorners - 1) * 3]), range);

	return range;
}

static void calcSmoothSteerDirection(const HmAgent* ag, float* dir)
{
	if (!ag->ncorners)
	{
		vecSet(dir, 0, 0, 0);
		return;
	}

	const int ip0 = 0;
	const int ip1 = std::min(1, ag->ncorners - 1);
	const float* p0 = &ag->cornerVerts[ip0 * 3];
	const float* p1 = &ag->cornerVerts[ip1 * 3];

	float dir0[3], dir1[3];
	vecSub(dir0, p0, ag->npos);
	vecSub(dir1, p1, ag->npos);
	dir0[1] = 0;
	dir1[1] = 0;

	float len0 = vecLen(dir0);
	float len1 = vecLen(dir1);
	if (len1 > 0.001f)
		vecScale(dir1, dir1, 1.0f / len1);

	dir[0] = dir0[0] - dir1[0] * len0*0.5f;
	dir[1] = 0;
	dir[2] = dir0[2] - dir1[2] * len0*0.5f;

	vecNormalize(dir);
}

static void calcStraightSteerDirection(const HmAgent* ag, float* dir)
{
	if (!ag->ncorners)
	{
		vecSet(dir, 0, 0, 0);
		return;
	}
	vecSub(dir, &ag->cornerVerts[0], ag->npos);
	dir[1] = 0;
	vecNormalize(dir);
}

int getNearestLog(int& num)
{
	int tmp = num;
	int n = 0;
	while (tmp>2)
	{
		int tmp_2 = tmp >> 1;
		tmp_2 = tmp_2 << 1;
		if (tmp - tmp_2 != 0)
		{
			tmp++;
			num += 1 << n;
		}
		tmp = tmp >> 1;
		n++;
	}
	return ++n;
}

void LoadMesh(NavMesh& navMesh, int vn, float* verts, int fn, int* inds)	// vn = 1/3 * length(verts); fn = 1/3 * length(inds);
{
	NavMesh::VertexIterator vi = vcg::tri::Allocator<NavMesh>::AddVertices(navMesh, vn);
	for (int i = 0; i < vn; i++) 
	{
		vi->P() = NavMesh::CoordType(verts[i * 3], verts[i * 3 + 1], verts[i * 3 + 2]);
		vi->vertID = i;
		vi++;
	}
	vcg::tri::Allocator<NavMesh>::AddFaces(navMesh, fn);
	for (int i = 0; i < fn; i++)
	{
		int vid0 = inds[i * 3];
		int vid1 = inds[i * 3 + 1];
		int vid2 = inds[i * 3 + 2];
		navMesh.face[i].V(0) = &navMesh.vert[vid0];
		navMesh.face[i].V(1) = &navMesh.vert[vid1];
		navMesh.face[i].V(2) = &navMesh.vert[vid2];
		navMesh.face[i].faceID = i;
		navMesh.face[i].CCID = 0;
		navMesh.face[i].walkable = true;
		navMesh.face[i].vertexIDs[0] = vid0;
		navMesh.face[i].vertexIDs[1] = vid1;
		navMesh.face[i].vertexIDs[2] = vid2;
	}
	vcg::tri::Allocator<NavMesh>::CompactFaceVector(navMesh);
	vcg::tri::Allocator<NavMesh>::CompactVertexVector(navMesh);
	vcg::tri::UpdateTopology<NavMesh>::VertexFace(navMesh);
	vcg::tri::UpdateTopology<NavMesh>::FaceFace(navMesh);
	vcg::tri::UpdateBounding<NavMesh>::Box(navMesh);
}


HybridFrameworkCtrl::HybridFrameworkCtrl() :m_agents(0), m_maxAgents(0), m_activeAgents(0)
											, m_rvosim(0), m_fluidsim(0), m_maxGrids(0), m_nav(0), m_tileCtrl(0)
											, m_groupRVOSim(0), m_groups(0), m_densityThreshold(0.3)
{

}

HybridFrameworkCtrl::~HybridFrameworkCtrl()
{
	purge();
}


void HybridFrameworkCtrl::purge()
{
	if (m_agents != 0)
		delete[] m_agents;
	m_agents = 0;
	m_maxAgents = 0;

	if (m_fluidsim != 0)
		delete m_fluidsim;
	m_fluidsim = 0;

	if (m_rvosim != 0) {
		m_rvosim->agents_.clear();
		delete m_rvosim;
	}
	m_rvosim = 0;

	if (m_groupRVOSim != 0)
		delete m_groupRVOSim;
	m_groupRVOSim = 0;

	if (m_groups != 0)
		delete m_groups;
	m_groups = 0;

	if (m_tileCtrl != 0)
		delete m_tileCtrl;
	m_tileCtrl = 0;

	if (m_nav != 0)
		delete m_nav;
	m_nav = 0;

	if (m_activeAgents != 0)
		free(m_activeAgents);
	m_activeAgents = 0;

	
	
	
	m_maxGrids = 0;

}

bool HybridFrameworkCtrl::init(int maxAgents, float renderAgentRadius, float grid_size, int vn, float* verts, int fn, int* inds)
{
	purge();

	m_maxAgents = maxAgents;
	m_maxAgentRadius = renderAgentRadius;
	m_gridsize = grid_size;	//真实比例
	m_cellsize = (int)(grid_size / renderAgentRadius / 2);	//h

	if (m_tileCtrl == 0) {
		m_tileCtrl = new TileCtrl();
	}
	m_tileCtrl->ClearTiles();
	if (m_nav == 0)
	{
		m_nav = new NavMesh();
	}
	else
	{
		m_nav->Clear();
	}
	LoadMesh(*m_nav, vn, verts, fn, inds);
	m_tileCtrl->BuildTiles(m_nav, grid_size);
	m_tileCtrl->BuildGrids(m_nav);
	int groupNum = m_tileCtrl->m_grids.size();
	m_groups = new std::vector < Group >;
	(*m_groups).resize(groupNum);
	for (int i = 0; i < groupNum; i++)
	{
		(*m_groups)[i] = m_tileCtrl->m_grids[i].m_group;
	}
	m_rvosim = new RVO::RVOSimulator();
	m_groupRVOSim = new RVO::RVOSimulator();
	agent_num = 0;
	// (pi*r^2/2) / maxAgentRadius;
	m_rvosim->setAgentDefaults(grid_size, PI*grid_size*grid_size / 8 / renderAgentRadius, 5.0f, grid_size*10.0f, renderAgentRadius, MAX_SPEED/*max speed*/);
	m_groupRVOSim->setAgentDefaults(grid_size, PI*grid_size*grid_size / 8 / renderAgentRadius, 15.0f, 15.0f, renderAgentRadius, MAX_SPEED/*max speed*/);
	// Allocate temp buffer for merging paths.

	//Initialize Agents
	m_agents = new HmAgent[m_maxAgents];
	m_activeAgents = (HmAgent**)malloc(m_maxAgents*sizeof(HmAgent*));
	if (!m_agents)
		return false;

	for (int i = 0; i < m_maxAgents; ++i)
	{
		m_agents[i].sim_ = m_rvosim;
		m_agents[i].HmAgent_id = i;
		m_agents[i].active = 0;
		m_agents[i].corridor.ClearPath();
	}

	//use navigation map to generate obstacles and grids
	float m_width = m_nav->bbox.DimX(); //这里必须是能够被2^n整除的数
	float m_height = m_nav->bbox.DimZ();
	float mapsize = (m_width>m_height) ? m_width : m_height;

	m_numofgridinrow = int(ceil(mapsize / grid_size));
	mapsize = m_numofgridinrow*grid_size;
	for (unsigned int i = 0; i<m_groups->size(); i++)
	{
		m_actGridIds.push_back(i);
	}

	m_fluidsim = new FluidSimulator();
	m_fluidsim->init(m_groups, m_activeAgents, &m_densegroups);


	return true;
}

const int HybridFrameworkCtrl::getAgentCount() const
{
	return m_maxAgents;
}

/// @par
/// 
/// Agents in the pool may not be in use.  Check #dtCrowdAgent.active before using the returned object.
HmAgent* HybridFrameworkCtrl::getAgent(int idx)
{
	return &m_agents[idx];
}

void HybridFrameworkCtrl::updateAgentParameters(int idx, float maxNeighborDist, int maxNeighborNum, float planHorizon, float radius, float maxSpeed)
{
	if (idx < 0 || idx > m_maxAgents)
		return;
	m_agents[idx].setHmAgent(maxNeighborDist, maxNeighborNum, planHorizon, planHorizon + m_gridsize, radius, maxSpeed);
}

/// @par
///
/// The agent's position will be constrained to the surface of the navigation mesh.
int HybridFrameworkCtrl::addAgent(float* pos
								, float maxNeighborDist, int maxNeighborNum, float planHorizon, float radius, float maxSpeed	// simulation parameters
								, float* target, float* vel, int color)
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
	float nearest[3];
	int tileID = -1;
	int faceID = -1;
	int gridID = -1;
	// Find nearest position on navmesh and place the agent there.
	bool res = GetNearestFace(pos, tileID, gridID, faceID, nearest);
	ag->curGrid = gridID;
	ag->curFace = faceID;
	ag->curTile = tileID;
	ag->color_id = color;
	ag->setHmAgent(maxNeighborDist, maxNeighborNum, planHorizon, planHorizon+m_gridsize, radius, maxSpeed);
	if (vel != NULL)
	{
		vecSet(ag->dvel, vel[0], vel[1], vel[2]);
		vecSet(ag->nvel, vel[0], vel[1], vel[2]);
		vecSet(ag->vel, vel[0], vel[1], vel[2]);
	}
	else
	{
		vecSet(ag->dvel, 0, 0, 0);
		vecSet(ag->nvel, 0, 0, 0);
		vecSet(ag->vel, 0, 0, 0);
	}
	vecCopy(ag->npos, nearest);

	ag->desiredSpeed = 0;

	ag->state = DT_CROWDAGENT_STATE_WALKING;
	
	ag->active = 1;
	ag->HmAgent_id = idx;
	if (gridID<0 || gridID>m_numofgridinrow*m_numofgridinrow){
		ag->active = 0;
		return -1;
	}
	return idx;
}

/// @par
///
/// The agent is deactivated and will no longer be processed.  Its #dtCrowdAgent object
/// is not removed from the pool.  It is marked as inactive so that it is available for reuse.
void HybridFrameworkCtrl::removeAgent(const int idx)
{
	if (idx >= 0 && idx < m_maxAgents)
	{
		m_agents[idx].active = 0;
		agent_num--;
	}
}

int HybridFrameworkCtrl::getActiveAgents(HmAgent** agents, const int maxAgents)
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

#include <fstream>

void HybridFrameworkCtrl::update(const float dt, int agentNum, int* agentIds, float* positions, float* velocities)
{
	TimeVal startTime = getPerfTime();
	m_rvosim->agents_.clear();
	m_rvosim->obstacles_.clear();
	
	m_groupRVOSim->agents_.clear();
	
	float agentSize = m_maxAgentRadius*m_maxAgentRadius * 4;
	// Check that all agents still have valid paths.
	//checkPathValidity(agents, nagents, dt);

	// Update async move request and path finder.
	m_maxGrids = m_actGridIds.size();

	for (int i = 0; i<m_maxGrids; i++)
	{
		(*m_groups)[m_actGridIds[i]].agentsInGroup.clear();
	}
	// Calculate steering.
	//printf("========================AGENT_INFO========================\n");
	int nagents = agentNum;
	//std::ofstream ofs("loglog.txt");
#pragma omp parallel for
	for (int i = 0; i < agentNum; ++i)
	{
		HmAgent* ag = &m_agents[agentIds[i]];
		ag->state = DT_CROWDAGENT_STATE_WALKING;
		ag->is_contour = false;
		ag->active = 1;
		m_activeAgents[i] = ag;
		float dvel[3] = { 0, 0, 0 };
		ag->corridor.GetSteerDir(ag->npos, dvel);
		std::cout << "dvel:" << " " << dvel[0] << " " << dvel[1] << " " << dvel[2] << std::endl;
		vcg::Point3f pt = ag->corridor.pts.front();
		std::cout << "corner size: " << ag->corridor.pts.size() << " " << pt.X() << " " << pt.Y() << " " << pt.Z() << std::endl;
		// Calculate speed scale, which tells the agent to slowdown at the end of the path.
		const float slowDownRadius = ag->radius_ * 2;	// TODO: make less hacky.
		const float speedScale = ag->corridor.GetDistanceToGoal(ag->npos, slowDownRadius) / slowDownRadius;
		
		ag->desiredSpeed = ag->maxSpeed_;
		vecScale(dvel, dvel, ag->desiredSpeed * speedScale);
		std::cout << "slowDownRadius: " << slowDownRadius << "speedScale: " << speedScale << "dvel:" << " " << dvel[0] << " " << dvel[1] << " " << dvel[2] << std::endl;
		ag->npos[0] = positions[i * 3];
		ag->npos[1] = positions[i * 3 + 1];
		ag->npos[2] = positions[i * 3 + 2];
		// Set the desired velocity.
		vecCopy(ag->dvel, dvel);
		ag->setHmAgent(ag->npos, ag->dvel);
		(*m_groups)[ag->curGrid].agentsInGroup.push_back(ag->HmAgent_id);
	}
	HmAgent** agents = m_activeAgents;
	
	//m_densityThreshold = EPSILON;
	for (int i = 0; i<m_maxGrids; i++)
	{
		(*m_groups)[m_actGridIds[i]].UpdateDensity(m_densityThreshold, agentSize);
		(*m_groups)[m_actGridIds[i]].avgVelocity.reset();
	}
	//
	TimeVal endTime = getPerfTime();
	std::printf("NavMesh Time: %f\n", getPerfDeltaTimeUsec(startTime, endTime) / 1000.0f);
	startTime = getPerfTime();
	MergeGrid();
	endTime = getPerfTime();
	std::printf("Merge Time: %f\n", getPerfDeltaTimeUsec(startTime, endTime) / 1000.0f);
	startTime = getPerfTime();

	m_fluidsim->doStep();
	for (int i = 0; i < ITER_REARRANGE; i++)
	{
		Rearrange();
	}
	endTime = getPerfTime();
	std::printf("Fluid Time: %f\n", getPerfDeltaTimeUsec(startTime, endTime) / 1000.0f);
	startTime = getPerfTime();
	for (unsigned int i = 0; i<m_densegroups.size(); i++)
	{

		/*for (unsigned int j = 0; j < m_densegroups[i].crowdGroup.size(); j++)
		{
			int gid = m_densegroups[i].crowdGroup[j];
			std::vector<Vector2> obstacle = (*m_groups)[gid].m_box.GetCorners();
			for (unsigned int k = 0; k<obstacle.size(); k++)
				obstacle[k] = obstacle[k] + (*m_groups)[gid].avgVelocity * dt;
			m_rvosim->addObstacle(obstacle, (float)(*m_groups)[gid].nowDensity, (*m_groups)[gid].avgVelocity);
		}
		*/
		for (unsigned int j = 0; j < m_densegroups[i].contours.size(); j++)
		{
			std::vector<Vector2> obstacle = m_densegroups[i].contours[j];
			for (unsigned int k = 0; k<obstacle.size(); k++)
				obstacle[k] = obstacle[k] + m_densegroups[i].avgVel * dt;
			m_rvosim->addObstacle(obstacle, m_densegroups[i].avgDensity, m_densegroups[i].avgVel);
		}
	}

	m_rvosim->processObstacles();
	m_rvosim->doStep();
	endTime = getPerfTime();
	std::printf("RVO Time: %f\n", getPerfDeltaTimeUsec(startTime, endTime) / 1000.0f);
	startTime = getPerfTime();

	// Integrate. and get new position
#pragma omp parallel for
	for (int i = 0; i < agentNum; ++i)
	{
		HmAgent* ag = agents[agentIds[i]];
		
		ag->nvel[0] = float(ag->velocity_.x());
		ag->nvel[1] = float(ag->dvel[1]);
		ag->nvel[2] = float(ag->velocity_.y());

		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;
		integrate(ag, dt);
		std::cout << "nvel:" << " " << ag->nvel[0] << " " << ag->nvel[1] << " " << ag->nvel[2] << std::endl;
		ag->corridor.UpdateDir(ag->npos, float(ag->radius_));
		int tileID = -1;
		int faceID = -1;
		int gridID = -1;
		velocities[i * 3] = ag->nvel[0];
		velocities[i * 3 + 1] = ag->nvel[1];
		velocities[i * 3 + 2] = ag->nvel[2];
		std::cout << "nvel:" << " " << ag->nvel[0] << " " << ag->nvel[1] << " " << ag->nvel[2] << std::endl;
		//ag->corridor.UpdateDir(ag->npos, float(ag->radius_));	// so here, we should refresh the navigation part every N frames
		float nearest[3];
		bool res = GetNearestFace(ag->npos, tileID, gridID, faceID, nearest);
		ag->curGrid = gridID;
		ag->curFace = faceID;
		ag->curTile = tileID;
		vecCopy(ag->npos, nearest);

		positions[i * 3] = ag->npos[0];
		positions[i * 3 + 1] = ag->npos[1];
		positions[i * 3 + 2] = ag->npos[2];

	}
	//ofs.close();
	endTime = getPerfTime();
	std::printf("Ajust Position Time: %f\n", getPerfDeltaTimeUsec(startTime, endTime) / 1000.0f);
}

void HybridFrameworkCtrl::MergeGrid()
{
	std::map<int, int> sameMark;
	int markid = 0;
	m_densegroups.clear();	//清空群体数组
	std::vector<int> regionMark;
	std::vector<int> denseGridIds;
	int gridNum = static_cast<int>(m_groups->size());
	regionMark.resize(gridNum);
	denseGridIds.resize(gridNum);

	for (int i = 0; i<gridNum; i++)
	{
		if ((*m_groups)[i].m_densityStatus == WHITE)
			continue;
		bool hasDense = false;
		for (vector<int>::iterator it = (*m_groups)[i].agentsInGroup.begin(); it != (*m_groups)[i].agentsInGroup.end(); it++)
		{
			// We add a bilinear interpolation to solve the continuity problem on the border of dense crowd
			std::vector<int> neighborIds;
			Vector2 leftBottom, rightTop;
			bool hasInterp = (*m_groups)[i].GetInterpolationNeighbors(m_agents[*it].position_, m_groups, neighborIds, leftBottom, rightTop);
			if (hasInterp)
			{
				float densities[4];
				memset(densities, 0.0f, 4 * sizeof(float));
				for (int di = 0; di < 4; di++)
				{
					if (neighborIds[di] >= 0)
					{
						densities[di] = (float)((*m_groups)[neighborIds[di]].nowDensity);
					}
				}
				float density = bilateralInterpolation(densities[0], densities[1], densities[2], densities[3],
					leftBottom, rightTop,
					m_agents[*it].position_);
				m_agents[*it].density = density;
				if (density >= m_densityThreshold)
				{
					hasDense = true;
				}
				else
				{
					m_agents[*it].active = 2;
					m_rvosim->agents_.push_back(&m_agents[*it]);
				}
			}
			else
			{
				m_agents[*it].density = 0;
				m_agents[*it].active = 2;
				m_rvosim->agents_.push_back(&m_agents[*it]);
			}
		}
		if (hasDense)
		{
			denseGridIds[markid] = i;
			regionMark[i] = markid++;	// initialize a mark for each grid
		}
	}
	int denseGridNum = markid;
	if (denseGridNum <= 0)
		return;
	for (int i = 0; i<denseGridNum; i++)
	{
		int gid = denseGridIds[i];
		for (int j = 0; j<4; j++)//遍历四个邻居
		{
			int ngid = (*m_groups)[gid].m_neighbor[j];
			if (ngid <= 0)
				continue;
			if ((*m_groups)[ngid].m_densityStatus == BLACK)
			{
				if (regionMark[ngid] >= regionMark[gid])
				{
					regionMark[ngid] = regionMark[gid];
				}
				else
				{
					std::map<int, int>::iterator iterPair = sameMark.find(regionMark[ngid]);
					int id = regionMark[ngid];
					while (iterPair != sameMark.end())	// maybe we don't need to have a loop, one level is enough
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
	m_densegroups.push_back(tempgroup);


	//处理同义区域，将其合并到同一个crowdgroup的vector里
	for (int i = 1; i<denseGridNum; i++)
	{
		int gid = denseGridIds[i];
		bool isNewGroup = true;
		for (unsigned int j = 0; j<m_densegroups.size(); j++)
		{
			if (regionMark[gid] == m_densegroups[j].RegionID)
			{
				m_densegroups[j].AddGrid(gid);
				isNewGroup = false;
				break;
			}
			std::map<int, int>::iterator iterPair = sameMark.find(regionMark[gid]);
			if (iterPair != sameMark.end() && iterPair->second == m_densegroups[j].RegionID)
			{
				regionMark[gid] = m_densegroups[j].RegionID;
				m_densegroups[j].AddGrid(gid);
				isNewGroup = false;
				break;
			}
		}
		if (isNewGroup == true)
		{
			CrowdGroup tempgroup;
			tempgroup.RegionID = regionMark[gid];
			tempgroup.AddGrid(gid);
			m_densegroups.push_back(tempgroup);
		}
	}
}

int compare(const void* a, const void* b)
{
	return (((elem_grid*)b)->score - ((elem_grid*)a)->score);
}

void HybridFrameworkCtrl::Rearrange()
{
	float elem_size = m_maxAgentRadius * 2;
	int grid_area = m_cellsize*m_cellsize;
	elem_grid* elem_groups = (elem_grid*)malloc(grid_area*sizeof(elem_grid));
#pragma omp parallel for
	for (unsigned int crowdi = 0; crowdi<m_densegroups.size(); crowdi++)
	{
		for (unsigned int gridi = 0; gridi < m_densegroups[crowdi].crowdGroup.size(); gridi++)
		{
			int gid = m_densegroups[crowdi].crowdGroup[gridi];
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
			int agentInRow = this->m_gridsize / m_maxAgentRadius / 2;
			float elem_grid_size = this->m_gridsize / (1.0*agentInRow);
			// Assign parameters from each agent to the agent list used for sorting.
			Vector2 mean_center(0, 0);
			int agentCount = 0;
			for (int i = 0; i<firstGroupNum; i++)
			{
				agent_grids[i].id = i;
				int agent_id = (*m_groups)[gid].agentsInGroup[i];
				agent_grids[i].center = Vector2(m_agents[agent_id].npos[0], m_agents[agent_id].npos[2]);
				agent_grids[i].agent = &m_agents[agent_id];
				agent_grids[i].radius = (float)m_agents[agent_id].radius_;
				mean_center += agent_grids[i].center;
				agentCount++;
			}
			mean_center /= firstGroupNum;

			for (int i = 0; i < 4; i++)
			{
				int nid = (*m_groups)[gid].m_neighbor[i];
				if (nid < 0)
					continue;
				for (unsigned int j = 0; j<(*m_groups)[nid].agentsInGroup.size(); j++)
				{
					agent_grids[agentCount].id = agentCount;
					int agent_id = (*m_groups)[nid].agentsInGroup[j];
					agent_grids[agentCount].center = Vector2(m_agents[agent_id].npos[0], m_agents[agent_id].npos[2]);
					agent_grids[agentCount].agent = &m_agents[agent_id];
					agent_grids[agentCount].radius = (float)m_agents[agent_id].radius_;
					agentCount++;
				}
			}

			for (int i = 0; i<people_num; i++)
			{
				agent_grids[i].score = float((agent_grids[i].center - mean_center).dot((*m_groups)[gid].avgVelocity));	// projection on velocity
			}

			//sort agents by projection
			qsort(agent_grids, people_num, sizeof(elem_grid), compare);

			//insert agent into buckets
			std::vector<std::list<elem_grid>> agent_bucket;
			Vector2 vertVelocity((*m_groups)[gid].avgVelocity.y(), -(*m_groups)[gid].avgVelocity.x());//顺时针旋转90度
			int count = -1;
			for (int i = 0; i<people_num; i++)
			{
				if (i == 0 || agent_grids[0].score - agent_grids[i].score>(count + 1)*elem_grid_size)	// create buckets when first guy in the bucket appears
				{
					std::list<elem_grid> bucket;
					bucket.push_back(agent_grids[i]);
					count++;
					agent_bucket.push_back(bucket);
				}
				else
				{
					std::list<elem_grid>::iterator iter;
					for (iter = agent_bucket[count].begin(); iter != agent_bucket[count].end(); iter++)
					{
						if (iter->center.dot(vertVelocity)>agent_grids[i].center.dot(vertVelocity))	// if the agent is on the left hand side of the tail
						{
							agent_bucket[count].insert(iter, agent_grids[i]);
							break;
						}
					}
					if (iter == agent_bucket[count].end())
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
				for (int j = 0; j<midId; j++)
				{
					riter++;
				}
				for (int j = midId + 1; j < bucketSize; j++)
				{
					liter++;
				}
				// liter is from the middle to the beginning, riter is from the middle to the end
				for (; liter != agent_bucket[i].rend(); liter++)
				{
					nextLiter = liter;
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
					if (i + 1<agent_bucket.size())
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

void HybridFrameworkCtrl::save(FILE* fp)
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

void HybridFrameworkCtrl::SetAgentCorridor(int aid, int cornerNum, float* corners)
{
	std::vector<vcg::Point3f> pathPts;
	pathPts.resize(cornerNum);
	for (int i = 0; i < cornerNum; i++)
	{
		pathPts[i].X() = corners[i * 3];
		pathPts[i].Y() = corners[i * 3 + 1];
		pathPts[i].Z() = corners[i * 3 + 2];
	}
	m_agents[aid].corridor.SetCorridor(pathPts);
}

bool HybridFrameworkCtrl::GetNearestFace(vcg::Point3f& pos, int& tileID, int& gridID, int& faceID, vcg::Point3f& nearestPos)
{
	Tile* tile = m_tileCtrl->GetTile(pos);
	tileID = tile->m_tileID;
	faceID = -1;
	return tile->GetNearestFace(pos, gridID, faceID, m_nav, m_tileCtrl->m_grids, nearestPos);
}

bool HybridFrameworkCtrl::GetNearestFace(const float* pos, int& tileID, int& gridID, int& faceID, float* nearestPos)
{
	vcg::Point3f p(pos[0], pos[1], pos[2]);
	vcg::Point3f n;
	bool res = GetNearestFace(p, tileID, gridID, faceID, n);
	nearestPos[0] = n.X();	nearestPos[1] = n.Y();	nearestPos[2] = n.Z();
	return res;
}

