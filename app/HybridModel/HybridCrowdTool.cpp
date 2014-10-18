#include "HybridCrowdTool.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <float.h>
#include "SDL.h"
#include "SDL_opengl.h"
#include "imgui.h"
#include "Sample.h"
#include "DetourDebugDraw.h"
#include "DetourObstacleAvoidance.h"
#include "DetourCommon.h"
#include "DetourNode.h"
#include "SampleInterfaces.h"
#include "CommonDef.h"
#include "HmAgent.h"
#include "HybridModel.h"

#ifdef WIN32
#	define snprintf _snprintf
#endif

static float frand()
{
	return (float)rand()/(float)RAND_MAX;
}

static bool isectSegAABB(const float* sp, const float* sq,
						 const float* amin, const float* amax,
						 float& tmin, float& tmax)
{
	static const float EPS = 1e-6f;
	
	float d[3];
	dtVsub(d, sq, sp);
	tmin = 0;  // set to -FLT_MAX to get first hit on line
	tmax = FLT_MAX;		// set to max distance ray can travel (for segment)
	
	// For all three slabs
	for (int i = 0; i < 3; i++)
	{
		if (fabsf(d[i]) < EPS)
		{
			// Ray is parallel to slab. No hit if origin not within slab
			if (sp[i] < amin[i] || sp[i] > amax[i])
				return false;
		}
		else
		{
			// Compute intersection t value of ray with near and far plane of slab
			const float ood = 1.0f / d[i];
			float t1 = (amin[i] - sp[i]) * ood;
			float t2 = (amax[i] - sp[i]) * ood;
			// Make t1 be intersection with near plane, t2 with far plane
			if (t1 > t2) dtSwap(t1, t2);
			// Compute the intersection of slab intersections intervals
			if (t1 > tmin) tmin = t1;
			if (t2 < tmax) tmax = t2;
			// Exit with no collision as soon as slab intersection becomes empty
			if (tmin > tmax) return false;
		}
	}
	
	return true;
}

static void getAgentBounds(const HmAgent* ag, float* bmin, float* bmax)
{
	const float* p = ag->npos;
	const float r = ag->params.radius;
	const float h = ag->params.height;
	bmin[0] = p[0] - r;
	bmin[1] = p[1];
	bmin[2] = p[2] - r;
	bmax[0] = p[0] + r;
	bmax[1] = p[1] + h;
	bmax[2] = p[2] + r;
}



HybridCrowdToolState::HybridCrowdToolState()
{
	m_toolParams.m_expandSelectedDebugDraw = true;
	m_toolParams.m_expandSimulationParameters = true;
	m_toolParams.m_expandAgentColor = false;
	m_toolParams.m_selectedAgentColor = 0;
	m_toolParams.m_showCorners = false;
	m_toolParams.m_showCollisionSegments = false;
	m_toolParams.m_showPath = false;
	m_toolParams.m_showVO = false;
	m_toolParams.m_showOpt = false;
	m_toolParams.m_showNeis = false;
	m_toolParams.m_expandDebugDraw = false;
	m_toolParams.m_showLabels = false;
	m_toolParams.m_showNodes = false;
	m_toolParams.m_showPerfGraph = false;
	m_toolParams.m_showDetailAll = false;
	m_toolParams.m_expandOptions = true;
	m_toolParams.m_anticipateTurns = true;
	m_toolParams.m_optimizeVis = true;
	m_toolParams.m_optimizeTopo = true;
	m_toolParams.m_obstacleAvoidance = true;
	m_toolParams.m_obstacleAvoidanceType = 3.0f;
	m_toolParams.m_separation = false;
	m_toolParams.m_separationWeight = 2.0f;
	m_toolParams.m_showGridLabels = false;
	m_toolParams.m_showContour = false;
	m_toolParams.m_showVelocityArrow = true;
	memset(m_trails, 0, sizeof(m_trails));
	
}

HybridCrowdToolState::~HybridCrowdToolState()
{

}

void HybridCrowdToolState::init(Sample *sample)
{
	if (m_sample != sample)
	{
		m_sample = (HybridModel*)sample;
	}
	
	// WARNING CHANGE NAV TO OURS
	//dtNavMesh* nav = m_sample->getNavMesh();
	NavMesh* nav = &m_sample->getInputMesh()->m_navMesh;
	HybridModelCrowd* crowd = ((HybridModel*)m_sample)->getHybridCrowd();
	NavMeshQuery* navquery = m_sample->getNavMeshQuery();
	if (nav && crowd && (m_nav != nav || m_crowd != crowd))
	{
		m_nav = nav;
		m_crowd = crowd;
	
		float gridsize;
		float ox,oy,ex,ey;
		((HybridModel*)m_sample)->getGridParams(gridsize, ox, oy, ex, ey);
		crowd->init(MAX_AGENTS, m_sample->getAgentRadius(), nav, navquery, gridsize, ox, oy, ex, ey);
	}
}

void HybridCrowdToolState::reset()
{
}

void HybridCrowdToolState::handleRender()
{
	//return;
	DebugDrawGL dd;
	const float rad = m_sample->getAgentRadius();
	
	//dtNavMesh* nav = m_sample->getNavMesh();
	NavMesh* nav = &m_sample->getInputMesh()->m_navMesh;
	HybridModelCrowd* crowd = ((HybridModel*)m_sample)->getHybridCrowd();
	if (!nav || !crowd)
		return;
	
/*	if (m_toolParams.m_showNodes && crowd->getPathQueue())
	{
		const dtNavMeshQuery* navquery = crowd->getPathQueue()->getNavQuery();
		if (navquery)
			duDebugDrawNavMeshNodes(&dd, *navquery);
	}
	*/
	dd.depthMask(false);
	
	// Draw paths
	if (m_toolParams.m_showPath)
	{
		for (int i = 0; i < crowd->getAgentCount(); i++)
		{
			const HmAgent* ag =crowd->getAgent(i);
			if (!ag->active)
				continue;
		}
	}
	
	if (m_targetFace >= 0)
		duDebugDrawCross(&dd, m_targetPos[0],m_targetPos[1]+0.1f,m_targetPos[2], rad, duRGBA(255,255,255,192), 2.0f);
	
	
	
	// Trail
	for (int i = 0; i < crowd->getAgentCount(); ++i)
	{
		const HmAgent* ag = crowd->getAgent(i);
		if (!ag->active) continue;
		
		const AgentTrail* trail = &m_trails[i];
		const float* pos = ag->npos;
		
		dd.begin(DU_DRAW_LINES,3.0f);
		float prev[3], preva = 1;
		dtVcopy(prev, pos);
		for (int j = 0; j < AGENT_MAX_TRAIL-1; ++j)
		{
			const int idx = (trail->htrail + AGENT_MAX_TRAIL-j) % AGENT_MAX_TRAIL;
			const float* v = &trail->trail[idx*3];
			float a = 1 - j/(float)AGENT_MAX_TRAIL;
			dd.vertex(prev[0],prev[1]+0.1f,prev[2], duRGBA(0,0,0,(int)(128*preva)));
			dd.vertex(v[0],v[1]+0.1f,v[2], duRGBA(0,0,0,(int)(128*a)));
			preva = a;
			dtVcopy(prev, v);
		}
		dd.end();
		
	}
	
	// Agent cylinders.
	for (int i = 0; i < crowd->getAgentCount(); ++i)
	{
		const HmAgent* ag = crowd->getAgent(i);
		if (!ag->active) continue;
		
		const float radius = ag->params.radius;
		const float* pos = ag->npos;
		
		unsigned int col = duRGBA(0,0,0,32);
		if (ag->is_selected == true)
		{
			col = duRGBA(255,0,0,128);
		}	
		duDebugDrawCircle(&dd, pos[0], pos[1], pos[2], radius, col, 2.0f);
	}
	
	for (int i = 0; i < crowd->getAgentCount(); ++i)
	{
		const HmAgent* ag = crowd->getAgent(i);
		if (!ag->active) continue;
		
		const float height = ag->params.height;
		const float radius = ag->params.radius;
		const float* pos = ag->npos;
		
		unsigned int col;
		switch(ag->color_id){
			case 0:
				col = duRGBA(220,0,0,128);
				break;
			case 1:
				col = duRGBA(0,220,0,128);
				break;
			case 2:
				col = duRGBA(0,0,220,128);
				break;
			case 3:
				col = duRGBA(220,220,0,128);
				break;
			case 4:
				col = duRGBA(0,220,220,128);
				break;
			case 5:
				col = duRGBA(220,0,220,128);
				break;
			default:
				col = duRGBA(220,220,220,128);
				break;
		}		
		if (ag->is_selected == true)
			col = duRGBA(218,112,214, 128);
		if (ag->targetState == DT_CROWDAGENT_TARGET_REQUESTING || ag->targetState == DT_CROWDAGENT_TARGET_WAITING_FOR_QUEUE)
			col = duLerpCol(col, duRGBA(128,0,255,128), 32);
		else if (ag->targetState == DT_CROWDAGENT_TARGET_WAITING_FOR_PATH)
			col = duLerpCol(col, duRGBA(128,0,255,128), 128);
		else if (ag->targetState == DT_CROWDAGENT_TARGET_FAILED)
			col = duRGBA(255,32,16,128);
		else if (ag->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
			col = duLerpCol(col, duRGBA(64,255,0,128), 128);

		if(ag->active == 3&&m_toolParams.m_showContour)
			col = duRGBA(0,0,220,128);
		
		duDebugDrawCylinder(&dd, pos[0]-radius, pos[1]+radius*0.1f, pos[2]-radius,
							pos[0]+radius, pos[1]+height, pos[2]+radius, col);

		
		//duDebugDrawBoxWire(&dd, pos[0]-radius, pos[1]+radius*0.1f, pos[2]-radius,
		//					pos[0]+radius, pos[1]+height, pos[2]+radius, col,2);
	}
	
	
	// Velocity stuff.
	for (int i = 0; i < crowd->getAgentCount(); ++i)
	{
		const HmAgent* ag = crowd->getAgent(i);
		if (!ag->active) continue;
		
		const float radius = ag->params.radius;
		const float height = ag->params.height;
		const float* pos = ag->npos;
		const float* vel = ag->vel;
		const float* dvel = ag->dvel;
		
		unsigned int col = duRGBA(220,220,220,192);
		if (ag->targetState == DT_CROWDAGENT_TARGET_REQUESTING || ag->targetState == DT_CROWDAGENT_TARGET_WAITING_FOR_QUEUE)
			col = duLerpCol(col, duRGBA(128,0,255,192), 32);
		else if (ag->targetState == DT_CROWDAGENT_TARGET_WAITING_FOR_PATH)
			col = duLerpCol(col, duRGBA(128,0,255,192), 128);
		else if (ag->targetState == DT_CROWDAGENT_TARGET_FAILED)
			col = duRGBA(255,32,16,192);
		else if (ag->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
			col = duLerpCol(col, duRGBA(64,255,0,192), 128);
		
		duDebugDrawCircle(&dd, pos[0], pos[1]+height, pos[2], radius, col, 2.0f);
		if(m_toolParams.m_showVelocityArrow)
		{
			duDebugDrawArrow(&dd, pos[0],pos[1]+height,pos[2],
							 pos[0]+dvel[0],pos[1]+height+dvel[1],pos[2]+dvel[2],
							 0.0f, 0.4f, duRGBA(0,192,255,192), 1.0f);
		
			duDebugDrawArrow(&dd, pos[0],pos[1]+height,pos[2],
							 pos[0]+vel[0],pos[1]+height+vel[1],pos[2]+vel[2],
							 0.0f, 0.4f, duRGBA(0,0,0,160), 2.0f);
		}
	}

	/*	
	int crowdNum = crowd->m_fluidgroups.size();
	unsigned int fcol[6];
	for (int i = 0; i < 6; i++)
		fcol[i] = duRGBA(64, 64, 128, 128);
	for (int i = 0; i < crowdNum; i++)
	{
		for (unsigned int j = 0; j < crowd->m_fluidgroups[i].crowdGrid.size(); j++)
		{
			int gid = crowd->m_fluidgroups[i].crowdGrid[j];
			std::vector<Vector2> obstacle = (*(crowd->m_grids))[gid].m_box.GetCorners();
			//for (unsigned int k = 0; k<obstacle.size(); k++)
			//	obstacle[k] = obstacle[k] + (*(crowd->m_grids))[gid].avgVelocity;
			duDebugDrawBox(&dd, obstacle[0].x(), 12.0f, obstacle[0].y(), obstacle[2].x(), 16.0f, obstacle[2].y(), fcol);
		}
	}
	*/
	int crowdNum = crowd->m_fluidgroups.size();
	unsigned int fcol[6];
	for (int i = 0; i < 6; i++)
		fcol[i] = duRGBA(64, 64, 128, 128);
	for (int i = 0; i < crowdNum; i++)
	{
		for (unsigned int j = 0; j < crowd->m_fluidgroups[i].crowdGroup.size(); j++)
		{
			int gid = crowd->m_fluidgroups[i].crowdGroup[j];
			std::vector<Vector2> obstacle = (*(crowd->m_groups))[gid].m_box.GetCorners();
			for (unsigned int k = 0; k<obstacle.size(); k++)
				obstacle[k] = obstacle[k] + (*(crowd->m_groups))[gid].avgVelocity * 0.03;
			duDebugDrawBox(&dd, obstacle[0].x(), 12.0f, obstacle[0].y(), obstacle[2].x(), 16.0f, obstacle[2].y(), fcol);
			/*
			std::vector<Vector2> obstacle = crowd->m_fluidgroups[i].contours[j];
			for (unsigned int k = 0; k<obstacle.size(); k++)
				obstacle[k] = obstacle[k] + crowd->m_fluidgroups[i].avgVel*0.03;
			dd.begin(DU_DRAW_QUADS, 10.0f);
			for (int k = 0; k < 4; k++){
				float pos[3];
				pos[0] = obstacle[k].x();
				pos[1] = 15.0f;
				pos[2] = obstacle[k].y();
				dd.vertex(pos, duRGBA(64, 64, 128, 128));
			}*/
			dd.end();
		}
	}
	dd.depthMask(true);
}

void HybridCrowdToolState::handleRenderOverlay(double* proj, double* model, int* view)
{
	GLdouble x, y, z;
	
	// Draw start and end point labels
	if (m_targetFace >= 0 && gluProject((GLdouble)m_targetPos[0], (GLdouble)m_targetPos[1], (GLdouble)m_targetPos[2],
								  model, proj, view, &x, &y, &z))
	{
		imguiDrawText((int)x, (int)(y+25), IMGUI_ALIGN_CENTER, "TARGET", imguiRGBA(0,0,0,220));
	}
	
	char label[32];
	
	if (m_toolParams.m_showLabels)
	{
		HybridModelCrowd* crowd = ((HybridModel*)m_sample)->getHybridCrowd();
		if (crowd)
		{
			for (int i = 0; i < crowd->getAgentCount(); ++i)
			{
				const HmAgent* ag = crowd->getAgent(i);
				if (!ag->active) continue;
				const float* pos = ag->npos;
				const float h = ag->params.height;
				if (gluProject((GLdouble)pos[0], (GLdouble)pos[1]+h, (GLdouble)pos[2],
							   model, proj, view, &x, &y, &z))
				{
					snprintf(label, 32, "%d", i);
					imguiDrawText((int)x, (int)y+15, IMGUI_ALIGN_CENTER, label, imguiRGBA(0,0,0,220));
				}
			}			
		}
	}	
	if (m_toolParams.m_showGridLabels)
	{
		HybridModelCrowd* crowd = ((HybridModel*)m_sample)->getHybridCrowd();
		const HmAgent* ag = crowd->getAgent(0);
		float height = ag->params.height;
		for(int i=0;i<crowd->m_numofgridinrow*crowd->m_numofgridinrow;i++)
		{
			
			if (gluProject((GLdouble)(*crowd->m_groups)[i].m_box.center_x(), (GLdouble)height, (GLdouble)(*crowd->m_groups)[i].m_box.center_y(), model, proj, view, &x, &y, &z))
			{
				snprintf(label, 32, "%d", (*crowd->m_groups)[i].groupID);
				imguiDrawText((int)x, (int)y+15, IMGUI_ALIGN_CENTER, label, imguiRGBA(255,255,255,220));
			}
		}
	}
}

void HybridCrowdToolState::handleUpdate(const float dt)
{
	if (m_run)
		updateTick(dt);
}

void HybridCrowdToolState::addAgent(const float* p, int color, SimulationParams simParams)
{
	if (!m_sample) return;
	HybridModelCrowd* crowd = ((HybridModel*)m_sample)->getHybridCrowd();
	
	dtCrowdAgentParams ap;
	memset(&ap, 0, sizeof(ap));
	ap.radius = m_sample->getAgentRadius();
	ap.height = m_sample->getAgentHeight();
	ap.maxAcceleration = 8.0f;
	ap.maxSpeed = simParams.m_preferSpeed;
	ap.updateFlags = 0; 
	int idx = crowd->addAgent(p, &ap, simParams, NULL, NULL, color);
	if (idx != -1)
	{
		if (m_targetFace >= 0)
			crowd->requestMoveTarget(idx, m_targetFace, m_targetPos);
		
		// Init trail
		AgentTrail* trail = &m_trails[idx];
		for (int i = 0; i < AGENT_MAX_TRAIL; ++i)
			dtVcopy(&trail->trail[i*3], p);
		trail->htrail = 0;
	}
}

void HybridCrowdToolState::removeAgent(const int idx)
{
	if (!m_sample) return;
	HybridModelCrowd* crowd = ((HybridModel*)m_sample)->getHybridCrowd();

	crowd->removeAgent(idx);
}


static void calcVel(float* vel, const float* pos, const float* tgt, const float speed)
{
	dtVsub(vel, tgt, pos);
	vel[1] = 0.0;
	dtVnormalize(vel);
	dtVscale(vel, vel, speed);
}

void HybridCrowdToolState::setMoveTarget(const float* p, bool adjust)
{
	if (!m_sample) return;
	
	// Find nearest point on navmesh and set move request to that location.
	NavMeshQuery* navquery = m_sample->getNavMeshQuery();
	HybridModelCrowd* crowd = ((HybridModel*)m_sample)->getHybridCrowd();
	NavMesh* nav = &m_sample->getInputMesh()->m_navMesh;
//	const dtQueryFilter* filter = crowd->getFilter();
	const float* ext = crowd->getQueryExtents();

	if (adjust)
	{
		crowd->m_selected.clear();
		float vel[3];
		// Request velocity
		for (int i = 0; i < crowd->getAgentCount(); ++i)
		{
			const HmAgent* ag = crowd->getAgent(i);
			if (!ag->active) continue;
			calcVel(vel, ag->npos, p, ag->params.maxSpeed);
			crowd->requestMoveVelocity(i, vel);
		}
	}
	else
	{
		//navquery->findNearestPoly(p, ext, filter, &m_targetRef, m_targetPos);
		navquery->GetNearestFace(p, m_targetFace, m_targetPos);
		if(crowd->m_selected.empty())
		{
			for (int i = 0; i < crowd->getAgentCount(); ++i)
			{
				const HmAgent* ag = crowd->getAgent(i);
				if (!ag->active) continue;
				crowd->requestMoveTarget(i, m_targetFace, m_targetPos);
			}
		}
		else
		{
			for (unsigned int i = 0; i < crowd->m_selected.size(); ++i)
			{
				const HmAgent* ag = crowd->m_selected[i];
				if (!ag->active) continue;
				crowd->requestMoveTarget(ag->HmAgent_id, m_targetFace, m_targetPos);
			}
		}
	}
}

void HybridCrowdToolState::updateAgentParams()
{
	if (!m_sample) return;
	HybridModelCrowd* crowd = ((HybridModel*)m_sample)->getHybridCrowd();
	if (!crowd) return;
	
	unsigned char updateFlags = 0;
	unsigned char obstacleAvoidanceType = 0;
	
	obstacleAvoidanceType = (unsigned char)m_toolParams.m_obstacleAvoidanceType;
	
	dtCrowdAgentParams params;
	
	for (int i = 0; i < crowd->getAgentCount(); ++i)
	{
		const HmAgent* ag = crowd->getAgent(i);
		if (!ag->active) continue;
		memcpy(&params, &ag->params, sizeof(dtCrowdAgentParams));
		params.updateFlags = updateFlags;
		params.obstacleAvoidanceType = obstacleAvoidanceType;
		params.separationWeight = m_toolParams.m_separationWeight;
		crowd->updateAgentParameters(i, &params);
	}	
}

/*
	最重要的函数，木有之一，所有的仿真输入输出就是从这里来的
*/
void HybridCrowdToolState::updateTick(const float dt)
{
	if (!m_sample) return;
	NavMesh* nav = m_sample->getNavMesh();
	HybridModelCrowd* crowd = ((HybridModel*)m_sample)->getHybridCrowd();
	if (!nav || !crowd) return;
	
	TimeVal startTime = getPerfTime();
	
	crowd->update(dt);
	
	TimeVal endTime = getPerfTime();
	
	// Update agent trails
	for (int i = 0; i < crowd->getAgentCount(); ++i)
	{
		HmAgent* ag = crowd->getAgent(i);
		AgentTrail* trail = &m_trails[i];
		if (!ag->active)
			continue;
		/*ag->position_.SetX(double(ag->npos[0]));
		ag->position_.SetY(double(ag->npos[2]));
		ag->prefVelocity_.SetX(double(ag->dvel[0]));
		ag->prefVelocity_.SetY(double(ag->dvel[2]));*/
		// Update agent movement trail.
		trail->htrail = (trail->htrail + 1) % AGENT_MAX_TRAIL;
		dtVcopy(&trail->trail[trail->htrail*3], ag->npos);
	}
	
	printf("Total Time: %f\n",getPerfDeltaTimeUsec(startTime, endTime) / 1000.0f);
}

void HybridCrowdToolState::hilightAgent(const int idx)
{
	
}

int HybridCrowdToolState::hitTestAgents(const float* s, const float* p)
{
	if (!m_sample) return -1;
	HybridModelCrowd* crowd = ((HybridModel*)m_sample)->getHybridCrowd();
	
	int isel = -1;
	float tsel = FLT_MAX;

	for (int i = 0; i < crowd->getAgentCount(); ++i)
	{
		const HmAgent* ag = crowd->getAgent(i);
		if (!ag->active) continue;
		float bmin[3], bmax[3];
		getAgentBounds(ag, bmin, bmax);
		float tmin, tmax;
		if (isectSegAABB(s, p, bmin,bmax, tmin, tmax))
		{
			if (tmin > 0 && tmin < tsel)
			{
				isel = i;
				tsel = tmin;
			} 
		}
	}

	return isel;
}


//=======================================================================

HybridCrowdTool::HybridCrowdTool(void)
{
}

HybridCrowdTool::~HybridCrowdTool(void)
{
}

void HybridCrowdTool::init(Sample *sample)
{
	if (m_sample != sample)
	{
		m_sample = (HybridModel*)sample;
	}
	
	if (!sample)
		return;
		
	m_state = (HybridCrowdToolState*)sample->getToolState(type());
	if (!m_state)
	{
		m_state = new HybridCrowdToolState();
		((HybridModel*)m_sample)->setToolState(type(), m_state);
	}
	m_state->init(sample);
	m_densityThreshold = 0.65;
	m_randomCrowdNumber = 0;
}
void HybridCrowdTool::reset()
{	
}

void HybridCrowdTool::handleMenu()
{
	if (!m_state)
		return;
	HybridCrowdToolParams* params = m_state->getToolParams();
	HybridModelCrowd* hmc = ((HybridModel*)m_sample)->getHybridCrowd();

	if (imguiCheck("Create Agent", m_mode == TOOLMODE_CREATE))//Create Agents
		m_mode = TOOLMODE_CREATE;
	if (imguiCheck("Set Target", m_mode == TOOLMODE_MOVE_TARGET))//Move Target
		m_mode = TOOLMODE_MOVE_TARGET;
	if (imguiCheck("Select Agent", m_mode == TOOLMODE_SELECT))//Select Agent
		m_mode = TOOLMODE_SELECT;
	imguiSeparatorLine();
	imguiSlider("Random Crowd Number", &m_randomCrowdNumber, 0.0f, 10000.0f, 50.0f);
	if (imguiButton("Random Crowd Generation"))
	{
		for (int i = 0; i < m_randomCrowdNumber; i++)
		{
			
			float pt[3];
			m_state->addAgent(pt);
		}
	}

	imguiSeparatorLine();

	if (imguiCollapse("Choose Agent Color", 0, params->m_expandAgentColor))//Selected Debug Draw
		params->m_expandAgentColor = !params->m_expandAgentColor;
	if (params->m_expandAgentColor)
	{
		imguiIndent();
		if (imguiCheck("red", params->m_selectedAgentColor == 0))
			params->m_selectedAgentColor = 0;
		if (imguiCheck("green", params->m_selectedAgentColor == 1))
			params->m_selectedAgentColor = 1;
		if (imguiCheck("blue", params->m_selectedAgentColor == 2))
			params->m_selectedAgentColor = 2;
		if (imguiCheck("yellow", params->m_selectedAgentColor == 3))
			params->m_selectedAgentColor = 3;
		if (imguiCheck("purple", params->m_selectedAgentColor == 4))
			params->m_selectedAgentColor = 4;
		if (imguiCheck("cyan", params->m_selectedAgentColor == 5))
			params->m_selectedAgentColor = 5;
		imguiUnindent();
	}
	imguiSeparatorLine();

	if (imguiButton("Save Simulation File"))
	{
		FILE* fp = fopen("Logs/log.txt", "w");
		hmc->save(fp);
		fclose(fp);
	}

	imguiSeparatorLine();
	imguiSlider("Minimum Density", &m_densityThreshold, 0.05f, 1.1f, 0.05f);
	
	hmc->SetDensityThreshold(m_densityThreshold);
	imguiSeparatorLine();
	
	if (imguiCollapse("Show Simulation Attributes", 0, params->m_expandSelectedDebugDraw))//Selected Debug Draw
		params->m_expandSelectedDebugDraw = !params->m_expandSelectedDebugDraw;
		
	if (params->m_expandSelectedDebugDraw)
	{
		imguiIndent();
		if (imguiCheck("Show Boundarys", params->m_showContour))//Show Contour
			params->m_showContour = !params->m_showContour;
		if (imguiCheck("Show Grid Ids", params->m_showGridLabels))//Show GridID
			params->m_showGridLabels = !params->m_showGridLabels;
		if (imguiCheck("Show Velocities", params->m_showVelocityArrow))//Show GridID
			params->m_showVelocityArrow = !params->m_showVelocityArrow;
		imguiUnindent();
	}
	imguiSeparatorLine();
	if (imguiCollapse("Modify Simulation Parameters", 0, params->m_expandSimulationParameters))//Selected Debug Draw
		params->m_expandSimulationParameters = !params->m_expandSimulationParameters;
	if (params->m_expandSimulationParameters)
	{
		imguiSlider("Max. Neighbor Dist.", &params->m_simParams.m_maxNeighborDist, 3.0f, 30, 1.0f);
		imguiSlider("Max. Num. Neighbors", &params->m_simParams.m_maxNumNeighbors, 1.0f, 100.0f, 1.0f);
		imguiSlider("Plan Horizon", &params->m_simParams.m_planHorizon, 1, 30, 1.0f);
		imguiSlider("Agent Radius.", &params->m_simParams.m_agentRadius, 0.3f, 2.2f, 0.1f);
		imguiSlider("Prefer Speed.", &params->m_simParams.m_preferSpeed, 1.2f, 2.2f, 0.1f);
		if (imguiButton("Set Params.")) {
			for (unsigned int i = 0; i < ((HybridModel*)m_sample)->getHybridCrowd()->m_selected.size(); i++) {
				((HybridModel*)m_sample)->getHybridCrowd()->m_selected[i]->setHmAgent(params->m_simParams);
			}
		}
		if (imguiButton("Reset Params.")) {
			params->m_simParams.m_maxNeighborDist = 15.0f;
			params->m_simParams.m_maxNumNeighbors = 10.0f;
			params->m_simParams.m_planHorizon = 10.0f;
			params->m_simParams.m_agentRadius = 0.8f;
			params->m_simParams.m_preferSpeed = 1.4f;
		}
	
		float pc1 = RVO::Agent::adjMat[0][0] * (params->m_simParams.m_maxNeighborDist - 15.0f) / 13.5f +
			RVO::Agent::adjMat[0][1] * (params->m_simParams.m_maxNumNeighbors - 10.0f) / 49.5f +
			RVO::Agent::adjMat[0][2] * (params->m_simParams.m_planHorizon - 30.0f) / 14.5f +
			RVO::Agent::adjMat[0][3] * (params->m_simParams.m_agentRadius - 0.8f) / 0.85f +
			RVO::Agent::adjMat[0][4] * (params->m_simParams.m_preferSpeed - 1.4f) / 0.5f;

		float pc2 = RVO::Agent::adjMat[1][0] * (params->m_simParams.m_maxNeighborDist - 15.0f) / 13.5f +
			RVO::Agent::adjMat[1][1] * (params->m_simParams.m_maxNumNeighbors - 10.0f) / 49.5f +
			RVO::Agent::adjMat[1][2] * (params->m_simParams.m_planHorizon - 30.0f) / 14.5f +
			RVO::Agent::adjMat[1][3] * (params->m_simParams.m_agentRadius - 0.8f) / 0.85f +
			RVO::Agent::adjMat[1][4] * (params->m_simParams.m_preferSpeed - 1.4f) / 0.5f;
		
		pc1 = -1 + (pc1 + 0.86) / 3.16 * 2;	// keep it between [-1, 1]
		pc2 = -1 + (pc2 + 2.21) / 3.6 * 2;

		imguiSlider("PC1", &pc1, -1.0f, 1.0f, 0.01f, false);
		imguiSlider("PC2", &pc2, -1.0f, 1.0, 0.01f, false);
	}
}



void HybridCrowdTool::handleClick(const float* s, const float* p, bool shift)
{
	if (!m_sample) return;
	if (!m_state) return;
	InputMesh* mesh = m_sample->getInputMesh();
	if (!mesh) return;
	HybridModelCrowd* crowd = ((HybridModel*)m_sample)->getHybridCrowd();
	if (!crowd) return;
	HybridCrowdToolParams* params = m_state->getToolParams();
	if (m_mode == TOOLMODE_CREATE)
	{
		if (shift)
		{
			// Delete
			int ahit = m_state->hitTestAgents(s,p);
			if (ahit != -1)
				m_state->removeAgent(ahit);
		}
		else
		{
			// Add
			m_state->addAgent(p, params->m_selectedAgentColor, params->m_simParams);
		}
	}
	else if (m_mode == TOOLMODE_MOVE_TARGET)
	{
		m_state->setMoveTarget(p, shift);
	}
	else if (m_mode == TOOLMODE_SELECT)
	{
		// Highlight
		int ahit = m_state->hitTestAgents(s,p);
		m_state->hilightAgent(ahit);
		((HybridModel*)m_sample)->getHybridCrowd()->m_selected.clear();
	}
	else if (m_mode == TOOLMODE_TOGGLE_POLYS)
	{
	}
	
}

void HybridCrowdTool::handleStep()
{
	if (!m_state) return;
	
	const float dt = 1.0f/20.0f;
	m_state->updateTick(dt);

	m_state->setRunning(false);
}

void HybridCrowdTool::handleToggle()
{
	if (!m_state) return;
	m_state->setRunning(!m_state->isRunning());
}

void HybridCrowdTool::handleUpdate(const float dt)
{
}

void HybridCrowdTool::handleRender()
{
}

void HybridCrowdTool::handleRenderOverlay(double* proj, double* model, int* view)
{
	// Tool help
	const int h = view[3];
	int ty = h-60;
	
	if (m_mode == TOOLMODE_CREATE)
	{
		imguiDrawText(280, ty, IMGUI_ALIGN_LEFT, "Left Mouse: Create Agent  Shift+Left Mouse: Remove Agent", imguiRGBA(255,255,255,192));	//LMB: add agent.  Shift+LMB: remove agent.
	}
	else if (m_mode == TOOLMODE_MOVE_TARGET)
	{
		imguiDrawText(280, ty, IMGUI_ALIGN_LEFT, "Left Mouse: Set Target  Shift+Left Mouse: Set Velocity", imguiRGBA(255,255,255,192));	//LMB: set move target.  Shift+LMB: adjust set velocity.
		ty -= 20;
		//imguiDrawText(280, ty, IMGUI_ALIGN_LEFT, "Setting velocity will move the agents without pathfinder.", imguiRGBA(255,255,255,192));	
	}
	else if (m_mode == TOOLMODE_SELECT)
	{
		imguiDrawText(280, ty, IMGUI_ALIGN_LEFT, "Left Mouse: Select Agent  B: Batch Selection On/Off", imguiRGBA(255,255,255,192));	//LMB: select agent.
	}
	ty -= 20;
	char agent_num[128];
	sprintf(agent_num, "Current Agent #: %d", ((HybridModel*)m_sample)->getHybridCrowd()->agent_num);
	imguiDrawText(280, ty, IMGUI_ALIGN_LEFT, agent_num, imguiRGBA(255,255,255,192));	//SPACE: Run/Pause simulation.  1: Step simulation.
	ty -= 20;
	imguiDrawText(280, ty, IMGUI_ALIGN_LEFT, "SPACE: Run/Pause simulation.  1: Step simulation.", imguiRGBA(255,255,255,192));	//SPACE: Run/Pause simulation.  1: Step simulation.
	ty -= 20;

	if (m_state && m_state->isRunning())
		imguiDrawText(280, ty, IMGUI_ALIGN_LEFT, "- Runing -", imguiRGBA(255,32,16,255));	
	else 
		imguiDrawText(280, ty, IMGUI_ALIGN_LEFT, "- Pause -", imguiRGBA(255,255,255,128));	
}

void HybridCrowdTool::handleDrag(const float* tl, const float* tr,const float* bl, const float* br )
{
	
	if (!m_sample) return;
	if (!m_state) return;
	InputMesh* mesh = m_sample->getInputMesh();
	if (!mesh) return;
	HybridModelCrowd* crowd = ((HybridModel*)m_sample)->getHybridCrowd();
	if (!crowd) return;
	
	Vector2 tlvec(tl[0],tl[2]);
	Vector2 trvec(tr[0],tr[2]);
	Vector2 blvec(bl[0],bl[2]);
	Vector2 brvec(br[0],br[2]);

	Vector2 hvec = blvec-tlvec;
	Vector2 wvec = trvec-tlvec;
	double hvalue = sqrt(hvec.dot(hvec));
	double wvalue = sqrt(wvec.dot(wvec));
	hvec = hvec/hvalue;
	wvec = wvec/wvalue;

	if (m_mode == TOOLMODE_SELECT)
	{
		// Highlight
		//int ahit = m_state->hitTestAgents(s,p);
		//m_state->hilightAgent(ahit);
		crowd->m_selected.clear();
		for (int i = 0; i < crowd->getAgentCount(); ++i)
		{
			HmAgent* ag = crowd->getAgent(i);
			ag->is_selected = false;
			if (!ag->active) continue;
			Vector2 pvec(ag->npos[0],ag->npos[2]);
			Vector2 dis2tl = pvec - tlvec;
			double hpos = dis2tl.dot(hvec);
			double wpos = dis2tl.dot(wvec);

			if(hpos>0&&wpos>0&&hpos<hvalue&&wpos<wvalue)
			{
				crowd->m_selected.push_back(ag);
				m_state->hilightAgent(i); //该函数还未实现
				ag->is_selected = true;
			}
		}

	}
}