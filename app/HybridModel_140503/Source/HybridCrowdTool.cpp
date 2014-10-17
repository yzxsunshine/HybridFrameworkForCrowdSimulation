#include "HybridCrowdTool.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <float.h>
#include "SDL.h"
#include "SDL_opengl.h"
#include "imgui.h"
#include "InputGeom.h"
#include "Sample.h"
#include "DetourCrowd.h"
#include "DetourDebugDraw.h"
#include "DetourObstacleAvoidance.h"
#include "DetourCommon.h"
#include "DetourNode.h"
#include "SampleInterfaces.h"

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

static void getAgentBounds(const dtCrowdAgent* ag, float* bmin, float* bmax)
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
//		m_oldFlags = m_sample->getNavMeshDrawFlags();
//		m_sample->setNavMeshDrawFlags(m_oldFlags & ~DU_DRAWNAVMESH_CLOSEDLIST);
	}
	
	// WARNING CHANGE NAV TO OURS
	//dtNavMesh* nav = m_sample->getNavMesh();
	NavMesh* nav = &m_sample->getInputMesh()->m_navMesh;
	HybridModelCrowd* crowd = m_sample->getHybridCrowd();
	
	if (nav && crowd && (m_nav != nav || m_crowd != crowd))
	{
		m_nav = nav;
		m_crowd = crowd;
	
		float gridsize;
		float ox,oy,ex,ey;
		m_sample->getGridParams(gridsize,ox,oy,ex,ey);
		crowd->init(MAX_AGENTS, m_sample->getAgentRadius(), nav, gridsize, ox, oy, ex, ey);
		
		// Make polygons with 'disabled' flag invalid.
///		crowd->getEditableFilter()->setExcludeFlags(SAMPLE_POLYFLAGS_DISABLED);
		/*
		// Setup local avoidance params to different qualities.
		dtObstacleAvoidanceParams params;
		// Use mostly default settings, copy from dtCrowd.
		memcpy(&params, crowd->getObstacleAvoidanceParams(0), sizeof(dtObstacleAvoidanceParams));
		
		// Low (11)
		params.velBias = 0.5f;
		params.adaptiveDivs = 5;
		params.adaptiveRings = 2;
		params.adaptiveDepth = 1;
		crowd->setObstacleAvoidanceParams(0, &params);
		
		// Medium (22)
		params.velBias = 0.5f;
		params.adaptiveDivs = 5; 
		params.adaptiveRings = 2;
		params.adaptiveDepth = 2;
		crowd->setObstacleAvoidanceParams(1, &params);
		
		// Good (45)
		params.velBias = 0.5f;
		params.adaptiveDivs = 7;
		params.adaptiveRings = 2;
		params.adaptiveDepth = 3;
		crowd->setObstacleAvoidanceParams(2, &params);
		
		// High (66)
		params.velBias = 0.5f;
		params.adaptiveDivs = 7;
		params.adaptiveRings = 3;
		params.adaptiveDepth = 3;
		
		crowd->setObstacleAvoidanceParams(3, &params);*/
	}
}

void HybridCrowdToolState::reset()
{
}

void HybridCrowdToolState::handleRender()
{
	DebugDrawGL dd;
	const float rad = m_sample->getAgentRadius();
	
	//dtNavMesh* nav = m_sample->getNavMesh();
	NavMesh* nav = &m_sample->getInputMesh()->m_navMesh;
	HybridModelCrowd* crowd = m_sample->getHybridCrowd();
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
			const dtCrowdAgent* ag =crowd->getAgent(i);
			if (!ag->active)
				continue;
			const dtPolyRef* path = ag->corridor.getPath();
			const int npath = ag->corridor.getPathCount();			
			//for (int j = 0; j < npath; ++j)
			//	duDebugDrawNavMeshPoly(&dd, *nav, path[j], duRGBA(255,255,255,24));
		}
	}
	
	if (m_targetRef)
		duDebugDrawCross(&dd, m_targetPos[0],m_targetPos[1]+0.1f,m_targetPos[2], rad, duRGBA(255,255,255,192), 2.0f);
	
	
	
	// Trail
	for (int i = 0; i < crowd->getAgentCount(); ++i)
	{
		const dtCrowdAgent* ag = crowd->getAgent(i);
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
	
	// Corners & co
	for (int i = 0; i < crowd->getAgentCount(); i++)
	{
		const dtCrowdAgent* ag =crowd->getAgent(i);
		if (!ag->active)
			continue;
			
		const float radius = ag->params.radius;
		const float* pos = ag->npos;
		
		if (m_toolParams.m_showCorners)
		{
			if (ag->ncorners)
			{
				dd.begin(DU_DRAW_LINES, 2.0f);
				for (int j = 0; j < ag->ncorners; ++j)
				{
					const float* va = j == 0 ? pos : &ag->cornerVerts[(j-1)*3];
					const float* vb = &ag->cornerVerts[j*3];
					dd.vertex(va[0],va[1]+radius,va[2], duRGBA(128,0,0,192));
					dd.vertex(vb[0],vb[1]+radius,vb[2], duRGBA(128,0,0,192));
				}
				if (ag->ncorners && ag->cornerFlags[ag->ncorners-1] & DT_STRAIGHTPATH_OFFMESH_CONNECTION)
				{
					const float* v = &ag->cornerVerts[(ag->ncorners-1)*3];
					dd.vertex(v[0],v[1],v[2], duRGBA(192,0,0,192));
					dd.vertex(v[0],v[1]+radius*2,v[2], duRGBA(192,0,0,192));
				}
				
				dd.end();
			}
		}
		
		if (m_toolParams.m_showCollisionSegments)
		{
			const float* center = ag->boundary.getCenter();
			duDebugDrawCross(&dd, center[0],center[1]+radius,center[2], 0.2f, duRGBA(192,0,128,255), 2.0f);
			duDebugDrawCircle(&dd, center[0],center[1]+radius,center[2], ag->params.collisionQueryRange,
							  duRGBA(192,0,128,128), 2.0f);
			
			dd.begin(DU_DRAW_LINES, 3.0f);
			for (int j = 0; j < ag->boundary.getSegmentCount(); ++j)
			{
				const float* s = ag->boundary.getSegment(j);
				unsigned int col = duRGBA(192,0,128,192);
				if (dtTriArea2D(pos, s, s+3) < 0.0f)
					col = duDarkenCol(col);
				
				duAppendArrow(&dd, s[0],s[1]+0.2f,s[2], s[3],s[4]+0.2f,s[5], 0.0f, 0.3f, col);
			}
			dd.end();
		}
		
		if (m_toolParams.m_showNeis)
		{
			duDebugDrawCircle(&dd, pos[0],pos[1]+radius,pos[2], ag->params.collisionQueryRange,
							  duRGBA(0,192,128,128), 2.0f);
			
			dd.begin(DU_DRAW_LINES, 2.0f);
			for (int j = 0; j < ag->nneis; ++j)
			{
				// Get 'n'th active agent.
				// TODO: fix this properly.
				const dtCrowdAgent* nei = crowd->getAgent(ag->neis[j].idx);
				if (nei)
				{
					dd.vertex(pos[0],pos[1]+radius,pos[2], duRGBA(0,192,128,128));
					dd.vertex(nei->npos[0],nei->npos[1]+radius,nei->npos[2], duRGBA(0,192,128,128));
				}
			}
			dd.end();
		}
		
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
		switch(ag->active){
			case 1:
				col = duRGBA(220,0,0,128);
				break;
			case 2:
				col = duRGBA(0,220,0,128);
				break;
			case 3:
				col = duRGBA(220,0,0,128);
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
		const dtCrowdAgent* ag = crowd->getAgent(i);
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
	
	dd.depthMask(true);
}

void HybridCrowdToolState::handleRenderOverlay(double* proj, double* model, int* view)
{
	GLdouble x, y, z;
	
	// Draw start and end point labels
	if (m_targetRef && gluProject((GLdouble)m_targetPos[0], (GLdouble)m_targetPos[1], (GLdouble)m_targetPos[2],
								  model, proj, view, &x, &y, &z))
	{
		imguiDrawText((int)x, (int)(y+25), IMGUI_ALIGN_CENTER, "TARGET", imguiRGBA(0,0,0,220));
	}
	
	char label[32];
	
	if (m_toolParams.m_showLabels)
	{
		HybridModelCrowd* crowd = m_sample->getHybridCrowd();
		if (crowd)
		{
			for (int i = 0; i < crowd->getAgentCount(); ++i)
			{
				const dtCrowdAgent* ag = crowd->getAgent(i);
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
		HybridModelCrowd* crowd = m_sample->getHybridCrowd();
		const dtCrowdAgent* ag = crowd->getAgent(0);
		float height = ag->params.height;
		for(int i=0;i<crowd->m_numofgridinrow*crowd->m_numofgridinrow;i++)
		{
			
			if (gluProject((GLdouble)crowd->m_grids[i].box.center_x(), (GLdouble)height, (GLdouble)crowd->m_grids[i].box.center_y(),  model, proj, view, &x, &y, &z))
			{
				snprintf(label, 32, "%d", crowd->m_grids[i].grid_id);
				imguiDrawText((int)x, (int)y+15, IMGUI_ALIGN_CENTER, label, imguiRGBA(255,255,255,220));
			}
		}
	}

	if (m_toolParams.m_showPerfGraph)
	{
		GraphParams gp;
		gp.setRect(300, 10, 500, 200, 8);
		gp.setValueRange(0.0f, 2.0f, 4, "ms");
		
		drawGraphBackground(&gp);
		drawGraph(&gp, &m_crowdTotalTime, 1, "Total", duRGBA(255,128,0,255));
		
		gp.setRect(300, 10, 500, 50, 8);
		gp.setValueRange(0.0f, 2000.0f, 1, "");
		drawGraph(&gp, &m_crowdSampleCount, 0, "Sample Count", duRGBA(96,96,96,128));
	}
	
}

void HybridCrowdToolState::handleUpdate(const float dt)
{
	if (m_run)
		updateTick(dt);
}

void HybridCrowdToolState::addAgent(const float* p)
{
	if (!m_sample) return;
	HybridModelCrowd* crowd = m_sample->getHybridCrowd();
	
	dtCrowdAgentParams ap;
	memset(&ap, 0, sizeof(ap));
	ap.radius = m_sample->getAgentRadius();
	ap.height = m_sample->getAgentHeight();
	ap.maxAcceleration = 8.0f;
	ap.maxSpeed = 3.5f;
	ap.collisionQueryRange = ap.radius * 12.0f;
	ap.pathOptimizationRange = ap.radius * 30.0f;
	ap.updateFlags = 0; 
	if (m_toolParams.m_anticipateTurns)
		ap.updateFlags |= DT_CROWD_ANTICIPATE_TURNS;
	if (m_toolParams.m_optimizeVis)
		ap.updateFlags |= DT_CROWD_OPTIMIZE_VIS;
	if (m_toolParams.m_optimizeTopo)
		ap.updateFlags |= DT_CROWD_OPTIMIZE_TOPO;
	if (m_toolParams.m_obstacleAvoidance)
		ap.updateFlags |= DT_CROWD_OBSTACLE_AVOIDANCE;
	if (m_toolParams.m_separation)
		ap.updateFlags |= DT_CROWD_SEPARATION;
	ap.obstacleAvoidanceType = (unsigned char)m_toolParams.m_obstacleAvoidanceType;
	ap.separationWeight = m_toolParams.m_separationWeight;
	
	int idx = crowd->addAgent(p, &ap);
	if (idx != -1)
	{
		if (m_targetRef)
			crowd->requestMoveTarget(idx, m_targetRef, m_targetPos);
		
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
	HybridModelCrowd* crowd = m_sample->getHybridCrowd();

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
	dtNavMeshQuery* navquery = m_sample->getNavMeshQuery();
	HybridModelCrowd* crowd = m_sample->getHybridCrowd();
//	const dtQueryFilter* filter = crowd->getFilter();
	const float* ext = crowd->getQueryExtents();

	if (adjust)
	{
		crowd->m_selected.clear();
		float vel[3];
		// Request velocity
		for (int i = 0; i < crowd->getAgentCount(); ++i)
		{
			const dtCrowdAgent* ag = crowd->getAgent(i);
			if (!ag->active) continue;
			calcVel(vel, ag->npos, p, ag->params.maxSpeed);
			crowd->requestMoveVelocity(i, vel);
		}
	}
	else
	{
//		navquery->findNearestPoly(p, ext, filter, &m_targetRef, m_targetPos);
		
		if(crowd->m_selected.empty())
		{
			for (int i = 0; i < crowd->getAgentCount(); ++i)
			{
				const dtCrowdAgent* ag = crowd->getAgent(i);
				if (!ag->active) continue;
				crowd->requestMoveTarget(i, m_targetRef, m_targetPos);
			}
		}
		else
		{
			for (int i = 0; i < crowd->m_selected.size(); ++i)
			{
				const HmAgent* ag = crowd->m_selected[i];
				if (!ag->active) continue;
				crowd->requestMoveTarget(ag->HmAgent_id, m_targetRef, m_targetPos);
			}
		}
	}
}

void HybridCrowdToolState::updateAgentParams()
{
	if (!m_sample) return;
	HybridModelCrowd* crowd = m_sample->getHybridCrowd();
	if (!crowd) return;
	
	unsigned char updateFlags = 0;
	unsigned char obstacleAvoidanceType = 0;
	
	if (m_toolParams.m_anticipateTurns)
		updateFlags |= DT_CROWD_ANTICIPATE_TURNS;
	if (m_toolParams.m_optimizeVis)
		updateFlags |= DT_CROWD_OPTIMIZE_VIS;
	if (m_toolParams.m_optimizeTopo)
		updateFlags |= DT_CROWD_OPTIMIZE_TOPO;
	if (m_toolParams.m_obstacleAvoidance)
		updateFlags |= DT_CROWD_OBSTACLE_AVOIDANCE;
	if (m_toolParams.m_obstacleAvoidance)
		updateFlags |= DT_CROWD_OBSTACLE_AVOIDANCE;
	if (m_toolParams.m_separation)
		updateFlags |= DT_CROWD_SEPARATION;
	
	obstacleAvoidanceType = (unsigned char)m_toolParams.m_obstacleAvoidanceType;
	
	dtCrowdAgentParams params;
	
	for (int i = 0; i < crowd->getAgentCount(); ++i)
	{
		const dtCrowdAgent* ag = crowd->getAgent(i);
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
	dtNavMesh* nav = m_sample->getNavMesh();
	HybridModelCrowd* crowd = m_sample->getHybridCrowd();
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
	m_crowdSampleCount.addSample((float)crowd->getVelocitySampleCount());
	m_crowdTotalTime.addSample(getPerfDeltaTimeUsec(startTime, endTime) / 1000.0f);
}

void HybridCrowdToolState::hilightAgent(const int idx)
{
	
}

int HybridCrowdToolState::hitTestAgents(const float* s, const float* p)
{
	if (!m_sample) return -1;
	HybridModelCrowd* crowd = m_sample->getHybridCrowd();
	
	int isel = -1;
	float tsel = FLT_MAX;

	for (int i = 0; i < crowd->getAgentCount(); ++i)
	{
		const dtCrowdAgent* ag = crowd->getAgent(i);
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
		sample->setToolState(type(), m_state);
	}
	m_state->init(sample);
	m_densityThreshold = 0.1;
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

	if (imguiCheck("创建智能体", m_mode == TOOLMODE_CREATE))//Create Agents
		m_mode = TOOLMODE_CREATE;
	if (imguiCheck("设置目标点", m_mode == TOOLMODE_MOVE_TARGET))//Move Target
		m_mode = TOOLMODE_MOVE_TARGET;
	if (imguiCheck("选择智能体", m_mode == TOOLMODE_SELECT))//Select Agent
		m_mode = TOOLMODE_SELECT;
	if (imguiCheck("删除场景多边形", m_mode == TOOLMODE_TOGGLE_POLYS))//Toggle Polys
		m_mode = TOOLMODE_TOGGLE_POLYS;
	imguiSeparatorLine();
	imguiSlider("随机人群个数", &m_randomCrowdNumber, 0.0f, 10000.0f, 50.0f);
	if (imguiButton("随机生成人群"))
	{
		const HybridModelCrowd* hmc = this->m_sample->getHybridCrowd();
		for (int i = 0; i < m_randomCrowdNumber; i++)
		{
			
			float pt[3];
			dtPolyRef ref;
//			dtStatus status = hmc->m_navquery->findRandomPoint(&hmc->m_filter, frand, &ref, pt);
//			if (dtStatusSucceed(status))
			{
				//dtVcopy(&m_randPoints[m_nrandPoints*3], pt);
				m_state->addAgent(pt);
			}
		}
	}
	imguiSeparatorLine();
	imguiSlider("稠密人群阈值", &m_densityThreshold, 0.05f, 1.0f, 0.05f);
	HybridModelCrowd* hmc = this->m_sample->getHybridCrowd();
	hmc->m_densityThreshold = m_densityThreshold;
	imguiSeparatorLine();
	/*	
	if (imguiCollapse("Options", 0, params->m_expandOptions))
		params->m_expandOptions = !params->m_expandOptions;
	
	if (params->m_expandOptions)
	{
		imguiIndent();
		if (imguiCheck("Optimize Visibility", params->m_optimizeVis))
		{
			params->m_optimizeVis = !params->m_optimizeVis;
			m_state->updateAgentParams();
		}
		if (imguiCheck("Optimize Topology", params->m_optimizeTopo))
		{
			params->m_optimizeTopo = !params->m_optimizeTopo;
			m_state->updateAgentParams();
		}
		if (imguiCheck("Anticipate Turns", params->m_anticipateTurns))
		{
			params->m_anticipateTurns = !params->m_anticipateTurns;
			m_state->updateAgentParams();
		}
		if (imguiCheck("Obstacle Avoidance", params->m_obstacleAvoidance))
		{
			params->m_obstacleAvoidance = !params->m_obstacleAvoidance;
			m_state->updateAgentParams();
		}
		if (imguiSlider("Avoidance Quality", &params->m_obstacleAvoidanceType, 0.0f, 3.0f, 1.0f))
		{
			m_state->updateAgentParams();
		}
		if (imguiCheck("Separation", params->m_separation))
		{
			params->m_separation = !params->m_separation;
			m_state->updateAgentParams();
		}
		if (imguiSlider("Separation Weight", &params->m_separationWeight, 0.0f, 20.0f, 0.01f))
		{
			m_state->updateAgentParams();
		}
		
		imguiUnindent();
	}
*/
	if (imguiCollapse("选择显示的属性", 0, params->m_expandSelectedDebugDraw))//Selected Debug Draw
		params->m_expandSelectedDebugDraw = !params->m_expandSelectedDebugDraw;
		
	if (params->m_expandSelectedDebugDraw)
	{
		imguiIndent();
		if (imguiCheck("显示拐点", params->m_showCorners))//Show Corners
			params->m_showCorners = !params->m_showCorners;
		if (imguiCheck("显示可能碰撞的区域", params->m_showCollisionSegments))//Show Collision Segs
			params->m_showCollisionSegments = !params->m_showCollisionSegments;
		if (imguiCheck("显示路径", params->m_showPath))//Show Path
			params->m_showPath = !params->m_showPath;
		if (imguiCheck("显示碰撞速度域", params->m_showVO))//Show VO
			params->m_showVO = !params->m_showVO;
		/*if (imguiCheck("Show Path Optimization", params->m_showOpt))
			params->m_showOpt = !params->m_showOpt;*/
		if (imguiCheck("显示邻居", params->m_showNeis))//Show Neighbours
			params->m_showNeis = !params->m_showNeis;
		if (imguiCheck("显示边界人群", params->m_showContour))//Show Contour
			params->m_showContour = !params->m_showContour;
		if (imguiCheck("显示网格ID", params->m_showGridLabels))//Show GridID
			params->m_showGridLabels = !params->m_showGridLabels;
		if (imguiCheck("显示速度箭头", params->m_showVelocityArrow))//Show GridID
			params->m_showVelocityArrow = !params->m_showVelocityArrow;
		imguiUnindent();
	}
		
}



void HybridCrowdTool::handleClick(const float* s, const float* p, bool shift)
{
	if (!m_sample) return;
	if (!m_state) return;
	InputMesh* mesh = m_sample->getInputMesh();
	if (!mesh) return;
	HybridModelCrowd* crowd = m_sample->getHybridCrowd();
	if (!crowd) return;

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
			m_state->addAgent(p);
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
		this->m_sample->getHybridCrowd()->m_selected.clear();
	}
	else if (m_mode == TOOLMODE_TOGGLE_POLYS)
	{
		dtNavMesh* nav = m_sample->getNavMesh();
		dtNavMeshQuery* navquery = m_sample->getNavMeshQuery();
		if (nav && navquery)
		{
			dtQueryFilter filter;
			const float* ext = crowd->getQueryExtents();
			float tgt[3];
			dtPolyRef ref;
			navquery->findNearestPoly(p, ext, &filter, &ref, tgt);
			if (ref)
			{
				unsigned short flags = 0;
				if (dtStatusSucceed(nav->getPolyFlags(ref, &flags)))
				{
					flags ^= SAMPLE_POLYFLAGS_DISABLED;
					nav->setPolyFlags(ref, flags);
				}
			}
		}
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
		imguiDrawText(280, ty, IMGUI_ALIGN_LEFT, "鼠标左键：创建智能体  Shift+鼠标左键：删除智能体", imguiRGBA(255,255,255,192));	//LMB: add agent.  Shift+LMB: remove agent.
	}
	else if (m_mode == TOOLMODE_MOVE_TARGET)
	{
		imguiDrawText(280, ty, IMGUI_ALIGN_LEFT, "鼠标左键：设置目标点  Shift+鼠标左键：临时改变速度方向", imguiRGBA(255,255,255,192));	//LMB: set move target.  Shift+LMB: adjust set velocity.
		ty -= 20;
		//imguiDrawText(280, ty, IMGUI_ALIGN_LEFT, "Setting velocity will move the agents without pathfinder.", imguiRGBA(255,255,255,192));	
	}
	else if (m_mode == TOOLMODE_SELECT)
	{
		imguiDrawText(280, ty, IMGUI_ALIGN_LEFT, "鼠标左键：选择智能体", imguiRGBA(255,255,255,192));	//LMB: select agent.
	}
	ty -= 20;
	char agent_num[128];
	sprintf(agent_num,"当前人群数量: %d",this->m_sample->getHybridCrowd()->agent_num);
	imguiDrawText(280, ty, IMGUI_ALIGN_LEFT, agent_num, imguiRGBA(255,255,255,192));	//SPACE: Run/Pause simulation.  1: Step simulation.
	ty -= 20;
	imguiDrawText(280, ty, IMGUI_ALIGN_LEFT, "空格： 启动/暂停仿真.  数字键“1”：仿真一帧", imguiRGBA(255,255,255,192));	//SPACE: Run/Pause simulation.  1: Step simulation.
	ty -= 20;

	if (m_state && m_state->isRunning())
		imguiDrawText(280, ty, IMGUI_ALIGN_LEFT, "- 仿真 -", imguiRGBA(255,32,16,255));	
	else 
		imguiDrawText(280, ty, IMGUI_ALIGN_LEFT, "- 暂停 -", imguiRGBA(255,255,255,128));	
}

void HybridCrowdTool::handleDrag(const float* tl, const float* tr,const float* bl, const float* br )
{
	/*
	if (!m_sample) return;
	if (!m_state) return;
	InputGeom* geom = m_sample->getInputGeom();
	if (!geom) return;
	HybridModelCrowd* crowd = m_sample->getHybridCrowd();
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

	}*/
}