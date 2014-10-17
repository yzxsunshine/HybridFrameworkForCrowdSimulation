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
	
	dtNavMesh* nav = m_sample->getNavMesh();
	HybridModelCrowd* crowd = m_sample->getHybridCrowd();
	
	//if (nav && crowd && (m_nav != nav || m_crowd != crowd))
	{
		m_nav = nav;
		m_crowd = crowd;
	
		float gridsize;
		float ox,oy,ex,ey;
		m_sample->getGridParams(gridsize,ox,oy,ex,ey);
		crowd->init(MAX_AGENTS, m_sample->getAgentRadius(), nav,gridsize,ox,oy,ex,ey);
		
		// Make polygons with 'disabled' flag invalid.
		crowd->getEditableFilter()->setExcludeFlags(SAMPLE_POLYFLAGS_DISABLED);
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
	float crowdpoints[] = {54.200912,0.794912,-17.517275,
54.508614,0.275351,-20.323996,
54.445210,-0.289378,-22.022718,
55.769768,0.889237,-18.799162,
53.781059,0.583329,-18.592268,
55.844910,0.605136,-19.825747,
52.667877,-0.097681,-19.677410,
51.689632,0.028122,-17.039301,
51.256023,-0.175291,-17.953684,
48.965851,-1.149656,-16.542004,
50.971161,-0.667170,-19.753225,
53.103859,-0.593972,-21.968443,
52.366287,-0.427156,-20.601244,
50.903927,-0.996489,-21.063875,
52.244995,-1.186037,-22.771038,
49.214863,-1.235917,-18.753063,
49.585732,-0.893759,-17.451759,
47.946999,-1.714422,-19.209019,
46.143696,-2.021061,-16.610397,
47.448959,-1.679642,-16.562088,
49.024418,-1.563183,-20.775290,
50.590473,-1.642596,-23.229935,
52.636147,-1.480711,-24.027245,
54.382572,-0.722255,-23.030518,
54.945240,-1.103396,-24.108261,
54.339619,-1.599471,-25.327175,
53.089100,-2.039904,-26.687588,
52.517994,-1.748835,-25.005875,
50.717682,-1.940048,-24.459496,
47.856224,-2.110790,-22.544634,
48.270145,-1.938198,-21.436621,
49.169930,-2.054213,-23.685974,
46.513088,-2.100682,-19.319431,
46.341156,-2.175391,-20.897415,
46.996914,-1.993890,-19.416620,
43.574314,-2.378636,-18.072931,
43.173737,-2.389266,-15.478016,
44.961750,-2.274155,-17.488485,
46.409393,-1.950988,-16.893702,
43.695847,-2.402145,-19.676090,
43.902466,-2.396076,-21.234964,
45.777969,-2.351983,-22.736105,
46.837822,-2.324884,-24.609177,
48.776398,-2.225845,-24.680609,
48.331703,-2.344719,-27.158142,
51.036156,-2.188134,-26.310152,
41.128063,-1.553068,-26.378864,
40.301556,-2.352332,-21.609434,
42.572250,-2.196980,-24.299746,
38.985062,-2.428548,-17.388659,
39.571724,-2.434609,-15.826063,
41.422020,-2.437552,-19.854080,
35.992626,-2.026866,-18.786268,
44.523270,-1.997865,-27.053665,
44.754795,-2.179487,-25.666733,
43.855438,-2.356870,-23.541412,
48.026363,-2.160284,-30.115051,
50.330551,-2.389188,-29.008373,
51.030468,-2.355586,-27.651096,
53.780334,-2.285984,-28.273075,
52.232712,-2.426088,-30.155661,
49.302990,-2.298928,-32.535950,
49.402351,-2.326486,-31.283226,
49.916950,-2.385224,-29.802225,
53.255119,-2.507041,-33.548023,
47.599480,-1.936486,-32.580227,
45.638760,-1.680827,-30.039642,
45.053612,-2.001510,-27.693035,
46.627594,-2.198456,-27.641436,
41.461830,-0.604966,-29.393070,
37.904301,-2.132590,-23.093445,
36.689426,-2.132337,-20.960606,
37.059570,-2.179250,-19.086521,
33.398403,-0.585669,-14.870102,
32.354626,-0.582786,-18.046358,
32.312256,-1.910433,-21.832380,
35.743629,-0.988670,-26.878147,
40.432629,-0.020238,-30.583941,
44.309464,-0.796426,-33.902500,
48.043938,-2.107648,-35.636238,
50.286961,-2.645415,-36.953083,
52.449806,-2.887829,-37.214115,
40.391582,-1.055961,-27.441673,
35.918785,-1.722738,-24.693813,
33.046673,-1.509080,-20.732994,
33.476063,-0.892174,-27.667494,
37.203548,0.729766,-32.097584,
34.237629,0.626415,-33.331085,
33.261288,-0.407451,-29.422121,
37.123222,0.033791,-29.513948,
49.358711,-2.353571,-26.299250,
47.518269,-2.318791,-25.408224,
47.611927,-2.305264,-27.918936,
46.583252,-2.095898,-28.658710,
46.304436,-1.844547,-30.242277,
44.973637,-1.377566,-31.221291,
43.481297,-1.462701,-28.339949,
45.442635,-2.301213,-25.356556,
46.087849,-2.296337,-26.089327,
52.397614,-2.342210,-28.573179};
	for(int ii=0;ii<5000;ii++)
	{
		int i = ii/50;
		float currentpoint[3];
		currentpoint[0] = crowdpoints[i*3]+0.001*(ii%50);
		currentpoint[1] = crowdpoints[i*3+1]+0.001*(ii%50);
		currentpoint[2] = crowdpoints[i*3+2]+0.001*(ii%50);
		addAgent(currentpoint);
	}
}

void HybridCrowdToolState::reset()
{
}

void HybridCrowdToolState::handleRender()
{
	return;
	DebugDrawGL dd;
	const float rad = m_sample->getAgentRadius();
	
	dtNavMesh* nav = m_sample->getNavMesh();
	HybridModelCrowd* crowd = m_sample->getHybridCrowd();
	if (!nav || !crowd)
		return;
	
	if (m_toolParams.m_showNodes && crowd->getPathQueue())
	{
		const dtNavMeshQuery* navquery = crowd->getPathQueue()->getNavQuery();
		if (navquery)
			duDebugDrawNavMeshNodes(&dd, *navquery);
	}

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
			for (int j = 0; j < npath; ++j)
				duDebugDrawNavMeshPoly(&dd, *nav, path[j], duRGBA(255,255,255,24));
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
				
				
				if (m_toolParams.m_anticipateTurns)
				{
					/* Deal with it later					
					 float dvel[3], pos[3];
					 calcSmoothSteerDirection(ag->pos, ag->cornerVerts, ag->ncorners, dvel);
					 pos[0] = ag->pos[0] + dvel[0];
					 pos[1] = ag->pos[1] + dvel[1];
					 pos[2] = ag->pos[2] + dvel[2];
					 
					 const float off = ag->radius+0.1f;
					 const float* tgt = &ag->cornerVerts[0];
					 const float y = ag->pos[1]+off;
					 
					 dd.begin(DU_DRAW_LINES, 2.0f);
					 
					 dd.vertex(ag->pos[0],y,ag->pos[2], duRGBA(255,0,0,192));
					 dd.vertex(pos[0],y,pos[2], duRGBA(255,0,0,192));
					 
					 dd.vertex(pos[0],y,pos[2], duRGBA(255,0,0,192));
					 dd.vertex(tgt[0],y,tgt[2], duRGBA(255,0,0,192));
					 
					 dd.end();*/
				}
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
		const dtCrowdAgent* ag = crowd->getAgent(i);
		if (!ag->active) continue;
		
		const float radius = ag->params.radius;
		const float* pos = ag->npos;
		
		unsigned int col;
		switch(ag->active){
			case 1:
				col = duRGBA(0,255,0,32);
				break;
			case 2:
				col = duRGBA(255,0,0,32);
				break;
			default:
				col = duRGBA(0,0,0,32);
				break;
		}		
		duDebugDrawCircle(&dd, pos[0], pos[1], pos[2], radius, col, 2.0f);
	}
	
	for (int i = 0; i < crowd->getAgentCount(); ++i)
	{
		const dtCrowdAgent* ag = crowd->getAgent(i);
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
			default:
				col = duRGBA(220,220,220,128);
				break;
		}		
		if (ag->targetState == DT_CROWDAGENT_TARGET_REQUESTING || ag->targetState == DT_CROWDAGENT_TARGET_WAITING_FOR_QUEUE)
			col = duLerpCol(col, duRGBA(128,0,255,128), 32);
		else if (ag->targetState == DT_CROWDAGENT_TARGET_WAITING_FOR_PATH)
			col = duLerpCol(col, duRGBA(128,0,255,128), 128);
		else if (ag->targetState == DT_CROWDAGENT_TARGET_FAILED)
			col = duRGBA(255,32,16,128);
		else if (ag->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
			col = duLerpCol(col, duRGBA(64,255,0,128), 128);
		
		duDebugDrawCylinder(&dd, pos[0]-radius, pos[1]+radius*0.1f, pos[2]-radius,
							pos[0]+radius, pos[1]+height, pos[2]+radius, col);
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
		
		duDebugDrawArrow(&dd, pos[0],pos[1]+height,pos[2],
						 pos[0]+dvel[0],pos[1]+height+dvel[1],pos[2]+dvel[2],
						 0.0f, 0.4f, duRGBA(0,192,255,192), 1.0f);
		
		duDebugDrawArrow(&dd, pos[0],pos[1]+height,pos[2],
						 pos[0]+vel[0],pos[1]+height+vel[1],pos[2]+vel[2],
						 0.0f, 0.4f, duRGBA(0,0,0,160), 2.0f);
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
	
	if (m_toolParams.m_showNodes)
	{
		HybridModelCrowd* crowd = m_sample->getHybridCrowd();
		if (crowd && crowd->getPathQueue())
		{
			const dtNavMeshQuery* navquery = crowd->getPathQueue()->getNavQuery();
			const dtNodePool* pool = navquery->getNodePool();
			if (pool)
			{
				const float off = 0.5f;
				for (int i = 0; i < pool->getHashSize(); ++i)
				{
					for (dtNodeIndex j = pool->getFirst(i); j != DT_NULL_IDX; j = pool->getNext(j))
					{
						const dtNode* node = pool->getNodeAtIdx(j+1);
						if (!node) continue;

						if (gluProject((GLdouble)node->pos[0],(GLdouble)node->pos[1]+off,(GLdouble)node->pos[2],
									   model, proj, view, &x, &y, &z))
						{
							const float heuristic = node->total;// - node->cost;
							snprintf(label, 32, "%.2f", heuristic);
							imguiDrawText((int)x, (int)y+15, IMGUI_ALIGN_CENTER, label, imguiRGBA(0,0,0,220));
						}
					}
				}
			}
		}
	}
	
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
	else
	{
		printf("fail haha\n");
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
	const dtQueryFilter* filter = crowd->getFilter();
	const float* ext = crowd->getQueryExtents();

	if (adjust)
	{
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
		navquery->findNearestPoly(p, ext, filter, &m_targetRef, m_targetPos);
		
		for (int i = 0; i < crowd->getAgentCount(); ++i)
		{
			const dtCrowdAgent* ag = crowd->getAgent(i);
			if (!ag->active) continue;
			crowd->requestMoveTarget(i, m_targetRef, m_targetPos);
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
}
void HybridCrowdTool::reset()
{	
}

void HybridCrowdTool::handleMenu()
{
	if (!m_state)
		return;
	HybridCrowdToolParams* params = m_state->getToolParams();

	if (imguiCheck("Create Agents", m_mode == TOOLMODE_CREATE))
		m_mode = TOOLMODE_CREATE;
	if (imguiCheck("Move Target", m_mode == TOOLMODE_MOVE_TARGET))
		m_mode = TOOLMODE_MOVE_TARGET;
	if (imguiCheck("Select Agent", m_mode == TOOLMODE_SELECT))
		m_mode = TOOLMODE_SELECT;
	if (imguiCheck("Toggle Polys", m_mode == TOOLMODE_TOGGLE_POLYS))
		m_mode = TOOLMODE_TOGGLE_POLYS;
	
	imguiSeparatorLine();
		
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

	if (imguiCollapse("Selected Debug Draw", 0, params->m_expandSelectedDebugDraw))
		params->m_expandSelectedDebugDraw = !params->m_expandSelectedDebugDraw;
		
	if (params->m_expandSelectedDebugDraw)
	{
		imguiIndent();
		if (imguiCheck("Show Corners", params->m_showCorners))
			params->m_showCorners = !params->m_showCorners;
		if (imguiCheck("Show Collision Segs", params->m_showCollisionSegments))
			params->m_showCollisionSegments = !params->m_showCollisionSegments;
		if (imguiCheck("Show Path", params->m_showPath))
			params->m_showPath = !params->m_showPath;
		if (imguiCheck("Show VO", params->m_showVO))
			params->m_showVO = !params->m_showVO;
		if (imguiCheck("Show Path Optimization", params->m_showOpt))
			params->m_showOpt = !params->m_showOpt;
		if (imguiCheck("Show Neighbours", params->m_showNeis))
			params->m_showNeis = !params->m_showNeis;
		imguiUnindent();
	}
		
}

void HybridCrowdTool::handleClick(const float* s, const float* p, bool shift)
{
	if (!m_sample) return;
	if (!m_state) return;
	InputGeom* geom = m_sample->getInputGeom();
	if (!geom) return;
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
	int ty = h-40;
	
	if (m_mode == TOOLMODE_CREATE)
	{
		imguiDrawText(280, ty, IMGUI_ALIGN_LEFT, "LMB: add agent.  Shift+LMB: remove agent.", imguiRGBA(255,255,255,192));	
	}
	else if (m_mode == TOOLMODE_MOVE_TARGET)
	{
		imguiDrawText(280, ty, IMGUI_ALIGN_LEFT, "LMB: set move target.  Shift+LMB: adjust set velocity.", imguiRGBA(255,255,255,192));	
		ty -= 20;
		imguiDrawText(280, ty, IMGUI_ALIGN_LEFT, "Setting velocity will move the agents without pathfinder.", imguiRGBA(255,255,255,192));	
	}
	else if (m_mode == TOOLMODE_SELECT)
	{
		imguiDrawText(280, ty, IMGUI_ALIGN_LEFT, "LMB: select agent.", imguiRGBA(255,255,255,192));	
	}
	ty -= 20;
	imguiDrawText(280, ty, IMGUI_ALIGN_LEFT, "SPACE: Run/Pause simulation.  1: Step simulation.", imguiRGBA(255,255,255,192));	
	ty -= 20;

	if (m_state && m_state->isRunning())
		imguiDrawText(280, ty, IMGUI_ALIGN_LEFT, "- RUNNING -", imguiRGBA(255,32,16,255));	
	else 
		imguiDrawText(280, ty, IMGUI_ALIGN_LEFT, "- PAUSED -", imguiRGBA(255,255,255,128));	
}

