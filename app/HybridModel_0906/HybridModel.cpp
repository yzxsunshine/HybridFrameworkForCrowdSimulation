//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//

#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <float.h>
#include <new>
#include "SDL.h"
#include "SDL_opengl.h"
#include "imgui.h"
#include "InputGeom.h"
#include "Sample.h"
#include "Recast.h"
#include "RecastDebugDraw.h"
#include "DetourAssert.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourDebugDraw.h"
#include "DetourCommon.h"
#include "DetourTileCache.h"
#include "OffMeshConnectionTool.h"
#include "HybridCrowdTool.h"
#include "RecastAlloc.h"
#include "RecastAssert.h"
#include "fastlz.h"
#include "HybridModel.h"
#include "Renderer.h"

#ifdef WIN32
#	define snprintf _snprintf
#endif


// This value specifies how many layers (or "floors") each navmesh tile is expected to have.
static const int EXPECTED_LAYERS_PER_TILE = 4;


static bool isectSegAABB(const float* sp, const float* sq,
						 const float* amin, const float* amax,
						 float& tmin, float& tmax)
{
	static const float EPS = 1e-6f;
	
	float d[3];
	rcVsub(d, sq, sp);
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
			if (t1 > t2) rcSwap(t1, t2);
			// Compute the intersection of slab intersections intervals
			if (t1 > tmin) tmin = t1;
			if (t2 < tmax) tmax = t2;
			// Exit with no collision as soon as slab intersection becomes empty
			if (tmin > tmax) return false;
		}
	}
	
	return true;
}

static int calcLayerBufferSize(const int gridWidth, const int gridHeight)
{
	const int headerSize = dtAlign4(sizeof(dtTileCacheLayerHeader));
	const int gridSize = gridWidth * gridHeight;
	return headerSize + gridSize*4;
}




struct FastLZCompressor : public dtTileCacheCompressor
{
	virtual int maxCompressedSize(const int bufferSize)
	{
		return (int)(bufferSize* 1.05f);
	}
	
	virtual dtStatus compress(const unsigned char* buffer, const int bufferSize,
							  unsigned char* compressed, const int /*maxCompressedSize*/, int* compressedSize)
	{
		*compressedSize = fastlz_compress((const void *const)buffer, bufferSize, compressed);
		return DT_SUCCESS;
	}
	
	virtual dtStatus decompress(const unsigned char* compressed, const int compressedSize,
								unsigned char* buffer, const int maxBufferSize, int* bufferSize)
	{
		*bufferSize = fastlz_decompress(compressed, compressedSize, buffer, maxBufferSize);
		return *bufferSize < 0 ? DT_FAILURE : DT_SUCCESS;
	}
};

struct LinearAllocator : public dtTileCacheAlloc
{
	unsigned char* buffer;
	int capacity;
	int top;
	int high;
	
	LinearAllocator(const int cap) : buffer(0), capacity(0), top(0), high(0)
	{
		resize(cap);
	}
	
	~LinearAllocator()
	{
		dtFree(buffer);
	}

	void resize(const int cap)
	{
		if (buffer) dtFree(buffer);
		buffer = (unsigned char*)dtAlloc(cap, DT_ALLOC_PERM);
		capacity = cap;
	}
	
	virtual void reset()
	{
		high = dtMax(high, top);
		top = 0;
	}
	
	virtual void* alloc(const int size)
	{
		if (!buffer)
			return 0;
		if (top+size > capacity)
			return 0;
		unsigned char* mem = &buffer[top];
		top += size;
		return mem;
	}
	
	virtual void free(void* /*ptr*/)
	{
		// Empty
	}
};

struct MeshProcess : public dtTileCacheMeshProcess
{
	InputGeom* m_geom;

	inline MeshProcess() : m_geom(0)
	{
	}

	inline void init(InputGeom* geom)
	{
		m_geom = geom;
	}
	
	virtual void process(struct dtNavMeshCreateParams* params,
						 unsigned char* polyAreas, unsigned short* polyFlags)
	{
		// Update poly flags from areas.
		for (int i = 0; i < params->polyCount; ++i)
		{
			if (polyAreas[i] == DT_TILECACHE_WALKABLE_AREA)
				polyAreas[i] = SAMPLE_POLYAREA_GROUND;

			if (polyAreas[i] == SAMPLE_POLYAREA_GROUND ||
				polyAreas[i] == SAMPLE_POLYAREA_GRASS ||
				polyAreas[i] == SAMPLE_POLYAREA_ROAD)
			{
				polyFlags[i] = SAMPLE_POLYFLAGS_WALK;
			}
			else if (polyAreas[i] == SAMPLE_POLYAREA_WATER)
			{
				polyFlags[i] = SAMPLE_POLYFLAGS_SWIM;
			}
			else if (polyAreas[i] == SAMPLE_POLYAREA_DOOR)
			{
				polyFlags[i] = SAMPLE_POLYFLAGS_WALK | SAMPLE_POLYFLAGS_DOOR;
			}
		}

		// Pass in off-mesh connections.
		if (m_geom)
		{
			params->offMeshConVerts = m_geom->getOffMeshConnectionVerts();
			params->offMeshConRad = m_geom->getOffMeshConnectionRads();
			params->offMeshConDir = m_geom->getOffMeshConnectionDirs();
			params->offMeshConAreas = m_geom->getOffMeshConnectionAreas();
			params->offMeshConFlags = m_geom->getOffMeshConnectionFlags();
			params->offMeshConUserID = m_geom->getOffMeshConnectionId();
			params->offMeshConCount = m_geom->getOffMeshConnectionCount();	
		}
	}
};




static const int MAX_LAYERS = 32;

struct TileCacheData
{
	unsigned char* data;
	int dataSize;
};

struct RasterizationContext
{
	RasterizationContext() :
		solid(0),
		triareas(0),
		lset(0),
		chf(0),
		ntiles(0)
	{
		memset(tiles, 0, sizeof(TileCacheData)*MAX_LAYERS);
	}
	
	~RasterizationContext()
	{
		rcFreeHeightField(solid);
		delete [] triareas;
		rcFreeHeightfieldLayerSet(lset);
		rcFreeCompactHeightfield(chf);
		for (int i = 0; i < MAX_LAYERS; ++i)
		{
			dtFree(tiles[i].data);
			tiles[i].data = 0;
		}
	}
	
	rcHeightfield* solid;
	unsigned char* triareas;
	rcHeightfieldLayerSet* lset;
	rcCompactHeightfield* chf;
	TileCacheData tiles[MAX_LAYERS];
	int ntiles;
};

		
dtObstacleRef hitTestObstacle(const dtTileCache* tc, const float* sp, const float* sq)
{
	float tmin = FLT_MAX;
	const dtTileCacheObstacle* obmin = 0;
	for (int i = 0; i < tc->getObstacleCount(); ++i)
	{
		const dtTileCacheObstacle* ob = tc->getObstacle(i);
		if (ob->state == DT_OBSTACLE_EMPTY)
			continue;
		
		float bmin[3], bmax[3], t0,t1;
		tc->getObstacleBounds(ob, bmin,bmax);
		
		if (isectSegAABB(sp,sq, bmin,bmax, t0,t1))
		{
			if (t0 < tmin)
			{
				tmin = t0;
				obmin = ob;
			}
		}
	}
	return tc->getObstacleRef(obmin);
}
	
void drawObstacles(duDebugDraw* dd, const dtTileCache* tc)
{
	// Draw obstacles
	for (int i = 0; i < tc->getObstacleCount(); ++i)
	{
		const dtTileCacheObstacle* ob = tc->getObstacle(i);
		if (ob->state == DT_OBSTACLE_EMPTY) continue;
		float bmin[3], bmax[3];
		tc->getObstacleBounds(ob, bmin,bmax);

		unsigned int col = 0;
		if (ob->state == DT_OBSTACLE_PROCESSING)
			col = duRGBA(255,255,0,128);
		else if (ob->state == DT_OBSTACLE_PROCESSED)
			col = duRGBA(255,192,0,192);
		else if (ob->state == DT_OBSTACLE_REMOVING)
			col = duRGBA(220,0,0,128);

		duDebugDrawCylinder(dd, bmin[0],bmin[1],bmin[2], bmax[0],bmax[1],bmax[2], col);
		duDebugDrawCylinderWire(dd, bmin[0],bmin[1],bmin[2], bmax[0],bmax[1],bmax[2], duDarkenCol(col), 2);
	}
}




class TempObstacleCreateTool : public SampleTool
{
	HybridModel* m_sample;
	
public:
	
	TempObstacleCreateTool()
	{
	}
	
	virtual ~TempObstacleCreateTool()
	{
	}
	
	virtual int type() { return TOOL_TEMP_OBSTACLE; }
	
	virtual void init(Sample* sample)
	{
		m_sample = (HybridModel*)sample; 
	}
	
	virtual void reset() {}
	
	virtual void handleMenu()
	{
		imguiLabel("创建动态障碍物");//Create Temp Obstacles
		
		if (imguiButton("删除所有动态障碍物"))//Remove All
			m_sample->clearAllTempObstacles();
		
		imguiSeparator();

	//	imguiValue("点击鼠标左键创建动态障碍物。");//Click LMB to create an obstacle.
	//	imguiValue("按住Shift键，并点击鼠标左键移除动态障碍物。");//Shift+LMB to remove an obstacle.
	}
	
	virtual void handleClick(const float* s, const float* p, bool shift)
	{
		if (m_sample)
		{
			if (shift)
				m_sample->removeTempObstacle(s,p);
			else
				m_sample->addTempObstacle(p);
		}
	}
	
	virtual void handleToggle() {}
	virtual void handleStep() {}
	virtual void handleUpdate(const float /*dt*/) {}
	virtual void handleRender() {}
	virtual void handleRenderOverlay(double* /*proj*/, double* /*model*/, int* /*view*/) { }
};





HybridModel::HybridModel() :
	m_keepInterResults(false),
	//m_tileCache(0),
	//m_cacheBuildTimeMs(0),
	//m_cacheCompressedSize(0),
	//m_cacheRawSize(0),
	//m_cacheLayerCount(0),
	//m_cacheBuildMemUsage(0),
	m_drawMode(DRAWMODE_NAVMESH),
	m_maxTiles(0),
	m_maxPolysPerTile(0),
	m_tileSize(40)
{
	resetCommonSettings();
	
//	m_talloc = new LinearAllocator(32000);
//	m_tcomp = new FastLZCompressor;
//	m_tmproc = new MeshProcess;
	m_hybridcrowd = new HybridModelCrowd();
	setTool(new TempObstacleCreateTool);
}

HybridModel::~HybridModel()
{
	//dtFreeNavMesh(m_navMesh);
	m_navMesh = 0;
	//dtFreeTileCache(m_tileCache);
}

void HybridModel::handleSettings()
{
	Sample::handleCommonSettings();

	if (imguiCheck("保存中间结果", m_keepInterResults))//Keep Itermediate Results
		m_keepInterResults = !m_keepInterResults;

	imguiLabel("网格划分");//Tiling
	imguiSlider("网格大小", &m_tileSize, 20.0f, 200.0f, 20.0f);//TileSize
	
	int gridSize = 1;
	//if (m_geom)
	if (m_mesh)
	{
		const float* bmin = m_mesh->getMeshBoundsMin();
		const float* bmax = m_mesh->getMeshBoundsMax();
		char text[64];
		int gw = 0, gh = 0;
		rcCalcGridSize(bmin, bmax, m_cellSize, &gw, &gh);
		const int ts = (int)m_tileSize;
		const int tw = (gw + ts-1) / ts;
		const int th = (gh + ts-1) / ts;
		//snprintf(text, 64, "网格个数  %d x %d", tw, th);//Tiles
		//imguiValue(text);

		// Max tiles and max polys affect how the tile IDs are caculated.
		// There are 22 bits available for identifying a tile and a polygon.
		int tileBits = rcMin((int)dtIlog2(dtNextPow2(tw*th*EXPECTED_LAYERS_PER_TILE)), 14);
		if (tileBits > 14) tileBits = 14;
		int polyBits = 22 - tileBits;
		m_maxTiles = 1 << tileBits;
		m_maxPolysPerTile = 1 << polyBits;
		//snprintf(text, 64, "最大网格个数  %d", m_maxTiles);//Max Tiles
		//imguiValue(text);
		//snprintf(text, 64, "最大多边形个数  %d", m_maxPolysPerTile);//Max Polys
		//imguiValue(text);
		gridSize = tw*th;
	}
	else
	{
		m_maxTiles = 0;
		m_maxPolysPerTile = 0;
	}
	
	imguiSeparator();
	
//	imguiLabel("网格缓存");	//Tile Cache
//	char msg[64];

//	const float compressionRatio = (float)m_cacheCompressedSize / (float)(m_cacheRawSize+1);
	
//	snprintf(msg, 64, "网格层数  %d", m_cacheLayerCount);	//Layers
//	imguiValue(msg);
	/*snprintf(msg, 64, "Layers (per tile)  %.1f", (float)m_cacheLayerCount/(float)gridSize);
	imguiValue(msg);

	snprintf(msg, 64, "内存使用  %.1f kB / %.1f kB (%.1f%%)", m_cacheCompressedSize/1024.0f, m_cacheRawSize/1024.0f, compressionRatio*100.0f);//Memory  %.1f kB / %.1f kB (%.1f%%)
	imguiValue(msg);
	snprintf(msg, 64, "导航网格建造时间  %.1f ms", m_cacheBuildTimeMs);//Navmesh Build Time
	imguiValue(msg);
	snprintf(msg, 64, "最大内存使用  %.1f kB", m_cacheBuildMemUsage/1024.0f); //Build Peak Mem Usage
	imguiValue(msg);
	*/	
	imguiSeparator();
}

void HybridModel::handleTools()
{
	int type = !m_tool ? TOOL_NONE : m_tool->type();

	if (imguiCheck("创建动态障碍物", type == TOOL_TEMP_OBSTACLE)) //Create Temp Obstacles
	{
		setTool(new TempObstacleCreateTool);
	}
	/*if (imguiCheck("Create Off-Mesh Links", type == TOOL_OFFMESH_CONNECTION))
	{
		setTool(new OffMeshConnectionTool);
	}*/
	if (imguiCheck("创建人群", type == TOOL_HYBRIDCROWD)) //Create Crowds
	{
		setTool(new HybridCrowdTool);
	}
	
	imguiSeparatorLine();

	imguiIndent();

	if (m_tool)
		m_tool->handleMenu();

	imguiUnindent();
}

void HybridModel::handleDebugMode()
{
	// Check which modes are valid.
	bool valid[MAX_DRAWMODE];
	for (int i = 0; i < MAX_DRAWMODE; ++i)
		valid[i] = false;
	
	//if (m_geom)
	if (m_mesh)
	{
		valid[DRAWMODE_NAVMESH] = m_navMesh != 0;
		valid[DRAWMODE_NAVMESH_TRANS] = m_navMesh != 0;
		valid[DRAWMODE_NAVMESH_BVTREE] = m_navMesh != 0;
		valid[DRAWMODE_NAVMESH_NODES] = m_navQuery != 0;
		valid[DRAWMODE_NAVMESH_PORTALS] = m_navMesh != 0;
		valid[DRAWMODE_NAVMESH_INVIS] = m_navMesh != 0;
		valid[DRAWMODE_MESH] = true;
		valid[DRAWMODE_CACHE_BOUNDS] = true;
	}
	
	int unavail = 0;
	for (int i = 0; i < MAX_DRAWMODE; ++i)
		if (!valid[i]) unavail++;
	
	if (unavail == MAX_DRAWMODE)
		return;
	
	imguiLabel("绘制信息");//Draw
	if (imguiCheck("原始场景", m_drawMode == DRAWMODE_MESH, valid[DRAWMODE_MESH]))//Input Mesh
		m_drawMode = DRAWMODE_MESH;
	if (imguiCheck("显示导航网格", m_drawMode == DRAWMODE_NAVMESH, valid[DRAWMODE_NAVMESH])) //Nevmesh
		m_drawMode = DRAWMODE_NAVMESH;
	//if (imguiCheck("Navmesh Invis", m_drawMode == DRAWMODE_NAVMESH_INVIS, valid[DRAWMODE_NAVMESH_INVIS]))
	//	m_drawMode = DRAWMODE_NAVMESH_INVIS;
	if (imguiCheck("仅显示导航网格", m_drawMode == DRAWMODE_NAVMESH_TRANS, valid[DRAWMODE_NAVMESH_TRANS]))//Navmesh Trans
		m_drawMode = DRAWMODE_NAVMESH_TRANS;
	/*if (imguiCheck("Navmesh BVTree", m_drawMode == DRAWMODE_NAVMESH_BVTREE, valid[DRAWMODE_NAVMESH_BVTREE]))
		m_drawMode = DRAWMODE_NAVMESH_BVTREE;
	if (imguiCheck("Navmesh Nodes", m_drawMode == DRAWMODE_NAVMESH_NODES, valid[DRAWMODE_NAVMESH_NODES]))
		m_drawMode = DRAWMODE_NAVMESH_NODES;
	if (imguiCheck("Navmesh Portals", m_drawMode == DRAWMODE_NAVMESH_PORTALS, valid[DRAWMODE_NAVMESH_PORTALS]))
		m_drawMode = DRAWMODE_NAVMESH_PORTALS;
	if (imguiCheck("Cache Bounds", m_drawMode == DRAWMODE_CACHE_BOUNDS, valid[DRAWMODE_CACHE_BOUNDS]))
		m_drawMode = DRAWMODE_CACHE_BOUNDS;
	*/
	/*if (unavail)
	{
		imguiValue("Tick 'Keep Itermediate Results'");
		imguiValue("rebuild some tiles to see");
		imguiValue("more debug mode options.");
	}*/
}

void HybridModel::handleRender()
{
	//if (!m_geom || !m_geom->getMesh())
	if (!m_mesh)
		return;
	
	DebugDrawGL dd;

	const float texScale = 1.0f / (m_cellSize * 10.0f);
	
	// Draw mesh
	if (m_drawMode != DRAWMODE_NAVMESH_TRANS)
	{
		// Draw mesh
		duDebugDrawMapMeshSlope(&dd, m_mesh->m_navMesh,
								m_agentMaxSlope, texScale);
		//m_geom->drawOffMeshConnections(&dd);
	}
	
	
	//if (m_tileCache)
	//	drawObstacles(&dd, m_tileCache);
	
	
	glDepthMask(GL_FALSE);
	
	// Draw bounds
	const float* bmin = m_mesh->getMeshBoundsMin();
	const float* bmax = m_mesh->getMeshBoundsMax();
	duDebugDrawBoxWire(&dd, bmin[0],bmin[1],bmin[2], bmax[0],bmax[1],bmax[2], duRGBA(255,255,255,128), 1.0f);
	
	// Tiling grid.
	int gw = 0, gh = 0;
	rcCalcGridSize(bmin, bmax, m_cellSize, &gw, &gh);
	const int tw = (gw + (int)m_tileSize-1) / (int)m_tileSize;
	const int th = (gh + (int)m_tileSize-1) / (int)m_tileSize;
	const float s = m_tileSize*m_cellSize;
	duDebugDrawGridXZ(&dd, bmin[0],bmin[1],bmin[2], tw,th, s, duRGBA(0,0,0,64), 1.0f);
	/*	
	if (m_navMesh && m_navQuery &&
		(m_drawMode == DRAWMODE_NAVMESH ||
		 m_drawMode == DRAWMODE_NAVMESH_TRANS ||
		 m_drawMode == DRAWMODE_NAVMESH_BVTREE ||
		 m_drawMode == DRAWMODE_NAVMESH_NODES ||
		 m_drawMode == DRAWMODE_NAVMESH_PORTALS ||
		 m_drawMode == DRAWMODE_NAVMESH_INVIS))
	{
		if (m_drawMode != DRAWMODE_NAVMESH_INVIS)
			duDebugDrawNavMeshWithClosedList(&dd, *m_navMesh, *m_navQuery, m_navMeshDrawFlags/*|DU_DRAWNAVMESH_COLOR_TILES*//*);
		duDebugDrawNavMeshPolysWithFlags(&dd, *m_navMesh, SAMPLE_POLYFLAGS_DISABLED, duRGBA(0,0,0,128));
	}*/
	
	
	glDepthMask(GL_TRUE);
		
	//m_geom->drawConvexVolumes(&dd);
	
	if (m_tool)
		m_tool->handleRender();
	renderToolStates();
	
	glDepthMask(GL_TRUE);
}


void HybridModel::handleRenderOverlay(double* proj, double* model, int* view)
{	
	if (m_tool)
		m_tool->handleRenderOverlay(proj, model, view);
	renderOverlayToolStates(proj, model, view);

	// Stats
/*	imguiDrawRect(280,10,300,100,imguiRGBA(0,0,0,64));
	
	char text[64];
	int y = 110-30;
	
	snprintf(text,64,"Lean Data: %.1fkB", m_tileCache->getRawSize()/1024.0f);
	imguiDrawText(300, y, IMGUI_ALIGN_LEFT, text, imguiRGBA(255,255,255,255));
	y -= 20;
	
	snprintf(text,64,"Compressed: %.1fkB (%.1f%%)", m_tileCache->getCompressedSize()/1024.0f,
			 m_tileCache->getRawSize() > 0 ? 100.0f*(float)m_tileCache->getCompressedSize()/(float)m_tileCache->getRawSize() : 0);
	imguiDrawText(300, y, IMGUI_ALIGN_LEFT, text, imguiRGBA(255,255,255,255));
	y -= 20;

	if (m_rebuildTileCount > 0 && m_rebuildTime > 0.0f)
	{
		snprintf(text,64,"Changed obstacles, rebuild %d tiles: %.3f ms", m_rebuildTileCount, m_rebuildTime);
		imguiDrawText(300, y, IMGUI_ALIGN_LEFT, text, imguiRGBA(255,192,0,255));
		y -= 20;
	}
	*/
}

void HybridModel::handleMeshChanged(class InputGeom* geom)
{
	/*Sample::handleMeshChanged(geom);

	dtFreeTileCache(m_tileCache);
	m_tileCache = 0;
	
	dtFreeNavMesh(m_navMesh);
	m_navMesh = 0;

	if (m_tool)
	{
		m_tool->reset();
		m_tool->init(this);
		m_tmproc->init(m_geom);
	}
	resetToolStates();
	initToolStates(this);*/
}

void HybridModel::handleMeshChanged(class InputMesh* mesh)
{
	Sample::handleMeshChanged(mesh);
	rcConfig cfg;
	memset(&cfg, 0, sizeof(cfg));
	cfg.cs = m_cellSize;
	cfg.ch = m_cellHeight;
	cfg.walkableSlopeAngle = m_agentMaxSlope;
	cfg.walkableHeight = (int)ceilf(m_agentHeight / cfg.ch);
	cfg.walkableClimb = (int)floorf(m_agentMaxClimb / cfg.ch);
	cfg.walkableRadius = (int)ceilf(m_agentRadius / cfg.cs);
	
	//m_mesh->FindWalkableArea(cfg.walkableSlopeAngle, cfg.walkableRadius);
	//m_mesh->BuildNavMesh();
	if (m_tool)
	{
		m_tool->reset();
		m_tool->init(this);
	}
	resetToolStates();
	initToolStates(this);
}

void HybridModel::addTempObstacle(const float* pos)
{
	/*if (!m_tileCache)
		return;
	float p[3];
	dtVcopy(p, pos);
	p[1] -= 0.5f;
	m_tileCache->addObstacle(p, 1.0f, 2.0f, 0);*/
}

void HybridModel::removeTempObstacle(const float* sp, const float* sq)
{
	/*if (!m_tileCache)
		return;
	dtObstacleRef ref = hitTestObstacle(m_tileCache, sp, sq);
	m_tileCache->removeObstacle(ref);*/
}

void HybridModel::clearAllTempObstacles()
{
	/*
	if (!m_tileCache)
		return;
	for (int i = 0; i < m_tileCache->getObstacleCount(); ++i)
	{
		const dtTileCacheObstacle* ob = m_tileCache->getObstacle(i);
		if (ob->state == DT_OBSTACLE_EMPTY) continue;
		m_tileCache->removeObstacle(m_tileCache->getObstacleRef(ob));
	}
	*/
}

bool HybridModel::handleBuild()
{
	dtStatus status;
	// Load map vertexes and faces //加载地图三角面片 
	if (!m_mesh)
	{
		m_ctx->log(RC_LOG_ERROR, "buildTiledNavigation: No vertices and triangles.");
		return false;
	}

	//m_tmproc->init(m_geom);
	
	// Init cache
	const float* bmin = m_mesh->getMeshBoundsMin();
	const float* bmax = m_mesh->getMeshBoundsMax();
	int gw = 0, gh = 0;
	rcCalcGridSize(bmin, bmax, m_cellSize, &gw, &gh);
	const int ts = (int)m_tileSize;
	const int tw = (gw + ts-1) / ts;
	const int th = (gh + ts-1) / ts;

	// Generation params.
	rcConfig cfg;
	memset(&cfg, 0, sizeof(cfg));
	cfg.cs = m_cellSize;
	cfg.ch = m_cellHeight;
	cfg.walkableSlopeAngle = m_agentMaxSlope;
	cfg.walkableHeight = (int)ceilf(m_agentHeight / cfg.ch);
	cfg.walkableClimb = (int)floorf(m_agentMaxClimb / cfg.ch);
	cfg.walkableRadius = (int)ceilf(m_agentRadius / cfg.cs);
	cfg.maxEdgeLen = (int)(m_edgeMaxLen / m_cellSize);
	cfg.maxSimplificationError = m_edgeMaxError;
	cfg.minRegionArea = (int)rcSqr(m_regionMinSize);		// Note: area = size*size
	cfg.mergeRegionArea = (int)rcSqr(m_regionMergeSize);	// Note: area = size*size
	cfg.maxVertsPerPoly = (int)m_vertsPerPoly;
	cfg.tileSize = (int)m_tileSize;
	cfg.borderSize = cfg.walkableRadius + 3; // Reserve enough padding.
	cfg.width = cfg.tileSize + cfg.borderSize*2;
	cfg.height = cfg.tileSize + cfg.borderSize*2;
	cfg.detailSampleDist = m_detailSampleDist < 0.9f ? 0 : m_cellSize * m_detailSampleDist;
	cfg.detailSampleMaxError = m_cellHeight * m_detailSampleMaxError;
	rcVcopy(cfg.bmin, bmin);
	rcVcopy(cfg.bmax, bmax);
	
	m_mesh->FindWalkableArea(cfg.walkableSlopeAngle, cfg.walkableRadius);
	m_mesh->BuildNavMesh();
	m_navMesh = &m_mesh->m_navMesh;
	m_tileCtrl->ClearTiles();
	m_tileCtrl->BuildTiles(m_navMesh, m_tileSize);
	m_tileCtrl->BuildGrids(m_navMesh);
	
	if(m_navQuery == NULL)
	{
		m_navQuery = new NavMeshQuery();
	}
	m_navQuery->SetNavMesh(m_navMesh);
	m_navQuery->SetMidPointCtrl(&m_mesh->m_midPtCtrl);
	m_navQuery->SetTileCtrl(m_tileCtrl);
	
	vcg::Point3f startPos(7.6912951, 10.100009, 51.732708);
	vcg::Point3f endPos(21.407295, 10.100000, -47.173748);

	std::vector<int> pathFaces;
	std::vector<vcg::Point3f> pathPts;
	m_navQuery->FindPath(startPos, endPos, pathFaces, pathPts);
	/*
	AStarGridNode node;
	std::vector<int> path, smoothPathIds;
	std::vector<vcg::Point3f> wayPoints, smoothWayPts;
	node.AStarSearch(0, 14, m_tileCtrl->m_grids[0].center, m_tileCtrl->m_grids[14].center, m_tileCtrl->m_grids, path, wayPoints);
	node.AStarPathSmooth(m_tileCtrl->m_grids, m_tileCtrl, m_navMesh, path, wayPoints, smoothWayPts, smoothPathIds);
	*/
	if (m_tool)
		m_tool->init(this);
	initToolStates(this);
	
	return true;
}

void HybridModel::handleUpdate(const float dt)
{
	Sample::handleUpdate(dt);
	
	if (!m_navMesh)
		return;
//	if (!m_tileCache)
//		return;
	
//	m_tileCache->update(dt, m_navMesh);
}

void HybridModel::getTilePos(const float* pos, int& tx, int& ty)
{
	//if (!m_geom) return;
	if (!m_mesh) return;
	const float* bmin = m_mesh->getMeshBoundsMin();
	
	const float ts = m_tileSize*m_cellSize;
	tx = (int)((pos[0] - bmin[0]) / ts);
	ty = (int)((pos[2] - bmin[2]) / ts);
}

void HybridModel::getGridParams(float& grid_size, float& ox, float& oy, float& ex, float& ey)
{
	const float* bmin = m_mesh->getMeshBoundsMin();
	const float* bmax = m_mesh->getMeshBoundsMax();
	
	// Tiling grid.
	/*int gw = 0, gh = 0;
	rcCalcGridSize(bmin, bmax, m_cellSize, &gw, &gh);
	const int tw = (gw + (int)m_tileSize-1) / (int)m_tileSize;
	const int th = (gh + (int)m_tileSize-1) / (int)m_tileSize;
	grid_size = m_tileSize*m_cellSize;*/
	grid_size = m_tileSize;
	ox = bmin[0];
	oy = bmin[2];
	ex = bmax[0];
	ey = bmax[2];
}