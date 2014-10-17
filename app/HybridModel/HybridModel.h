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


/*
* 接口文件，这个文件的任务是界面管理以及场景渲染，负责将仿真内容与main.cpp中的渲染联系起来
* 最重要的成员是HybridCrowdTool类型的工具文件，该类负责传递人群信息
*/

#ifndef HYBRIDMODEL_H
#define HYBRIDMODEL_H

#include "Sample.h"
#include "Recast.h"
#include "HybridModelCrowd.h"
#include "HybridCrowdTool.h"

class HybridModel : public Sample
{
protected:
	bool m_keepInterResults;

	//struct LinearAllocator* m_talloc;
	//struct FastLZCompressor* m_tcomp;
	//struct MeshProcess* m_tmproc;

	//class dtTileCache* m_tileCache;
	
	//float m_cacheBuildTimeMs;
	//int m_cacheCompressedSize;
	//int m_cacheRawSize;
	//int m_cacheLayerCount;
	//int m_cacheBuildMemUsage;
	
	enum DrawMode
	{
		DRAWMODE_NAVMESH,
		DRAWMODE_NAVMESH_TRANS,
		DRAWMODE_NAVMESH_BVTREE,
		DRAWMODE_NAVMESH_NODES,
		DRAWMODE_NAVMESH_PORTALS,
		DRAWMODE_NAVMESH_INVIS,
		DRAWMODE_MESH,
		DRAWMODE_CACHE_BOUNDS,
		MAX_DRAWMODE
	};
	
	DrawMode m_drawMode;
	HybridModelCrowd *m_hybridcrowd;
	int m_maxTiles;
	int m_maxPolysPerTile;
	float m_tileSize;

public:
	HybridModel();
	virtual ~HybridModel();
	
	virtual void handleSettings();
	virtual void handleTools();
	virtual void handleDebugMode();
	virtual void handleRender();
	virtual void handleRenderOverlay(double* proj, double* model, int* view);
	virtual void handleMeshChanged(class InputGeom* geom);
	virtual void handleMeshChanged(class InputMesh* mesh);
	virtual bool handleBuild();
	virtual void handleUpdate(const float dt);

	void getTilePos(const float* pos, int& tx, int& ty);
	
	void getGridParams(float& grid_size, float& ox, float& oy, float& ex, float& ey);	
	void addTempObstacle(const float* pos);
	void removeTempObstacle(const float* sp, const float* sq);
	void clearAllTempObstacles();

	inline HybridModelCrowd* getHybridCrowd() { return m_hybridcrowd; }
};


#endif // HYBRIDMODEL_H
