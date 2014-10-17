#pragma once
#include "NavMesh.h"
#include "MidPointCtrl.h"

class InputMesh
{
public:
	InputMesh(void);
	~InputMesh(void);

	NavMesh m_navMesh;
	MidPointCtrl m_midPtCtrl;
	float m_minBox[3]; 
	float m_maxBox[3];
public:
	bool ReadMap(char* mapPath);
	float* getMeshBoundsMin() {return m_minBox;}
	float* getMeshBoundsMax() {return m_maxBox;}
	void FindWalkableArea(float walkableSlopeAngle, float agentRadius);
	int SetConnectComponentRecursively(NavMesh::FaceIterator beginfi, int offset);
	void BuildNavMesh();
	bool raycastMesh(float* src, float* dst, float& tmin);
	int getVertCount() { return m_navMesh.vn; }
	int getTriCount() { return m_navMesh.fn; }
private:
	inline float HelenArea(const float a, const float b, const float c)
	{
		float p = (a+b+c)/2;
		float S = sqrtf(p*(p-a)*(p-b)*(p-c));
		return S;
	}

	inline float getDistance(const vcg::Point3f& p0, const vcg::Point3f& p1)
	{
		return sqrtf((p0-p1).dot(p0-p1));
	}
};

