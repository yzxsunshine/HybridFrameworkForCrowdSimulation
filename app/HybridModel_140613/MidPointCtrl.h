#pragma once
#include <vcg/complex/complex.h>
#include "NavMesh.h"

// MidPoint
class MidPoint
{
public:
	vcg::Point3f position;
	int neighbor[4]; //
	int walkable;
	float cost[4];
	int faceIds[2];	// every mid point belongs to two faces;
	bool isBorder;
	int openID;
	bool isClose;
public:
	MidPoint(void);
	~MidPoint(void);
};
// MidPointCtrl

class MidPointCtrl
{
public:
	std::vector<MidPoint> m_midPointList;
public:
	MidPointCtrl(void);
	~MidPointCtrl(void);
	void BuildMidPointList(NavMesh& navMesh);
	void ResetMPList();
};
