#pragma once
#include <vcg\complex\complex.h>
#include <queue>

class NavCorridor
{
public:
	NavCorridor(void);
	~NavCorridor(void);

	void SetCorridor(std::vector<vcg::Point3f> path);
	int UpdateDir(const float* pos, float radius);
	void GetSteerDir(const float* pos, float* dir);
	float GetDistanceToGoal(const float* pos, const float range);
	void ClearPath();
public:
	std::queue<vcg::Point3f> pts;	//路径中的点
};

