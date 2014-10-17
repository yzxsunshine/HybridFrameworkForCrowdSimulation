#include "NavCorridor.h"


NavCorridor::NavCorridor(void)
{
}


NavCorridor::~NavCorridor(void)
{
}

void NavCorridor::SetCorridor(std::vector<vcg::Point3f> path)
{
	for(int i=0; i<path.size(); i++)
	{
		pts.push(path[i]);
	}
}

int NavCorridor::UpdateDir(const float* pos, float radius)
{
	if(pts.empty())
		return 0;
	vcg::Point3f nextPt = pts.front();
	vcg::Point3f curPt(pos[0], pos[1], pos[2]);
	float dist = (nextPt - curPt).Norm();
	if(dist < radius)
	{
		pts.pop();
	}
	return pts.size();
}

void NavCorridor::GetSteerDir(const float* pos, float* dir)
{
	if(pts.empty())
	{
		dir[0] = 0;	dir[1] = 0;	dir[2] = 0;
		return;
	}
	vcg::Point3f curPt(pos[0], pos[1], pos[2]);
	vcg::Point3f nextPt = pts.front();
	vcg::Point3f dirVec = nextPt - curPt;
	//dirVec.Y() = 0;
	dirVec.Normalize();
	dir[0] = dirVec.X();
	dir[1] = dirVec.Y();
	dir[2] = dirVec.Z();
}

float NavCorridor::GetDistanceToGoal(const float* pos, const float range)
{
	if(pts.empty())
		return range;
	
	const bool endOfPath = (pts.size() == 1) ? true : false;
	if (endOfPath)
	{
		vcg::Point3f curPt(pos[0], pos[1], pos[2]);
		vcg::Point3f nextPt = pts.front();
		return std::min((nextPt - curPt).Norm(), range);
	}
	return range;
}

void NavCorridor::ClearPath()
{
	pts.swap(std::queue<vcg::Point3f>());
}