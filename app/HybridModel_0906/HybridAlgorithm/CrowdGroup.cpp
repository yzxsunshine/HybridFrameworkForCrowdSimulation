#include "CrowdGroup.h"
#include "../CommonDef.h"
#include <map>

CrowdGroup::CrowdGroup(void):RegionID(-1)
{
}

CrowdGroup::~CrowdGroup(void)
{
	
}
void CrowdGroup::AddGrid(int gridId)
{
	if(gridId >= 0)
	{
		crowdGrid.push_back(gridId);
	}
}

int compareVertical(const elem_grid& a, const elem_grid& b)
{
	return ( a.center.y() < b.center.y()  );
}

int compareHorizontal(const elem_grid& a, const elem_grid& b)
{
	return ( a.center.x() < b.center.x()  );
}

#define  S(arr,a,b,c) ((arr[b].x()-arr[a].x())*(arr[c].y()-arr[a].y())-(arr[c].x()-arr[a].x())*(arr[b].y()-arr[a].y()))    
#define  J(arr,a,b,c,d)  ((arr[b].x()-arr[a].x())*(arr[d].y()-arr[c].y())-(arr[d].x()-arr[c].x())*(arr[b].y()-arr[a].y()))   
#define  Q(x) ((x)*(x))   
#define  D(arr,a,b) (Q(arr[a].x()-arr[b].x())+Q(arr[a].y()-arr[b].y()))

void CrowdGroup::TrackContour(std::vector<Grid>* grids, HmAgent** agents)
{
	std::vector<int> agentsInGroup;
	avgVel.reset();
	for(int i=0; i<crowdGrid.size(); i++)
	{
		int gid = crowdGrid[i];
		for(int j=0; j<(*grids)[gid].agentsInGrid.size(); j++)
		{
			int aid = (*grids)[gid].agentsInGrid[j];
			//avgVel += (*agents)[aid].velocity_;
			avgVel += (*agents[aid]).velocity_;
			agentsInGroup.push_back(aid);
		}
	}

	int agentNum = agentsInGroup.size();
	avgVel = avgVel / agentNum;
	std::vector<elem_grid> agentList;
	agentList.resize(agentNum);
	for(int i=0; i<agentNum; i++)
	{
		int aid = agentsInGroup[i];
		//agentList[i].center.SetX((*agents)[aid].npos[0]);
		//agentList[i].center.SetY((*agents)[aid].npos[2]);
		agentList[i].center.SetX((*agents[aid]).npos[0]);
		agentList[i].center.SetY((*agents[aid]).npos[2]);
		agentList[i].id = aid;
	}

	std::sort(agentList.begin(), agentList.end(), compareHorizontal);
	float radius = agents[0]->radius_;
	float xmin = agentList[0].center.x() - radius;
	float xmax = agentList[agentNum-1].center.x() + radius;
	float range = xmax - xmin;
	int bucketNum = ceil(range / radius / 2);
	if(bucketNum>=agentNum)
		return;
	std::vector<std::vector<elem_grid>> agent_bucket;
	agent_bucket.resize(bucketNum);
	for(int i=0; i<agentNum; i++)
	{
		int bucketID = floor( (agentList[i].center.x() - xmin)/ (radius*2) );
		agent_bucket[bucketID].push_back(agentList[i]);
	}
	for(int i=0; i<bucketNum; i++)
	{
		std::sort(agent_bucket[i].begin(), agent_bucket[i].end(), compareVertical);
	}
	contourAgent.clear();
	contourPts.clear();
	contourAgent.reserve(bucketNum*2+2);
	contourPts.reserve(bucketNum*2+2);
	contourAgent.push_back(agentList[0].id);
	contourPts.push_back(agentList[0].center);

	std::vector<Vector2> tmpContour;
	tmpContour.push_back(agentList[0].center);
	for(int i=0; i<bucketNum; i++)
	{
		if(agent_bucket[i].empty())
			continue;
		contourAgent.push_back(agent_bucket[i][0].id);
		contourPts.push_back(agent_bucket[i][0].center);

		int bucketSize = agent_bucket[i].size();
		tmpContour.push_back(agent_bucket[i][0].center);
		if(bucketSize > 1)
		{
			tmpContour.push_back(agent_bucket[i][bucketSize-1].center);
		}
	}
	tmpContour.push_back(agentList[agentNum-1].center);

	contourAgent.push_back(agentList[agentNum-1].id);
	contourPts.push_back(agentList[agentNum-1].center);

	for(int i=bucketNum-1; i>=0; i--)
	{
		if(agent_bucket[i].empty())
			continue;
		int bucketSize = agent_bucket[i].size();
		if(bucketSize > 1)
		{
			contourAgent.push_back(agent_bucket[i][bucketSize-1].id);
			contourPts.push_back(agent_bucket[i][bucketSize-1].center);
		}
	}

	int pointNum = tmpContour.size();
	int  *ptIds = new int [2*pointNum];   
    int  p,q;   
    p =q =pointNum;   
	for ( int  i=0;i<pointNum;i++){   
		ptIds[p] =ptIds[q] =i;   
			while (p-pointNum>=2&&S(tmpContour, ptIds[p], ptIds[p-1], ptIds[p-2])<=0){ptIds[p-1] =ptIds[p]; p--;}   
			while (pointNum-q>=2&&S(tmpContour, ptIds[q+2], ptIds[q+1], ptIds[q])>=0){ptIds[q+1] =ptIds[q]; q++;}   
		p++;   
		q--;   
	}   
	int  len = p -q -2;   
	contourPts.clear();
	contourPts.resize(len+2);
	for ( int i=q+1; i<p; i++) 
		contourPts[i-q] =tmpContour[ptIds[i]];    
	contourPts[0] = tmpContour[len];   

	/*
	contours.clear();
	std::vector<Vector2> contour, contourNext;
	contour.push_back(agentList[0].center);
	int id = 0;
	for(int i=0; i<bucketNum; i++)
	{
		if(agent_bucket[i].empty()|| agent_bucket[i].size() < 2)
			continue;
		int bucketSize = agent_bucket[i].size();
		contour.push_back(agent_bucket[i][0].center);
		contour.push_back(agent_bucket[i][bucketSize-1].center);
		id = i;
		break;
	}
	contours.push_back(contour);
	bool lastSingle = false;
	for(int i=id+1; i<bucketNum; i++)
	{
		if(agent_bucket[i].empty())
			continue;
		int bucketSize = agent_bucket[i].size();
		if(bucketSize < 2)
		{
			if(!lastSingle)
			{
				contourNext.push_back(contour[1]);
				contourNext.push_back(contour[2]);
				contourNext.push_back(agent_bucket[i][0].center);
				contour = contourNext;
				contourNext.clear();
				contours.push_back(contour);
			}
			lastSingle = true;
			continue;
		}
		if(lastSingle)
		{
			contourNext.push_back(contour[2]);
			contourNext.push_back(agent_bucket[i][0].center);
			contourNext.push_back(agent_bucket[i][bucketSize-1].center);
			contour = contourNext;
			contourNext.clear();
			contours.push_back(contour);
		}
		else
		{
			contourNext.push_back(contour[1]);
			contourNext.push_back(contour[2]);
			contourNext.push_back(agent_bucket[i][0].center);	
			contour = contourNext;
			contourNext.clear();
			contours.push_back(contour);

			contourNext.push_back(contour[1]);
			contourNext.push_back(contour[2]);
			contourNext.push_back(agent_bucket[i][bucketSize-1].center);	
			contour = contourNext;
			contourNext.clear();
			contours.push_back(contour);
		}
		lastSingle = false;
	}*/
}

void CrowdGroup::GetGroupAgent(Vector2& pos, Vector2& vel, float& radius, std::vector<Grid>* grids, HmAgent** agents)
{
	vel.reset();
	pos.reset();
	int agentNum = 0;
	for(int i=0; i<crowdGrid.size(); i++)
	{
		int gid = crowdGrid[i];
		for(int j=0; j<(*grids)[gid].agentsInGrid.size(); j++)
		{
			int aid = (*grids)[gid].agentsInGrid[j];
			//avgVel += (*agents)[aid].velocity_;
			vel += (*agents[aid]).velocity_;
			pos += Vector2((*agents[aid]).npos[0], (*agents[aid]).npos[2]);
			agentNum++;
		}
	}
	vel = vel / agentNum;
	pos = pos / agentNum;
	std::vector<float> dists;
	dists.resize(agentNum);
	int count = 0;
	for(int i=0; i<crowdGrid.size(); i++)
	{
		int gid = crowdGrid[i];
		for(int j=0; j<(*grids)[gid].agentsInGrid.size(); j++)
		{
			int aid = (*grids)[gid].agentsInGrid[j];
			Vector2 p((*agents[aid]).npos[0], (*agents[aid]).npos[2]);
			Vector2 vec = p - pos;
			dists[count] = sqrt(vec.dot(vec));
			count++;
		}
	}
	std::sort(dists.begin(), dists.end());
	float maxVal = dists[agentNum-1];
	float medianVal = dists[agentNum/2];
	radius = std::min(medianVal*1.2f, maxVal);
}