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

	std::sort(agentList.begin(), agentList.end(), compareVertical);
	float radius = agents[0]->radius_;
	float ymin = agentList[0].center.y() - radius;
	float ymax = agentList[agentNum-1].center.y() + radius;
	float range = ymax - ymin;
	int bucketNum = ceil(range / radius / 2);
	assert(bucketNum<=agentNum);
	std::vector<std::vector<elem_grid>> agent_bucket;
	agent_bucket.resize(bucketNum);
	for(int i=0; i<agentNum; i++)
	{
		int bucketID = floor( (agentList[i].center.y() - ymin)/ (radius*2) );
		agent_bucket[bucketID].push_back(agentList[i]);
	}
	for(int i=0; i<bucketNum; i++)
	{
		std::sort(agent_bucket[i].begin(), agent_bucket[i].end(), compareHorizontal);
	}
	contourAgent.clear();
	contourPts.clear();
	contourAgent.reserve(bucketNum*2+2);
	contourPts.reserve(bucketNum*2+2);
	contourAgent.push_back(agentList[0].id);
	contourPts.push_back(agentList[0].center);

	for(int i=0; i<bucketNum; i++)
	{
		if(agent_bucket[i].empty())
			continue;
		contourAgent.push_back(agent_bucket[i][0].id);
		contourPts.push_back(agent_bucket[i][0].center);
	}

	contourAgent.push_back(agentList[agentNum-1].id);
	contourPts.push_back(agentList[agentNum-1].center);

	for(int i=bucketNum-1; i>=0; i--)
	{
		if(agent_bucket[i].empty())
			continue;
		int bucketSize = agent_bucket[i].size();
		contourAgent.push_back(agent_bucket[i][bucketSize-1].id);
		contourPts.push_back(agent_bucket[i][bucketSize-1].center);
	}

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
	}
}

/*
		int flag = 0;
		int flag2 = 0;
		int gid = crowdGrid[i];
		for(int j=0;j<4;j++)
		{
			int ngid = (*grids)[gid].m_neighbor[j];
			if(ngid < 0)
				continue;
			flag2++;
			if((*grids)[ngid].m_obstacleStatus == BLACK)
				flag++;
			else 
			{
				break;
			}

		}
		if(flag < flag2)
		{
			contourGrid.push_back(crowdGrid[i]);
			(*grids)[crowdGrid[i]].isContour = true;
		}
		*/