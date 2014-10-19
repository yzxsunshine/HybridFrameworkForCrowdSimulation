#include "FluidSimulator.h"
#include <vector>
#include <list>
#include "stdlib.h"
#include <GCoptimization.h>
#include "../CommonDef.h"

#define PI 3.14159265359
float FluidSimulator::m_alpha = -100;
float FluidSimulator::m_beta = 100;
static const float GC_WEIGHT = 0.7;

FluidSimulator::FluidSimulator(void)
{
	m_xLabelNum = 0;
	m_yLabelNum = 0;
	m_xLabels = NULL;
	m_yLabels = NULL;
	m_dataCostScale = 200;
	m_smoothCostScale = 10;
	m_maxCost = 100;
	m_lastCallEM = 0;
}

FluidSimulator::~FluidSimulator(void)
{
	if(!m_xLabels)
		delete[] m_xLabels;
	if(!m_yLabels)
		delete[] m_yLabels;
}

void FluidSimulator::init(std::vector<Group>* groups, HmAgent** agents, std::vector<CrowdGroup>* fluid)
{
	m_groups = groups;
	m_agents = agents;
	m_fluidGroups = fluid;

	// divide space into n pieces
	if(m_xLabelNum <= 0)
		m_xLabelNum = 81;
	if(m_yLabelNum <= 0)
		m_yLabelNum = 81;
	if(!m_xLabels)
		delete[] m_xLabels;
	if(!m_yLabels)
		delete[] m_yLabels;

	float maxSpeed = MAX_SPEED;
	float xDelta = 2*maxSpeed / (m_xLabelNum-1);
	float yDelta = 2*maxSpeed / (m_yLabelNum-1);

	m_xLabels = new float[m_xLabelNum];
	m_yLabels = new float[m_yLabelNum];
	for(int i=0; i<m_xLabelNum; i++)
	{
		m_xLabels[i] = -maxSpeed + xDelta*i;
	}

	for(int i=0; i<m_yLabelNum; i++)
	{
		m_yLabels[i] = -maxSpeed + yDelta*i;
	}
}

Vector2 FluidSimulator::GetGroupVelocity(int groupid)
{
	Vector2 AvgVelocity(0,0);
	int agents_num = (*m_groups)[groupid].agentsInGroup.size();
	for(int i=0;i<agents_num;i++)
	{
		int agentid = (*m_groups)[groupid].agentsInGroup[i];			
		//AvgVelocity += Vector2((*m_agents)[agentid].dvel[0], (*m_agents)[agentid].dvel[2]);
		AvgVelocity += Vector2((*m_agents[agentid]).dvel[0], (*m_agents[agentid]).dvel[2]);
	}
	AvgVelocity /= agents_num;
	return AvgVelocity;
}

void FluidSimulator::GetGroupTraits(int groupid, float& pc1, float& pc2)
{
	pc1 = 0;
	pc2 = 0;
	int agents_num = (*m_groups)[groupid].agentsInGroup.size();
	for (int i = 0; i<agents_num; i++)
	{
		int agentid = (*m_groups)[groupid].agentsInGroup[i];
		//AvgVelocity += Vector2((*m_agents)[agentid].dvel[0], (*m_agents)[agentid].dvel[2]);
		pc1 += (*m_agents[agentid]).m_pc1;
		pc2 += (*m_agents[agentid]).m_pc2;
	}
	pc1 /= agents_num;
	pc2 /= agents_num;
	return;
}

int smoothFn(int p1, int p2, int l1, int l2, void* data)
{
	
	FluidSimulator* ptr = (FluidSimulator*)data;
	int gid1 = ptr->getCrowdGroups(ptr->getCurGroupID()).crowdGroup[p1];
	int gid2 = ptr->getCrowdGroups(ptr->getCurGroupID()).crowdGroup[p2];
	
	if(ptr->getGroupConst(gid1).nowDensity >= 1.0)	// we need to change 1.0 to some other constant later
	{	// the movement of gid2 is limited, as gid1 is full
		if(l1 - l2 > 0)	// gid2 is a horizontal neighbor of gid1
		{
			return ptr->getMaxCost() * ptr->getSmoothCostScale();
		}
	}

	if(ptr->getGroupConst(gid2).nowDensity >= 1.0)	// we need to change 1.0 to some other constant later
	{	// the movement of gid1 is limited, as gid2 is full
		if(l2 - l1 > 0)	// gid2 is a horizontal neighbor of gid1
		{
			return ptr->getMaxCost() * ptr->getSmoothCostScale();
		}
	}
	//  the contribution of group personal traits
	return abs(l1 - l2) * ptr->getSmoothCostScale();	
	//( abs(labelVal1.x() - labelVal2.x()) + abs(labelVal1.y() - labelVal2.y()) ) * (abs(l1-l2)+2) / (abs(l1-l2)+1) * ptr->getSmoothCostScale();
}


void FluidSimulator::doStep()
{
//#pragma omp parallel for
	for (int crowdi = 0; crowdi<(int)m_fluidGroups->size(); crowdi++)
	{
		m_curGroup = crowdi;
		int groupNum = (*m_fluidGroups)[crowdi].crowdGroup.size();
		for(int groupi = 0; groupi < groupNum; groupi++)
		{
			int gid = (*m_fluidGroups)[crowdi].crowdGroup[groupi];
			(*m_groups)[gid].avgVelocity = GetGroupVelocity(gid);
			GetGroupTraits(gid, (*m_groups)[gid].PC1, (*m_groups)[gid].PC2);
		}
		m_lastCallEM++;
		if(groupNum > 1 && m_lastCallEM >= 10)
		{
			m_lastCallEM = 0;
			// graph cut minimal energy
			GCoptimizationGeneralGraph *gcHorizontal = new GCoptimizationGeneralGraph(groupNum, m_xLabelNum);
			GCoptimizationGeneralGraph *gcVertical = new GCoptimizationGeneralGraph(groupNum, m_yLabelNum);
			int* dataCostHorizontal = new int[groupNum*m_xLabelNum];
			int* dataCostVertical = new int[groupNum*m_yLabelNum];
			std::map<int, int> groupDict;
			// compute data cost
			for(int i=0; i < groupNum; i++)
			{
				int gid = (*m_fluidGroups)[crowdi].crowdGroup[i];
				groupDict.insert(std::pair<int, int>(gid, i));
				Vector2 curDir = (*m_groups)[gid].avgVelocity;
				// x Dir
				float weight = 1 + (*m_groups)[gid].PC1 + (*m_groups)[gid].PC2;	// vary from 0.5 to 1.5
				for(int j=0; j<m_xLabelNum; j++)
				{
					dataCostHorizontal[i*m_xLabelNum + j] = weight * abs(curDir.x() - m_xLabels[j]) * m_dataCostScale;
				}
				for(int j=0; j<m_yLabelNum; j++)
				{
					dataCostVertical[i*m_yLabelNum + j] = weight * abs(curDir.y() - m_yLabels[j]) * m_dataCostScale;
				}
			}
			gcHorizontal->setDataCost(dataCostHorizontal);
			gcVertical->setDataCost(dataCostVertical);

			gcHorizontal->setSmoothCost(smoothFn, this);
			gcVertical->setSmoothCost(smoothFn, this);
			/*int* smoothCostHorizontal = new int[m_xLabelNum*m_xLabelNum];
			for(unsigned int i=0; i<m_xLabelNum; i++)
			{
				for(int j=0; j<m_xLabelNum; j++)
				{
					smoothCostHorizontal[i*m_xLabelNum + j] = abs(i-j) * m_smoothCostScale;
				}
			}
			int* smoothCostVerticalal = new int[m_yLabelNum*m_yLabelNum];
			for(unsigned int i=0; i<m_yLabelNum; i++)
			{
				for(int j=0; j<m_yLabelNum; j++)
				{
					smoothCostVerticalal[i*m_yLabelNum + j] = abs(i-j) * m_smoothCostScale;
				}
			}
			gcHorizontal->setSmoothCost(smoothCostHorizontal);
			gcVertical->setSmoothCost(smoothCostVerticalal);*/
			// build neighbor
			std::map<int, int>::iterator iter;
			printf("======================SET NEIGHBOR======================\n");
			std::vector<std::vector<int>> neighbors;
			neighbors.resize(groupNum);
			int smaller, larger;
			for(int i=0; i < groupNum; i++)
			{
				int gid = (*m_fluidGroups)[crowdi].crowdGroup[i];
				for(int j=0; j<4; j++)
				{
					int nid = (*m_groups)[gid].m_neighbor[j];
					if(nid < 0)
						continue;
					iter = groupDict.find(nid);
					if(iter != groupDict.end())
					{
						if( i < iter->second)
						{
							smaller = i;
							larger = iter->second;
						}
						else
						{
							smaller = iter->second;
							larger = i;
						}
						bool notIn = true;
						for (int k = 0; k<(int)neighbors[smaller].size(); k++)
						{
							if(neighbors[smaller][k] == larger)
							{
								notIn = false;
							}
						}
						if(notIn)
						{
							neighbors[smaller].push_back(larger);
							if(j<2)
								gcVertical->setNeighbors(i, iter->second);
							if(j>=2)
								gcHorizontal->setNeighbors(i, iter->second);
							printf("(%d , %d)\n", i, iter->second);
						}
					}
				}
			}

			printf("\nBefore optimization horizontal energy is %d",gcHorizontal->compute_energy());
			gcHorizontal->expansion(2);// run expansion for 2 iterations. For swap use gc->swap(num_iterations);
			printf("\nAfter optimization horizontal energy is %d",gcHorizontal->compute_energy());

			printf("\nBefore optimization vertical energy is %d",gcVertical->compute_energy());
			gcVertical->expansion(2);// run expansion for 2 iterations. For swap use gc->swap(num_iterations);
			printf("\nAfter optimization vertical energy is %d",gcVertical->compute_energy());
		
			for(int i=0; i < groupNum; i++)
			{
				int gid = (*m_fluidGroups)[crowdi].crowdGroup[i];
				int xLabelID = gcHorizontal->whatLabel(i);
				int yLabelID = gcVertical->whatLabel(i);
				(*m_groups)[gid].avgVelocity.SetX(m_xLabels[xLabelID] * GC_WEIGHT + (1 - GC_WEIGHT)*(*m_groups)[gid].avgVelocity.x());
				(*m_groups)[gid].avgVelocity.SetY(m_yLabels[yLabelID] * GC_WEIGHT + (1 - GC_WEIGHT)*(*m_groups)[gid].avgVelocity.y());
			}
			delete[] dataCostHorizontal;
			delete[] dataCostVertical;
			groupDict.clear();
			delete gcHorizontal;
			delete gcVertical;
		}
		// interpolation
		for(int groupi = 0; groupi < groupNum; groupi++)
		{
			int gid = (*m_fluidGroups)[crowdi].crowdGroup[groupi];
			//(*m_groups)[gid].avgVelocity = GetGroupVelocity(gid);
			for (int i = 0; i<(int)(*m_groups)[gid].agentsInGroup.size(); i++)
			{
				int agent_id = (*m_groups)[gid].agentsInGroup[i];			
				Vector2 interpVel = (*m_groups)[gid].avgVelocity;
				(*m_agents[agent_id]).velocity_ = (*m_agents)[agent_id].prefVelocity_ 
												+ (*m_agents[agent_id]).density * (interpVel - (*m_agents)[agent_id].prefVelocity_);
			}
			(*m_groups)[gid].isContour = false;
			//cur_group->obstacle.clear();
		}
		(*m_fluidGroups)[crowdi].TrackContour(m_groups, m_agents);
		
		for (int i = 0; i < (int)(*m_fluidGroups)[crowdi].contourAgent.size(); i++)
		{
			int aid = (*m_fluidGroups)[crowdi].contourAgent[i];
			//(*m_agents)[aid].is_contour = true;
			//(*m_agents)[aid].active = 3;
			(*m_agents[aid]).is_contour = true;
			(*m_agents[aid]).active = 3;
		}
	}
	
}

void FluidSimulator::SetTraitCoeff(float alpha, float beta) {
	m_alpha = alpha;
	m_beta = beta;
}
