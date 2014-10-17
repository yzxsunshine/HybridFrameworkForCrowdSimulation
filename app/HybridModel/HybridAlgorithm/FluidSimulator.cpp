#include "FluidSimulator.h"
#include <vector>
#include <list>
#include "stdlib.h"
#include <GCoptimization.h>
#include "../CommonDef.h"

#define PI 3.14159265359
float FluidSimulator::m_alpha = 0.3;
float FluidSimulator::m_beta = -0.3;

FluidSimulator::FluidSimulator(void)
{
	m_xLabelNum = 0;
	m_yLabelNum = 0;
	m_xLabels = NULL;
	m_yLabels = NULL;
	m_dataCostScale = 60;
	m_smoothCostScale = 100;
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

void FluidSimulator::init(std::vector<Grid>* grids, HmAgent** agents, std::vector<CrowdGroup>* fluid)
{
	m_grids = grids;
	m_agents = agents;
	m_fluidGroups = fluid;

	// divide space into n pieces
	if(m_xLabelNum <= 0)
		m_xLabelNum = 41;
	if(m_yLabelNum <= 0)
		m_yLabelNum = 41;
	if(!m_xLabels)
		delete[] m_xLabels;
	if(!m_yLabels)
		delete[] m_yLabels;

	float maxSpeed = MAX_SPEED;
	float xDelta = 2*maxSpeed / m_xLabelNum;
	float yDelta = 2*maxSpeed / m_yLabelNum;

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

Vector2 FluidSimulator::GetGridVelocity(int gridid)
{
	Vector2 AvgVelocity(0,0);
	int agents_num = (*m_grids)[gridid].agentsInGrid.size();
	for(int i=0;i<agents_num;i++)
	{
		int agentid = (*m_grids)[gridid].agentsInGrid[i];			
		//AvgVelocity += Vector2((*m_agents)[agentid].dvel[0], (*m_agents)[agentid].dvel[2]);
		AvgVelocity += Vector2((*m_agents[agentid]).dvel[0], (*m_agents[agentid]).dvel[2]);
	}
	AvgVelocity /= agents_num;
	return AvgVelocity;
}

void FluidSimulator::GetGridTraits(int gridid, float& pc1, float& pc2)
{
	pc1 = 0;
	pc2 = 0;
	int agents_num = (*m_grids)[gridid].agentsInGrid.size();
	for (int i = 0; i<agents_num; i++)
	{
		int agentid = (*m_grids)[gridid].agentsInGrid[i];
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
	int gid1 = ptr->getGroupConst(ptr->getCurGroupID()).crowdGrid[p1];
	int gid2 = ptr->getGroupConst(ptr->getCurGroupID()).crowdGrid[p2];
	
	if(ptr->getGridConst(gid1).nowDensity >= 1.0)	// we need to change 1.0 to some other constant later
	{	// the movement of gid2 is limited, as gid1 is full
		if(l1 - l2 > 0)	// gid2 is a horizontal neighbor of gid1
		{
			return ptr->getMaxCost() * ptr->getSmoothCostScale();
		}
	}

	if(ptr->getGridConst(gid2).nowDensity >= 1.0)	// we need to change 1.0 to some other constant later
	{	// the movement of gid1 is limited, as gid2 is full
		if(l2 - l1 > 0)	// gid2 is a horizontal neighbor of gid1
		{
			return ptr->getMaxCost() * ptr->getSmoothCostScale();
		}
	}
	//  the contribution of group personal traits
	float weight = 1 + ptr->GetAlpha()*ptr->getGridConst(gid1).PC1 + ptr->GetBeta()*ptr->getGridConst(gid1).PC2;	// vary from 0.5 to 1.5

	return weight * abs(l1 - l2) * ptr->getSmoothCostScale();	
	//( abs(labelVal1.x() - labelVal2.x()) + abs(labelVal1.y() - labelVal2.y()) ) * (abs(l1-l2)+2) / (abs(l1-l2)+1) * ptr->getSmoothCostScale();
}


void FluidSimulator::doStep()
{
//#pragma omp parallel for
	for (int crowdi = 0; crowdi<(int)m_fluidGroups->size(); crowdi++)
	{
		m_curGroup = crowdi;
		int gridNum = (*m_fluidGroups)[crowdi].crowdGrid.size();
		for(int gridi = 0; gridi < gridNum; gridi++)
		{
			int gid = (*m_fluidGroups)[crowdi].crowdGrid[gridi];
			(*m_grids)[gid].avgVelocity = GetGridVelocity(gid);
			GetGridTraits(gid, (*m_grids)[gid].PC1, (*m_grids)[gid].PC2);
		}
		m_lastCallEM++;
		if(gridNum > 1 && m_lastCallEM >= 10)
		{
			m_lastCallEM = 0;
			// graph cut minimal energy
			GCoptimizationGeneralGraph *gcHorizontal = new GCoptimizationGeneralGraph(gridNum, m_xLabelNum);
			GCoptimizationGeneralGraph *gcVertical = new GCoptimizationGeneralGraph(gridNum, m_yLabelNum);
			int* dataCostHorizontal = new int[gridNum*m_xLabelNum];
			int* dataCostVertical = new int[gridNum*m_yLabelNum];
			std::map<int, int> gridDict;
			// compute data cost
			for(int i=0; i < gridNum; i++)
			{
				int gid = (*m_fluidGroups)[crowdi].crowdGrid[i];
				gridDict.insert(std::pair<int, int>(gid, i));
				Vector2 curDir = (*m_grids)[gid].avgVelocity;
				// x Dir
				for(int j=0; j<m_xLabelNum; j++)
				{
					dataCostHorizontal[i*m_xLabelNum+j] = abs( curDir.x() - m_xLabels[j] ) * m_dataCostScale;
				}
				for(int j=0; j<m_yLabelNum; j++)
				{
					dataCostVertical[i*m_yLabelNum+j] = abs( curDir.y() - m_yLabels[j] ) * m_dataCostScale;
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
			neighbors.resize(gridNum);
			int smaller, larger;
			for(int i=0; i < gridNum; i++)
			{
				int gid = (*m_fluidGroups)[crowdi].crowdGrid[i];
				for(int j=0; j<4; j++)
				{
					int nid = (*m_grids)[gid].m_neighbor[j];
					if(nid < 0)
						continue;
					iter = gridDict.find(nid);
					if(iter != gridDict.end())
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
		
			for(int i=0; i < gridNum; i++)
			{
				int gid = (*m_fluidGroups)[crowdi].crowdGrid[i];
				int xLabelID = gcHorizontal->whatLabel(i);
				int yLabelID = gcVertical->whatLabel(i);
				(*m_grids)[gid].avgVelocity.SetX(m_xLabels[xLabelID]);
				(*m_grids)[gid].avgVelocity.SetY(m_yLabels[yLabelID]);
			}
			delete[] dataCostHorizontal;
			delete[] dataCostVertical;
			gridDict.clear();
			delete gcHorizontal;
			delete gcVertical;
		}
		// interpolation
		for(int gridi = 0; gridi < gridNum; gridi++)
		{
			int gid = (*m_fluidGroups)[crowdi].crowdGrid[gridi];
			//(*m_grids)[gid].avgVelocity = GetGridVelocity(gid);
			for (int i = 0; i<(int)(*m_grids)[gid].agentsInGrid.size(); i++)
			{
				int agent_id = (*m_grids)[gid].agentsInGrid[i];			
				Vector2 interpVel = (*m_grids)[gid].avgVelocity;
				(*m_agents[agent_id]).velocity_ = (*m_agents)[agent_id].prefVelocity_ 
												+ (*m_agents[agent_id]).density * (interpVel - (*m_agents)[agent_id].prefVelocity_);
			}
			(*m_grids)[gid].isContour = false;
			//cur_grid->obstacle.clear();
		}
		(*m_fluidGroups)[crowdi].TrackContour(m_grids, m_agents);
		
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
