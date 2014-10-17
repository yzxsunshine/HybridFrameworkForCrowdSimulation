#include "FluidSimulator.h"
#include <vector>
#include <list>
#include "stdlib.h"
#include <GCoptimization.h>

#define PI 3.14159265359

FluidSimulator::FluidSimulator(void)
{
	m_labelNum = 0;
	m_xLabelNum = 0;
	m_yLabelNum = 0;
	m_labels = NULL;
	m_dataCostScale = 60;
	m_smoothCostScale = 50;
	m_maxCost = 100;
}

FluidSimulator::~FluidSimulator(void)
{
	if(!m_labels)
		delete[] m_labels;
}

void FluidSimulator::init(std::vector<Grid>* grids, HmAgent** agents, std::vector<CrowdGroup>* fluid)
{
	m_grids = grids;
	m_agents = agents;
	m_fluidGroups = fluid;

	// divide space into n pieces
	if(m_xLabelNum <= 0)
		m_xLabelNum = 11;
	if(m_yLabelNum <= 0)
		m_yLabelNum = 11;
	if(!m_labels)
		delete[] m_labels;
	m_labelNum = m_xLabelNum * m_yLabelNum;
	float maxSpeed = 3.5f;
	float xDelta = 2*maxSpeed / m_xLabelNum;
	float yDelta = 2*maxSpeed / m_yLabelNum;

	m_labels = new Vector2[m_labelNum];
	double deltaAngle = 2*PI/m_labelNum;
	for(int i=0; i<m_xLabelNum; i++)
	{
		for(int j=0; j<m_yLabelNum; j++)
		{
			int id = i*m_yLabelNum + j;
			m_labels[id].SetX(-maxSpeed + xDelta*i);
			m_labels[id].SetY(-maxSpeed + yDelta*j);
		}
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

int smoothFn(int p1, int p2, int l1, int l2, void* data)
{
	
	FluidSimulator* ptr = (FluidSimulator*)data;
	int gid1 = ptr->getGroupConst(ptr->getCurGroupID()).crowdGrid[p1];
	int gid2 = ptr->getGroupConst(ptr->getCurGroupID()).crowdGrid[p2];
	
	int dir12 = -1;	// neighbor direction from 1 to 2. the value from 0 to 3, means up, down, left, right
	int dir21 = -1;	// neighbor direction from 2 to 1

	for(int i=0; i<4; i++)
	{
		if(ptr->getGridConst(gid1).m_neighbor[i] == gid2)
		{
			dir12 = i;		
		}
		if(ptr->getGridConst(gid2).m_neighbor[i] == gid1)
		{
			dir21 = i;		
		}
	}
	Vector2 labelVal1 = ptr->getLabel(l1);
	Vector2 labelVal2 = ptr->getLabel(l2);
	/*
	if(ptr->getGridConst(gid1).nowDensity >= 1.0)	// we need to change 1.0 to some other constant later
	{	// the movement of gid2 is limited, as gid1 is full
		if(dir12 >= 2 && labelVal1.x() - labelVal2.x() > 0)	// gid2 is a horizontal neighbor of gid1
		{
			return ptr->getMaxCost() * ptr->getSmoothCostScale();
		}
		if(dir12 < 2 && labelVal1.y() - labelVal2.y() > 0)	// gid2 is a vertical neighbor of gid1
		{
			return ptr->getMaxCost() * ptr->getSmoothCostScale();
		}
	}

	if(ptr->getGridConst(gid2).nowDensity >= 1.0)	// we need to change 1.0 to some other constant later
	{	// the movement of gid1 is limited, as gid2 is full
		if(dir21 >= 2 && labelVal2.x() - labelVal1.x() > 0)	// gid2 is a horizontal neighbor of gid1
		{
			return ptr->getMaxCost() * ptr->getSmoothCostScale();
		}
		if(dir21 < 2 && labelVal2.y() - labelVal1.y() > 0)	// gid2 is a vertical neighbor of gid1
		{
			return ptr->getMaxCost() * ptr->getSmoothCostScale();
		}
	}
	*/
	return ( abs(labelVal1.x() - labelVal2.x()) + abs(labelVal1.y() - labelVal2.y()) ) * (abs(l1-l2)+2) / (abs(l1-l2)+1) * ptr->getSmoothCostScale();
}

void FluidSimulator::doStep()
{
//#pragma omp parallel for
	for(int crowdi = 0; crowdi<m_fluidGroups->size(); crowdi++)
	{
		m_curGroup = crowdi;
		int gridNum = (*m_fluidGroups)[crowdi].crowdGrid.size();
		for(int gridi = 0; gridi < gridNum; gridi++)
		{
			int gid = (*m_fluidGroups)[crowdi].crowdGrid[gridi];
			(*m_grids)[gid].avgVelocity = GetGridVelocity(gid);
		}
		if(gridNum > 1)
		{
			// graph cut minimal energy
			GCoptimizationGeneralGraph *gc = new GCoptimizationGeneralGraph(gridNum, m_labelNum);
			int* dataCost = new int[gridNum*m_labelNum];
			std::map<int, int> gridDict;
			// compute data cost
			for(int i=0; i < gridNum; i++)
			{
				int gid = (*m_fluidGroups)[crowdi].crowdGrid[i];
				gridDict.insert(std::pair<int, int>(gid, i));
				Vector2 curDir = (*m_grids)[gid].avgVelocity;
				for(int j=0; j<m_labelNum; j++)
				{
					if( sqrt(m_labels[j].dot(m_labels[j])) > m_agents[0]->params.maxSpeed )
					{
						dataCost[i*m_labelNum+j] = m_maxCost*m_dataCostScale;
					}
					else
					{
						dataCost[i*m_labelNum+j] = ( abs(curDir.x() - m_labels[j].x()) + abs(curDir.y() - m_labels[j].y()) ) * m_dataCostScale;
					}
					//(1 - curDir.dot(m_labels[j])) / 2 * m_dataCostScale;
				}
			}
			gc->setDataCost(dataCost);
			gc->setSmoothCost(smoothFn, this);
			/*int* smoothCost = new int[m_labelNum*m_labelNum];
			for(int i=0; i<m_labelNum; i++)
			{
				for(int j=0; j<m_labelNum; j++)
				{
					smoothCost[i*m_labelNum + j] = ( abs(m_labels[i].x() - m_labels[j].x()) + abs(m_labels[i].y() - m_labels[j].y()) ) * m_smoothCostScale;
				}
			}
			gc->setSmoothCost(smoothCost);*/
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
						for(int k=0; k<neighbors[smaller].size(); k++)
						{
							if(neighbors[smaller][k] == larger)
							{
								notIn = false;
							}
						}
						if(notIn)
						{
							neighbors[smaller].push_back(larger);
							gc->setNeighbors(i, iter->second);
							printf("(%d , %d)\n", i, iter->second);
						}
					}
				}
			}

			printf("\nBefore optimization energy is %d",gc->compute_energy());
			gc->expansion(2);// run expansion for 2 iterations. For swap use gc->swap(num_iterations);
			printf("\nAfter optimization energy is %d",gc->compute_energy());
		
			for(int i=0; i < gridNum; i++)
			{
				int gid = (*m_fluidGroups)[crowdi].crowdGrid[i];
				int labelID = gc->whatLabel(i);
				(*m_grids)[gid].avgVelocity = m_labels[labelID];
			}
			delete[] dataCost;
			gridDict.clear();
			delete gc;
		}
		// interpolation
		for(int gridi = 0; gridi < gridNum; gridi++)
		{
			int gid = (*m_fluidGroups)[crowdi].crowdGrid[gridi];
			//(*m_grids)[gid].avgVelocity = GetGridVelocity(gid);
			for(int i=0;i<(*m_grids)[gid].agentsInGrid.size();i++)
			{
				int agent_id = (*m_grids)[gid].agentsInGrid[i];			
				//(*m_agents)[agent_id].velocity_ = (*m_agents)[agent_id].prefVelocity_ 
				(*m_agents[agent_id]).velocity_ = (*m_agents)[agent_id].prefVelocity_ 
												+ (*m_grids)[gid].nowDensity*((*m_grids)[gid].avgVelocity - (*m_agents)[agent_id].prefVelocity_);
			}
			(*m_grids)[gid].isContour = false;
			//cur_grid->obstacle.clear();
		}
		(*m_fluidGroups)[crowdi].TrackContour(m_grids, m_agents);
		
		for(int i = 0; i < (*m_fluidGroups)[crowdi].contourAgent.size(); i++)
		{
			int aid = (*m_fluidGroups)[crowdi].contourAgent[i];
			//(*m_agents)[aid].is_contour = true;
			//(*m_agents)[aid].active = 3;
			(*m_agents[aid]).is_contour = true;
			(*m_agents[aid]).active = 3;
		}
	}
	
}
