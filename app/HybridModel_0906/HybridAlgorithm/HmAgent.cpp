#include "HmAgent.h"
//vector<HmAgent> HmAgent::test_array;
float HmAgent::adjMat[2][5] = { {0, -0.04, 0.04, 0.75, 0.66}, {0.14, 0.5, 0.8, 0.15, -0.19} };

HmAgent::HmAgent():HmAgent_id(0),grid_former_id(0),grid_now_id(0)
{
	color_id = 0;
}
HmAgent::HmAgent(RVO::RVOSimulator* sim):HmAgent_id(0),grid_former_id(0),grid_now_id(0),Agent(sim)
{
	color_id = 0;
}
HmAgent::HmAgent(RVO::RVOSimulator* sim,int p_id, int g_former_id, int g_now_id, double pos_x,double pos_y):HmAgent_id(p_id),grid_former_id(g_former_id),grid_now_id(g_now_id),Agent(sim)
{
	color_id = 0;
	position_.SetX(pos_x);
	position_.SetY(pos_y);
}

HmAgent::~HmAgent(void)
{
}
void HmAgent::setHmAgent(double neighborDist, size_t maxNeighbors, double timeHorizon, double timeHorizonObst, double radius, double maxSpeed)
{
	this->maxNeighbors_ = maxNeighbors;
	this->maxSpeed_ = maxSpeed;
	this->neighborDist_ = neighborDist;
	this->radius_ = radius;
	this->timeHorizon_ = timeHorizon;
	this->timeHorizonObst_ = timeHorizonObst;
	this->m_pc1 = adjMat[0][0] * (neighborDist_ - 15) / 13.5 + 
				  adjMat[0][1] * (maxNeighbors_ - 10) / 49.5 + 
				  adjMat[0][2] * (timeHorizon_ - 30) / 14.5 + 
				  adjMat[0][3] * (radius_ - 0.8) / 0.85 + 
				  adjMat[0][4] * (maxSpeed_ - 1.4) / 0.5;

	this->m_pc2 = adjMat[1][0] * (neighborDist_ - 15) / 13.5 + 
				  adjMat[1][1] * (maxNeighbors_ - 10) / 49.5 + 
				  adjMat[1][2] * (timeHorizon_ - 30) / 14.5 + 
				  adjMat[1][3] * (radius_ - 0.8) / 0.85 + 
				  adjMat[1][4] * (maxSpeed_ - 1.4) / 0.5;
}

void HmAgent::setHmAgent(const SimulationParams& simParams) 
{
	setHmAgent(simParams.m_maxNeighborDist, simParams.m_maxNumNeighbors
			 , simParams.m_planHorizon, simParams.m_planHorizon
			 , simParams.m_agentRadius, simParams.m_preferSpeed);
}

void HmAgent::setHmAgent(float pc1, float pc2)
{
	this->m_pc1 = pc1;
	this->m_pc2 = pc2;

	this->maxNeighbors_ = pc1*adjMat[0][1] + pc2*adjMat[1][1];
	this->maxSpeed_ = pc1*adjMat[0][4] + pc2*adjMat[1][4];
	this->neighborDist_ = pc1*adjMat[0][0] + pc2*adjMat[1][0];
	this->radius_ = pc1*adjMat[0][3] + pc2*adjMat[1][3];
	this->timeHorizon_ = pc1*adjMat[0][2] + pc2*adjMat[1][2];
	this->timeHorizonObst_ = pc1*adjMat[0][2] + pc2*adjMat[1][2];
}

void HmAgent::setHmAgent(const HmAgent& hmAgent)
{
	this->setHmAgent(hmAgent.neighborDist_, hmAgent.maxNeighbors_, hmAgent.timeHorizon_, 
					 hmAgent.timeHorizonObst_, hmAgent.radius_, hmAgent.maxSpeed_);
}

void HmAgent::setHmAgent(const float* position, const float* vel )
{
	position_ = Vector2(position[0], position[2]);
    maxNeighbors_ = sim_->defaultAgent_->maxNeighbors_;
	maxSpeed_ = params.maxSpeed; //desiredSpeed;
    neighborDist_ = sim_->defaultAgent_->neighborDist_;
    radius_ = sim_->defaultAgent_->radius_;
    timeHorizon_ = sim_->defaultAgent_->timeHorizon_;
    timeHorizonObst_ = sim_->defaultAgent_->timeHorizonObst_;
    prefVelocity_ = Vector2(vel[0], vel[2]);
}