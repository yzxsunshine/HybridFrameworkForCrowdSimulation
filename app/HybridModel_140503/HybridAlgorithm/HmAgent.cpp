#include "HmAgent.h"
//vector<HmAgent> HmAgent::test_array;
HmAgent::HmAgent():HmAgent_id(0),grid_former_id(0),grid_now_id(0)
{

}
HmAgent::HmAgent(RVO::RVOSimulator* sim):HmAgent_id(0),grid_former_id(0),grid_now_id(0),Agent(sim)
{

}
HmAgent::HmAgent(RVO::RVOSimulator* sim,int p_id, int g_former_id, int g_now_id, double pos_x,double pos_y):HmAgent_id(p_id),grid_former_id(g_former_id),grid_now_id(g_now_id),Agent(sim)
{
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
}
void HmAgent::setHmAgent(const HmAgent& HmAgent)
{
	this->setHmAgent(HmAgent.neighborDist_,HmAgent.maxNeighbors_,HmAgent.timeHorizon_,HmAgent.timeHorizonObst_,HmAgent.radius_,HmAgent.maxSpeed_);
}

void HmAgent::setHmAgent(const float* position, const float* vel )
{
	position_ = Vector2(position[0], position[2]);
    maxNeighbors_ = sim_->defaultAgent_->maxNeighbors_;
	maxSpeed_ = desiredSpeed;
    neighborDist_ = sim_->defaultAgent_->neighborDist_;
    radius_ = sim_->defaultAgent_->radius_;
    timeHorizon_ = sim_->defaultAgent_->timeHorizon_;
    timeHorizonObst_ = sim_->defaultAgent_->timeHorizonObst_;
    prefVelocity_ = Vector2(vel[0], vel[2]);
}