#include "FluidSimulator.h"
#include <vector>
#include <list>
#include "stdlib.h"
#define PI 3.14159265359

FluidSimulator::FluidSimulator(void)
{
}

FluidSimulator::~FluidSimulator(void)
{
}

void FluidSimulator::init(HybridModelCrowd *hmc)
{
	this->m_hmcrowd = hmc;
}

Vector2 FluidSimulator::GetGridVelocity(QuadTree *node)
{
	return GetGridVelocity(node->gridid);
}

Vector2 FluidSimulator::GetGridVelocity(int gridid)
{
	Vector2 AvgVelocity(0,0);
	int agents_num = m_hmcrowd->m_grids[gridid].people_in_grid.size();
	for(int i=0;i<agents_num;i++)
	{
		int agentid = m_hmcrowd->m_grids[gridid].people_in_grid[i];			
		AvgVelocity += m_hmcrowd->m_agents[agentid].prefVelocity_;
	}
	AvgVelocity /= agents_num;
	return AvgVelocity;
}

void FluidSimulator::doStep()
{
	for(int crowdi = 0; crowdi<m_hmcrowd->m_fluidgroups.size(); crowdi++)
	{
		for(int gridi = 0; gridi < m_hmcrowd->m_fluidgroups[crowdi].crowd_grid.size(); gridi++)
		{
			Grid* cur_grid = m_hmcrowd->m_fluidgroups[crowdi].crowd_grid[gridi];
			int gridid = cur_grid->grid_id;
			cur_grid->avgVelocity = GetGridVelocity(gridid);
			for(int i=0;i<cur_grid->people_in_grid.size();i++)
			{
				int agent_id = cur_grid->people_in_grid[i];			
				m_hmcrowd->m_agents[agent_id].velocity_ = m_hmcrowd->m_agents[agent_id].prefVelocity_ + cur_grid->now_density*(cur_grid->avgVelocity-m_hmcrowd->m_agents[agent_id].prefVelocity_);
			}
			cur_grid->isContour = false;
			//cur_grid->obstacle.clear();
		}
		m_hmcrowd->m_fluidgroups[crowdi].TrackContour();
		for(int gridi = 0; gridi < m_hmcrowd->m_fluidgroups[crowdi].contour_grid.size(); gridi++)
		{
			Grid* cur_grid = m_hmcrowd->m_fluidgroups[crowdi].contour_grid[gridi];
			for(int i=0;i<cur_grid->people_in_grid.size();i++)
			{
				int agent_id = cur_grid->people_in_grid[i];			
				m_hmcrowd->m_agents[agent_id].is_contour = true;
				m_hmcrowd->m_agents[agent_id].active = 3;
			}
		}
		break;
	}
	rearrange();
}

int compare(const void* a, const void* b)
{
	return ( ((elem_grid*)b)->score - ((elem_grid*)a)->score  );
}

void FluidSimulator::rearrange()
{
	float elem_size = m_hmcrowd->m_maxAgentRadius*2;
	int grid_area = m_hmcrowd->m_cellsize*m_hmcrowd->m_cellsize;
	elem_grid* elem_grids = (elem_grid*)malloc(grid_area*sizeof(elem_grid));
	for(int crowdi = 0; crowdi<m_hmcrowd->m_fluidgroups.size(); crowdi++)
	{
		for(int gridi = 0; gridi < m_hmcrowd->m_fluidgroups[crowdi].crowd_grid.size(); gridi++)
		{
			Grid* cur_grid = m_hmcrowd->m_fluidgroups[crowdi].crowd_grid[gridi];
			int people_num = cur_grid->people_in_grid.size();
			elem_grid* agent_grids = (elem_grid*)malloc(people_num*sizeof(elem_grid));
			//先对格子中的每个单位区域和人都计算一下在速度方向上的投影，然后根据投影结果进行排序
			/***********************************************************************************************
			*     重排人群这里想改一下算法，之前那个直接投影太生硬了，我们可以算一次平均速度，再算一次方差，
			*     方差越大，代表大家越不齐心协力，阵型面积就散的越大；方差越小，代表所有人的目标是相同的，
			*     阵型的面积也就越小。阵型的形状可以用高斯函数2*pi*exp((x-avg)^2/sigma)，不过高斯的计算量大，
			*     可以改成cos(lamda*x)函数，通过修改lamda来实现面积的变化；如果需要继续提高计算速度，可以直接
			*     用等腰三角形，不过这样效果不会很好，此处需要注意的是高斯函数中sigma或者cos函数中lamda的取值
			*     都是从一个常数到无穷大的（无穷大的时候会站成直线），所以方差不能直接转化为sigma，可能需要找
			*     一个公式趋近一下，可能e的n次方比较符合这个要求，不过需要加一些常数处理一下，这些常数需要具体
			*     实现的时候试一下
			*     新的流程应该如下：
			*     1. 我们应该先预处理一下，给速度的方向大概定16个区间（把360度分成16份），这样做的话只要预处理
			*        一次就能把所有的格子的情况都给定，
			*     2. 根据格子的大小，即每个格子里能有多少个小格子，来顶一下高斯函数或者cos函数的几个基本取值，
			*        因为高斯函数和cos函数是连续的，但格子是离散的，所以存在固定个数的高斯函数或者cos函数的定
			*        义域分段范围（即从X.XX~X.XXX范围内，函数在格子中离散化后的面积一样），定好这个模式后，可以
			*        都算出来，做一个查找表，到时候根据方差和速度就能锁定高斯函数或者cos函数的朝向和面积。
			*        在后期可以考虑加上每个速度方向上的起始位置，来解决人不是很满的时候如何不让人突然蹦到格子
            *        最边缘的问题，初期的时候可以先减少格子的大小来解决这个问题。
			*     3. 在实际仿真过程中，先计算速度，在计算方差，再统计人数，这三项和重排列的结果有关，首先按照
		    *        方向选择函数的方向，再根据人数（即人最少占的面积，是函数面积的下限），得到一个最小的方差
			*        结果，如果方差大于当前的结果，选更大的面积函数，最大为正方形。如果小于当前结果，则还是用
			*        人数得到的函数  PS：这里的函数其实是一个离散的查找表，用来查找函数的中每个格子相对于速度
			*        轴方向上的位置
			*     4. 选好函数模型（查找表）之后，对人群进行排序，排序的判断标准是在速度方向上投影的大小，这样
			*        做的原因是要从人群的领头部位逐渐往后按顺序排下来才能保证Agent最后都能一个萝卜一个坑，然后
			*        每次选一个在队首的人，选择他现在的位置在函数模型中最近且没有被占用的地方安置好，完成重排
			************************************************************************************************/
			int agentInRow = this->m_hmcrowd->m_gridsize/m_hmcrowd->m_maxAgentRadius/2;
			float elem_grid_size = this->m_hmcrowd->m_gridsize/(1.0*agentInRow);
			for(int r = 0;r<agentInRow;r++) {
				for(int c=0;c<agentInRow;c++) {
					int id = r*agentInRow+c;
					elem_grids[id].id = id;
					elem_grids[id].center = Vector2(cur_grid->box.x()+c*elem_size , cur_grid->box.y()+r*elem_size);
					elem_grids[id].score = elem_grids[id].center.dot(cur_grid->avgVelocity);
				}
			}
			Vector2 mean_center(0,0);
			for(int i=0;i<people_num;i++)
			{
				agent_grids[i].id = i;
				int agent_id = cur_grid->people_in_grid[i];		
				agent_grids[i].center = Vector2(m_hmcrowd->m_agents[agent_id].npos[0] , m_hmcrowd->m_agents[agent_id].npos[2]);
				agent_grids[i].agent = &m_hmcrowd->m_agents[agent_id];
				//agent_grids[i].score = agent_grids[i].center.dot(cur_grid->avgVelocity);
				mean_center += agent_grids[i].center;
			}
			for(int i=0;i<people_num;i++)
			{
				agent_grids[i].score = (agent_grids[i].center-mean_center).dot(cur_grid->avgVelocity);
			}

			//排序
			qsort(elem_grids,grid_area,sizeof(elem_grid),compare);

			qsort(agent_grids,people_num,sizeof(elem_grid),compare);

			//重排人群
			std::vector<std::list<elem_grid>> agent_bucket;
			Vector2 vertVelocity(cur_grid->avgVelocity.y(),-cur_grid->avgVelocity.x());//顺时针旋转90度
			int count = -1;
			for(int i=0;i<people_num;i++)
			{
				if(i==0||agent_grids[0].score-agent_grids[i].score>(count+1)*elem_grid_size)
				{
					std::list<elem_grid> bucket;
					bucket.push_back(agent_grids[i]);
					count++;
					agent_bucket.push_back(bucket);
				}
				else
				{
					std::list<elem_grid>::iterator iter;
					for(iter=agent_bucket[count].begin();iter!=agent_bucket[count].end();iter++)
					{
						if(iter->center.dot(vertVelocity)>agent_grids[i].center.dot(vertVelocity))
						{
							agent_bucket[count].insert(iter,agent_grids[i]);
							break;
						}
					}
					if(iter==agent_bucket[count].end())
						agent_bucket[count].push_back(agent_grids[i]);//此处需要插入排序
				}
				//agent_grids[i]

			}
			
			for(int i=0;i<((int)agent_bucket.size());i++)
			{
				//同层
				std::list<elem_grid>::iterator iter,nextiter;
				for(iter=agent_bucket[i].begin();iter!=agent_bucket[i].end();iter++)
				{
					nextiter=iter;
					nextiter++;
					for(;nextiter!=agent_bucket[i].end();nextiter++)
					{
						if(iter->center.getDistance(nextiter->center)<elem_grid_size)
						{
							Vector2 dir = normalize(nextiter->center-iter->center);
							nextiter->center = iter->center+dir*elem_grid_size;
							nextiter->agent->npos[0] = nextiter->center.x();
							nextiter->agent->npos[2] = nextiter->center.y();
						}
					}
					if(i+1<agent_bucket.size())
					{
						for(nextiter=agent_bucket[i+1].begin();nextiter!=agent_bucket[i+1].end();nextiter++)
						{
							if(iter->center.getDistance(nextiter->center)<elem_grid_size)
							{
								Vector2 dir = normalize(nextiter->center-iter->center);
								nextiter->center = iter->center+dir*elem_grid_size;
								nextiter->agent->npos[0] = nextiter->center.x();
								nextiter->agent->npos[2] = nextiter->center.y();
							}	
						}
					}
				}
			}

			//将格子中的外围人群添加为障碍物
			if(cur_grid->isContour==true)
			{
				std::list<elem_grid>::iterator iter;
				std::vector<Vector2> obstacle;
				for(iter=agent_bucket[0].begin();iter!=agent_bucket[0].end();iter++)//上
				{
					obstacle.push_back(iter->center);
				}
				for(int i=1;i<((int)agent_bucket.size())-1;i++)	//右
				{
					obstacle.push_back(agent_bucket[i].rbegin()->center);
				}
				
				if(agent_bucket.size()>1)	//下
				{
					std::list<elem_grid>::reverse_iterator riter;
					int i=((int)agent_bucket.size())-1;
					for(riter=agent_bucket[i].rbegin();riter!=agent_bucket[i].rend();riter++)
					{
						obstacle.push_back(riter->center);
					}
				}
				for(int i=((int)agent_bucket.size())-2;i>0;i--)	//左
				{
					obstacle.push_back(agent_bucket[i].begin()->center);
				}
				this->m_hmcrowd->m_rvosim->addObstacle(obstacle);
			}
			free(agent_grids);
		}
	}
	free(elem_grids);
}

/*缺少更新格子和人群对应关系的步骤
int gridid = Grid::GetGridId(People::people_array[peopleid].position_);
		
				if(gridid!=People::people_array[peopleid].grid_now_id)
				{
					Grid::grid_array[gridid].people_in_grid.push_back(People::people_array[peopleid].people_id);
					vector<int>::iterator gridpeopleit = Grid::grid_array[People::people_array[peopleid].grid_now_id].people_in_grid.begin();
					for(;!Grid::grid_array[People::people_array[peopleid].grid_now_id].people_in_grid.empty()&&gridpeopleit!=Grid::grid_array[People::people_array[peopleid].grid_now_id].people_in_grid.end() && *gridpeopleit !=People::people_array[peopleid].people_id ;gridpeopleit++);
					if(*gridpeopleit ==People::people_array[peopleid].people_id)
					{
						Grid::grid_array[People::people_array[peopleid].grid_now_id].people_in_grid.erase(gridpeopleit);
					}
					People::people_array[peopleid].grid_former_id = People::people_array[peopleid].grid_now_id;
					People::people_array[peopleid].grid_now_id = gridid;
				}*/

void FluidSimulator::generateTemplate()
{
	/*
		1. 先求出一个基，该基为格子的两端为
		2. 二维平移、二维缩放和旋转	
	*/
	crowdTemplate ct_basis;
	ct_basis.xtrans = 0;
	ct_basis.ytrans = 0;
	int agentInRow = this->m_hmcrowd->m_gridsize/m_hmcrowd->m_maxAgentRadius/2;
	ct_basis.peak = agentInRow;	//这里只是格子的大小，而不是和人大小的比例关系
	ct_basis.cycle = PI/(agentInRow-1);
	float elem_grid_size = this->m_hmcrowd->m_gridsize/(1.0*agentInRow);

	//base template assignment
	for(int r=0;r<agentInRow;r++)
	{
		for(int c=0;c<agentInRow;c++)
		{
			int y_ = agentInRow-1-r;
			int y = ct_basis.peak*sin(ct_basis.cycle*c+ct_basis.xtrans)+ct_basis.ytrans;
			if(y_<=y)	//在模板内的点
			{
				elem_grid eg;
				eg.center.SetX(elem_grid_size*c+elem_grid_size/2);
				eg.center.SetY(elem_grid_size*r+elem_grid_size/2);
				eg.id = r*agentInRow+c;
				eg.score = 0;
				ct_basis.elem_grids.push_back(eg);
			}
		}
	}

	for(int r=0;r<agentInRow;r++) //r为y轴的循环
	{
		for(int c=0;c<agentInRow;c++) //c为x轴的循环
		{
			for(int cyclei=agentInRow-1;cyclei>0;cyclei--)	//周期变短
			{
				for(int peaki=agentInRow;peaki>0;peaki--)	//波峰变小
				{
					for(int i=0;i<8;i++)	//八个朝向	
					{
						crowdTemplate ct;
						ct.xtrans = c;
						ct.ytrans = r;
						ct.peak = peaki;	//这里只是格子的大小，而不是和人大小的比例关系
						ct.cycle = PI/cyclei;

						//base template assignment
						for(int r=0;r<agentInRow;r++)
						{
							for(int c=0;c<agentInRow;c++)
							{
								int y_ = agentInRow-1-r;
								int y = ct_basis.peak*sin(ct_basis.cycle*c+ct_basis.xtrans)+ct_basis.ytrans;
								if(y_<=y)	//在模板内的点
								{
									elem_grid eg;
									eg.center.SetX(elem_grid_size*c+elem_grid_size/2);
									eg.center.SetY(elem_grid_size*r+elem_grid_size/2);
									eg.id = r*agentInRow+c;
									eg.score = 0;
									ct.elem_grids.push_back(eg);
								}
							}
						}
					}
				}
			}
		}
	}
	
}