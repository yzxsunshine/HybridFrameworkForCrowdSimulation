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
			//�ȶԸ����е�ÿ����λ������˶�����һ�����ٶȷ����ϵ�ͶӰ��Ȼ�����ͶӰ�����������
			/***********************************************************************************************
			*     ������Ⱥ�������һ���㷨��֮ǰ�Ǹ�ֱ��ͶӰ̫��Ӳ�ˣ����ǿ�����һ��ƽ���ٶȣ�����һ�η��
			*     ����Խ�󣬴�����Խ������Э�������������ɢ��Խ�󣻷���ԽС�����������˵�Ŀ������ͬ�ģ�
			*     ���͵����Ҳ��ԽС�����͵���״�����ø�˹����2*pi*exp((x-avg)^2/sigma)��������˹�ļ�������
			*     ���Ըĳ�cos(lamda*x)������ͨ���޸�lamda��ʵ������ı仯�������Ҫ������߼����ٶȣ�����ֱ��
			*     �õ��������Σ���������Ч������ܺã��˴���Ҫע����Ǹ�˹������sigma����cos������lamda��ȡֵ
			*     ���Ǵ�һ�������������ģ�������ʱ���վ��ֱ�ߣ������Է����ֱ��ת��Ϊsigma��������Ҫ��
			*     һ����ʽ����һ�£�����e��n�η��ȽϷ������Ҫ�󣬲�����Ҫ��һЩ��������һ�£���Щ������Ҫ����
			*     ʵ�ֵ�ʱ����һ��
			*     �µ�����Ӧ�����£�
			*     1. ����Ӧ����Ԥ����һ�£����ٶȵķ����Ŷ�16�����䣨��360�ȷֳ�16�ݣ����������Ļ�ֻҪԤ����
			*        һ�ξ��ܰ����еĸ��ӵ������������
			*     2. ���ݸ��ӵĴ�С����ÿ�����������ж��ٸ�С���ӣ�����һ�¸�˹��������cos�����ļ�������ȡֵ��
			*        ��Ϊ��˹������cos�����������ģ�����������ɢ�ģ����Դ��ڹ̶������ĸ�˹��������cos�����Ķ�
			*        ����ֶη�Χ������X.XX~X.XXX��Χ�ڣ������ڸ�������ɢ��������һ�������������ģʽ�󣬿���
			*        �����������һ�����ұ���ʱ����ݷ�����ٶȾ���������˹��������cos�����ĳ���������
			*        �ں��ڿ��Կ��Ǽ���ÿ���ٶȷ����ϵ���ʼλ�ã�������˲��Ǻ�����ʱ����β�����ͻȻ�ĵ�����
            *        ���Ե�����⣬���ڵ�ʱ������ȼ��ٸ��ӵĴ�С�����������⡣
			*     3. ��ʵ�ʷ�������У��ȼ����ٶȣ��ڼ��㷽���ͳ��������������������еĽ���йأ����Ȱ���
		    *        ����ѡ�����ķ����ٸ�����������������ռ��������Ǻ�����������ޣ����õ�һ����С�ķ���
			*        ��������������ڵ�ǰ�Ľ����ѡ�����������������Ϊ�����Ρ����С�ڵ�ǰ�����������
			*        �����õ��ĺ���  PS������ĺ�����ʵ��һ����ɢ�Ĳ��ұ��������Һ�������ÿ������������ٶ�
			*        �᷽���ϵ�λ��
			*     4. ѡ�ú���ģ�ͣ����ұ�֮�󣬶���Ⱥ��������������жϱ�׼�����ٶȷ�����ͶӰ�Ĵ�С������
			*        ����ԭ����Ҫ����Ⱥ����ͷ��λ������˳�����������ܱ�֤Agent�����һ���ܲ�һ���ӣ�Ȼ��
			*        ÿ��ѡһ���ڶ��׵��ˣ�ѡ�������ڵ�λ���ں���ģ���������û�б�ռ�õĵط����úã��������
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

			//����
			qsort(elem_grids,grid_area,sizeof(elem_grid),compare);

			qsort(agent_grids,people_num,sizeof(elem_grid),compare);

			//������Ⱥ
			std::vector<std::list<elem_grid>> agent_bucket;
			Vector2 vertVelocity(cur_grid->avgVelocity.y(),-cur_grid->avgVelocity.x());//˳ʱ����ת90��
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
						agent_bucket[count].push_back(agent_grids[i]);//�˴���Ҫ��������
				}
				//agent_grids[i]

			}
			
			for(int i=0;i<((int)agent_bucket.size());i++)
			{
				//ͬ��
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

			//�������е���Χ��Ⱥ���Ϊ�ϰ���
			if(cur_grid->isContour==true)
			{
				std::list<elem_grid>::iterator iter;
				std::vector<Vector2> obstacle;
				for(iter=agent_bucket[0].begin();iter!=agent_bucket[0].end();iter++)//��
				{
					obstacle.push_back(iter->center);
				}
				for(int i=1;i<((int)agent_bucket.size())-1;i++)	//��
				{
					obstacle.push_back(agent_bucket[i].rbegin()->center);
				}
				
				if(agent_bucket.size()>1)	//��
				{
					std::list<elem_grid>::reverse_iterator riter;
					int i=((int)agent_bucket.size())-1;
					for(riter=agent_bucket[i].rbegin();riter!=agent_bucket[i].rend();riter++)
					{
						obstacle.push_back(riter->center);
					}
				}
				for(int i=((int)agent_bucket.size())-2;i>0;i--)	//��
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

/*ȱ�ٸ��¸��Ӻ���Ⱥ��Ӧ��ϵ�Ĳ���
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
		1. �����һ�������û�Ϊ���ӵ�����Ϊ
		2. ��άƽ�ơ���ά���ź���ת	
	*/
	crowdTemplate ct_basis;
	ct_basis.xtrans = 0;
	ct_basis.ytrans = 0;
	int agentInRow = this->m_hmcrowd->m_gridsize/m_hmcrowd->m_maxAgentRadius/2;
	ct_basis.peak = agentInRow;	//����ֻ�Ǹ��ӵĴ�С�������Ǻ��˴�С�ı�����ϵ
	ct_basis.cycle = PI/(agentInRow-1);
	float elem_grid_size = this->m_hmcrowd->m_gridsize/(1.0*agentInRow);

	//base template assignment
	for(int r=0;r<agentInRow;r++)
	{
		for(int c=0;c<agentInRow;c++)
		{
			int y_ = agentInRow-1-r;
			int y = ct_basis.peak*sin(ct_basis.cycle*c+ct_basis.xtrans)+ct_basis.ytrans;
			if(y_<=y)	//��ģ���ڵĵ�
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

	for(int r=0;r<agentInRow;r++) //rΪy���ѭ��
	{
		for(int c=0;c<agentInRow;c++) //cΪx���ѭ��
		{
			for(int cyclei=agentInRow-1;cyclei>0;cyclei--)	//���ڱ��
			{
				for(int peaki=agentInRow;peaki>0;peaki--)	//�����С
				{
					for(int i=0;i<8;i++)	//�˸�����	
					{
						crowdTemplate ct;
						ct.xtrans = c;
						ct.ytrans = r;
						ct.peak = peaki;	//����ֻ�Ǹ��ӵĴ�С�������Ǻ��˴�С�ı�����ϵ
						ct.cycle = PI/cyclei;

						//base template assignment
						for(int r=0;r<agentInRow;r++)
						{
							for(int c=0;c<agentInRow;c++)
							{
								int y_ = agentInRow-1-r;
								int y = ct_basis.peak*sin(ct_basis.cycle*c+ct_basis.xtrans)+ct_basis.ytrans;
								if(y_<=y)	//��ģ���ڵĵ�
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