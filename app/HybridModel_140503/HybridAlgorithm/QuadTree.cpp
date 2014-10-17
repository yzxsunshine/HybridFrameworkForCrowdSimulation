#include "QuadTree.h"
#include "Grid.h"
#include "CrowdGroup.h"
#include "HmAgent.h"
#include "RVO/Agent.h"
#include "math.h"
#include <queue>
#include <map>
using namespace std;
int QuadTree::maxlayer = 6;
RVO::RVOSimulator* sim1;


QuadTree::QuadTree(HybridModelCrowd* hmc, int l, int ld, QuadTree *p, CRectangle<double> r):layernum(l),leaf_direction(ld),parent(p),rect(r),RegionMark(-1)
{
	hmcrowd = hmc;
	for(int i=0;i<4;i++)
	{
		this->neighbor[i] =NULL;
	}
	if(layernum==maxlayer||rect.getwidth()<=hmcrowd->m_gridsize||rect.getheight()<=hmcrowd->m_gridsize)
	{	//��С�㣬��Ҫ��gridid���и�ֵ
		gridid = hmcrowd->m_defaultGrid->GetGridId(rect.center_x(),rect.center_y());
		hmcrowd->m_grids[gridid].tpnode = this;
		grid = &hmcrowd->m_grids[gridid];
		//�������˸��Ӵ���Ĺ��̣���ÿ��������δ�ͼ����������ȡ����Ⱥ�������������ܶȡ�
		//��Ҫͼ��ȫ�ֱ�����������ȥrect��������ͼ���rect�����������ҵ���ֵͼ���Ҷ�ͼ����ֵΪ�˵����أ�ͳ���˵ĸ�������������ܶ�
		if(hmcrowd->m_grids[gridid].now_density <= 0)
		{
			leaf_state = WHITE;
			former_leaf_state = leaf_state;
		}
		else if(hmcrowd->m_grids[gridid].now_density >= hmcrowd->m_densityThreshold)
		{
			leaf_state = BLACK;
			former_leaf_state = leaf_state;
		}
		else
		{
			leaf_state = GRAY;
			former_leaf_state = leaf_state;
		}
		memset(node,0,4*sizeof(QuadTree*));
		return;
	}
	
	//root�ڵ��layernumΪ0��leaf_directionΪ-1��parentΪNULL
	CRectangle<double> NWRect(rect.x(),rect.y(),rect.w()/2,rect.h()/2);
	node[0] = new QuadTree(hmc,layernum+1,0,this,NWRect);

	CRectangle<double> NERect(rect.x()+rect.w()/2,rect.y(),rect.w()/2,rect.h()/2);
	node[1] = new QuadTree(hmc,layernum+1,1,this,NERect);

	CRectangle<double> SERect(rect.x()+rect.w()/2,rect.y()+rect.h()/2,rect.w()/2,rect.h()/2);
	node[2] = new QuadTree(hmc,layernum+1,2,this,SERect);

	CRectangle<double> SWRect(rect.x(),rect.y()+rect.h()/2,rect.w()/2,rect.h()/2);
	node[3] = new QuadTree(hmc,layernum+1,3,this,SWRect);

	int stateflag = 0;	//����ӽڵ��кڸ��ӵĸ���
	for(int i=0;i<4;i++)
	{
		if(node[i]->leaf_state==BLACK)
		{
			stateflag++;
		}
		else if(node[i]->leaf_state==GRAY)
		{
			stateflag = 5;
			break;
		}
	}
	if(stateflag==0)
	{
		leaf_state = WHITE;
	}
	else if(stateflag==4)
	{
		leaf_state = BLACK;
	}
	else
	{
		leaf_state = GRAY;
	}

}

QuadTree::~QuadTree(void)
{
	for(int i=0;i<4;i++)
	{
		delete node[i];
	}
}
void QuadTree::MergeQuadTree()//д�˺ö඼��ɾ���ˣ�����//д�˺ö��ֱ�ɾ����������
{
	QuadTree *temp = this;
	queue<QuadTree*> QueueQuad;
	vector<QuadTree*> MarkedVector;
	map<long,long> SameMarkPair;
	map<long,long>::iterator SameMarkPairItr;
	long markid = 0;
	hmcrowd->m_fluidgroups.clear();	//���Ⱥ������
	QueueQuad.push(this);
	hmcrowd->m_rvoagents.clear();
	while(QueueQuad.size()!=0)	//��������������ÿ����ɫ�����鸳һ��markid
	{
		temp = QueueQuad.front();
		for(int i=0;i<4;i++)
		{
			if(temp->node[i]==NULL)//�˴�Ϊ��ɫ��Ҷ�ӽڵ㣬�����ڴ˴����˼��뵽RVO��vector�������people�ｨ���µ�һ��vector����������RVO������ˣ�Ȼ����ݸ�����ÿ���˵�id��people��ָ��ŵ���vector�и��£�
			{
				if(temp->leaf_state == GRAY)
				{
				
					for(vector<int>::iterator it =hmcrowd->m_grids[temp->gridid].people_in_grid.begin();it!=hmcrowd->m_grids[temp->gridid].people_in_grid.end();it++)
					{
						//People::test_array.push_back(People::people_array[*it]);
						hmcrowd->m_agents[*it].active = 2;
						hmcrowd->m_rvosim->agents_.push_back(&hmcrowd->m_agents[*it]);
						
					}
				}
				break;

			}
			if(temp->node[i]->leaf_state==WHITE)
				continue;
			if(temp->node[i]->leaf_state==BLACK)
			{
				temp->node[i]->RegionMark = markid++;
				MarkedVector.push_back(temp->node[i]);
				continue;		
			}
			QueueQuad.push(temp->node[i]);	//��ɫ�����
			
		}
		QueueQuad.pop();
	}
	//MarkedVector.reserve();//���ﷴ����Ϊ�����������㣬�����Ϊ��Ч�ʣ���ʵ����ֱ�����ܴ�С��ȥi����i=0
	//�ϲ���ͬ������������뷨�ǣ������е���������Ϊ����ͨ��Ȼ��ͨ�����޸��ھӵ�markidȻ����ͨ������idͳһ
	for(int i=MarkedVector.size()-1;i>=0;i--)
	{
		for(int j=0;j<4;j++)//�����ĸ��ھ�
		{
			if(MarkedVector[i]->neighbor[j]==NULL)
				continue;
			if(MarkedVector[i]->neighbor[j]->leaf_state==BLACK)
			{
				if(MarkedVector[i]->neighbor[j]->RegionMark<0)	//�丸�ڵ�����Ŀ���markid����û�жԸò�����趨
				{
					QuadTree* tempneighbor = MarkedVector[i]->neighbor[j];
					while(tempneighbor->RegionMark<0)
					{
						tempneighbor = tempneighbor->parent;
						//assert(tempneighbor!=NULL);//����˴�Ϊ�գ�˵�����Ľ���������mark�ĸ�ֵ�����⣬�Żᵼ�´Ӹ��ڵ㿪ʼ���нڵ㶼û�б��
					}
					if(tempneighbor->RegionMark <= MarkedVector[i]->RegionMark)
						tempneighbor->RegionMark = MarkedVector[i]->RegionMark;	//ͬ��mark
					else
					{

						bool existflag = false;
						bool existsecondflag = false;
						SameMarkPairItr = SameMarkPair.find(tempneighbor->RegionMark);	//�����ھӵ�һ����Ŀǰ�Ĵ󣬷����������Ѿ���ͬ���ˣ��������ұ��ھӸ�������ڱ��
						if(SameMarkPairItr!=SameMarkPair.end())//�ھ��Ѿ�������ͬ�����,�б��ھӸ����
						{
							//�µĶԱ�� MarkedVector[i]->RegionMark,SameMarkPairItr->second
							map<long,long>::iterator itr = SameMarkPair.find(MarkedVector[i]->RegionMark);
							if(itr!=SameMarkPair.end())
							{
								if(itr->second == SameMarkPairItr->second)
								{
									existflag = true;
								}
								else
								{
									int itr_second = itr->second;
									SameMarkPair.erase(itr);//���������α�������ŵ�������
									SameMarkPair.insert(make_pair(MarkedVector[i]->RegionMark,SameMarkPairItr->second));
									SameMarkPair.insert(make_pair(itr_second,SameMarkPairItr->second));
								}
							}
							else
							{
								SameMarkPair.insert(make_pair(MarkedVector[i]->RegionMark,SameMarkPairItr->second));
							}
						}
						else//û�б��ھӸ����
						{
							SameMarkPair.insert(make_pair(MarkedVector[i]->RegionMark,tempneighbor->RegionMark));
						}
					}
				}
				else
				{
					if(MarkedVector[i]->neighbor[j]->RegionMark <= MarkedVector[i]->RegionMark)
						MarkedVector[i]->neighbor[j]->RegionMark = MarkedVector[i]->RegionMark;
					else
					{
						//����ط���find��һ��
						bool existflag = false;
						bool existsecondflag = false;
						SameMarkPairItr = SameMarkPair.find(MarkedVector[i]->neighbor[j]->RegionMark);	//�����ھӵ�һ����Ŀǰ�Ĵ󣬷����������Ѿ���ͬ���ˣ��������ұ��ھӸ�������ڱ��
						if(SameMarkPairItr!=SameMarkPair.end())//�ھ��Ѿ�������ͬ�����,�б��ھӸ����
						{
							//�µĶԱ�� MarkedVector[i]->RegionMark,SameMarkPairItr->second
							map<long,long>::iterator itr = SameMarkPair.find(MarkedVector[i]->RegionMark);
							if(itr!=SameMarkPair.end())
							{
								if(itr->second == SameMarkPairItr->second)
								{
									existflag = true;
								}
								else
								{
									int itr_second = itr->second;
									SameMarkPair.erase(itr);//���������α�������ŵ�������
									SameMarkPair.insert(make_pair(MarkedVector[i]->RegionMark,SameMarkPairItr->second));
									SameMarkPair.insert(make_pair(itr_second,SameMarkPairItr->second));
								}
							}
							else
							{
								SameMarkPair.insert(make_pair(MarkedVector[i]->RegionMark,SameMarkPairItr->second));
							}
						}
						else//û�б��ھӸ����
						{
							SameMarkPair.insert(make_pair(MarkedVector[i]->RegionMark,MarkedVector[i]->neighbor[j]->RegionMark));
						}
						
					}
				}
			}
		}
	}

	//����ͬ�����򣬽���ϲ���ͬһ��crowdgroup��vector��
	for(int i=MarkedVector.size()-1;i>=0;i--)
	{
		if(i==MarkedVector.size()-1)//��һ��������Ҫ�½�һ��group����ʼ��һ��regionid���Ա��ں�֮�����Ա�
		{
			CrowdGroup tempgroup;
			tempgroup.RegionID = MarkedVector[i]->RegionMark;
			tempgroup.AddGrid(MarkedVector[i]);
			hmcrowd->m_fluidgroups.push_back(tempgroup);
			continue;
		}
		bool isNewGroup = true;
		for(unsigned int j=0;j<hmcrowd->m_fluidgroups.size();j++)
		{
			if(MarkedVector[i]->RegionMark==hmcrowd->m_fluidgroups[j].RegionID)
			{
				hmcrowd->m_fluidgroups[j].AddGrid(MarkedVector[i]);
				isNewGroup = false;
				break;
			}
			SameMarkPairItr = SameMarkPair.find(MarkedVector[i]->RegionMark);
			if(SameMarkPairItr != SameMarkPair.end())
			{
				if(SameMarkPairItr->second==hmcrowd->m_fluidgroups[j].RegionID)
				{
					MarkedVector[i]->RegionMark = hmcrowd->m_fluidgroups[j].RegionID;
					hmcrowd->m_fluidgroups[j].AddGrid(MarkedVector[i]);
					isNewGroup = false;
					break;
				}
			}
		}
		if(isNewGroup==true)
		{
			CrowdGroup tempgroup;
			tempgroup.RegionID = MarkedVector[i]->RegionMark;
			tempgroup.AddGrid(MarkedVector[i]);
			hmcrowd->m_fluidgroups.push_back(tempgroup);
		}
	}


}

void QuadTree::InitNeighbor()
{
	//ÿ��Ҫ�ȸ����Լ����׵��ھ����Լ����ھӣ�Ȼ���ڵݹ�����Լ��ӽڵ��InitNeighbor�����Լ����ӽڵ����ھӡ�
	if(this->parent == NULL)
	{
		//���ڵ�û���ھӣ�ֱ�Ӽ����ӽڵ���ھӡ�
	}
	else
	{
		switch(this->leaf_direction)
		{
		case 0:
			//�·����ҷ��Ľڵ����Լ����ڵ���ӽڵ�
			this->neighbor[1] = this->parent->node[3];
			this->neighbor[3] = this->parent->node[1];
			//�Ϸ������ǲ����ڵ���ӽڵ�
			if(this->parent->neighbor[0] == NULL)
			{	//���ڵ���ӽڵ�
				this->neighbor[0] = NULL;
			}
			else
			{
				this->neighbor[0] = this->parent->neighbor[0]->node[3];
			}
			if(this->parent->neighbor[2] == NULL)
			{
				this->neighbor[2] = NULL;
			}
			else
			{
				this->neighbor[2] = this->parent->neighbor[2]->node[1];
			}
			break;
		case 1:
			//�·����󷽵Ľڵ����Լ����ڵ���ӽڵ㣬�Ϸ����ҷ��ǲ����ڵ���ӽڵ�
			this->neighbor[1] = this->parent->node[2];
			this->neighbor[2] = this->parent->node[0];
			//�Ϸ����ҷ��ǲ����ڵ���ӽڵ�
			if(this->parent->neighbor[0] == NULL)
			{	//���ڵ���ӽڵ�
				this->neighbor[0] = NULL;
			}
			else 
			{
				this->neighbor[0] = this->parent->neighbor[0]->node[2];
			}
			if(this->parent->neighbor[3] == NULL)
			{
				this->neighbor[3] = NULL;
			}
			else
			{
				this->neighbor[3] = this->parent->neighbor[3]->node[0];
			}
			break;
		case 2:
			//�Ϸ����󷽵Ľڵ����Լ����ڵ���ӽڵ㣬�·����ҷ��ǲ����ڵ���ӽڵ�
			this->neighbor[0] = this->parent->node[1];
			this->neighbor[2] = this->parent->node[3];
			//�·����ҷ��ǲ����ڵ���ӽڵ�
			if(this->parent->neighbor[1] == NULL)
			{	//���ڵ���ӽڵ�
				this->neighbor[1] = NULL;
			}
			else 
			{
				this->neighbor[1] = this->parent->neighbor[1]->node[1];
			}
			if(this->parent->neighbor[3] == NULL)
			{
				this->neighbor[3] = NULL;
			}
			else
			{
				this->neighbor[3] = this->parent->neighbor[3]->node[3];
			}
			break;
		case 3:
			//�Ϸ����ҷ��Ľڵ����Լ����ڵ���ӽڵ㣬�·������ǲ����ڵ���ӽڵ�
			this->neighbor[0] = this->parent->node[0];
			this->neighbor[3] = this->parent->node[2];
			//�·������ǲ����ڵ���ӽڵ�
			if(this->parent->neighbor[1] == NULL)
			{	//���ڵ���ӽڵ�
				this->neighbor[1] = NULL;
			}
			else 
			{
				this->neighbor[1] = this->parent->neighbor[1]->node[0];
			}
			if(this->parent->neighbor[2] == NULL)
			{
				this->neighbor[2] = NULL;
			}
			else
			{
				this->neighbor[2] = this->parent->neighbor[2]->node[2];
			}
			break;
		}
	}
	//�����ӽڵ��ھ�
	if(this->node[0]!=NULL)
	{
		//������Ҷ�ӽڵ�ʱ���ݹ�������ӽڵ�ļ����ھӺ���
		for(int i=0;i<4;i++)
		{
			this->node[i]->InitNeighbor();
		}
	}
}

QuadTreeState QuadTree::UpdateQuadTree()
{
	QuadTreeState tempstate;
	int count=0;
	this->RegionMark = -1;
	if(this->node[0] == NULL)
	{
		return this->leaf_state;	
	}
	for(int i=0;i<4;i++)
	{
		tempstate = this->node[i]->UpdateQuadTree();
		if(tempstate ==WHITE)
			count--;
		else if(tempstate == BLACK)
			count++;

	}
	if(count==-4)
		this->leaf_state = WHITE;
	else if(count ==4)
		this->leaf_state = BLACK;
	else
		this->leaf_state = GRAY;
	return this->leaf_state;
}