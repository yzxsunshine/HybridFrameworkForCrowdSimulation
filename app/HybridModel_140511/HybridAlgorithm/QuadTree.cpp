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
	{	//最小层，需要对gridid进行赋值
		gridid = hmcrowd->m_defaultGrid->GetGridId(rect.center_x(),rect.center_y());
		hmcrowd->m_grids[gridid].tpnode = this;
		grid = &hmcrowd->m_grids[gridid];
		//这里少了格子处理的过程，即每个格子如何从图像区域中提取出人群个数，并计算密度。
		//需要图像（全局变量），传进去rect变量，在图像的rect区域搜索，找到二值图像或灰度图像中值为人的像素，统计人的个数，并计算出密度
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
	
	//root节点的layernum为0，leaf_direction为-1，parent为NULL
	CRectangle<double> NWRect(rect.x(),rect.y(),rect.w()/2,rect.h()/2);
	node[0] = new QuadTree(hmc,layernum+1,0,this,NWRect);

	CRectangle<double> NERect(rect.x()+rect.w()/2,rect.y(),rect.w()/2,rect.h()/2);
	node[1] = new QuadTree(hmc,layernum+1,1,this,NERect);

	CRectangle<double> SERect(rect.x()+rect.w()/2,rect.y()+rect.h()/2,rect.w()/2,rect.h()/2);
	node[2] = new QuadTree(hmc,layernum+1,2,this,SERect);

	CRectangle<double> SWRect(rect.x(),rect.y()+rect.h()/2,rect.w()/2,rect.h()/2);
	node[3] = new QuadTree(hmc,layernum+1,3,this,SWRect);

	int stateflag = 0;	//标记子节点中黑格子的个数
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
void QuadTree::MergeQuadTree()//写了好多都被删掉了！！！//写了好多又被删掉啦！！！
{
	QuadTree *temp = this;
	queue<QuadTree*> QueueQuad;
	vector<QuadTree*> MarkedVector;
	map<long,long> SameMarkPair;
	map<long,long>::iterator SameMarkPairItr;
	long markid = 0;
	hmcrowd->m_fluidgroups.clear();	//清空群体数组
	QueueQuad.push(this);
	hmcrowd->m_rvoagents.clear();
	while(QueueQuad.size()!=0)	//遍历整个树，对每个黑色的最大块赋一个markid
	{
		temp = QueueQuad.front();
		for(int i=0;i<4;i++)
		{
			if(temp->node[i]==NULL)//此处为灰色的叶子节点，可以在此处把人加入到RVO的vector里，（先在people里建立新的一个vector，用来存用RVO仿真的人，然后根据格子里每个人的id将people的指针放到该vector中更新）
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
			QueueQuad.push(temp->node[i]);	//灰色入队列
			
		}
		QueueQuad.pop();
	}
	//MarkedVector.reserve();//这里反向是为了用起来方便，但如果为了效率，其实可以直接用总大小减去i代替i=0
	//合并不同的区域，这里的想法是，将所有的区域先视为不连通，然后通过逐渐修改邻居的markid然后将连通的区域id统一
	for(int i=MarkedVector.size()-1;i>=0;i--)
	{
		for(int j=0;j<4;j++)//遍历四个邻居
		{
			if(MarkedVector[i]->neighbor[j]==NULL)
				continue;
			if(MarkedVector[i]->neighbor[j]->leaf_state==BLACK)
			{
				if(MarkedVector[i]->neighbor[j]->RegionMark<0)	//其父节点或更大的块有markid，并没有对该层进行设定
				{
					QuadTree* tempneighbor = MarkedVector[i]->neighbor[j];
					while(tempneighbor->RegionMark<0)
					{
						tempneighbor = tempneighbor->parent;
						//assert(tempneighbor!=NULL);//如果此处为空，说明树的建立或者是mark的赋值有问题，才会导致从根节点开始所有节点都没有标记
					}
					if(tempneighbor->RegionMark <= MarkedVector[i]->RegionMark)
						tempneighbor->RegionMark = MarkedVector[i]->RegionMark;	//同化mark
					else
					{

						bool existflag = false;
						bool existsecondflag = false;
						SameMarkPairItr = SameMarkPair.find(tempneighbor->RegionMark);	//由于邻居的一定比目前的大，否则在上面已经被同化了，所以找找比邻居更大的相邻标记
						if(SameMarkPairItr!=SameMarkPair.end())//邻居已经存在于同义对中,有比邻居更大的
						{
							//新的对变成 MarkedVector[i]->RegionMark,SameMarkPairItr->second
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
									SameMarkPair.erase(itr);//类似三角形保持最大编号的两条边
									SameMarkPair.insert(make_pair(MarkedVector[i]->RegionMark,SameMarkPairItr->second));
									SameMarkPair.insert(make_pair(itr_second,SameMarkPairItr->second));
								}
							}
							else
							{
								SameMarkPair.insert(make_pair(MarkedVector[i]->RegionMark,SameMarkPairItr->second));
							}
						}
						else//没有比邻居更大的
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
						//这个地方用find改一下
						bool existflag = false;
						bool existsecondflag = false;
						SameMarkPairItr = SameMarkPair.find(MarkedVector[i]->neighbor[j]->RegionMark);	//由于邻居的一定比目前的大，否则在上面已经被同化了，所以找找比邻居更大的相邻标记
						if(SameMarkPairItr!=SameMarkPair.end())//邻居已经存在于同义对中,有比邻居更大的
						{
							//新的对变成 MarkedVector[i]->RegionMark,SameMarkPairItr->second
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
									SameMarkPair.erase(itr);//类似三角形保持最大编号的两条边
									SameMarkPair.insert(make_pair(MarkedVector[i]->RegionMark,SameMarkPairItr->second));
									SameMarkPair.insert(make_pair(itr_second,SameMarkPairItr->second));
								}
							}
							else
							{
								SameMarkPair.insert(make_pair(MarkedVector[i]->RegionMark,SameMarkPairItr->second));
							}
						}
						else//没有比邻居更大的
						{
							SameMarkPair.insert(make_pair(MarkedVector[i]->RegionMark,MarkedVector[i]->neighbor[j]->RegionMark));
						}
						
					}
				}
			}
		}
	}

	//处理同义区域，将其合并到同一个crowdgroup的vector里
	for(int i=MarkedVector.size()-1;i>=0;i--)
	{
		if(i==MarkedVector.size()-1)//第一个处理，需要新建一个group，初始化一个regionid，以便于和之后做对比
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
	//每层要先根据自己父亲的邻居算自己的邻居，然后在递归调用自己子节点的InitNeighbor帮助自己的子节点找邻居。
	if(this->parent == NULL)
	{
		//根节点没有邻居，直接计算子节点的邻居。
	}
	else
	{
		switch(this->leaf_direction)
		{
		case 0:
			//下方和右方的节点是自己父节点的子节点
			this->neighbor[1] = this->parent->node[3];
			this->neighbor[3] = this->parent->node[1];
			//上方和左方是伯父节点的子节点
			if(this->parent->neighbor[0] == NULL)
			{	//根节点的子节点
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
			//下方和左方的节点是自己父节点的子节点，上方和右方是伯父节点的子节点
			this->neighbor[1] = this->parent->node[2];
			this->neighbor[2] = this->parent->node[0];
			//上方和右方是伯父节点的子节点
			if(this->parent->neighbor[0] == NULL)
			{	//根节点的子节点
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
			//上方和左方的节点是自己父节点的子节点，下方和右方是伯父节点的子节点
			this->neighbor[0] = this->parent->node[1];
			this->neighbor[2] = this->parent->node[3];
			//下方和右方是伯父节点的子节点
			if(this->parent->neighbor[1] == NULL)
			{	//根节点的子节点
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
			//上方和右方的节点是自己父节点的子节点，下方和左方是伯父节点的子节点
			this->neighbor[0] = this->parent->node[0];
			this->neighbor[3] = this->parent->node[2];
			//下方和左方是伯父节点的子节点
			if(this->parent->neighbor[1] == NULL)
			{	//根节点的子节点
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
	//计算子节点邻居
	if(this->node[0]!=NULL)
	{
		//当不是叶子节点时，递归调用其子节点的计算邻居函数
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