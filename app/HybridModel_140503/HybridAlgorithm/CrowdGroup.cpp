#include "CrowdGroup.h"
#include <map>

CrowdGroup::CrowdGroup(void):RegionID(-1)
{
}

CrowdGroup::~CrowdGroup(void)
{
	
}
void CrowdGroup::AddGrid(QuadTree* p)
{
	if(p->node[0]==NULL)
	{
		this->crowd_grid.push_back(p->grid);
	}
	else
	{
		for(int i=0;i<4;i++)
		{
			AddGrid(p->node[i]);	//此处用了递归，如果后面效率有问题的话，需要改成循环实现
		}
	}
}

void CrowdGroup::TrackContour()
{
	for(int i=0;i<crowd_grid.size();i++)
	{
		int flag = 0;
		for(int j=0;j<4;j++)
		{
			if(crowd_grid[i]->tpnode->neighbor[j]==0)
				continue;
			if(crowd_grid[i]->tpnode->neighbor[j]->leaf_state == BLACK)
				flag++;
			else 
			{
				break;
			}

		}
		if(flag < 4)
		{
			contour_grid.push_back(crowd_grid[i]);
			crowd_grid[i]->isContour = true;
		}
	}
}

