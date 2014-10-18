#include "Group.h"


Group::Group()
{
}


Group::~Group()
{
}

void Group::UpdateDensity(float threshold, float agentSize)
{
	formerDensity = nowDensity;
	nowDensity = GetAgentNum() * agentSize / m_area;
	if (nowDensity > threshold)
	{
		m_densityStatus = BLACK;
	}
	else if (nowDensity > 0)
	{
		m_densityStatus = GRAY;
	}
	else
	{
		m_densityStatus = WHITE;
	}
}

bool Group::GetInterpolationNeighbors(Vector2 pos, std::vector<Group>* grids, std::vector<int>& nids
	, Vector2& leftBottom, Vector2& rightTop)
{
	nids.clear();
	if (pos.x() > m_box.center_x() && pos.y() > m_box.center_y())	// first quadrant
	{
		int cornerGridID;
		if (m_neighbor[1] >= 0)
			cornerGridID = (*grids)[m_neighbor[1]].m_neighbor[3];
		else if (m_neighbor[3] >= 0)
			cornerGridID = (*grids)[m_neighbor[3]].m_neighbor[1];
		else
			return false;
		nids.push_back(groupID);
		nids.push_back(m_neighbor[3]);
		nids.push_back(cornerGridID);
		nids.push_back(m_neighbor[1]);
		leftBottom.SetX(m_box.center_x());
		leftBottom.SetY(m_box.center_y());
		rightTop.SetX(m_box.center_x() + m_box.w());
		rightTop.SetY(m_box.center_y() + m_box.h());
	}
	else if (pos.x() > m_box.center_x() && pos.y() < m_box.center_y())	// second quadrant
	{
		int cornerGridID;
		if (m_neighbor[0] >= 0)
			cornerGridID = (*grids)[m_neighbor[0]].m_neighbor[3];
		else if (m_neighbor[3] >= 0)
			cornerGridID = (*grids)[m_neighbor[3]].m_neighbor[0];
		else
			return false;
		nids.push_back(m_neighbor[0]);
		nids.push_back(cornerGridID);
		nids.push_back(m_neighbor[3]);
		nids.push_back(groupID);
		leftBottom.SetX(m_box.center_x());
		leftBottom.SetY(m_box.center_y() - m_box.h());
		rightTop.SetX(m_box.center_x() + m_box.w());
		rightTop.SetY(m_box.center_y());
	}
	else if (pos.x() < m_box.center_x() && pos.y() < m_box.center_y())	// first quadrant
	{
		int cornerGridID;
		if (m_neighbor[0] >= 0)
			cornerGridID = (*grids)[m_neighbor[0]].m_neighbor[2];
		else if (m_neighbor[2] >= 0)
			cornerGridID = (*grids)[m_neighbor[2]].m_neighbor[0];
		else
			return false;
		nids.push_back(cornerGridID);
		nids.push_back(m_neighbor[0]);
		nids.push_back(groupID);
		nids.push_back(m_neighbor[2]);
		leftBottom.SetX(m_box.center_x() - m_box.w());
		leftBottom.SetY(m_box.center_y() - m_box.h());
		rightTop.SetX(m_box.center_x());
		rightTop.SetY(m_box.center_y());
	}
	else
	{
		int cornerGridID;
		if (m_neighbor[1] >= 0)
			cornerGridID = (*grids)[m_neighbor[1]].m_neighbor[2];
		else if (m_neighbor[2] >= 0)
			cornerGridID = (*grids)[m_neighbor[2]].m_neighbor[1];
		else
			return false;
		nids.push_back(m_neighbor[2]);
		nids.push_back(groupID);
		nids.push_back(m_neighbor[1]);
		nids.push_back(cornerGridID);
		leftBottom.SetX(m_box.center_x() - m_box.w());
		leftBottom.SetY(m_box.center_y());
		rightTop.SetX(m_box.center_x());
		rightTop.SetY(m_box.center_y() + m_box.h());
	}
}