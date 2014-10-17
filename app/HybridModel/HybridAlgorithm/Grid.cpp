#include "Grid.h"
#include "../CommonDef.h"

Grid::Grid(void)
{
	//avaliable_region_rate = 1;
	m_obstacleStatus = NO_OBSTACLE;
	openID = -1;
	isClose = false;
}

Grid::~Grid(void)
{
}

bool Grid::GetNearestFace(vcg::Point3f& pos, int& faceID, NavMesh* nav, std::vector<Grid>& grids, vcg::Point3f& nearestPos)
{
	float p[3], nearest[3];
	p[0] = pos.X();	p[1] = pos.Y();	p[2] = pos.Z();
	bool res = GetNearestFace(p, faceID, nav, grids, nearest);
	nearestPos = vcg::Point3f(nearest[0], nearest[1], nearest[2]);
	return res;
}


bool Grid::GetNearestFace(const float* pos, int& faceID, NavMesh* nav, std::vector<Grid>& grids, float* nearestPos)
{
	vcg::Point2f a, b, p;
	p.X() = pos[0];
	p.Y() = pos[2];
	for (int j = 0; j<(int)m_faces.size(); j++)
	{
		int fid = m_faces[j];
		int sideCount = 0;
		for(int k=0; k<3; k++)
		{
			int aid = k;
			int bid = (k+1)%3;
			a.X() = nav->face[fid].V(aid)->P().X();
			a.Y() = nav->face[fid].V(aid)->P().Z();
			b.X() = nav->face[fid].V(bid)->P().X();
			b.Y() = nav->face[fid].V(bid)->P().Z();
			int status = GetPointSide(a, b, p);
			if(status == RIGHT_SIDE || status == ON_LINE)
			{
				sideCount++;
			}
		}
		if(sideCount==3)
		{
			faceID = fid;
			float height;
			vcg::Point3f posVec(pos[0], pos[1], pos[2]);
			nearestPos[0] = pos[0];
			nearestPos[2] = pos[2];
			if(ClosestHeightPointTriangle(posVec, nav->face[fid].V(0)->P(), nav->face[fid].V(1)->P(), nav->face[fid].V(2)->P(), height))
			{
				nearestPos[1] = height;
			}
			else
			{
				nearestPos[1] = pos[1];
			}
			return true;
		}
	}
	return false;
}

bool Grid::GetEdgePoint(vcg::Point3f startPt, vcg::Point3f endPt
					, vcg::Point3f& entryPt, vcg::Point3f& exitPt
					, int& entryFace, int& exitFace
					, NavMesh* nav, std::vector<Grid>& grids)
{
	vcg::Point2f ptA, ptB;
	vcg::Point2f lineA, lineB;
	vcg::Point2f intersect;
	// box edges
	vcg::Point2f edges[4][2];
	edges[0][0].X() = m_box.x();	// up
	edges[0][0].Y() = m_box.y();
	edges[0][1].X() = m_box.x() + m_box.w();
	edges[0][1].Y() = m_box.y();

	edges[1][0].X() = m_box.x();	//down
	edges[1][0].Y() = m_box.y() + m_box.h();
	edges[1][1].X() = m_box.x() + m_box.w();
	edges[1][1].Y() = m_box.y() + m_box.h();

	edges[2][0].X() = m_box.x();	//left
	edges[2][0].Y() = m_box.y();
	edges[2][1].X() = m_box.x();
	edges[2][1].Y() = m_box.y() + m_box.h();

	edges[3][0].X() = m_box.x() + m_box.w();	//right
	edges[3][0].Y() = m_box.y();
	edges[3][1].X() = m_box.x() + m_box.w();
	edges[3][1].Y() = m_box.y() + m_box.h();
	
	// entry
	ptA.X() = startPt.X();
	ptA.Y() = startPt.Z();
	ptB.X() = center.X();
	ptB.Y() = center.Z();
	vcg::Point3f pt;
	bool foundIntersect = false;
	for(unsigned int i=0; i<4; i++)
	{
		lineA = edges[i][0];
		lineB = edges[i][1];
		int status = GetIntersection(ptA, ptB, lineA, lineB, &intersect);
		if(status == SEGMENTS_INTERSECT)
		{
			pt.X() = intersect.X();
			pt.Y() = center.Y();
			pt.Z() = intersect.Y();
			foundIntersect = true;
			break;
		}
	}
	if(!foundIntersect)
	{
		printf("[Grid - GetEdgePoint] Some logic is wrong here, we can't find intersection points.\n");
		return false;
	}
	if(!GetNearestFace(pt, entryFace, nav, grids, entryPt))
	{
		printf("[Grid - GetEdgePoint] Some logic is wrong here, we can't find face where intersection point lies on.\n");
		return false;
	}

	// exit
	ptA.X() = center.X();
	ptA.Y() = center.Z();
	ptB.X() = endPt.X();
	ptB.Y() = endPt.Z();
	foundIntersect = false;
	for(unsigned int i=0; i<4; i++)
	{
		lineA = edges[i][0];
		lineB = edges[i][1];
		int status = GetIntersection(ptA, ptB, lineA, lineB, &intersect);
		if(status == SEGMENTS_INTERSECT)
		{
			pt.X() = intersect.X();
			pt.Y() = center.Y();
			pt.Z() = intersect.Y();
			foundIntersect = true;
			break;
		}
	}
	if(!foundIntersect)
	{
		printf("[Grid - GetEdgePoint] Some logic is wrong here, we can't find intersection points.\n");
		return false;
	}
	if(!GetNearestFace(pt, exitFace, nav, grids, exitPt))
	{
		printf("[Grid - GetEdgePoint] Some logic is wrong here, we can't find face where intersection point lies on.\n");
		return false;
	}
	return true;
}

void Grid::UpdateDensity(float threshold, float agentSize)
{
	formerDensity = nowDensity;
	nowDensity = GetAgentNum() * agentSize / m_area;
	if(nowDensity > threshold)
	{
		m_densityStatus = BLACK;
	}
	else if(nowDensity > 0)
	{
		m_densityStatus = GRAY;
	}
	else
	{
		m_densityStatus = WHITE;
	}
}

bool Grid::GetInterpolationNeighbors(Vector2 pos, std::vector<Grid>* grids, std::vector<int>& nids
									,Vector2& leftBottom, Vector2& rightTop)
{
	nids.clear();
	if(pos.x() > m_box.center_x() && pos.y() > m_box.center_y())	// first quadrant
	{
		int cornerGridID;
		if(m_neighbor[1] >= 0)
			cornerGridID = (*grids)[m_neighbor[1]].m_neighbor[3];
		else if(m_neighbor[3] >= 0)
			cornerGridID = (*grids)[m_neighbor[3]].m_neighbor[1];
		else
			return false;
		nids.push_back(gridID);
		nids.push_back(m_neighbor[3]);
		nids.push_back(cornerGridID);
		nids.push_back(m_neighbor[1]);
		leftBottom.SetX(m_box.center_x());
		leftBottom.SetY(m_box.center_y());
		rightTop.SetX(m_box.center_x() + m_box.w());
		rightTop.SetY(m_box.center_y() + m_box.h());
	}
	else if(pos.x() > m_box.center_x() && pos.y() < m_box.center_y())	// second quadrant
	{
		int cornerGridID;
		if(m_neighbor[0] >= 0)
			cornerGridID = (*grids)[m_neighbor[0]].m_neighbor[3];
		else if(m_neighbor[3] >= 0)
			cornerGridID = (*grids)[m_neighbor[3]].m_neighbor[0];
		else
			return false;
		nids.push_back(m_neighbor[0]);
		nids.push_back(cornerGridID);
		nids.push_back(m_neighbor[3]);
		nids.push_back(gridID);
		leftBottom.SetX(m_box.center_x());
		leftBottom.SetY(m_box.center_y() - m_box.h());
		rightTop.SetX(m_box.center_x() + m_box.w());
		rightTop.SetY(m_box.center_y());
	}
	else if(pos.x() < m_box.center_x() && pos.y() < m_box.center_y())	// first quadrant
	{
		int cornerGridID;
		if(m_neighbor[0] >= 0)
			cornerGridID = (*grids)[m_neighbor[0]].m_neighbor[2];
		else if(m_neighbor[2] >= 0)
			cornerGridID = (*grids)[m_neighbor[2]].m_neighbor[0];
		else
			return false;
		nids.push_back(cornerGridID);
		nids.push_back(m_neighbor[0]);
		nids.push_back(gridID);
		nids.push_back(m_neighbor[2]);
		leftBottom.SetX(m_box.center_x() - m_box.w());
		leftBottom.SetY(m_box.center_y() - m_box.h());
		rightTop.SetX(m_box.center_x());
		rightTop.SetY(m_box.center_y());
	}
	else
	{
		int cornerGridID;
		if(m_neighbor[1] >= 0)
			cornerGridID = (*grids)[m_neighbor[1]].m_neighbor[2];
		else if(m_neighbor[2] >= 0)
			cornerGridID = (*grids)[m_neighbor[2]].m_neighbor[1];
		else 
			return false;
		nids.push_back(m_neighbor[2]);
		nids.push_back(gridID);
		nids.push_back(m_neighbor[1]);		
		nids.push_back(cornerGridID);
		leftBottom.SetX(m_box.center_x() - m_box.w());
		leftBottom.SetY(m_box.center_y());
		rightTop.SetX(m_box.center_x());
		rightTop.SetY(m_box.center_y() + m_box.h());
	}
}