#include "Grid.h"

long int Grid::QuadTreeGridID = 0;
Grid::Grid(void)
{
	avaliable_region_rate = 1;
}

Grid::~Grid(void)
{
}

int Grid::GetGridId(double i, double j)
{
	int row = (j - hmcrowd->m_min[1])/hmcrowd->m_gridsize;
	int col = (i - hmcrowd->m_min[0])/hmcrowd->m_gridsize;

	return row*hmcrowd->m_numofgridinrow + col;
}

int Grid::GetGridId(const Vector2& pos)
{
	return Grid::GetGridId(pos.x(),pos.y());
}