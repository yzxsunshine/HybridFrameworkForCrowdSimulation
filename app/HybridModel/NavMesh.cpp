#include "NavMesh.h"

NavMeshFace::NavMeshFace(void) : walkable(false), interactionType(0),CCID(-1)
{
}

NavMeshFace::~NavMeshFace(void)
{
}

/*
NavMeshFace::NavMeshFace(const NavMeshFace& face)
{
	this->CCID = face.CCID;
	this->walkable = face.walkable;
	this->interactionType = face.interactionType;
	memcpy(this->midPointIds, face.midPointIds, 3*sizeof(int));
	memcpy(this->vertexIDs, face.vertexIDs, 3*sizeof(int));
}*/

NavMeshFace& NavMeshFace::operator = (const NavMeshFace& face)
{
	memcpy(this, &face, sizeof(NavMeshFace));
	return *this;
}