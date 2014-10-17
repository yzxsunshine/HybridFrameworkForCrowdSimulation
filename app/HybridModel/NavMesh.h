#pragma once

#include <vcg/complex/complex.h>

class NavMeshVertex; class NavMeshFace;
struct MapUsedTypes : public vcg::UsedTypes<vcg::Use<NavMeshVertex>   ::AsVertexType,
											vcg::Use<NavMeshFace>     ::AsFaceType>{};

class NavMeshVertex  : public vcg::Vertex< MapUsedTypes, vcg::vertex::Coord3f, vcg::vertex::Normal3f, vcg::vertex::BitFlags, vcg::vertex::VFAdj>
{
public:
	int vertID;
	std::vector<int> edgeIDs;
};

class NavMeshEdge
{
public:
	int vertIDs[2];	//id of vertex
	int vertRelIDs[2];	// Relative id of vertex
	int faceIDs[2];	//id of faces;
	int edgeID;
	bool isBorder;
};

class NavMeshFace    : public vcg::Face< MapUsedTypes, vcg::face::FFAdj, vcg::face::Normal3f, vcg::face::VFAdj, vcg::face::VertexRef, vcg::face::BitFlags > 
{
public:
	NavMeshFace(void);
	~NavMeshFace(void);
	//NavMeshFace(const NavMeshFace& face);
	NavMeshFace& operator = (const NavMeshFace& face);
public:
	bool walkable;
	int interactionType;	// not implemented, leave for future API.
	int CCID;  // connect component id
	int midPointIds[3]; 
	int vertexIDs[3];
	int edgeIDs[3];
	int faceID;
	int tileID;	// record the tile it's belongs to
};


class NavMesh    : public vcg::tri::TriMesh< std::vector<NavMeshVertex>, std::vector<NavMeshFace>> 
{
public:
	std::vector<NavMeshEdge> edges;
};