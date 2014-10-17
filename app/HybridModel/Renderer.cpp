#include "Renderer.h"


void duDebugDrawMapMeshSlope(duDebugDraw* dd, NavMesh& mesh,
							 const float walkableSlopeAngle, const float texScale)
{
	if (!dd) return;
	
	
	const float walkableThr = cosf(walkableSlopeAngle/180.0f*DU_PI);
	
	float uva[2];
	float uvb[2];
	float uvc[2];
	
	dd->texture(true);

	const unsigned int unwalkable = duRGBA(192,128,0,255);
	
	dd->begin(DU_DRAW_TRIS);
	for(unsigned int i=0; i< mesh.face.size(); i++)
	{ 
		// calculate the normal of each face in order to see which face is walkable
		vcg::Point3f norm = mesh.face[i].N();
		unsigned int color;
		unsigned char a = (unsigned char)(220*(2+norm[0]+norm[1])/4);
		if (norm[1] < walkableThr)
			color = duRGBA(a,0,0,255);//duLerpCol(duRGBA(a,a,a,255), unwalkable, 64);
		else
			color = duRGBA(a,a,a,255);
		
		NavMeshVertex* vert0 = mesh.face[i].V(0);
		NavMeshVertex* vert1 = mesh.face[i].V(1);
		NavMeshVertex* vert2 = mesh.face[i].V(2);
		// calculate color and texture for xoy or yoz or xoz plane, depends on the dominant direction z or x or y.
		int ax = 0, ay = 0;
		if (fabs(norm[1]) > fabs(norm[ax]))
			ax = 1;
		if (fabs(norm[2]) > fabs(norm[ax]))
			ax = 2;
		ax = (1<<ax)&3; // +1 mod 3
		ay = (1<<ax)&3; // +1 mod 3
		
		uva[0] = vert0->P()[ax]*texScale;
		uva[1] = vert0->P()[ay]*texScale;
		uvb[0] = vert1->P()[ax]*texScale;
		uvb[1] = vert1->P()[ay]*texScale;
		uvc[0] = vert2->P()[ax]*texScale;
		uvc[1] = vert2->P()[ay]*texScale;

		//dd->vertex(va, color, uva);
		//dd->vertex(vb, color, uvb);
		//dd->vertex(vc, color, uvc);
		dd->vertex(vert0->P()[0], vert0->P()[1], vert0->P()[2], color, uva[0], uva[1]);
		dd->vertex(vert1->P()[0], vert1->P()[1], vert1->P()[2], color, uva[0], uva[1]);
		dd->vertex(vert2->P()[0], vert2->P()[1], vert2->P()[2], color, uva[0], uva[1]);
	}
	dd->end();

	dd->texture(false);
}