//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//

#include <stdio.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include "SDL.h"
#include "SDL_opengl.h"
#include "imgui.h"
#include "imguiRenderGL.h"
#include "Recast.h"
#include "RecastDebugDraw.h"
#include "InputGeom.h"
#include "TestCase.h"
#include "Filelist.h"
#include "SlideShow.h"
#include "HybridModel.h"

//*/
#ifdef WIN32
#	define snprintf _snprintf
#	define putenv _putenv
#endif

struct RegionPoint	//批量框选用到的数据结构
{
	float x;
	float y;
	float rays[3];
	float raye[3];
};

struct SampleItem
{
	Sample* (*create)();
	const char* name;
};

//Sample* createSolo() { return new Sample_SoloMesh(); }
//Sample* createTile() { return new Sample_TileMesh(); }
//Sample* createTempObstacle() { return new Sample_TempObstacles(); }
//Sample* createDebug() { return new Sample_Debug(); }
Sample* createHybridModel() { return new HybridModel(); };

static SampleItem g_samples[] =
{
//	{ createSolo, "Solo Mesh" },
//	{ createTile, "Tile Mesh" },
//	{ createTempObstacle, "Temp Obstacles" },
//	{ createDebug, "Debug" },
	{createHybridModel, "Hybrid Model"},
};
static const int g_nsamples = sizeof(g_samples)/sizeof(SampleItem); 


int main(int /*argc*/, char** /*argv*/)
{
	// Init SDL
	if (SDL_Init(SDL_INIT_EVERYTHING) != 0)
	{
		printf("Could not initialise SDL\n");
		return -1;
	}
	
	// Center window
	char env[] = "SDL_VIDEO_CENTERED=1";
	putenv(env);

	// Init OpenGL
	SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
	SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
	SDL_GL_SetAttribute(SDL_GL_RED_SIZE, 8);
	SDL_GL_SetAttribute(SDL_GL_GREEN_SIZE, 8);
	SDL_GL_SetAttribute(SDL_GL_BLUE_SIZE, 8);
	SDL_GL_SetAttribute(SDL_GL_ALPHA_SIZE, 8);
//#ifndef WIN32
	SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 1);
	SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 4);
//#endif

	const SDL_VideoInfo* vi = SDL_GetVideoInfo();

	bool presentationMode = false;

	int width, height;
	SDL_Surface* screen = 0;
	
	if (presentationMode)
	{
		width = vi->current_w;
		height = vi->current_h;
		screen = SDL_SetVideoMode(width, height, 0, SDL_OPENGL|SDL_FULLSCREEN);
	}
	else
	{	
		width = vi->current_w - 20;
		height = vi->current_h - 80;
		screen = SDL_SetVideoMode(width, height, 0, SDL_OPENGL);
	}
	
	if (!screen)
	{
		printf("Could not initialise SDL opengl\n");
		return -1;
	}

	glEnable(GL_MULTISAMPLE);

	SDL_WM_SetCaption("CrowdVisual", 0);
	
	if (!imguiRenderGLInit("DroidSans.ttf"))
	{
		printf("Could not init GUI renderer.\n");
		SDL_Quit();
		return -1;
	}
	
	float t = 0.0f;
	float timeAcc = 0.0f;
	Uint32 lastTime = SDL_GetTicks();
	int mx = 0, my = 0;
	float rx = 45;
	float ry = -45;
	float moveW = 0, moveS = 0, moveA = 0, moveD = 0;
	float camx = 0, camy = 0, camz = 0, camr = 1000;
	float origrx = 0, origry = 0;
	int origx = 0, origy = 0;
	float scrollZoom = 0;
	bool rotate = false;
	bool movedDuringRotate = false;
	float rays[3], raye[3]; 
	bool mouseOverMenu = false;
	bool showMenu = !presentationMode;
	bool showLog = false;
	bool showTools = true;
	bool showLevels = false;
	bool showSample = false;
	bool showTestCases = false;
	bool showTests = false;
	bool selectRegion = false;
	bool selectRegionMoving = false;
	RegionPoint region[4];
	GLdouble proj[16];
	GLdouble model[16];
	GLint view[4];
	int propScroll = 0;
	int logScroll = 0;
	int toolsScroll = 0;
	
	char sampleName[64] = "选择仿真工具..."; 
	
	FileList files;
	char meshName[128] = "选择场景...";

	FileList testFiles;
	char testName[128] = "选择用例...";
	
	float mpos[3] = {0,0,0};
	bool mposSet = false;
	
	SlideShow slideShow;
	slideShow.init("slides/");
	
	//InputGeom* geom = 0;
	InputMesh* mesh = 0;
	Sample* sample = 0;
	//TestCase* test = 0;

	BuildContext ctx;
	
	glEnable(GL_CULL_FACE);
	
	float fogCol[4] = { 0.32f, 0.31f, 0.30f, 1.0f };
	glEnable(GL_FOG);
	glFogi(GL_FOG_MODE, GL_LINEAR);
	glFogf(GL_FOG_START, camr*0.1f);
	glFogf(GL_FOG_END, camr*1.25f);
	glFogfv(GL_FOG_COLOR, fogCol);
	
	glDepthFunc(GL_LEQUAL);
	
	bool done = false;
	while(!done)
	{
		// Handle input events.
		int mscroll = 0;
		bool processHitTest = false;
		bool processHitTestShift = false;
		SDL_Event event;
		
		while (SDL_PollEvent(&event))
		{
			switch (event.type)
			{
				case SDL_KEYDOWN:
					// Handle any key presses here.
					if (event.key.keysym.sym == SDLK_ESCAPE)
					{
						done = true;
					}
					else if (event.key.keysym.sym == SDLK_TAB)
					{
						showMenu = !showMenu;
					}
					else if (event.key.keysym.sym == SDLK_SPACE)
					{
						if (sample)
							sample->handleToggle();
					}
					else if (event.key.keysym.sym == SDLK_1)
					{
						if (sample)
							sample->handleStep();
					}
					else if (event.key.keysym.sym == SDLK_RIGHT)
					{
						slideShow.nextSlide();
					}
					else if (event.key.keysym.sym == SDLK_LEFT)
					{
						slideShow.prevSlide();
					}
					else if (event.key.keysym.sym == SDLK_b)
					{
						selectRegion = true;
					}
					break;
					
				case SDL_MOUSEBUTTONDOWN:
					if (event.button.button == SDL_BUTTON_RIGHT)
					{
						if (!mouseOverMenu)
						{
							// Rotate view
							rotate = true;
							movedDuringRotate = false;
							origx = mx;
							origy = my;
							origrx = rx;
							origry = ry;
						}
					}	
					else if (event.button.button == SDL_BUTTON_WHEELUP)
					{
						if (mouseOverMenu)
							mscroll--;
						else
							scrollZoom -= 1.0f;
					}
					else if (event.button.button == SDL_BUTTON_WHEELDOWN)
					{
						if (mouseOverMenu)
							mscroll++;
						else
							scrollZoom += 1.0f;
					}
					else if (event.button.button == SDL_BUTTON_LEFT)
					{
						if(selectRegion&&!mouseOverMenu)
						{
							region[0].x = mx;
							region[0].y = my;
							//memcpy(region[0].rays,rays,3*sizeof(float));
							//memcpy(region[0].raye,raye,3*sizeof(float));
							selectRegionMoving = true;
						}
					}
					break;
					
				case SDL_MOUSEBUTTONUP:
					// Handle mouse clicks here.
					if (event.button.button == SDL_BUTTON_RIGHT)
					{
						rotate = false;
						if (!mouseOverMenu)
						{
							if (!movedDuringRotate)
							{
								processHitTest = true;
								processHitTestShift = true;
							}
						}
					}
					else if (event.button.button == SDL_BUTTON_LEFT)
					{
						if (!mouseOverMenu)
						{
							processHitTest = true;
							processHitTestShift = (SDL_GetModState() & KMOD_SHIFT) ? true : false;
							if(selectRegion)
							{
								region[1].x = mx;
								region[1].y = region[0].y;
								region[2].x = mx;
								region[2].y = my;
								region[3].x = region[0].x;
								region[3].y = my;
								selectRegionMoving = false;
							}
						}
					}
					
					break;
					
				case SDL_MOUSEMOTION:
					mx = event.motion.x;
					my = height-1 - event.motion.y;
					if (rotate)
					{
						int dx = mx - origx;
						int dy = my - origy;
						rx = origrx - dy*0.25f;
						ry = origry + dx*0.25f;
						if (dx*dx+dy*dy > 3*3)
							movedDuringRotate = true;
					}
					if(selectRegion)
					{
						region[1].x = mx;
						region[1].y = region[0].y;
						region[2].x = mx;
						region[2].y = my;
						region[3].x = region[0].x;
						region[3].y = my;
					}
					break;
					
				case SDL_QUIT:
					done = true;
					break;
					
				default:
					break;
			}
		}

		unsigned char mbut = 0;
		if (SDL_GetMouseState(0,0) & SDL_BUTTON_LMASK)
			mbut |= IMGUI_MBUT_LEFT;
		if (SDL_GetMouseState(0,0) & SDL_BUTTON_RMASK)
			mbut |= IMGUI_MBUT_RIGHT;
		
		Uint32	time = SDL_GetTicks();
		float	dt = (time - lastTime) / 1000.0f;
		lastTime = time;
		
		t += dt;


		// Hit test mesh.
		if (processHitTest && mesh && sample)
		{
			if(selectRegion)
			{
				selectRegion = false;
				float hitt;
				//GLdouble proj[16];
				//GLdouble model[16];
				//GLint view[4];
				//glGetDoublev(GL_PROJECTION_MATRIX, proj);
				//glGetDoublev(GL_MODELVIEW_MATRIX, model);
				//glGetIntegerv(GL_VIEWPORT, view);
				for(int i=0;i<4;i++)
				{
					GLdouble x, y, z;
					gluUnProject(region[i].x, region[i].y, 0.0f, model, proj, view, &x, &y, &z);
					region[i].rays[0] = (float)x; region[i].rays[1] = (float)y; region[i].rays[2] = (float)z;
					gluUnProject(region[i].x, region[i].y, 1.0f, model, proj, view, &x, &y, &z);
					region[i].raye[0] = (float)x; region[i].raye[1] = (float)y; region[i].raye[2] = (float)z;
				}
				bool hit = mesh->raycastMesh(region[0].rays, region[0].raye, hitt);
				if(hit)
				{
					float topleft[3];
					topleft[0] = region[0].rays[0] + (region[0].raye[0] - region[0].rays[0])*hitt;
					topleft[1] = region[0].rays[1] + (region[0].raye[1] - region[0].rays[1])*hitt;
					topleft[2] = region[0].rays[2] + (region[0].raye[2] - region[0].rays[2])*hitt;
					

					hit = mesh->raycastMesh(region[1].rays, region[1].raye, hitt);
					if(hit)
					{
						float topright[3];
						topright[0] = region[1].rays[0] + (region[1].raye[0] - region[1].rays[0])*hitt;
						topright[1] = region[1].rays[1] + (region[1].raye[1] - region[1].rays[1])*hitt;
						topright[2] = region[1].rays[2] + (region[1].raye[2] - region[1].rays[2])*hitt;

						hit = mesh->raycastMesh(region[2].rays, region[2].raye, hitt);
						if(hit)
						{
							float bottomright[3];
							bottomright[0] = region[2].rays[0] + (region[2].raye[0] - region[2].rays[0])*hitt;
							bottomright[1] = region[2].rays[1] + (region[2].raye[1] - region[2].rays[1])*hitt;
							bottomright[2] = region[2].rays[2] + (region[2].raye[2] - region[2].rays[2])*hitt;
							hit = mesh->raycastMesh(region[3].rays, region[3].raye, hitt);
							if(hit)
							{
								float bottomleft[3];
								bottomleft[0] = region[3].rays[0] + (region[3].raye[0] - region[3].rays[0])*hitt;
								bottomleft[1] = region[3].rays[1] + (region[3].raye[1] - region[3].rays[1])*hitt;
								bottomleft[2] = region[3].rays[2] + (region[3].raye[2] - region[3].rays[2])*hitt;

								sample->handleDrag(topleft,topright,bottomleft,bottomright);
							}
						}
					}
				}
			}
			else
			{
				float hitt;
				bool hit = mesh->raycastMesh(rays, raye, hitt);
				
				if (hit)
				{
					if (SDL_GetModState() & KMOD_CTRL)
					{
						// Marker
						mposSet = true;
						mpos[0] = rays[0] + (raye[0] - rays[0])*hitt;
						mpos[1] = rays[1] + (raye[1] - rays[1])*hitt;
						mpos[2] = rays[2] + (raye[2] - rays[2])*hitt;
					}
					else
					{
						float pos[3];
						pos[0] = rays[0] + (raye[0] - rays[0])*hitt;
						pos[1] = rays[1] + (raye[1] - rays[1])*hitt;
						pos[2] = rays[2] + (raye[2] - rays[2])*hitt;
						sample->handleClick(rays, pos, processHitTestShift);
					}
				}
				else
				{
					if (SDL_GetModState() & KMOD_CTRL)
					{
						// Marker
						mposSet = false;
					}
				}
			}
		}
		
		// Update sample simulation.
		const float SIM_RATE = 20;
		const float DELTA_TIME = 1.0f/SIM_RATE;
		timeAcc = rcClamp(timeAcc+dt, -1.0f, 1.0f);
		int simIter = 0;
		while (timeAcc > DELTA_TIME)
		{
			timeAcc -= DELTA_TIME;
			if (simIter < 5)
			{
				if (sample)
					sample->handleUpdate(DELTA_TIME);
			}
			simIter++;
		}

		// Clamp the framerate so that we do not hog all the CPU.
		const float MIN_FRAME_TIME = 1.0f/40.0f;
		if (dt < MIN_FRAME_TIME)
		{
			int ms = (int)((MIN_FRAME_TIME - dt)*1000.0f);
			if (ms > 10) ms = 10;
			if (ms >= 0)
				SDL_Delay(ms);
		}
		
		
		// Update and render
		glViewport(0, 0, width, height);
		glClearColor(0.3f, 0.3f, 0.32f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glDisable(GL_TEXTURE_2D);
		
		// Render 3d
		glEnable(GL_DEPTH_TEST);
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluPerspective(50.0f, (float)width/(float)height, 1.0f, camr);
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		glRotatef(rx,1,0,0);
		glRotatef(ry,0,1,0);
		glTranslatef(-camx, -camy, -camz);
		
		// Get hit ray position and direction.
		
		glGetDoublev(GL_PROJECTION_MATRIX, proj);
		glGetDoublev(GL_MODELVIEW_MATRIX, model);
		glGetIntegerv(GL_VIEWPORT, view);
		GLdouble x, y, z;
		gluUnProject(mx, my, 0.0f, model, proj, view, &x, &y, &z);
		rays[0] = (float)x; rays[1] = (float)y; rays[2] = (float)z;
		gluUnProject(mx, my, 1.0f, model, proj, view, &x, &y, &z);
		raye[0] = (float)x; raye[1] = (float)y; raye[2] = (float)z;
		
		// Handle keyboard movement.
		Uint8* keystate = SDL_GetKeyState(NULL);
		moveW = rcClamp(moveW + dt * 4 * (keystate[SDLK_w] ? 1 : -1), 0.0f, 1.0f);
		moveS = rcClamp(moveS + dt * 4 * (keystate[SDLK_s] ? 1 : -1), 0.0f, 1.0f);
		moveA = rcClamp(moveA + dt * 4 * (keystate[SDLK_a] ? 1 : -1), 0.0f, 1.0f);
		moveD = rcClamp(moveD + dt * 4 * (keystate[SDLK_d] ? 1 : -1), 0.0f, 1.0f);
		
		float keybSpeed = 22.0f;
		if (SDL_GetModState() & KMOD_SHIFT)
			keybSpeed *= 4.0f;
		
		float movex = (moveD - moveA) * keybSpeed * dt;
		float movey = (moveS - moveW) * keybSpeed * dt;
		
		movey += scrollZoom * 2.0f;
		scrollZoom = 0;
		
		camx += movex * (float)model[0];
		camy += movex * (float)model[4];
		camz += movex * (float)model[8];
		
		camx += movey * (float)model[2];
		camy += movey * (float)model[6];
		camz += movey * (float)model[10];

		glEnable(GL_FOG);

		if (sample)
			sample->handleRender();
		
		glDisable(GL_FOG);
		
		// Render GUI
		glDisable(GL_DEPTH_TEST);
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluOrtho2D(0, width, 0, height);
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		
		mouseOverMenu = false;
		
		imguiBeginFrame(mx,my,mbut,mscroll);
		
		if (sample)
		{
			sample->handleRenderOverlay((double*)proj, (double*)model, (int*)view);
		}

		// Help text.
		if (showMenu)
		{
			const char msg[] = "W/S/A/D：移动  鼠标右键：旋转视角";
			imguiDrawText(280, height-20, IMGUI_ALIGN_LEFT, msg, imguiRGBA(255,255,255,128));
		}
		
		if (showMenu)
		{
			if (imguiBeginScrollArea("场景属性", width-250-10, 10, 250, height-20, &propScroll))
				mouseOverMenu = true;

			if (imguiCheck("显示日志", showLog))
				showLog = !showLog;
			if (imguiCheck("显示仿真工具", showTools))
				showTools = !showTools;

			imguiSeparator();
			imguiLabel("仿真工具");
			if (imguiButton(sampleName))
			{
				if (showSample)
				{
					showSample = false;
				}
				else
				{
					showSample = true;
					showLevels = false;
					showTestCases = false;
					showTests = false;
				}
			}
			
			imguiSeparator();
			imguiLabel("输入场景"); //Input Mesh
			if (imguiButton(meshName))
			{
				if (showLevels)
				{
					showLevels = false;
				}
				else
				{
					showSample = false;
					showTestCases = false;
					showLevels = true;
					showTests = false;
					scanDirectory("Meshes", ".obj", files);
				}
			}

			if (mesh)
			{
				char text[64];
				snprintf(text, 64, "顶点数: %.1fk  三角形数: %.1fk", //Verts: %.1fk  Tris: %.1fk
						 mesh->getVertCount()/1000.0f,
						 mesh->getTriCount()/1000.0f);
				imguiValue(text);
			}
			imguiSeparator();
			
			if (imguiButton(testName))
			{
				if(showTests)
					showTests = false;
				else
				{
					showLevels = false;
					showTests = true;
					scanDirectory("Tests", ".txt", testFiles);
				}
			}

			if (mesh && sample)
			{
				imguiSeparatorLine();
				
				sample->handleSettings();

				if (imguiButton("建立导航网格"))//Build
				{
					ctx.resetLog();
					if (!sample->handleBuild())
					{
						showLog = true;
						logScroll = 0;
					}
					ctx.dumpLog("建立导航网格 %s:", meshName);//"Build log %s:"
					
					// Clear test.
					//delete test;
					//test = 0;
				}

				imguiSeparator();
			}
			
			if (sample)
			{
				imguiSeparatorLine();
				sample->handleDebugMode();
			}

			imguiEndScrollArea();
		}
		
		// Sample selection dialog.
		if (showSample)
		{
			static int levelScroll = 0;
			if (imguiBeginScrollArea("选择方案", width-10-250-10-200, height-10-250, 200, 250, &levelScroll))//Choose Sample
				mouseOverMenu = true;

			Sample* newSample = 0;
			for (int i = 0; i < g_nsamples; ++i)
			{
				if (imguiItem(g_samples[i].name))
				{
					newSample = g_samples[i].create();
					if (newSample)
						strcpy(sampleName, g_samples[i].name);
				}
			}
			
			if (newSample)
			{
				delete sample;
				sample = newSample;
				sample->setContext(&ctx);
				if (mesh && sample)
				{
					sample->handleMeshChanged(mesh);
				}
				showSample = false;
			}

			if (mesh || sample)
			{
				const float* bmin = 0;
				const float* bmax = 0;
				if (sample)
				{
					bmin = sample->getBoundsMin();
					bmax = sample->getBoundsMax();
				}
				else if (mesh)
				{
					bmin = mesh->getMeshBoundsMin();
					bmax = mesh->getMeshBoundsMax();
				}
				// Reset camera and fog to match the mesh bounds.
				if (bmin && bmax)
				{
					camr = sqrtf(rcSqr(bmax[0]-bmin[0]) +
								 rcSqr(bmax[1]-bmin[1]) +
								 rcSqr(bmax[2]-bmin[2])) / 2;
					camx = (bmax[0] + bmin[0]) / 2 + camr;
					camy = (bmax[1] + bmin[1]) / 2 + camr;
					camz = (bmax[2] + bmin[2]) / 2 + camr;
					camr *= 3;
				}
				rx = 45;
				ry = -45;
				glFogf(GL_FOG_START, camr*0.1f);
				glFogf(GL_FOG_END, camr*1.25f);
			}
			
			imguiEndScrollArea();
		}
		
		// Level selection dialog.
		if (showLevels)
		{
			static int levelScroll = 0;
			if (imguiBeginScrollArea("选择场景", width-10-250-10-200, height-10-450, 200, 450, &levelScroll))//Choose Level
				mouseOverMenu = true;
			
			int levelToLoad = -1;
			for (int i = 0; i < files.size; ++i)
			{
				if (imguiItem(files.files[i]))
					levelToLoad = i;
			}
			
			if (levelToLoad != -1)
			{
				strncpy(meshName, files.files[levelToLoad], sizeof(meshName));
				meshName[sizeof(meshName)-1] = '\0';
				showLevels = false;
				
				delete mesh;
				mesh = 0;
				
				char path[256];
				strcpy(path, "Meshes/");
				strcat(path, meshName);
				
				mesh = new InputMesh;
				if (!mesh || !mesh->ReadMap(path))
				{
					delete mesh;
					mesh = 0;
					
					showLog = true;
					logScroll = 0;
					ctx.dumpLog("读取场景 %s:", meshName);//Geom load log %s:
				}
				else
				{
					//mesh->FindWalkableArea();
				}
				
				if (sample && mesh)
				{
					sample->handleMeshChanged(mesh);
				}

				if (mesh || sample)
				{
					const float* bmin = 0;
					const float* bmax = 0;
					if (sample)
					{
						bmin = sample->getBoundsMin();
						bmax = sample->getBoundsMax();
					}
					else if (mesh)
					{
						bmin = mesh->getMeshBoundsMin();
						bmax = mesh->getMeshBoundsMax();
					}
					// Reset camera and fog to match the mesh bounds.
					if (bmin && bmax)
					{
						camr = sqrtf(rcSqr(bmax[0]-bmin[0]) +
									 rcSqr(bmax[1]-bmin[1]) +
									 rcSqr(bmax[2]-bmin[2])) / 2;
						camx = (bmax[0] + bmin[0]) / 2 + camr;
						camy = (bmax[1] + bmin[1]) / 2 + camr;
						camz = (bmax[2] + bmin[2]) / 2 + camr;
						camr *= 3;
					}
					rx = 45;
					ry = -45;
					glFogf(GL_FOG_START, camr*0.1f);
					glFogf(GL_FOG_END, camr*1.25f);
				}
			}
			
			imguiEndScrollArea();
			
		}

		if (showTests)
		{
			static int levelScroll = 0;
			if (imguiBeginScrollArea("选择用例", width-10-250-10-200, height-10-450, 200, 450, &levelScroll))//Choose Level
				mouseOverMenu = true;
			
			int levelToLoad = -1;
			for (int i = 0; i < testFiles.size; ++i)
			{
				if (imguiItem(testFiles.files[i]))
					levelToLoad = i;
			}
			
			if (levelToLoad != -1)
			{
				strncpy(testName, testFiles.files[levelToLoad], sizeof(testName));
				testName[sizeof(testName)-1] = '\0';
				showTests = false;
				HybridModelCrowd* hmc = ((HybridModel*)sample)->getHybridCrowd();
				dtCrowdAgentParams ap;
				memset(&ap, 0, sizeof(ap));
				ap.radius = sample->getAgentRadius();
				ap.height = sample->getAgentHeight();
				ap.maxAcceleration = 8.0f;
				ap.maxSpeed = 3.5f;
				ap.collisionQueryRange = ap.radius * 12.0f;
				ap.pathOptimizationRange = ap.radius * 30.0f;
				ap.updateFlags = 0; 
				float p[3], v[3], t[3];
				int colorID;
				char path[256];
				strcpy(path, "Tests/");
				strcat(path, testName);
				FILE* fpTest = fopen(path, "r");
				while(!feof(fpTest))
				{
					fscanf(fpTest, "%f %f %f %f %f %f %d", &p[0], &p[1], &p[2], &t[0], &t[1], &t[2], &colorID);
					hmc->addAgent(p, &ap, t, NULL, colorID);
				}
				fclose(fpTest);
			}
			imguiEndScrollArea();
			
		}

		
		// Log
		if (showLog && showMenu)
		{
			if (imguiBeginScrollArea("系统日志", 250+20, 10, width - 300 - 250, 200, &logScroll)) //Log
				mouseOverMenu = true;
			for (int i = 0; i < ctx.getLogCount(); ++i)
				imguiLabel(ctx.getLogText(i));
			imguiEndScrollArea();
		}
		
		// Tools
		if (!showTestCases && showTools && showMenu) // && geom && sample)
		{
			if (imguiBeginScrollArea("仿真工具", 10, 10, 250, height-20, &toolsScroll)) //Tools
				mouseOverMenu = true;

			if (sample)
				sample->handleTools();
			
			imguiEndScrollArea();
		}
		
		slideShow.updateAndDraw(dt, (float)width, (float)height);
		
		// Marker
		if (mposSet && gluProject((GLdouble)mpos[0], (GLdouble)mpos[1], (GLdouble)mpos[2],
								  model, proj, view, &x, &y, &z))
		{
			// Draw marker circle
			glLineWidth(5.0f);
			glColor4ub(240,220,0,196);
			glBegin(GL_LINE_LOOP);
			const float r = 25.0f;
			for (int i = 0; i < 20; ++i)
			{
				const float a = (float)i / 20.0f * RC_PI*2;
				const float fx = (float)x + cosf(a)*r;
				const float fy = (float)y + sinf(a)*r;
				glVertex2f(fx,fy);
			}
			glEnd();
			glLineWidth(1.0f);
		}

		//Select Region Rect 选取框
		if(selectRegion&&selectRegionMoving)
		{
			glLineWidth(1.0f);
			glColor4ub(220,220,220,196);
			glBegin(GL_LINE_LOOP);
			glVertex2f(region[0].x,region[0].y);
			glVertex2f(region[2].x,region[0].y);
			glVertex2f(region[2].x,region[2].y);
			glVertex2f(region[0].x,region[2].y);
			glEnd();
			glLineWidth(1.0f);
		}
		
		imguiEndFrame();
		imguiRenderGLDraw();		
		
		glEnable(GL_DEPTH_TEST);
		SDL_GL_SwapBuffers();
	}
	
	imguiRenderGLDestroy();
	
	SDL_Quit();
	
	delete sample;
	//delete geom;
	delete mesh;
	return 0;
}
