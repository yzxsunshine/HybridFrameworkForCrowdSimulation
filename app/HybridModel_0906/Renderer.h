#pragma once
#include "DebugDraw.h"
#include "InputMesh.h"

void duDebugDrawMapMeshSlope(duDebugDraw* dd, NavMesh& mesh,
							 const float walkableSlopeAngle, const float texScale);