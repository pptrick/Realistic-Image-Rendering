#pragma once
#ifndef TRACINGTOOLS_H
#define TRACINGTOOLS_H

#include <iostream>
#include <cmath>
#include "vector3D.h"

using namespace std;

Vector3D GetTransBeam(Vector3D incidentBeam, Vector3D normal, double refractRate_in, double refractRate_out);

Vector3D GetReflectBeam(Vector3D incidentBeam, Vector3D normal);

double Distance_PL(Vector3D Point, Vector3D direction,Vector3D Point_On_Line);

bool Plane_BBox(double x0, double x1, double y0, double y1,Vector3D StartPoint_Plane,Vector3D direction_Plane);

bool BBox(double x0, double x1, double y0, double y1, double z0, double z1, Vector3D StartPoint, Vector3D direction);


#endif TRACINGTOOLS_H
