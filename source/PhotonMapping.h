#pragma once
#ifndef PHOTONMAPPING_H
#define PHOTONMAPPING_H

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <string>

#include "vector3D.h"
#include "tracingtools.h"
#include "object.h"
#include "bezier.h"

using namespace std;
using namespace cv;

struct node
{
    double node_x;
    double node_y;
    double node_z;
    int direction;  //0表示x方向，1表示y方向，2表示z方向
    node *l_child;
    node *r_child;
};

class PhotonMap
{
public:
    node * root;
    vector<Vector3D> Photons;
    void SpreadPhoton(Vector3D center, double radius, Vector3D LightPos, Sphere ball, Room myroom);
    void SpreadPhoton_ALL(Vector3D LightPos, Sphere ball1, Sphere ball2, Room myroom);
    void trace(Vector3D StartPoint, Vector3D direction, Sphere ball1, Sphere ball2, Room myRoom);
    void FindNear(Vector3D startPoint, Vector3D direction, double& NearestParami, int& NearestObject, Vector3D& NearestPoint, bool& InObject, Sphere ball1, Sphere ball2, Room myRoom);
    double distance(Vector3D a, node *b){return sqrt((a.x - b->node_x)*(a.x - b->node_x) + (a.y - b->node_y)*(a.y - b->node_y) + (a.z - b->node_z)*(a.z - b->node_z));}
    void init_tree(vector<Vector3D> Photons);
    bool build_tree(int depth, vector<Vector3D> Photons, node *&p);
    void RegionFind(Vector3D Goal, double range, node* p, int& photon_num);
    bool inrange(Vector3D Goal, double range, node* p);
};

#endif PHOTONMAPPING_H

