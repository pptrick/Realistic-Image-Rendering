#pragma once
#ifndef CAMERA_H
#define CAMERA_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <math.h>
#include "vector3D.h"
#include "tracingtools.h"
#include "object.h"
#include "bezier.h"
#include "PhotonMapping.h"

#define HEIGHT 600
#define WIDTH 800

using namespace std;
using namespace cv;

class Camera
{
private:
    Vector3D ViewPos;
    Vector3D mainDirect;
    Vector3D LightColor;
    double reflectloss;
    double transmitloss;
    double focus;
    double MinWeight;
    double aperture;
public:
    Vector3D LightPos;
    Camera()
    {
        //ÄÚÖÃ²ÎÊý
        ViewPos.ChangeValue(0, 0, 0);
        LightPos.ChangeValue(0, 1000, 500);
        mainDirect.ChangeValue(0, 0, 1);
        LightColor.ChangeValue(255, 255, 255);
        reflectloss = 0.3;
        transmitloss = 0;
        focus = 200;
        MinWeight = 0.1;
        aperture = 1.0;
    }
    void init(const char* filename);
    void TakePhoto(Mat& picture, Sphere ball1, Sphere ball2, Room myRoom, Cup myCup, PhotonMap* Map);
    void RayTracing(Vector3D StartPoint, Vector3D direction, double weight, Vector3D& Color, Sphere ball1, Sphere ball2, Room myRoom, Cup myCup, PhotonMap* Map);
    void FindNearest(Vector3D startPoint, Vector3D direction, double& NearestParami, int& NearestObject, Vector3D& NearestPoint, bool& InObject, int& plane_num, CoorBezier& Cup_Para, Sphere ball1, Sphere ball2, Room myRoom, Cup myCup);
    void Shading(Vector3D startPoint, Vector3D direction, double& NearestParami, int& NearestObject, Vector3D& NearestPoint, CoorBezier& Cup_Para, Sphere ball1, Sphere ball2, Cup myCup);
};
#endif CAMERA_H
