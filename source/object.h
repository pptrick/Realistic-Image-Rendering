#pragma once
#ifndef OBJECT_H
#define OBJECT_H

#include <iostream>
#include <fstream>
#include <math.h>
#include <opencv2/opencv.hpp>
#include "vector3D.h"
#include "tracingtools.h"

using namespace std;
using namespace cv;

class Object
{
public:
    double ReflectRate;
    double TransmitRate;
    double RefractRate;

    Vector3D ambientStrength;
    Vector3D diffuseStrength;
    Vector3D specularStrength;
    int ReflectFactor;
    Vector3D ObjectColor;
    Vector3D Locallight(Vector3D PointPos, Vector3D LightPos, Vector3D Viewdirect, Vector3D normal, Vector3D LightColor);
};

class Sphere :public Object
{
public:
    Vector3D Center;
    double Radius;
    Sphere() { Center.ChangeValue(0, 0, 0); Radius = 0; }
    Sphere(double center_x, double center_y, double center_z, double input_Radius)
    {
        Center.ChangeValue(center_x, center_y, center_z);
        Radius = input_Radius;

        //内置参数
        ReflectRate = 0.3;
        TransmitRate = 0;
        RefractRate = 1.5;
        ReflectFactor = 20;
        ambientStrength.ChangeValue(0.1, 0.1, 0.1);
        diffuseStrength.ChangeValue(0.5, 0.1, 0.1);
        specularStrength.ChangeValue(0.4, 0.4, 0.4);
        ObjectColor.ChangeValue(0, 255, 0);
    }
    void init(const char* filename);
    bool OnSphere(double input_x, double input_y, double input_z);
    bool InSphere(double input_x, double input_y, double input_z);
    Vector3D GetNormal(double input_x, double input_y, double input_z);
    double GetIntersectParami(Vector3D StartPoint, Vector3D direction, bool& InObject);

};

class Plane : public Object
{
private:
    Vector3D normal;
    Vector3D point;
    Mat texture;
public:
    Plane() { 
        normal.ChangeValue(0, 1, 0); point.ChangeValue(0, 0, 0); 

        //内置参数
        ReflectRate = 0.0;
        TransmitRate = 0;
        RefractRate = 1;
        ReflectFactor = 10;
        ambientStrength.ChangeValue(0.1, 0.1, 0.1);
        diffuseStrength.ChangeValue(0.8, 0.3, 0.3);
        specularStrength.ChangeValue(0.1, 0.1, 0.1);
        ObjectColor.ChangeValue(255, 0, 0);
    }
    void texture_init(const char* filename) { texture = imread(filename); };
    void init(const char* filename);
    void setvalue(double norm_x, double norm_y, double norm_z, double x, double y, double z)
    {
        normal.ChangeValue(norm_x, norm_y, norm_z);
        normal = normal.Normalize();
        point.ChangeValue(x, y, z);
    }
    double GetIntersectParami(Vector3D StartPoint, Vector3D direction);
    Vector3D GetNormal() { return normal; }
    Vector3D Local(int num, Vector3D PointPos, Vector3D LightPos, Vector3D Viewdirect, Vector3D normal, Vector3D LightColor);
};

class Room
{
//房间将自动与视口对齐
private:
    double Height;
    double Width;
    double Depth;
public:
    Plane plane[6];
    Room(double height, double width, double depth)
    {
        Height = height;
        Width = width;
        Depth = depth;
    }
    void init(const char* filename1, const char* filename2, const char* filename3, const char* filename4, const char* filename5, const char* filename6, const char* text_filename1, const char* text_filename2, const char* text_filename3,const char* text_filename4);
    void SetPlanes();
    double GetIntersectParami(Vector3D StartPoint, Vector3D direction,int& plane_num);
};

#endif OBJECT_H//OBJECT_H

