#pragma once
#ifndef BEZIER_H
#define BEZIER_H
#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <vector>
#include "vector3D.h"
#include "object.h"

//计算贝塞尔曲线和贝塞尔旋转体的工具函数

//Bezier参数结构体
struct CoorBezier
{
    int num;//0为FOOT，1为BODY
    double t; //Bezier曲线参数
    double angle; //Bezier曲面旋转角
    double line_t; //相交线参数
    Vector3D point; //交点坐标
};

//组合数公式函数
double Comb(int i, int n);

//伯恩斯坦基计算函数
double Bernstein(int i, int n, double t);

//伯恩斯坦基求导函数
double Bernstein_t_derivative(int i, int n, double t);

//贝塞尔曲线建模函数
Vector3D Point_Bezier(vector<Vector3D> Points, double t);

//贝塞尔曲线显示函数
void Show_Bezier(vector<Vector3D> Points);
void Show_2Bezier(vector<Vector3D> Points1, vector<Vector3D> Points2);

//旋转Bezier面建模函数
Vector3D Point_SurfaceBezier(vector<Vector3D> Points, double t, double angle_d);

//单段Bezier求交函数
CoorBezier GetIntersect(vector<Vector3D> Points, Vector3D StartPoint, Vector3D direction);

//贝塞尔建模
class Cup :public Object
{
public:
    Mat texture;
    double trans_x;
    double trans_y;
    double trans_z;
    double x0, x1, y0, y1, z0, z1;
    vector<Vector3D> FootController;
    vector<Vector3D> BodyController;
    Cup(double x, double y, double z) { trans_x = x; trans_y = y; trans_z = z; }
    void init_texture(const char* filename);
    void init(const char* filename);
    Vector3D Local(CoorBezier Point, Vector3D PointPos, Vector3D LightPos, Vector3D Viewdirect, Vector3D normal, Vector3D LightColor);
    CoorBezier GetIntersectParami(Vector3D StartPoint, Vector3D direction);
    Vector3D GetNormal(vector<Vector3D> Points,double t, double angle_d);
};



//光线求交函数（迭代求解）——已解决
//求交大问题：如何处理有多个交点的情况——已解决
//法向计算函数——已解决
//包围盒加速——已解决

#endif BEZIER_H
