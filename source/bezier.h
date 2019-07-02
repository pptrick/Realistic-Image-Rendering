#pragma once
#ifndef BEZIER_H
#define BEZIER_H
#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <vector>
#include "vector3D.h"
#include "object.h"

//���㱴�������ߺͱ�������ת��Ĺ��ߺ���

//Bezier�����ṹ��
struct CoorBezier
{
    int num;//0ΪFOOT��1ΪBODY
    double t; //Bezier���߲���
    double angle; //Bezier������ת��
    double line_t; //�ཻ�߲���
    Vector3D point; //��������
};

//�������ʽ����
double Comb(int i, int n);

//����˹̹�����㺯��
double Bernstein(int i, int n, double t);

//����˹̹���󵼺���
double Bernstein_t_derivative(int i, int n, double t);

//���������߽�ģ����
Vector3D Point_Bezier(vector<Vector3D> Points, double t);

//������������ʾ����
void Show_Bezier(vector<Vector3D> Points);
void Show_2Bezier(vector<Vector3D> Points1, vector<Vector3D> Points2);

//��תBezier�潨ģ����
Vector3D Point_SurfaceBezier(vector<Vector3D> Points, double t, double angle_d);

//����Bezier�󽻺���
CoorBezier GetIntersect(vector<Vector3D> Points, Vector3D StartPoint, Vector3D direction);

//��������ģ
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



//�����󽻺�����������⣩�����ѽ��
//�󽻴����⣺��δ����ж���������������ѽ��
//������㺯�������ѽ��
//��Χ�м��١����ѽ��

#endif BEZIER_H
