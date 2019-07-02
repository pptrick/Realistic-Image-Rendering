#pragma once
#ifndef VECTOR3D_H
#define VECTOR3D_H

#include <iostream>
#include <math.h>

using namespace std;

class Vector3D
{
private:
public:
    double x;
    double y;
    double z;
    Vector3D(double input_x, double input_y, double input_z)
    {
        x = input_x;
        y = input_y;
        z = input_z;
    }
    Vector3D()
    {
        x = 0; y = 0; z = 0;
    }
    Vector3D operator+(Vector3D a)
    {
        Vector3D result(x + a.x, y + a.y, z + a.z);
        return result;
    }
    Vector3D operator-(Vector3D a)
    {
        Vector3D result(x - a.x, y - a.y, z - a.z);
        return result;
    }
    Vector3D operator*(double a)
    {
        Vector3D result(a*x, a*y, a*z);
        return result;
    }
    Vector3D operator*(Vector3D a)
    {
        Vector3D result(x*a.x, y*a.y, z*a.z);
        return result;
    }
    void operator=(Vector3D a)
    {
        x = a.x;
        y = a.y;
        z = a.z;
    }
    double dot(Vector3D a)
    {
        return x * a.x + y * a.y + z * a.z;
    }
    Vector3D cross(Vector3D a)
    {
        double result_x = y * a.z - z * a.y;
        double result_y = z * a.x - x * a.z;
        double result_z = x * a.y - y * a.x;
        Vector3D result(result_x, result_y, result_z);
        return result;
    }
    Vector3D Normalize()
    {
        double sum = sqrt(x * x + y * y + z * z);
        Vector3D result(0, 0, 0);
        if (sum == 0)return result;
        result.ChangeValue(x / sum, y / sum, z / sum);
        return result;
    }
    double Length()
    {
        double length = sqrt(x * x + y * y + z * z);
        return length;
    }
    void ChangeValue(double input_x, double input_y, double input_z)
    {
        x = input_x;
        y = input_y;
        z = input_z;
    }
    void print()
    {
        cout << "(" << x << ", " << y << ", " << z << ")" << endl;
    }

};

#endif VECTOR3D_H


