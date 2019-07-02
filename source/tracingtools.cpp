#include "tracingtools.h"

using namespace std;

//获取透射光方向
Vector3D GetTransBeam(Vector3D incidentBeam, Vector3D normal, double refractRate_in, double refractRate_out)
{
    Vector3D Normal = normal.Normalize();
    double lenth_Normal = -incidentBeam.dot(Normal);
    Normal = Normal * lenth_Normal;
    double refractRate = refractRate_in / refractRate_out;
    if (lenth_Normal == 0 && refractRate_in<=refractRate_out) //相切射入情形
    {
        Vector3D x = incidentBeam.Normalize()*refractRate_in;
        Vector3D y = normal.Normalize()*(-sqrt(refractRate_out*refractRate_out - refractRate_in * refractRate_in));
        Vector3D result = x + y;
        result = result.Normalize();
        return result;
    }
    double cosine1 = abs(incidentBeam.dot(Normal)) / (incidentBeam.Length()*Normal.Length());
    double refractJudger = 1 - refractRate * refractRate*(1 - cosine1 * cosine1);
    if (refractJudger <= 0) //光密进入光疏，全反射情形
    {
        Vector3D result(0, 0, 0);
        return result;
    }
    double cosine2 = sqrt(refractJudger);
    double tanRate = refractRate * cosine1 / cosine2;
    Vector3D result = Normal * (-1) + (incidentBeam + Normal)*tanRate;
    result = result.Normalize();
    return result;
}

//获取反射光方向
Vector3D GetReflectBeam(Vector3D incidentBeam, Vector3D normal)
{
    Vector3D Normal = normal.Normalize();
    double lenth_Normal = -incidentBeam.dot(Normal);
    Normal = Normal * lenth_Normal;
    Vector3D result = (Normal * 2) + incidentBeam;
    result = result.Normalize();
    return result;
}

//点到直线距离计算函数
double Distance_PL(Vector3D Point, Vector3D direction,Vector3D Point_On_Line)
{
    Vector3D i = direction.Normalize();
    Vector3D P_to_PL = Point_On_Line - Point;
    double d = abs(P_to_PL.dot(i));
    double l = P_to_PL.Length();
    return sqrt(l * l - d * d);
}

bool Plane_BBox(double x0, double x1, double y0, double y1, Vector3D StartPoint_Plane, Vector3D direction_Plane)
{
    Vector3D Direction = direction_Plane.Normalize();
    double t_y0 = (y0 - StartPoint_Plane.y) / Direction.y;
    double t_y1 = (y1 - StartPoint_Plane.y) / Direction.y;
    double t_x0 = (x0 - StartPoint_Plane.x) / Direction.x;
    double t_x1 = (x1 - StartPoint_Plane.x) / Direction.x;
    if ((t_y0 < 0 && t_y1 < 0) || (t_x0 < 0 && t_x1 < 0))return false;
    else if (t_y0 < t_y1&&t_x0 < t_x1)
    {
        if (t_y1 <= t_x0 || t_x1 <= t_y0)return false;
    }
    else if (t_y0 < t_y1&&t_x0 >= t_x1)
    {
        if (t_y1 <= t_x1 || t_x0 <= t_y0)return false;
    }
    else if (t_y0 >= t_y1&&t_x0 < t_x1)
    {
        if (t_y0 <= t_x0 || t_x1 <= t_y1)return false;
    }
    else if (t_y0 >= t_y1&&t_x0 >= t_x1)
    {
        if (t_x0 <= t_y1 || t_y0 <= t_x1)return false;
    }
    return true;
}

bool BBox(double x0, double x1, double y0, double y1, double z0, double z1, Vector3D StartPoint, Vector3D direction)
{
    Vector3D sp_xy(StartPoint.x, StartPoint.y, 0);
    Vector3D dr_xy(direction.x, direction.y, 0);
    Vector3D sp_yz(StartPoint.y, StartPoint.z, 0);
    Vector3D dr_yz(direction.y, direction.z, 0);
    Vector3D sp_zx(StartPoint.z, StartPoint.x, 0);
    Vector3D dr_zx(direction.z, direction.x, 0);
    return Plane_BBox(x0, x1, y0, y1, sp_xy, dr_xy) && Plane_BBox(y0, y1, z0, z1, sp_yz, dr_yz) && Plane_BBox(z0, z1, x0, x1, sp_zx, dr_zx);
}
