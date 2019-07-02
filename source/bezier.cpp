#include "bezier.h"
#include "tracingtools.h"

#define PI 3.1415926535
#define INF 999999999

using namespace std;
using namespace cv;

double Comb(int i, int n)
{
    if (i == n || i == 0)return 1;
    else if (i < n)return Comb(i, n - 1) + Comb(i - 1, n - 1);
    else return 0;
}

double Bernstein(int i, int n, double t)
{
    double first_term = 1;
    double second_term = 1;
    for (int k = 0; k < i; k++)first_term *= t;
    for (int k = 0; k < n - i; k++)second_term *= 1 - t;
    //cout << Comb(i, n)*first_term*second_term << endl;
    return Comb(i, n)*first_term*second_term;
}

double Bernstein_t_derivative(int i, int n, double t)
{
    double first_term = 1;
    double second_term = 1;
    if (i == 0)
    {
        for (int k = 0; k < n - 1; k++)first_term *= 1 - t;
        return -n * first_term;
    }
    else if (n == i)
    {
        for (int k = 0; k < n - 1; k++)first_term *= t;
        return n * first_term;
    }
    for (int k = 0; k < i - 1; k++)first_term *= t;
    for (int k = 0; k < n - i - 1; k++)second_term *= 1 - t;
    return Comb(i, n)*first_term*second_term*(i - n * t);
}

Vector3D Point_Bezier(vector<Vector3D> Points, double t)
{
    Vector3D result(0, 0, 0);
    int n = Points.size();
    for (int i = 0; i < n; i++)result = result + Points[i] * Bernstein(i, n-1, t);
    return result;
}

void Show_Bezier(vector<Vector3D> Points)
{
    Mat BezierTest(800, 600, CV_8UC3);
    namedWindow("BezierTest");

    double delta_t = 0.01;
    double t = 0;
    while (t <= 1)
    {
        Vector3D point;
        point = Point_Bezier(Points, t);
        //point.print();
        BezierTest.at<Vec3b>((int)point.x, (int)point.y) = Vec3b(0, 255, 0);
        t += delta_t;
    }
    for (int i = 0; i < Points.size(); i++)
    {
        BezierTest.at<Vec3b>((int)Points[i].x, (int)Points[i].y) = Vec3b(255, 255, 255);
    }

    imshow("BezierTest", BezierTest);
    waitKey(0);
}

void Show_2Bezier(vector<Vector3D> Points1, vector<Vector3D> Points2)
{
    Mat BezierTest(800, 600, CV_8UC3);
    namedWindow("BezierTest");

    double delta_t = 0.01;
    double t = 0;
    while (t <= 1)
    {
        Vector3D point;
        point = Point_Bezier(Points1, t);
        //point.print();
        BezierTest.at<Vec3b>(600-(int)point.x, (int)point.y) = Vec3b(0, 255, 0);
        t += delta_t;
    }
    for (int i = 0; i < Points1.size(); i++)
    {
        BezierTest.at<Vec3b>(600-(int)Points1[i].x, (int)Points1[i].y) = Vec3b(255, 255, 255);
    }
    t = 0;
    while (t <= 1)
    {
        Vector3D point;
        point = Point_Bezier(Points2, t);
        //point.print();
        BezierTest.at<Vec3b>(600-(int)point.x, (int)point.y) = Vec3b(0, 255, 0);
        t += delta_t;
    }
    for (int i = 0; i < Points2.size(); i++)
    {
        BezierTest.at<Vec3b>(600-(int)Points2[i].x, (int)Points2[i].y) = Vec3b(255, 255, 255);
    }


    //imshow("BezierTest", BezierTest);
    imwrite("../output/Bezier.png",BezierTest);
    //waitKey(0);
}

Vector3D Point_SurfaceBezier(vector<Vector3D> Points, double t, double angle_d)
{
    Vector3D result = Point_Bezier(Points, t);
    //以下计算只针对绕y轴旋转，且Bezier曲线在x-y平面建模的情况
    double l = result.x;
    double y = result.y;
    result.ChangeValue(l*cos(angle_d*PI / 180), y, -l * sin(angle_d*PI / 180));//旋转方向为y的右手螺旋
    return result;
}

CoorBezier GetIntersect(vector<Vector3D> Points, Vector3D StartPoint, Vector3D direction)
{
    //迭代点初始化
    double t_b[8];
    double angle[8];
    for (int i = 0; i < 8; i++)
    {
        t_b[i] = 0.5;
        angle[i] = 45 * i;
    }
    //设置迭代参数
    const int iter = 20000;
    const double learning_rate = 0.0001;
    const double delta_t = 0.001;
    const double delta_angle = 0.01;
    const double dist_limit = 0.2;
    const double para_limit = 1; 
    const double change_limit = 0.001;
    const double t_rate = 0.1;
    const double angle_rate = 3000;
    //开始迭代
    int num = 0;
    vector<CoorBezier> result;
    for (int i = 0; i < 8; i++)
    {
        double buffer_old = 0;
        while (num <= iter)
        {
            Vector3D now_Point = Point_SurfaceBezier(Points, t_b[i], angle[i]);
            double old_dist = Distance_PL(now_Point, direction, StartPoint);
            //cout << i << " " << t_b[i] << " " << angle[i] << " " << old_dist << endl;
            if (abs(old_dist - buffer_old) < change_limit)break;
            if (old_dist < dist_limit)
            {
                CoorBezier buffer_result;
                buffer_result.point = now_Point;
                buffer_result.t = t_b[i];
                buffer_result.angle = angle[i];
                result.push_back(buffer_result);
                break;
            }

            //此处还可优化
            double radius = sqrt(now_Point.x*now_Point.x + now_Point.z*now_Point.z);
            Vector3D new_Point_t = Point_SurfaceBezier(Points, t_b[i] + delta_t, angle[i]);
            Vector3D new_Point_angle = Point_SurfaceBezier(Points, t_b[i], angle[i] + delta_angle);
            double new_dist_t= Distance_PL(new_Point_t, direction, StartPoint);
            double new_dist_angle = Distance_PL(new_Point_angle, direction, StartPoint);
            double Grad_t = (new_dist_t - old_dist) / delta_t;
            double Grad_angle = (new_dist_angle - old_dist) / delta_angle;
            t_b[i] -= learning_rate * old_dist*Grad_t*t_rate;
            if (radius < 100)angle[i] -= learning_rate * old_dist*Grad_angle * angle_rate * 2;
            else angle[i] -= learning_rate * old_dist*Grad_angle * angle_rate;
            if (t_b[i] > 1)t_b[i] = 0.999;
            else if (t_b[i] < 0)t_b[i] = 0.001;

            num++;
            buffer_old = old_dist;
        }
        num = 0;
    }

    Vector3D direct = direction.Normalize();
    double min_t=INF;
    int term = -1;
    for (int i = 0; i < result.size(); i++)
    {
        Vector3D RS = result[i].point - StartPoint;
        double para = RS.Length();
        if (RS.dot(direct) <= 0)para = -para;
        result[i].line_t = para;
        if (para < min_t&&para>para_limit)
        {
            min_t = para;
            term = i;
        }
    }
    if (term==-1||min_t == INF)
    {
        CoorBezier NONE;
        NONE.line_t = -1; 
        return NONE;
    }
    else return result[term];
}

void Cup::init(const char* filename)
{   
    Vector3D point4(88, 20, 0);
    Vector3D point3(68, -15, 0);
    Vector3D point2(88, -50, 0);
    Vector3D point1(140, -70, 0);

    Vector3D pointe(88, 300, 0);
    Vector3D pointd(143, 230, 0);
    Vector3D pointc(148, 160, 0);
    Vector3D pointb(133, 90, 0);
    Vector3D pointa(88, 20, 0);

    FootController.push_back(point1);
    FootController.push_back(point2);
    FootController.push_back(point3);
    FootController.push_back(point4);

    BodyController.push_back(pointa);
    BodyController.push_back(pointb);
    BodyController.push_back(pointc);
    BodyController.push_back(pointd);
    BodyController.push_back(pointe);

    y0 = -90; y1 = 300;
    x0 = -140; x1 = 140;
    z0 = -140; z1 = 140;

    ifstream fin(filename);
    char label;
    int buffer_int;
    double buffer;
    Vector3D buffer_vector;
    fin >> label;
    if (label == 'b')
    {
        fin >> buffer; ReflectRate = buffer;
        fin >> buffer; TransmitRate = buffer;
        fin >> buffer; RefractRate = buffer;
        fin >> buffer_int; ReflectFactor = buffer_int;
        fin >> buffer_vector.x >> buffer_vector.y >> buffer_vector.z;
        ambientStrength = buffer_vector;
        fin >> buffer_vector.x >> buffer_vector.y >> buffer_vector.z;
        diffuseStrength = buffer_vector;
        fin >> buffer_vector.x >> buffer_vector.y >> buffer_vector.z;
        specularStrength = buffer_vector;
        fin >> buffer_vector.x >> buffer_vector.y >> buffer_vector.z;
        ObjectColor = buffer_vector;

        cout << "Cup Bezier Init Success" << endl;
    }
    else
    {
        cout << "Cup Bezier Init Failed" << endl;
    }
}

CoorBezier Cup::GetIntersectParami(Vector3D StartPoint, Vector3D direction)
{
    Vector3D newStartPoint(StartPoint.x - trans_x, StartPoint.y - trans_y, StartPoint.z - trans_z);
    //之后可在此函数中加入包围盒求交
    if (!BBox(x0, x1, y0, y1, z0, z1, newStartPoint, direction))
    {
        CoorBezier NONE;
        NONE.line_t = -1;
        return NONE;
    }
    CoorBezier Foot_Para = GetIntersect(FootController, newStartPoint, direction);
    Foot_Para.num = 0;
    Foot_Para.point.x += trans_x;
    Foot_Para.point.z += trans_z;
    Foot_Para.point.y += trans_y;
    CoorBezier Body_Para = GetIntersect(BodyController, newStartPoint, direction);
    Body_Para.num = 1;
    Body_Para.point.x += trans_x;
    Body_Para.point.z += trans_z;
    Body_Para.point.y += trans_y;
    if (Body_Para.line_t == -1) return Foot_Para;
    else if (Foot_Para.line_t == -1)return Body_Para;
    else if (Body_Para.line_t < Foot_Para.line_t)return Body_Para;
    else if (Body_Para.line_t >= Foot_Para.line_t)return Foot_Para;
    else cout << "Error: Bezier Get Intersect Failed" << endl;
    return Foot_Para;
}

Vector3D Cup::GetNormal(vector<Vector3D> Points,double t, double angle_d)
{
    Vector3D tan(0, 0, 0);
    int n = Points.size();
    for (int i = 0; i < n; i++)tan = tan + Points[i] * Bernstein_t_derivative(i, n - 1, t);
    Vector3D normal(tan.y, -tan.x, 0);
    double n_l = normal.x;
    double n_y = normal.y;
    normal.ChangeValue(n_l*cos(angle_d*PI / 180), n_y, -n_l * sin(angle_d*PI / 180));
    return normal.Normalize();
}

void Cup::init_texture(const char* filename)
{
    texture = imread(filename);
}

Vector3D Cup::Local(CoorBezier Point, Vector3D PointPos, Vector3D LightPos, Vector3D Viewdirect, Vector3D normal, Vector3D LightColor)
{
    int color_x = (int)(((int)Point.angle)%180*(texture.rows - 1) / 180);
    if (Point.angle < 0)color_x = ((int)(360 + Point.angle)%180*(texture.rows-1) / 180);
    int color_y;
    if(Point.num==1)color_y = (int)(Point.t * (texture.cols - 200));
    else color_y = (int)(Point.t * (texture.cols - 200) / 3);
    Vec3b color = texture.at<Vec3b>(color_y, color_x);
    Vector3D color_result(color[0], color[1], color[2]);
    //Vector3D color_result(0, 0, 0);

    Vector3D ambient;
    Vector3D diffuse;
    Vector3D specular;
    //ambient:
    ambient = color_result * ambientStrength;
    //diffuse:
    Vector3D inputlight = color_result;
    Vector3D lightdirect = PointPos - LightPos;
    lightdirect = lightdirect.Normalize();
    Vector3D Normal = normal.Normalize();
    double cosine1 = lightdirect.dot(Normal);//Normal必须向外，否则无法判断阴影(之后修改)
    if (cosine1 < 0)diffuse = (inputlight * diffuseStrength) * (-cosine1);
    else diffuse.ChangeValue(0, 0, 0);
    //specular:
    Vector3D reflectdirect = GetReflectBeam(lightdirect, normal);
    Vector3D viewdirect = Viewdirect.Normalize();
    double cosine2 = reflectdirect.dot(viewdirect);
    if (cosine2 < 0)specular = (LightColor*specularStrength)*pow((-cosine2), ReflectFactor);
    else specular.ChangeValue(0, 0, 0);

    return ambient + diffuse + specular;
}