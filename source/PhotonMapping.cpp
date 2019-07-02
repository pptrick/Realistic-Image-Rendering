#include "PhotonMapping.h"

#define PI 3.1415926535

using namespace std;

bool comparison_x(Vector3D a, Vector3D b) { return a.x < b.x; }
bool comparison_y(Vector3D a, Vector3D b) { return a.y < b.y; }
bool comparison_z(Vector3D a, Vector3D b) { return a.z < b.z; }

//kd-tree初始化
void PhotonMap::init_tree(vector<Vector3D> Photons)
{
    int depth = 0;
    node *p = new node;
    root = p;
    int central;

    sort(Photons.begin(), Photons.end(), comparison_x);
    central = Photons.size() / 2;
    p->node_x = Photons[central].x; p->node_y = Photons[central].y; p->node_z = Photons[central].z;
    p->direction = 0;

    vector<Vector3D> left;
    vector<Vector3D> right;
    for (int i = 0; i < Photons.size(); i++)
    {
        if (Photons[i].x < Photons[central].x)left.push_back(Photons[i]);
        else if (Photons[i].x >= Photons[central].x&&i != central)right.push_back(Photons[i]);
    }

    depth++;
    build_tree(depth, left, p->l_child);
    build_tree(depth, right, p->r_child);

    left.clear();
    right.clear();
}

//迭代构建kd-tree
bool PhotonMap::build_tree(int depth, vector<Vector3D> Photons, node *&p)
{
    p = new node;
    if (Photons.empty()) { p = NULL; return 0; }
    int central;
    vector<Vector3D> left;
    vector<Vector3D> right;
    if (depth % 3 == 1)
    {
        sort(Photons.begin(), Photons.end(), comparison_y);
        central = Photons.size() / 2;
        p->node_x = Photons[central].x; p->node_y = Photons[central].y; p->node_z = Photons[central].z;
        p->direction = 1;
        for (int i = 0; i < Photons.size(); i++)
        {
            if (Photons[i].y < Photons[central].y)left.push_back(Photons[i]);
            else if (Photons[i].y >= Photons[central].y&&i!=central)right.push_back(Photons[i]);
        }
    }
    else if (depth % 3 == 2)
    {
        sort(Photons.begin(), Photons.end(), comparison_z);
        central = Photons.size() / 2;
        p->node_x = Photons[central].x; p->node_y = Photons[central].y; p->node_z = Photons[central].z;
        p->direction = 2; 
        for (int i = 0; i < Photons.size(); i++)
        {
            if (Photons[i].z < Photons[central].z)left.push_back(Photons[i]);
            else if (Photons[i].z >= Photons[central].z&&i!=central)right.push_back(Photons[i]);
        }
    }
    else
    {
        sort(Photons.begin(), Photons.end(), comparison_x);
        central = Photons.size() / 2;
        p->node_x = Photons[central].x; p->node_y = Photons[central].y; p->node_z = Photons[central].z;
        p->direction = 0; 
        for (int i = 0; i < Photons.size(); i++)
        {
            if (Photons[i].x < Photons[central].x)left.push_back(Photons[i]);
            else if (Photons[i].x >= Photons[central].x&&i!=central)right.push_back(Photons[i]);
        }
    }

    build_tree(depth + 1, left, p->l_child);
    build_tree(depth + 1, right, p->r_child);
}

//判断光子是否在范围内
bool PhotonMap::inrange(Vector3D Goal, double range, node* p)
{
    return p->node_x < Goal.x + range && p->node_x > Goal.x - range && p->node_y < Goal.y + range && p->node_y > Goal.y - range && p->node_z < Goal.z + range && p->node_z > Goal.z - range;
}

//光子数查找函数
void PhotonMap::RegionFind(Vector3D Goal,double range, node* p, int& photon_num)
{
    //if (p != NULL)cout << p->node_y << endl;
    if (p == NULL)return;
    else if (inrange(Goal, range, p))
    {
        photon_num++;
        RegionFind(Goal, range, p->l_child, photon_num);
        RegionFind(Goal, range, p->r_child, photon_num);
    }
    else if (p->direction == 0)
    {
        if (Goal.x + range < p->node_x)RegionFind(Goal, range, p->l_child, photon_num);
        else if(Goal.x - range > p->node_x)RegionFind(Goal, range, p->r_child, photon_num);
        else
        {
            RegionFind(Goal, range, p->l_child, photon_num);
            RegionFind(Goal, range, p->r_child, photon_num);
        }
    }
    else if (p->direction == 1)
    {
        if (Goal.y + range < p->node_y)RegionFind(Goal, range, p->l_child, photon_num);
        else if (Goal.y - range > p->node_y)RegionFind(Goal, range, p->r_child, photon_num);
        else
        {
            RegionFind(Goal, range, p->l_child, photon_num);
            RegionFind(Goal, range, p->r_child, photon_num);
        }
    }
    else
    {
        if (Goal.z + range < p->node_z)RegionFind(Goal, range, p->l_child, photon_num);
        else if (Goal.z - range > p->node_z)RegionFind(Goal, range, p->r_child, photon_num);
        else
        {
            RegionFind(Goal, range, p->l_child, photon_num);
            RegionFind(Goal, range, p->r_child, photon_num);
        }
    }
}

//光子散布器
void PhotonMap::SpreadPhoton_ALL(Vector3D LightPos, Sphere ball1, Sphere ball2, Room myroom)
{
    double theta = 0;
    double phi = 0;
    double delta_theta = 0.1;
    double delta_phi = 0.1;
    for (int i = 0; i<(int)180 / delta_theta; i++)
    {
        for (int j = 0; j<(int)360 / delta_phi; j++)
        {
            double x = sin(theta*PI / 180)*sin(phi*PI / 180);
            double z = sin(theta*PI / 180)*cos(phi*PI / 180);
            double y = cos(theta*PI / 180);
            Vector3D direction(x, y, z);
            trace(LightPos, direction.Normalize(), ball1, ball2, myroom);
            phi += delta_phi;
        }
        theta += delta_theta;
        phi = 0;
    }

    /*
    ofstream fout("../output/model.obj");
    for (int i = 0; i < Photons.size(); i++)
    {
        //Photons[i].print();
        if (Photons[i].z>0)fout << 'v' << " " << Photons[i].x << " " << Photons[i].y << " " << Photons[i].z << endl;
    }
    */
    cout << "Spread Photons Success" << endl;
}

/*
void PhotonMap::SpreadPhoton(Vector3D center, double radius, Vector3D LightPos, Sphere ball, Room myroom)
{
    double theta = 0;
    double phi = 0;
    double delta_theta = 0.5;
    double delta_phi = 0.5;
    for(int i=0;i<(int)180/delta_theta;i++)
    {
        for(int j=0;j<(int)360/delta_phi;j++)
        {
            double x = radius * sin(theta*PI / 180)*sin(phi*PI / 180);
            double y = radius * sin(theta*PI / 180)*cos(phi*PI / 180);
            double z = radius * cos(theta*PI / 180);
            Vector3D ball_array(x, y, z);
            Vector3D Light_Center = center - LightPos;
            Vector3D direction = Light_Center + ball_array;
            trace(LightPos, direction.Normalize(), ball, myroom);
            phi += delta_phi;
        }
        theta += delta_theta;
        phi = 0;
    }

    
    ofstream fout("../output/model.obj");
    for (int i = 0; i < Photons.size(); i++)
    {
        //Photons[i].print();
        if(Photons[i].z>0)fout << 'v' << " " << Photons[i].x << " " << Photons[i].y << " " << Photons[i].z << endl;
    }
}
*/

//光子追踪函数
void PhotonMap::trace(Vector3D StartPoint, Vector3D direction, Sphere ball1, Sphere ball2, Room myRoom)
{
    double NearestParami = -1;
    int NearestObject=0;
    int plane_num = 0;
    Vector3D NearestPoint;
    bool InObject = 0;
    FindNear(StartPoint, direction, NearestParami, NearestObject, NearestPoint, InObject, ball1, ball2, myRoom);

    Vector3D normal;
    Vector3D transmitdirect;

    if (NearestObject == 1)
    {
        normal = ball1.GetNormal(NearestPoint.x, NearestPoint.y, NearestPoint.z);
        if (InObject == 0)
            transmitdirect = GetTransBeam(direction, normal, 1, ball1.RefractRate);
        else transmitdirect = GetTransBeam(direction, normal, ball1.RefractRate, 1);
        if (transmitdirect.Length() > 0)trace(NearestPoint, transmitdirect, ball1, ball2, myRoom);
    }
    else if (NearestObject == 2)
    {
        normal = ball2.GetNormal(NearestPoint.x, NearestPoint.y, NearestPoint.z);
        if (InObject == 0)
            transmitdirect = GetTransBeam(direction, normal, 1, ball2.RefractRate);
        else transmitdirect = GetTransBeam(direction, normal, ball2.RefractRate, 1);
        if (transmitdirect.Length() > 0)trace(NearestPoint, transmitdirect, ball1, ball2, myRoom);
    }
    else
    {
        Photons.push_back(NearestPoint);
        return;
    }
}

//光子求交函数
void PhotonMap::FindNear(Vector3D startPoint, Vector3D direction, double& NearestParami, int& NearestObject, Vector3D& NearestPoint, bool& InObject, Sphere ball1, Sphere ball2, Room myRoom)
{
    bool InBall1 = 0;
    bool InBall2 = 0;
    int plane_num;
    //先交于ball1
    NearestParami = ball1.GetIntersectParami(startPoint, direction, InBall1);
    NearestObject = 1;
    InObject = InBall1;
    //先交于ball2
    double buffer = ball2.GetIntersectParami(startPoint, direction, InBall2);
    if ((buffer >= 0 && NearestParami == -1) || (buffer >= 0 && NearestParami >= 0 && buffer<NearestParami))
    {
        NearestParami = buffer;
        NearestObject = 2;
        InObject = InBall2;
    }
    //先交于room
    buffer = myRoom.GetIntersectParami(startPoint, direction, plane_num);
    if ((buffer >= 0 && NearestParami == -1) || (buffer >= 0 && NearestParami >= 0 && buffer<NearestParami))
    {
        NearestParami = buffer;
        NearestObject = 3;
    }
    NearestPoint = startPoint + (direction.Normalize())*NearestParami;
}