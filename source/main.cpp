#include <iostream>
#include <fstream>
#include <cmath>
#include <opencv2/opencv.hpp>

#include "vector3D.h"
#include "tracingtools.h"
#include "object.h"
#include "camera.h"
#include "bezier.h"
#include "PhotonMapping.h"

#define WIDTH 600
#define HEIGHT 800

using namespace std;
using namespace cv;

/*
int main()
{
    Sphere ball1(-200, 180, 150, 120);
    Sphere ball2(300, -100, 600, 100);
    Room myRoom(600, 800, 1000);
    Camera mycamera;
    Cup myCup(50, 0, 650);
    PhotonMap Map;

    const char* filename_ball1 = "../data/parami_ball1.txt";
    const char* filename_ball2 = "../data/parami_ball2.txt";
    const char* filename_plane1 = "../data/parami_plane1.txt";
    const char* filename_plane2 = "../data/parami_plane2.txt";
    const char* filename_plane3 = "../data/parami_plane3.txt";
    const char* filename_plane4 = "../data/parami_plane4.txt";
    const char* filename_plane5 = "../data/parami_plane5.txt";
    const char* filename_plane6 = "../data/parami_plane6.txt";
    const char* filename_camera = "../data/parami_camera.txt";
    const char* filename_Cup = "../data/parami_cup.txt";
    const char* filename_cuptexture = "../data/texture.jpg";
    const char* filename_walltexture = "../data/wall1.jpg";
    const char* filename_floortexture = "../data/floor.jpg";
    const char* filename_backtexture = "../data/back.jpg";
    const char* filename_celingtexture = "../data/celing.jpg";

    ball1.init(filename_ball1);
    ball2.init(filename_ball2);
    myRoom.init(filename_plane1, filename_plane2, filename_plane3, filename_plane4, filename_plane5, filename_plane6, filename_walltexture, filename_floortexture, filename_backtexture, filename_celingtexture);
    myCup.init(filename_Cup);
    myCup.init_texture(filename_cuptexture);
    mycamera.init(filename_camera);

    Vector3D LightPos(0, -200, 500);

    Map.SpreadPhoton_ALL(LightPos, ball1, ball2, myRoom);
    Map.init_tree(Map.Photons);
    PhotonMap* myMap = &Map;

    Mat picture(1200,1600,CV_8UC3);
    mycamera.TakePhoto(picture, ball1, ball2, myRoom, myCup, myMap);

    cout << "success" << endl;
    imwrite("../output/test.png", picture);
    return 0;
}
*/


/*
int main()
{
    Vector3D point1(0, 0, 0);
    Vector3D point2(1, 0, 0);
    Vector3D point3(2, 0, 0);
    Vector3D point4(3, 0, 0);
    Vector3D point5(0, 1, 0);
    Vector3D point6(0, 0, 2);

    vector<Vector3D> Photons;
    Photons.push_back(point1);
    Photons.push_back(point2);
    Photons.push_back(point3);
    Photons.push_back(point4);
    Photons.push_back(point5);
    Photons.push_back(point6);

    PhotonMap Map;
    int num=0;
    Map.init_tree(Photons);
    Map.RegionFind(point1, 6, Map.root, num);
    cout << num << endl;
    return 0;
}
*/

/*
int main()
{
    Plane p(0, 0, 1, 0, 0, 0);
    Vector3D startpoint(0, 0, 0);
    Vector3D direction(0, 0, 1);
    double para = p.GetIntersectParami(startpoint, direction);
    cout << para << endl;
    return 0;
}
*/


int main()
{
    vector<Vector3D> FootController;
    vector<Vector3D> BodyController;
    
    Vector3D point4(88, 90, 0);
    Vector3D point3(68, 55, 0);
    Vector3D point2(88, 20, 0);
    Vector3D point1(140, 0, 0);

    Vector3D pointe(88, 370, 0);
    Vector3D pointd(143, 300, 0);
    Vector3D pointc(148, 230, 0);
    Vector3D pointb(133, 160, 0);
    Vector3D pointa(88, 90, 0);

    FootController.push_back(point1);
    FootController.push_back(point2);
    FootController.push_back(point3);
    FootController.push_back(point4);

    BodyController.push_back(pointa);
    BodyController.push_back(pointb);
    BodyController.push_back(pointc);
    BodyController.push_back(pointd);
    BodyController.push_back(pointe);

    //Vector3D StartPoint(148, 295, 90);
    //Vector3D direction(-1, 0, 0);
    Show_2Bezier(FootController, BodyController);
    //GetIntersect(BodyController, StartPoint, direction).point.print();
    //Show_Bezier(Points1);
    //Point_Bezier(Points1, 0.5).print();
    return 0;
}


/*
int main()
{
    Cup myCup;
    vector<Vector3D> Points1;

    Vector3D point1(8, 0, 0);
    Vector3D point2(8, -50, 0);
    Vector3D point3(8, -100, 0);
    Vector3D point4(8, -150, 0);
    Vector3D point5(43, -163, 0);
    Vector3D point6(83, -175, 0);

    Points1.push_back(point1);
    Points1.push_back(point2);
    Points1.push_back(point3);
    Points1.push_back(point4);
    Points1.push_back(point5);
    Points1.push_back(point6);


    for (double i = 0; i <= 1; i += 0.1)
    {
        myCup.GetNormal(Points1, i, 30).print();
    }
}
*/

//亟待解决的问题：
//1、模型的存储问题，当前需要手动遍历所有模型，非常非常不方便，而且会让函数变得冗长
//2、投射反射综合，解决颜色叠加超限的问题，继续修复一些artifact ——已解决
//3、可开始探索复杂模型求交以及贝塞尔建模——已解决
//4、开始Photon Mapping绘制球体焦散效果