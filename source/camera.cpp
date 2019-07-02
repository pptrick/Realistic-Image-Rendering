#include "camera.h"

using namespace std;
using namespace cv;

//相机初始化函数
void Camera::init(const char* filename)
{
    ifstream fin(filename);
    char label;
    Vector3D buffer_vector;
    double buffer;
    fin >> label;
    if (label = 'c')
    {
        fin >> buffer_vector.x >> buffer_vector.y >> buffer_vector.z;
        ViewPos = buffer_vector;
        fin >> buffer_vector.x >> buffer_vector.y >> buffer_vector.z;
        LightPos = buffer_vector;
        fin >> buffer_vector.x >> buffer_vector.y >> buffer_vector.z;
        mainDirect = buffer_vector;
        fin >> buffer_vector.x >> buffer_vector.y >> buffer_vector.z;
        LightColor = buffer_vector;
        fin >> buffer; reflectloss = buffer;
        fin >> buffer; transmitloss = buffer;
        fin >> buffer; focus = buffer;
        fin >> buffer; MinWeight = buffer;
        fin >> buffer; aperture = buffer;

        cout << "Camera Init Success" << endl;
    }
    else
    {
        cout << "Error: Camera Init failed" << endl;
    }

}

//画图函数
void Camera::TakePhoto(Mat& picture,Sphere ball1, Sphere ball2, Room myRoom, Cup myCup, PhotonMap* Map)
{
    for (int row = 0; row < picture.rows; row++)
    {
        for (int col = 0; col < picture.cols; col++)
        {
            Vector3D Color;
            Vector3D direction(0.5*((double)col - picture.cols / 2), 0.5*((double)row - picture.rows / 2), 1000);
            RayTracing(ViewPos, direction, 1, Color, ball1, ball2, myRoom, myCup, Map);
            //Color.print();
            picture.at<Vec3b>(row, col) = Vec3b(Color.x, Color.y, Color.z)*aperture;
        }
        cout << "Picture Painting, " << row << "/"<<picture.rows<<" has done" << endl;
    }
}

//光线追踪函数
void Camera::RayTracing(Vector3D StartPoint, Vector3D direction, double weight, Vector3D& Color, Sphere ball1, Sphere ball2, Room myRoom, Cup myCup,PhotonMap* Map)
{
    if (weight < MinWeight)
    {
        Color.ChangeValue(0, 0, 0);
    }
    else
    {
        double NearestParami = -1;
        int NearestObject;
        int plane_num = 0;
        CoorBezier Cup_para;
        int rate = 0;
        Vector3D NearestPoint;
        bool InObject = 0;
        FindNearest(StartPoint, direction, NearestParami, NearestObject, NearestPoint, InObject, plane_num, Cup_para,ball1, ball2, myRoom,myCup);
        Vector3D LP = LightPos - NearestPoint;
        if (NearestParami == -1)
        {
            Color.ChangeValue(0, 0, 0);
        }
        else
        {
            Vector3D local;
            Vector3D normal;
            Vector3D reflectdirect;
            Vector3D transmitdirect;
            Vector3D reflectlight;
            Vector3D transmitlight;
            Vector3D Light;
            switch (NearestObject)
            {
            case 1://先交于第一个球
                normal = ball1.GetNormal(NearestPoint.x, NearestPoint.y, NearestPoint.z);
                local = ball1.Locallight(NearestPoint, LightPos, direction, normal, LightColor);
                reflectdirect = GetReflectBeam(direction, normal);
                RayTracing(NearestPoint, reflectdirect, weight*reflectloss, reflectlight, ball1, ball2,myRoom,myCup,Map);
                if(InObject==0) 
                    transmitdirect = GetTransBeam(direction, normal, 1, ball1.RefractRate);
                else transmitdirect = GetTransBeam(direction, normal, ball1.RefractRate, 1);
                if (transmitdirect.Length() > 0)RayTracing(NearestPoint, transmitdirect, weight*transmitloss, transmitlight, ball1, ball2, myRoom, myCup, Map);
                Color = local + reflectlight * ball1.ReflectRate + transmitlight * ball1.TransmitRate;
                break;
            case 2://先交于第二个球
                normal = ball2.GetNormal(NearestPoint.x, NearestPoint.y, NearestPoint.z);
                local = ball2.Locallight(NearestPoint, LightPos, direction, normal, LightColor);
                reflectdirect = GetReflectBeam(direction, normal);
                RayTracing(NearestPoint, reflectdirect, weight*reflectloss, reflectlight, ball1, ball2,myRoom,myCup,Map);
                if (InObject == 0) 
                    transmitdirect = GetTransBeam(direction, normal, 1, ball2.RefractRate);
                else transmitdirect = GetTransBeam(direction, normal, ball2.RefractRate, 1);
                if (transmitdirect.Length() > 0)RayTracing(NearestPoint, transmitdirect, weight*transmitloss, transmitlight, ball1, ball2, myRoom, myCup, Map);
                Color = local + reflectlight * ball2.ReflectRate + transmitlight * ball2.TransmitRate;
                break;
            case 3://先交于墙壁
                normal = myRoom.plane[plane_num - 1].GetNormal();
                Map->RegionFind(NearestPoint, 4, Map->root, rate);
                Light = LightColor * 0.5 + LightColor * (rate / 10);
                if (Light.x > 255)Light.ChangeValue(255, 255, 255);
                local = myRoom.plane[plane_num - 1].Local(plane_num - 1, NearestPoint, LightPos, direction, normal, Light);
                Color = local + reflectlight * myRoom.plane[plane_num - 1].ReflectRate ;
                NearestParami = -1;
                Shading(NearestPoint, LP.Normalize(), NearestParami, NearestObject, NearestPoint, Cup_para, ball1, ball2, myCup);
                if (NearestParami != -1 && NearestParami < LP.Length() - 0.1)Color = Color * 0.4;
                break;
            case 4://先交于杯子
                if (Cup_para.num == 0)normal = myCup.GetNormal(myCup.FootController, Cup_para.t, Cup_para.angle);
                else if(Cup_para.num==1)normal= myCup.GetNormal(myCup.BodyController, Cup_para.t, Cup_para.angle);
                else cout << "Error: Trace Cup Normal Failed" << endl;
                local = myCup.Local(Cup_para, NearestPoint, LightPos, direction, normal, LightColor);
                //reflectdirect = GetReflectBeam(direction, normal);
                //RayTracing(NearestPoint, reflectdirect, weight*reflectloss, reflectlight, ball1, ball2, myRoom, myCup);
                //transmitdirect = GetTransBeam(direction, normal, 1, myCup.RefractRate);
                //if (transmitdirect.Length() > 0)RayTracing(NearestPoint, transmitdirect, weight*transmitloss, transmitlight, ball1, ball2, myRoom, myCup);
                Color = local + reflectlight * myCup.ReflectRate + transmitlight * myCup.TransmitRate;
                break;
            default:
                Color.ChangeValue(0, 0, 0);
                break;
            }
        }
    }
}

//寻找最近交点函数
void Camera::FindNearest(Vector3D startPoint, Vector3D direction,double& NearestParami,int& NearestObject,Vector3D& NearestPoint,bool& InObject, int& plane_num, CoorBezier& Cup_para, Sphere ball1, Sphere ball2, Room myRoom, Cup myCup)
{
    bool InBall1 = 0;
    bool InBall2 = 0;
    //先交于ball1
    NearestParami = ball1.GetIntersectParami(startPoint, direction,InBall1);
    NearestObject = 1;
    InObject = InBall1;
    //先交于ball2
    double buffer = ball2.GetIntersectParami(startPoint, direction,InBall2);
    if ((buffer>=0&&NearestParami==-1)||(buffer>=0&&NearestParami>=0&&buffer<NearestParami))
    {
        NearestParami = buffer;
        NearestObject = 2;
        InObject = InBall2;
    }
    //先交于room
    buffer = myRoom.GetIntersectParami(startPoint, direction,plane_num);
    if ((buffer >= 0 && NearestParami == -1) || (buffer >= 0 && NearestParami >= 0 && buffer<NearestParami))
    {
        NearestParami = buffer;
        NearestObject = 3;
    }
    
    //先交于Cup
    CoorBezier Cup_buffer = myCup.GetIntersectParami(startPoint, direction);
    if ((Cup_buffer.line_t >= 0 && NearestParami == -1) || (Cup_buffer.line_t >= 0 && NearestParami >= 0 && Cup_buffer.line_t < NearestParami))
    {
        NearestParami = Cup_buffer.line_t;
        Cup_para = Cup_buffer;
        NearestObject = 4;
    }
    
    
    NearestPoint = startPoint + (direction.Normalize())*NearestParami;
}

//阴影计算函数
void Camera::Shading(Vector3D startPoint, Vector3D direction, double& NearestParami, int& NearestObject, Vector3D& NearestPoint, CoorBezier& Cup_Para, Sphere ball1, Sphere ball2, Cup myCup)
{
    bool InBall1 = 0;
    bool InBall2 = 0;
    /*
    //先交于ball1
    NearestParami = ball1.GetIntersectParami(startPoint, direction, InBall1);
    NearestObject = 1;
    //先交于ball2
    double buffer = ball2.GetIntersectParami(startPoint, direction, InBall2);
    if ((buffer >= 0 && NearestParami == -1) || (buffer >= 0 && NearestParami >= 0 && buffer<NearestParami))
    {
        NearestParami = buffer;
        NearestObject = 2;
    }
    */
    
    //先交于Cup
    CoorBezier Cup_buffer = myCup.GetIntersectParami(startPoint, direction);
    if (Cup_buffer.line_t >= 0)
    {
    NearestParami = Cup_buffer.line_t;
    Cup_Para = Cup_buffer;
    NearestObject = 4;
    }
    
    NearestPoint = startPoint + (direction.Normalize())*NearestParami;
}

//以上三个函数在以下方面需进行调整：
//1、颜色上限值存储，避免出现颜色超限情况
//2、模型存储方式