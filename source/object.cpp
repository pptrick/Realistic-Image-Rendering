#include "object.h"

#define ERRORLIMIT 1000 //球表面误差范围
#define PI 3.1415926535

//Phong局部模型渲染（单光源）
Vector3D Object::Locallight(Vector3D PointPos, Vector3D LightPos, Vector3D Viewdirect, Vector3D normal, Vector3D LightColor)
{
    Vector3D ambient;
    Vector3D diffuse;
    Vector3D specular;
    //ambient:
    ambient = LightColor * ambientStrength;
    //diffuse:
    Vector3D inputlight = LightColor;
    Vector3D lightdirect = PointPos-LightPos;
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

//球体初始化函数
void Sphere::init(const char* filename)
{
    ifstream fin(filename);
    char label;
    int buffer_int;
    double buffer;
    Vector3D buffer_vector;
    fin >> label;
    if (label == 's')
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

        cout << "Sphere Init Success" << endl;
    }
    else
    {
        cout << "Error: Sphere Init Failed" << endl;
    }

}

//判断点是否在球面上（误差范围内）
bool Sphere::OnSphere(double input_x, double input_y, double input_z)
{
    double delta = sqrt((input_x - Center.x)*(input_x - Center.x) + (input_y - Center.y)*(input_y - Center.y) + (input_z - Center.z)*(input_z - Center.z)) - Radius;
    double minValue = Radius / ERRORLIMIT;
    return delta < minValue;
}

//判断点是否在球内部（误差范围内）
bool Sphere::InSphere(double input_x, double input_y, double input_z)
{
    double delta = Radius - sqrt((input_x - Center.x)*(input_x - Center.x) + (input_y - Center.y)*(input_y - Center.y) + (input_z - Center.z)*(input_z - Center.z));
    double minValue= Radius / ERRORLIMIT;
    return delta >= minValue;
}

//获取球面上某点的法向量
Vector3D Sphere::GetNormal(double input_x, double input_y, double input_z)
{
    Vector3D result(0, 0, 0);
    if(OnSphere(input_x, input_y, input_z))
    {
        result.x = input_x - Center.x;
        result.y = input_y - Center.y;
        result.z = input_z - Center.z;
        result = result.Normalize();
    }
    else
    {
        cout << "Error: The Point (" << input_x << "," << input_y << "," << input_z << ") is not on the sphere" << endl;
    }
    return result;
}

//求直线与球体交点参数t
double Sphere::GetIntersectParami(Vector3D StartPoint, Vector3D direction,bool& InObject)
{
    Vector3D Direction = direction.Normalize();
    Vector3D start_center = Center - StartPoint;
    Vector3D vertical = Direction * (start_center.dot(Direction)) - start_center;
    if (vertical.Length() > Radius)
    {
        return -1;
    }
    if (!InSphere(StartPoint.x, StartPoint.y, StartPoint.z))
    {
        if (direction.dot(start_center) < 0)return -1;
    }
    Vector3D v_i = Direction * (-sqrt(Radius*Radius - vertical.Length()*vertical.Length()));
    Vector3D result = start_center + vertical + v_i;
    if (OnSphere(StartPoint.x, StartPoint.y, StartPoint.z) || InSphere(StartPoint.x, StartPoint.y, StartPoint.z))
    {
        result = start_center + vertical - v_i;
        InObject = 1;
    }
    return result.Length();
}

//平面初始化函数
void Plane::init(const char* filename)
{
    ifstream fin(filename);
    char label;
    int buffer_int;
    double buffer;
    Vector3D buffer_vector;
    fin >> label;
    if (label == 'p')
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
    }
}

//平面局部光照（含纹理映射）
Vector3D Plane::Local(int num, Vector3D PointPos, Vector3D LightPos, Vector3D Viewdirect, Vector3D normal, Vector3D LightColor)
{
    const double Width = 800;
    const double Height = 600;
    const double depth = 1000;
    Vector3D color_result = LightColor;
    Vec3b color;
    double color_x;
    double color_y;

    switch (num)
    {
    case 0:
        color_y = (int)((texture.rows-1) * (((int)(PointPos.x + 400)) / Width));
        color_x = (int)((texture.cols-1) * ((((int)abs(PointPos.z) % 1000))) / depth);
        color = texture.at<Vec3b>(color_y, color_x);
        color_result.ChangeValue(color[0], color[1], color[2]);
        break;
    case 1:
        color_y = (int)((texture.rows-1) * (((int)(PointPos.x + 400))/ Width));
        color_x = (int)((texture.cols-1) * ((((int)abs(PointPos.z) % 1000))) / depth);
        color = texture.at<Vec3b>(color_y, color_x);
        color_result.ChangeValue(color[0], color[1], color[2]);
        break;
    case 2:
        color_y = (int)((texture.rows-1) * (PointPos.y+300) / Height);
        color_x = (int)((texture.cols-1) * ((int)abs(PointPos.z)%1000) / depth);
        color = texture.at<Vec3b>(color_y, color_x);
        color_result.ChangeValue(color[0], color[1], color[2]);
        break;
    case 3:
        color_y = (int)((texture.rows-1) * (PointPos.y+300) / Height);
        color_x = (int)((texture.cols-1) * ((int)abs(PointPos.z) % 1000) / depth);
        color = texture.at<Vec3b>(color_y, color_x);
        color_result.ChangeValue(color[0], color[1], color[2]);
        break;
    case 4:
        color_y = (int)((texture.rows-1) * (PointPos.y + 300) / Height);
        color_x = (int)((texture.cols-1) * (PointPos.x + 400) / depth);
        color = texture.at<Vec3b>(color_y, color_x);
        color_result.ChangeValue(color[0], color[1], color[2]);
        break;
    default:
        break;
    }

    color_result = color_result * (LightColor.x / 255);

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

//平面求交函数
double Plane::GetIntersectParami(Vector3D StartPoint, Vector3D direction)
{
    Vector3D start_point = point - StartPoint;
    Vector3D vertical = normal * (start_point.dot(normal));
    if (direction.dot(vertical) <= 0)return -1;
    else return vertical.Length()*direction.Length() / abs(direction.dot(normal));
}

//房间初始化函数
void Room::init(const char* filename1, const char* filename2, const char* filename3, const char* filename4, const char* filename5, const char* filename6,const char* text_filename1,const char* text_filename2,const char* text_filename3,const char* text_filename4)
{
    SetPlanes();
    plane[0].init(filename1);
    plane[1].init(filename2);
    plane[2].init(filename3);
    plane[3].init(filename4);
    plane[4].init(filename5);
    plane[5].init(filename6);

    plane[2].texture_init(text_filename1);
    plane[3].texture_init(text_filename1);
    plane[1].texture_init(text_filename2);
    plane[4].texture_init(text_filename3);
    plane[0].texture_init(text_filename4);
    cout << "Room Init Success" << endl;
}

//房间设置平面参数函数
void Room::SetPlanes()
{
    plane[0].setvalue(0, 1, 0, 0, -Height / 2, Depth / 2);
    plane[1].setvalue(0, -1, 0, 0, Height / 2, Depth / 2);
    plane[2].setvalue(1, 0, 0, -Width / 2, 0, Depth / 2);
    plane[3].setvalue(-1, 0, 0, Width / 2, 0, Depth / 2);
    plane[4].setvalue(0, 0, -1, 0, 0, Depth);
    plane[5].setvalue(0, 0, 1, 0, 0, 0);
}

//房间求交函数（用于比较各平面求交参数的大小）
double Room::GetIntersectParami(Vector3D StartPoint, Vector3D direction,int& plane_num)
{
    double dist[6];
    for (int i = 0; i < 5; i++)
    {
        dist[i] = plane[i].GetIntersectParami(StartPoint, direction);
    }
    double min_dist = 999999999;
    int num_plane =0;
    for (int i = 0; i < 5; i++)
    {
        if (dist[i] < min_dist&&dist[i]>0)
        {
            min_dist = dist[i];
            num_plane = i + 1;
        }
    }
    plane_num = num_plane;
    return min_dist;
}