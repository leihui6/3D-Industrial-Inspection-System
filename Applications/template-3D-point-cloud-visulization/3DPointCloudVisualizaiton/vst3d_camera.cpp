#include "vst3d_camera.h"

VST3D_Camera::VST3D_Camera()
{

}

int VST3D_Camera::init(const std::string &path)
{
    m_path = path;
    int result = VST3D_Init(m_path.c_str());
    if (result != VST3D_RESULT_OK)
    {
#ifdef _DEBUG
        cerr << "Could not Start Scanner software" << endl;
#endif
        VST3D_Exit();
        return VST3D_RESULT_ERROR;
    }

    result = connect_align();
    if (VST3D_RESULT_OK != result)
        return VST3D_RESULT_ERROR;

    return result;
}

int VST3D_Camera::connect()
{
    int result = VST3D_Connect();     // 正常单幅扫描一次
    if (result != VST3D_RESULT_OK)
    {
#ifdef _DEBUG
        cerr << "Could not connect to Scanner" << endl;
#endif
        VST3D_Exit();
        return VST3D_RESULT_ERROR;
    }
    return result;
}

int VST3D_Camera::connect_align()
{
    // 正常单幅扫描一次，并识别标记点，同时进行点云旋转对齐
    int result = VST3D_Connect_ALIGN();
    if (result != VST3D_RESULT_OK)
    {
#ifdef _DEBUG
        cerr << "Could not connect to Scanner." << endl;
#endif
        VST3D_Exit();
        return VST3D_RESULT_ERROR;
    }
    return result;
}

int VST3D_Camera::del_background(std::vector<float> &plane_func)
{
    // 设置扫描基准平面后，调用去除基准面以下的杂点
    float* plane = nullptr;
    int result = VST3D_DelBackground(&plane);

    if (result != VST3D_RESULT_OK)
    {
#ifdef _DEBUG
        cerr << "Could not Set backgroud palne parameters."<<endl;
#endif
        VST3D_Exit();
        return -1;
    }

    float cen[3] = { 0.0f };  // 平面的中心坐标
    float normals[3] = { 0.0f }; // 平面的法向量

    cen[0] = plane[3];
    cen[1] = plane[4];
    cen[2] = plane[5];
    normals[0] = plane[0];
    normals[1] = plane[1];
    normals[2] = plane[2];

    plane_func.push_back(cen[0]);
    plane_func.push_back(cen[1]);
    plane_func.push_back(cen[2]);
    plane_func.push_back(normals[0]);
    plane_func.push_back(normals[1]);
    plane_func.push_back(normals[2]);

    return result;
}

int VST3D_Camera::scan()
{
    // 开始单次扫描
    int result = VST3D_Scan();
    if (result != VST3D_RESULT_OK)
    {
#ifdef _DEBUG
        cerr << "Could not scan" << endl;
#endif
        VST3D_Exit();
        return VST3D_RESULT_ERROR;
    }
    return result;
}

int VST3D_Camera::get_point_cloud(std::vector<point_3d> &pointcloud)
{
    VST3D_PT * p_pointcloud = nullptr;
    int totalNum = 0;

    int result = get_num_points(totalNum);  // 得到当前单次扫描总点数
    if (result != 0)
    {
        // 扫描不成功
        return -1;
    }

    // 得到当前单次扫描总点数和点云
    VST3D_GetPointClouds(totalNum, &p_pointcloud);

    // 遍历单次采集到的所有点云，得到点云坐标等信息，通过索引方式[0 1 2 ....]
    for (int i = 0; i < totalNum; i++)
    {
        VST3D_PT * pt = nullptr;
        point_3d p;
        get_each_point_by_index(i, &pt);
        p.x = pt->x;
        p.y = pt->y;
        p.z = pt->z;
        pointcloud.push_back(p);
    }

    return result;
}

int VST3D_Camera::get_each_point_by_index(int nIndex, VST3D_PT ** pt)
{
    return VST3D_GetEachPointByIndex(nIndex, pt);
}

int VST3D_Camera::get_num_points(int &nPts)
{
    return VST3D_GetNumPoints(nPts);
}

int VST3D_Camera::clear()
{
    return VST3D_ClearAllData();
}

int VST3D_Camera::reset()
{
    return VST3D_Reset(m_path.c_str());
}

int VST3D_Camera::exit()
{
    return VST3D_Exit();
}
