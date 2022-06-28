#ifndef VST3D_CAMERA_H
#define VST3D_CAMERA_H

#include "camera_3d.h"
#include "VisionBooster.h"

class VST3D_Camera : public camera_3d_com
{
public:
    VST3D_Camera();

    ~VST3D_Camera(){}

    //! 初始化相机
    /*!
    \param[in] path VST软件路径
    */
    virtual int init(const std::string  & path);

    virtual int get_point_cloud(std::vector<point_3d> & point_cloud);

private:
    //! 尝试连接
    /*!
    \retval VST3D_RESULT_ERROR 成功
    \retval VST3D_RESULT_OK 失败
    */
    int connect();

    //! 以拼接的形式连接
    /*!
    \retval VST3D_RESULT_ERROR 成功
    \retval VST3D_RESULT_OK 失败
    */
    int connect_align();

    //! 消除基准面
    /*!
    \param[out] plane_func 基准面方程(点法式)
    \retval VST3D_RESULT_ERROR 成功
    \retval VST3D_RESULT_OK 失败
    */
    int del_background(std::vector<float> &plane_func);

    //! 触发扫描
    /*!
    \retval VST3D_RESULT_ERROR 成功
    \retval VST3D_RESULT_OK 失败
    */
    int scan();

    //! 获取上一帧的点云数据
    /*!
    \param[out] totalNum 点云点集数量
    \param[out] pPointClouds 点云
    \retval VST3D_RESULT_ERROR 成功
    \retval VST3D_RESULT_OK 失败
    */
    //int get_point_cloud(std::vector<point_3d> &pointcloud);

    //! 清楚点云数据
    /*!
    \retval VST3D_RESULT_ERROR 成功
    \retval VST3D_RESULT_OK 失败
    */
    int clear();

    int reset();

    int exit();
public:

    //! 获取点云大小
    /*!
    \param[out] nPts 用于获取大小的值
    \retval VST3D_RESULT_ERROR 成功
    \retval VST3D_RESULT_OK 失败
    */
    int	get_num_points(int &nPts);

    //! 索引点云
    /*!
    \param[in] nIndex 点云索引值
    \param[out] pt 点云
    \retval VST3D_RESULT_ERROR 成功
    \retval VST3D_RESULT_OK 失败
    */
    int get_each_point_by_index(int nIndex, VST3D_PT** pt);

private:
    std::string m_path;

};

#endif // VST3D_CAMERA_H
