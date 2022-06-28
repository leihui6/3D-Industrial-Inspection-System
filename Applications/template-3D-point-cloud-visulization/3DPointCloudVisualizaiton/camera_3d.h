#ifndef CAMERA_3D_H
#define CAMERA_3D_H

#include <vector>
#include <string>

#include "cloud_geometry.h"

class camera_3d_com
{
public:

    camera_3d_com(){}

    virtual ~camera_3d_com() {}

    virtual int init(const std::string  & path) = 0;

    virtual int get_point_cloud(std::vector<point_3d> & points) = 0;
};

#endif // CAMERA_3D_H
