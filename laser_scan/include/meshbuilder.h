#ifndef MESHBUILDER_H
#define MESHBUILDER_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class MeshBuilder
{
public:
    MeshBuilder();
    static void build(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::string& filename);
};

#endif // MESHBUILDER_H
