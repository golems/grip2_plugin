#ifndef PCDLOADER_H
#define PCDLOADER_H

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <osg/Geometry>
#include <osg/Geode>

class PCDLoader
{
public:
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    osg::ref_ptr<osg::Geode> geode;
    PCDLoader(std::string pcd_file);
    ~PCDLoader();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr passThroughFilter();
};

#endif // PCDLOADER_H

