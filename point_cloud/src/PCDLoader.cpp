#include "PCDLoader.h"
#include <osg/StateAttribute>
#include <osg/Point>

PCDLoader::PCDLoader(std::string pcd_file)
{
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (pcd_file, cloud) == -1)  // load the file
        std::cerr << "Couldn't read file " << pcd_file << std::endl;
    else
    {
        std::cout << "Loaded " << cloud.width * cloud.height << " data points from " << pcd_file << std::endl;

        geode=osg::ref_ptr<osg::Geode>(new osg::Geode());
        osg::ref_ptr<osg::Geometry> geometry (new osg::Geometry());

        osg::ref_ptr<osg::Vec3Array> vertices (new osg::Vec3Array());
        osg::ref_ptr<osg::Vec4Array> colors (new osg::Vec4Array());

        for (int i=0; i<cloud.points.size(); i++) {
            vertices->push_back (osg::Vec3 (cloud.points[i].x/1000, cloud.points[i].y/1000, cloud.points[i].z/1000));
            //uint32_t rgb_val_;
            //memcpy(&rgb_val_, &(cloud.points[i].rgb), sizeof(uint32_t));

            colors->push_back (osg::Vec4f (1.0f, 0.0f, 0.0f, 0.0f));
        }

        geometry->setVertexArray (vertices.get());
        geometry->setColorArray (colors.get());
        geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

        geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS,0,vertices->size()));
        geometry->getOrCreateStateSet()->setAttribute(new osg::Point(3.0f), osg::StateAttribute::ON);
        geode->addDrawable (geometry.get());
        osg::StateSet* state = geometry->getOrCreateStateSet();
        state->setMode( GL_LIGHTING,osg::StateAttribute::OFF);
    }
}

PCDLoader::~PCDLoader(){}
