#include "scanner3d.h"
#include "urgtoosg.h"
#include <osg/Point>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

Scanner3d::Scanner3d(URGCPPWrapper* urg, Dxl* dxl,
                     int start_angle_degree, int end_angle_degree, double scan_step_degree)
    : urg(urg), dxl(dxl),
      start_angle_degree(start_angle_degree), end_angle_degree(end_angle_degree), scan_step_degree(scan_step_degree)
{
    updateScanParam();
}


void Scanner3d::scan()
{
    moveHeadToInitialPosition();

    // Start measurement
    urg->start(false);

    raw_scan3d_result.distances.clear();

    for(unsigned int i=0 ; i<raw_scan3d_result.number_of_scans ; ++i)
    {

        dxl->moveToDegree(start_angle_degree - i * scan_step_degree, 3);

        raw_scan3d_result.jointsValue[3 * i] = dxl->getCurrentAngleRadian(1);
        raw_scan3d_result.jointsValue[3 * i + 1] = dxl->getCurrentAngleRadian(2);
        raw_scan3d_result.jointsValue[3 * i + 2] = dxl->getCurrentAngleRadian(3);

        //Launch scan
        urg->grabScan();
        // Add scan result to distance vector
        raw_scan3d_result.distances.insert(raw_scan3d_result.distances.end(), urg->getDistance().begin(), urg->getDistance().end());
    }

    // Stop measurement
    urg->stop();

}

void Scanner3d::getScan3dGeode(osg::ref_ptr<osg::Geode> geode)
{
    osg::ref_ptr<osg::Vec3Array> vertices(new osg::Vec3Array());
    UrgToOsg::getOsg3DPointsts(urg, vertices, raw_scan3d_result);

    osg::ref_ptr<osg::Geometry> geometry(new osg::Geometry);
    geometry->setVertexArray(vertices);
    geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, vertices->size()));
    geometry->getOrCreateStateSet()->setAttribute(new osg::Point(1), osg::StateAttribute::ON);

    // Color
    osg::ref_ptr<osg::Vec4Array> color(new osg::Vec4Array(1));
    geometry->setColorBinding(osg::Geometry::BIND_OVERALL);
    (*color)[0] = osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f);
    geometry->setColorArray(color);

    geometry->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

    geode->addDrawable(geometry);
}

void Scanner3d::setScanParameters(int start_angle_degree, int end_angle_degree, double scan_step_degree)
{
    this->start_angle_degree = start_angle_degree;
    this->end_angle_degree = end_angle_degree;
    this->scan_step_degree = scan_step_degree;

    updateScanParam();
}

void Scanner3d::updateScanParam()
{
    // Hubo's head has 3 dx motors
    raw_scan3d_result.number_of_joints = 3;

    raw_scan3d_result.number_of_scans = abs(start_angle_degree - end_angle_degree) / scan_step_degree;
    raw_scan3d_result.number_of_points_per_scan = urg->getNumberOfPoints();
    raw_scan3d_result.number_of_points = raw_scan3d_result.number_of_scans * raw_scan3d_result.number_of_points_per_scan;

    raw_scan3d_result.distances.reserve(raw_scan3d_result.number_of_points);
    raw_scan3d_result.jointsValue.resize(raw_scan3d_result.number_of_scans * raw_scan3d_result.number_of_joints);
}

void Scanner3d::moveHeadToInitialPosition()
{
    // Move to initial position
    dxl->moveToDegree(180, 1);
    dxl->moveToDegree(180, 2);
    dxl->moveToDegree(start_angle_degree, 3);

    // Wait end of move
    while(dxl->isMoving(1)){asm("nop");}
    while(dxl->isMoving(2)){asm("nop");}
    while(dxl->isMoving(3)){asm("nop");}

}
