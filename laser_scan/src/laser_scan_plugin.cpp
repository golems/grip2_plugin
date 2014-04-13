/*
* Copyright (c) 2014, Georgia Tech Research Corporation
* All rights reserved.
*
* Author: Paul Bernier <bernier.pja@gmail.com>
* Date: Jan 2014
*
* Humanoid Robotics Lab Georgia Institute of Technology
* Director: Mike Stilman http://www.golems.org
*
*
* This file is provided under the following "BSD-style" License:
* Redistribution and use in source and binary forms, with or
* without modification, are permitted provided that the following
* conditions are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
*
* * Neither the name of the Humanoid Robotics Lab nor the names of
* its contributors may be used to endorse or promote products
* derived from this software without specific prior written
* permission
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
* CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
* INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
* CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
* LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
* USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/

#include "laser_scan_plugin.h"
#include <iostream>
#include "urgcppwrapper.h"
#include "dxl.h"
#include "scanner3d.h"
#include "meshbuilder.h"

#include <dart/dynamics/BodyNode.h>
#include <dart/dynamics/FreeJoint.h>
#include <dart/dynamics/MeshShape.h>
#include <dart/dynamics/Skeleton.h>
#include <dart/constraint/ConstraintDynamics.h>

LaserScanPlugin::LaserScanPlugin(QWidget *) : ui(new Ui::LaserScanPlugin){
    ui->setupUi(this);
}

LaserScanPlugin::~LaserScanPlugin(){}

void LaserScanPlugin::scan_slot()
{
    // Laser params
    const QString ip = ui->ip_input->text();
    const QString ip_port = ui->ip_port_input->text();
    const QString min_detection_angle = ui->min_detection_angle->text();
    const QString max_detection_angle = ui->max_detection_angle->text();

    // Dxl params
    const QString ttyUSB_dxl = ui->ttyUSB_input->text();
    const QString start_angle = ui->start_angle_input->text();
    const QString end_angle = ui->end_angle_input->text();
    const QString step_angle = ui->step_angle_input->text();

    try
    {
        // Laser
        URGCPPWrapper urg(ip.toStdString(), ip_port.toInt());
        urg.setDetectionAngleDegree(min_detection_angle.toInt(), max_detection_angle.toInt());
        std::cout << urg.getAllInfo() << std::endl;

        // Motors
        Dxl dxl(ttyUSB_dxl.toInt());

        Scanner3d scanner(&urg, &dxl, start_angle.toDouble(), end_angle.toDouble(), step_angle.toDouble());
        scanner.scan();

        // Get pcl point cloud
        pcl::PointCloud<pcl::PointXYZ> pcl_point_cloud;
        scanner.getPointCloud(pcl_point_cloud);
        // Scale point cloud
        scalePointCloudToWorld(pcl_point_cloud);

        // Create a mesh and save as obj file
        pcl::PointCloud<pcl::PointXYZ>::Ptr pc_ptr = pcl_point_cloud.makeShared();
        MeshBuilder::buildAndSave(pc_ptr, 0.2, tmp_filename);

        // Create skeleton from mesh (loaded from obj file)
        dart::dynamics::Skeleton* point_cloud_skel = createSkeletonFromMesh(tmp_filename);
        // Transform to good frame
        changePointCloudFrame(point_cloud_skel);
        // Add to scene
        _world->addSkeleton(point_cloud_skel);

        // Remove collision between the mesh and the ground
         _world->getConstraintHandler()->getCollisionDetector()->disablePair(_world->getSkeleton("ground")->getBodyNode("ground"), _world->getSkeleton("point_cloud")->getBodyNode("rootNode"));
    }
    catch(const std::runtime_error& e)
    {
        std::cout << e.what() << std::endl;
    }
}


void LaserScanPlugin::scalePointCloudToWorld(pcl::PointCloud<pcl::PointXYZ>& pcl_point_cloud)
{
    const size_t size = pcl_point_cloud.points.size();

    for(unsigned int i=0 ; i<size ; ++i)
    {
        // Convert mm to m
        pcl_point_cloud.points[i].x /= 1000;
        pcl_point_cloud.points[i].y /= 1000;
        pcl_point_cloud.points[i].z /= 1000;
    }
}

dart::dynamics::Skeleton* LaserScanPlugin::createSkeletonFromMesh(const std::string& filename)
{
    dart::dynamics::Skeleton* point_cloud = new dart::dynamics::Skeleton("point_cloud");
    const aiScene* model = dart::dynamics::MeshShape::loadMesh(filename);

    if(model)
    {
        dart::dynamics::Shape* shape = new dart::dynamics::MeshShape(Eigen::Vector3d(1, 1, 1), model);

        dart::dynamics::BodyNode* rootNode = new dart::dynamics::BodyNode("rootNode");
        dart::dynamics::FreeJoint* rootJoint = new dart::dynamics::FreeJoint("rootJoint");
        rootNode->setParentJoint(rootJoint);

        rootNode->addCollisionShape(shape);
        rootNode->addVisualizationShape(shape);

        point_cloud->addBodyNode(rootNode);
        point_cloud->setMobile(false);
    }
    else
    {
        std::cerr << "Error loading .obj mesh" << std::endl;
    }

    return point_cloud;
}

void LaserScanPlugin::changePointCloudFrame(dart::dynamics::Skeleton* skeleton)
{
    dart::dynamics::BodyNode* n = _world->getSkeleton("huboplus")->getBodyNode("Body_HNP");
    Eigen::Isometry3d transformWorld = n->getWorldTransform();
    dart::dynamics::Joint* joint = skeleton->getRootBodyNode()->getParentJoint();

    Eigen::Matrix<double, 6, 1> q = dart::math::logMap(transformWorld);
    joint->set_q(q);
}

void LaserScanPlugin::Refresh() {}

Q_EXPORT_PLUGIN2(LaserScanPlugin, LaserScanPlugin)
