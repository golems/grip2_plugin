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

#include "urgtopcl.h"

void UrgToPcl::getPCLCloud(URGCPPWrapper* urg, pcl::PointCloud<pcl::PointXYZ>& cloud, const RawScan3dResult &raw_scan3d_result)
{
    const unsigned int nb_pts = raw_scan3d_result.number_of_points;
    const unsigned int nb_joints = raw_scan3d_result.number_of_joints;
    const long max_distance = urg->getMaxDistance() - EPSILON;
    const long min_distance = urg->getMinDistance() + EPSILON;

    // Header cloud
    cloud.width = raw_scan3d_result.number_of_points_per_scan;
    cloud.height = raw_scan3d_result.number_of_scans;
    cloud.is_dense = true;
    cloud.points.resize(cloud.width * cloud.height);

    for(unsigned int i=0 ; i< nb_pts ; ++i)
    {
        const double distance = raw_scan3d_result.distances[i];
        // Remove extreme points
        if(distance < max_distance && distance > min_distance)
        {
            const double phi = raw_scan3d_result.jointsValue[nb_joints * (i / raw_scan3d_result.number_of_points_per_scan + 1) - 1];
            const double theta = urg->index2rad(i % raw_scan3d_result.number_of_points_per_scan) - 3.1415926 / 2;

            cloud.points[i] = sphericalToCartesian(distance, theta, phi);
        }
        else
        {
            cloud.points[i].x = nan("");
            cloud.points[i].y = nan("");
            cloud.points[i].z = nan("");
        }
    }
}

void UrgToPcl::getPCLCloudUnorganized(URGCPPWrapper* urg, pcl::PointCloud<pcl::PointXYZ>& cloud, const RawScan3dResult &raw_scan3d_result)
{
    const unsigned int nb_pts = raw_scan3d_result.number_of_points;
    const unsigned int nb_joints = raw_scan3d_result.number_of_joints;
    const long max_distance = urg->getMaxDistance() - EPSILON;
    const long min_distance = urg->getMinDistance() + EPSILON;

    // Header cloud
    cloud.height = 1;
    cloud.is_dense = false;
    cloud.points.resize(raw_scan3d_result.number_of_points_per_scan * raw_scan3d_result.number_of_scans);

    unsigned long int count = 0;
    for(unsigned int i=0 ; i< nb_pts ; ++i)
    {
        const double distance = raw_scan3d_result.distances[i];

        // Remove extreme points
        if(distance < max_distance && distance > min_distance)
        {
            count++;
            const double phi = raw_scan3d_result.jointsValue[nb_joints * (i / raw_scan3d_result.number_of_points_per_scan + 1) - 1];
            const double theta = urg->index2rad(i % raw_scan3d_result.number_of_points_per_scan) - 3.1415926 / 2;

            cloud.points[i] = sphericalToCartesian(distance, theta, phi);
        }
    }

    // Resize to actual size
    cloud.width = count;
    cloud.points.resize(count);
}

pcl::PointXYZ UrgToPcl::sphericalToCartesian(const long distance, const double theta, const double phi)
{
    return pcl::PointXYZ(distance * cos(phi) * sin(theta),
                         distance * cos(theta),
                         distance * sin(phi) * sin(theta));
}
