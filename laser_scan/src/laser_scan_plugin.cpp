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
#include <qplugin.h>
#include <QtGui>
#include "urgcppwrapper.h"
#include "dxl.h"
#include "scanner3d.h"

LaserScanPlugin::LaserScanPlugin(QWidget *) : ui(new Ui::LaserScanPlugin){
    ui->setupUi(this);
}

LaserScanPlugin::~LaserScanPlugin(){}

void LaserScanPlugin::scan_slot()
{
    const QString ip = ui->ip_input->text();
    const QString ip_port = ui->ip_port_input->text();
    const QString ttyUSB_dxl = ui->ttyUSB_input->text();

    const QString start_angle = ui->start_angle_input->text();
    const QString end_angle = ui->end_angle_input->text();
    const QString step_angle = ui->step_angle_input->text();

    try
    {
        URGCPPWrapper urg(ip.toStdString(), ip_port.toInt());
        std::cout << urg.getAllInfo() << std::endl;
        Dxl dxl(ttyUSB_dxl.toInt());

        Scanner3d scanner(&urg, &dxl, start_angle.toDouble(), end_angle.toDouble(), step_angle.toDouble());
        scanner.scan();

        // Display point cloud
        osg::ref_ptr<osg::Geode> geode = new osg::Geode;
        scanner.getScan3dGeode(geode, 0.001);
        viewWidget->addNodeToScene(geode);
    }
    catch(const std::runtime_error& e)
    {
        std::cout << e.what() << std::endl;
    }
}

void LaserScanPlugin::GRIPEventSimulationBeforeTimestep()
{
}

void LaserScanPlugin::GRIPEventSimulationAfterTimestep(){}
void LaserScanPlugin::GRIPEventSimulationStart(){}
void LaserScanPlugin::GRIPEventSimulationStop(){}
void LaserScanPlugin::GRIPEventTreeViewSelectionChanged(){}
void LaserScanPlugin::Load(TreeViewReturn* ret, ViewerWidget *viewer)
{
    activeNode = ret;
    std::cout << ret->dType << std::endl;
    viewWidget = viewer;
}

void LaserScanPlugin::GRIPEventPlaybackBeforeFrame() {}

/**
 * \brief called from the main window whenever the simulation history slider is being played
 * This method is executed after every playback time step
 */
void LaserScanPlugin::GRIPEventPlaybackAfterFrame() {}

/**
 * \brief called from the main window whenever the simulation history slider is being played
 * This method is executed at the start of the playback
 */
void LaserScanPlugin::GRIPEventPlaybackStart() {}

/**
 * \brief called from the main window whenever the simulation history slider is being played
 * This method is executed at the end of the playback
 */
void LaserScanPlugin::GRIPEventPlaybackStop() {}

void LaserScanPlugin::Refresh() {}

Q_EXPORT_PLUGIN2(LaserScanPlugin, LaserScanPlugin)
