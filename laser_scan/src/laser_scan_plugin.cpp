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
        scanner.getScan3dGeode(geode);
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
