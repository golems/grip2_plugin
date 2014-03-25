#include "laser_scan_plugin.h"
#include <iostream>
#include <qplugin.h>
#include <QtGui>

LaserScanPlugin::LaserScanPlugin(QWidget *parent) : ui(new Ui::LaserScanPlugin){
    ui->setupUi(this);
}

LaserScanPlugin::~LaserScanPlugin(){}

void LaserScanPlugin::scan_slot()
{
    QStringList fileNames; //stores the entire path of the file that it attempts to open

    QStringList filters; //setting file filters
    filters << "Point Cloud file (*.pcd)"
            << "Any files (*)";

    //initializing the File dialog box
    //the static QFileDialog does not seem to be working correctly in Ubuntu 12.04 with unity.
    //as per the documentation it may work correctly with gnome
    //the method used below should work correctly on all desktops and is supposedly more powerful
    QFileDialog dialog(this);
    dialog.setNameFilters(filters);
    dialog.setAcceptMode(QFileDialog::AcceptOpen);
    dialog.setFileMode(QFileDialog::ExistingFile);
    if (dialog.exec())
        fileNames = dialog.selectedFiles();

//    if (!fileNames.isEmpty())
//    {
//        std::cerr<<"Attempting to open the following world file: "<<fileNames.front().toStdString() <<std::endl;
//        loader = new PCDLoader(fileNames.front().toStdString());
//        viewWidget->addNodeToScene(loader->geode);
//    }
}

void LaserScanPlugin::GRIPEventSimulationBeforeTimestep()
{
    std::cout<<"This is a test"<<std::endl;
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
