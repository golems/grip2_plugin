#include "point_cloud_plugin.h"
#include <iostream>
#include <qplugin.h>
#include <QtGui>

PointCloudPlugin::PointCloudPlugin(QWidget *parent) : ui(new Ui::PointCloudPlugin){
    ui->setupUi(this);
}
PointCloudPlugin::~PointCloudPlugin(){}
void PointCloudPlugin::testslot()
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

    if (!fileNames.isEmpty())
    {
        std::cerr<<"Attempting to open the following world file: "<<fileNames.front().toStdString() <<std::endl;
        loader = new PCDLoader(fileNames.front().toStdString());
        _viewWidget->addNodeToScene(loader->geode);
    }
}

void PointCloudPlugin::GRIPEventSimulationBeforeTimestep()
{
    std::cout<<"This is a test"<<std::endl;
}

void PointCloudPlugin::GRIPEventSimulationAfterTimestep(){}
void PointCloudPlugin::GRIPEventSimulationStart(){}
void PointCloudPlugin::GRIPEventSimulationStop(){}
void PointCloudPlugin::GRIPEventTreeViewSelectionChanged(){}
void PointCloudPlugin::Load(TreeViewReturn* ret, ViewerWidget *viewer)
{
    _activeNode = ret;
    _viewWidget = viewer;
}

void PointCloudPlugin::GRIPEventPlaybackBeforeFrame() {}

/**
 * \brief called from the main window whenever the simulation history slider is being played
 * This method is executed after every playback time step
 */
void PointCloudPlugin::GRIPEventPlaybackAfterFrame() {}

/**
 * \brief called from the main window whenever the simulation history slider is being played
 * This method is executed at the start of the playback
 */
void PointCloudPlugin::GRIPEventPlaybackStart() {}

/**
 * \brief called from the main window whenever the simulation history slider is being played
 * This method is executed at the end of the playback
 */
void PointCloudPlugin::GRIPEventPlaybackStop() {}

void PointCloudPlugin::Refresh() {}

Q_EXPORT_PLUGIN2(PointCloudPlugin, PointCloudPlugin)
