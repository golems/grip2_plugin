#ifndef POINT_CLOUD_PLUGIN_H
#define POINT_CLOUD_PLUGIN_H

#include "ui_point_cloud_plugin.h"
#include "PCDLoader.h"
#include <grip/qtWidgets/GripTab.h>
#include <grip/qtWidgets/TreeViewReturn.h>
#include <grip/osgGolems/ViewerWidget.h>

class PointCloudPlugin : public GripTab
{
    Q_OBJECT
    Q_INTERFACES(GripTab)
public:
    PointCloudPlugin(QWidget *parent = 0);
    ~PointCloudPlugin();

private:
    Ui::PointCloudPlugin *ui;
    PCDLoader* loader;

public slots:
    void testslot();

public:
    void Load(TreeViewReturn* ret, ViewerWidget* viewer);

    void GRIPEventSimulationBeforeTimestep();
    void GRIPEventSimulationAfterTimestep();
    void GRIPEventSimulationStart();
    void GRIPEventSimulationStop();
    void GRIPEventPlaybackBeforeFrame();
    void GRIPEventPlaybackAfterFrame();
    void GRIPEventPlaybackStart();
    void GRIPEventPlaybackStop();
    void GRIPEventTreeViewSelectionChanged();
    void Refresh();

};

#endif // TESTGRIPPLUGIN_H
