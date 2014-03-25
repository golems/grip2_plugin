#ifndef POINT_CLOUD_PLUGIN_H
#define POINT_CLOUD_PLUGIN_H

#include "ui_laser_scan_plugin.h"
#include "qtWidgets/include/GripTab.h"
#include "qtWidgets/include/TreeViewReturn.h"
#include "osgNodes/include/ViewerWidget.h"

class LaserScanPlugin : public GripTab
{
    Q_OBJECT
    Q_INTERFACES(GripTab)
public:
    LaserScanPlugin(QWidget *parent = 0);
    ~LaserScanPlugin();

private:
    Ui::LaserScanPlugin *ui;

public slots:
    void scan_slot();

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

#endif
