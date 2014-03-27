#ifndef POINT_CLOUD_PLUGIN_H
#define POINT_CLOUD_PLUGIN_H

#include "ui_planning_plugin.h"
#include "qtWidgets/include/GripTab.h"
#include "qtWidgets/include/TreeViewReturn.h"
#include "osgNodes/include/ViewerWidget.h"

class PlanningPlugin : public GripTab
{
    Q_OBJECT
    Q_INTERFACES(GripTab)
public:
    PlanningPlugin(QWidget *parent = 0);
    ~PlanningPlugin();

private:
    Ui::PlanningPlugin *ui;

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
