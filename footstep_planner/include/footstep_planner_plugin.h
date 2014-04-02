#ifndef FOOTSTEP_PLANNER_PLUGIN_H
#define FOOTSTEP_PLANNER_PLUGIN_H

#include "ui_footstep_planner_plugin.h"
#include <grip/qtWidgets/GripTab.h>
#include <grip/qtWidgets/TreeViewReturn.h>
#include <grip/osgGolems/ViewerWidget.h>
#include <string>

class FootstepPlannerPlugin : public GripTab
{
    Q_OBJECT
    Q_INTERFACES(GripTab)
public:
    FootstepPlannerPlugin(QWidget *parent = 0);
    ~FootstepPlannerPlugin();

private:
    Ui::FootstepPlannerPlugin *ui;

    void log(std::string msg);

public slots:
    void runPlanner();
    void showStartPosition();
    void showGoalPosition();
    void showObstacles();
    void showFootsteps();
    void showTiles();

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
