#ifndef FOOTSTEP_PLANNER_PLUGIN_H
#define FOOTSTEP_PLANNER_PLUGIN_H

#include "ui_footstep_planner_plugin.h"
#include "FootstepPlanner.h"
#include "FootLocation.h"
#include "Line.h"
#include "Foot.h"
#include "FootConstraint.h"
#include "FootstepPlanner.h"
#include "FootstepPlanVisualizer.h"

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
    Ui::FootstepPlannerPlugin * _ui;
    std::vector<fsp::Foot> _feet;
    std::vector<fsp::FootConstraint> _constraints;

    void log(std::string msg);

public slots:
    void runPlanner();
    void showStartPosition(bool checked);
    void showGoalPosition(bool checked);
    void showObstacles(bool checked);
    void showFootsteps(bool checked);
    void showTiles(bool checked);

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
