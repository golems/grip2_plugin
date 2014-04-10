#ifndef POINT_CLOUD_PLUGIN_H
#define POINT_CLOUD_PLUGIN_H

#include "ui_planning_plugin.h"
#include <grip/qtWidgets/GripTab.h>
#include <grip/qtWidgets/TreeViewReturn.h>
#include <grip/osgGolems/ViewerWidget.h>
#include <PCDLoader.h>

class PlanningPlugin : public GripTab
{
    Q_OBJECT
    Q_INTERFACES(GripTab)
public:
    PlanningPlugin(QWidget *parent = 0);
    ~PlanningPlugin();

//    const int joint_order[];

private:
    Ui::PlanningPlugin *ui;
    dart::dynamics::Skeleton* _skel;
    std::vector<int> _index;
    Eigen::VectorXd _startConf;
    Eigen::VectorXd _goalConf;
    PCDLoader* loader;

    bool updateIndex();

public:
//    void Load(TreeViewReturn* ret, ViewerWidget* viewer);

//    void GRIPEventSimulationBeforeTimestep();
//    void GRIPEventSimulationAfterTimestep();
//    void GRIPEventSimulationStart();
//    void GRIPEventSimulationStop();
//    void GRIPEventPlaybackBeforeFrame();
//    void GRIPEventPlaybackAfterFrame();
//    void GRIPEventPlaybackStart();
//    void GRIPEventPlaybackStop();
    void GRIPEventTreeViewSelectionChanged();
    void Refresh();

private slots:
    void on_saveStartButton_clicked();
    void on_saveGoalButton_clicked();
    void on_PlanMoveButton_clicked();
    void on_commandLinkButton_clicked();
};

#endif // TESTGRIPPLUGIN_H
