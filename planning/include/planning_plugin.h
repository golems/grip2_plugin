#ifndef POINT_CLOUD_PLUGIN_H
#define POINT_CLOUD_PLUGIN_H

#include "ui_planning_plugin.h"
#include <grip/qtWidgets/GripTab.h>
#include <grip/qtWidgets/TreeViewReturn.h>
#include <grip/osgGolems/ViewerWidget.h>
#include <HuboPath/Operator.hpp>

class PlanningPlugin : public GripTab
{
    Q_OBJECT
    Q_INTERFACES(GripTab)
public:
    PlanningPlugin(QWidget *parent = 0);
    ~PlanningPlugin();

private:
    static const std::string ROBOT_SKELETON_NAME;
    static const unsigned int CONTROLLED_JOINTS_COUNT = 6;
    static const std::vector<std::string> CONTROLLED_JOINTS_NAMES;

    Ui::PlanningPlugin *ui;
    dart::dynamics::Skeleton* _skel;

    std::vector<int> _controlled_joints_index;

    Eigen::VectorXd _startConf;
    Eigen::VectorXd _goalConf;

    HuboPath::Operator _operator;
    std::list<Eigen::VectorXd> _path;

    bool setUp();
    void setUpJointsIndex();
    void setUpCollision();

public:
    void Refresh();

private slots:
    void on_saveStartButton_clicked();
    void on_saveGoalButton_clicked();
    void on_planAndMoveButton_clicked();
    void on_runButton_clicked();
};

#endif // PLANNING_PLUGIN_H
