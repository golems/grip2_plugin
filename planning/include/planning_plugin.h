#ifndef POINT_CLOUD_PLUGIN_H
#define POINT_CLOUD_PLUGIN_H

#include "ui_planning_plugin.h"
#include <grip/qtWidgets/GripTab.h>
#include <grip/qtWidgets/TreeViewReturn.h>
#include <grip/osgGolems/ViewerWidget.h>

class PlanningPlugin : public GripTab
{
    Q_OBJECT
    Q_INTERFACES(GripTab)
public:
    PlanningPlugin(QWidget *parent = 0);
    ~PlanningPlugin();

private:
    Ui::PlanningPlugin *ui;
    dart::dynamics::Skeleton* _skel;
    std::vector<int> _index;
    Eigen::VectorXd _startConf;
    Eigen::VectorXd _goalConf;

    bool updateIndex();
    void interpolate(std::list<Eigen::VectorXd>& path, std::list<Eigen::VectorXd>& interpolation);

public:

    void GRIPEventTreeViewSelectionChanged();
    void Refresh();

private slots:
    void on_saveStartButton_clicked();
    void on_saveGoalButton_clicked();
    void on_planAndMoveButton_clicked();
};

#endif // TESTGRIPPLUGIN_H
