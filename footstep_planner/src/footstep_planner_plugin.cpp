#include "footstep_planner_plugin.h"
#include <iostream>
#include <qplugin.h>
#include <QtGui>

FootstepPlannerPlugin::FootstepPlannerPlugin(QWidget *parent) : ui(new Ui::FootstepPlannerPlugin)
{
    ui->setupUi(this);
}
FootstepPlannerPlugin::~FootstepPlannerPlugin()
{

}

void FootstepPlannerPlugin::runPlanner()
{
    log("Running Planner");
}
void FootstepPlannerPlugin::showStartPosition()
{
    log("Show start position");
}
void FootstepPlannerPlugin::showGoalPosition()
{
    log("Show goal position");
}
void FootstepPlannerPlugin::showObstacles()
{
    log("Show obstacles");
}
void FootstepPlannerPlugin::showFootsteps()
{
    log("Show footsteps");
}
void FootstepPlannerPlugin::showTiles()
{
    log("Show tiles");
}

void FootstepPlannerPlugin::log(std::string msg)
{
    std::cout << "[FootstepPlanner]: " << msg << std::endl;
}

void FootstepPlannerPlugin::GRIPEventSimulationBeforeTimestep(){}
void FootstepPlannerPlugin::GRIPEventSimulationAfterTimestep(){}
void FootstepPlannerPlugin::GRIPEventSimulationStart(){}
void FootstepPlannerPlugin::GRIPEventSimulationStop(){}
void FootstepPlannerPlugin::GRIPEventTreeViewSelectionChanged(){}
void FootstepPlannerPlugin::Load(TreeViewReturn* ret, ViewerWidget *viewer)
{
    _activeNode = ret;
    _viewWidget = viewer;
}

void FootstepPlannerPlugin::GRIPEventPlaybackBeforeFrame(){}

/**
 * \brief called from the main window whenever the simulation history slider is being played
 * This method is executed after every playback time step
 */
void FootstepPlannerPlugin::GRIPEventPlaybackAfterFrame(){}

/**
 * \brief called from the main window whenever the simulation history slider is being played
 * This method is executed at the start of the playback
 */
void FootstepPlannerPlugin::GRIPEventPlaybackStart(){}

/**
 * \brief called from the main window whenever the simulation history slider is being played
 * This method is executed at the end of the playback
 */
void FootstepPlannerPlugin::GRIPEventPlaybackStop(){}

void FootstepPlannerPlugin::Refresh(){}

Q_EXPORT_PLUGIN2(FootstepPlannerPlugin, FootstepPlannerPlugin)
