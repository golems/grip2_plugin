#include "footstep_planner_plugin.h"
#include <iostream>
#include <qplugin.h>
#include <QtGui>

using namespace std;
using namespace fsp;
using namespace Eigen;

FootstepPlannerPlugin::FootstepPlannerPlugin(QWidget *parent) : ui(new Ui::FootstepPlannerPlugin)
{
    ui->setupUi(this);
    // Initialize random seed
    srand(time(NULL));

    // Initialize the feet
    _feet.push_back(Foot(2.0f, 4.0f, "Left"));
    _feet.push_back(Foot(2.0f, 4.0f, "Right"));

    // Initialize the foot constraints
    _constraints.push_back(FootConstraint(0, 1, -4.0d, 4.0d, 3.0d, 5.0d, -30.0d, 30.0d));
    _constraints.push_back(FootConstraint(1, 0, -4.0d, 4.0d, -3.0d, -5.0d, -30.0d, 30.0d));
}
FootstepPlannerPlugin::~FootstepPlannerPlugin()
{
}

void FootstepPlannerPlugin::runPlanner()
{
    log("Initializing Current Location");
    // Initialize the current location
    vector<FootLocation> currentLoc;
    currentLoc.push_back(FootLocation(Vector2d(2.0d, 2.5d), 0.0f, 0.0f, 0, &_feet));
    currentLoc.push_back(FootLocation(Vector2d(2.0d, 0.0d), 0.0f, 0.0f, 1, &_feet));

    log("Initializing Goal Location");
    // Initialize the goal location
    vector<FootLocation> goalLoc;
    goalLoc.push_back(FootLocation(Vector2d(100.0d, -2.5d), 0.0f, 0.0f, 0, &_feet));
    goalLoc.push_back(FootLocation(Vector2d(100.0d, -0.0d), 0.0f, 0.0f, 1, &_feet));

    log("Initializing Obstacles");
    // Initialize the obstacles
    vector<Line> obs;

    /*
    // Random obstacles
    obs.push_back(Line(Vector2d(-10.0d, 7.0d), Vector2d(55.0d, 18.0d)));
    obs.push_back(Line(Vector2d(55.0d, 18.0d), Vector2d(0.0d, 23.0d)));
    obs.push_back(Line(Vector2d(0.0d, 23.0d), Vector2d(-10.0d, 7.0d)));
    obs.push_back(Line(Vector2d(20.0d, -7.0d), Vector2d(25.0d, -38.0d)));
    obs.push_back(Line(Vector2d(25.0d, -38.0d), Vector2d(25.0d, -50.0d)));
    obs.push_back(Line(Vector2d(25.0d, -50.0d), Vector2d(10.0d, -17.0d)));
    obs.push_back(Line(Vector2d(10.0d, -17.0d), Vector2d(20.0d, -7.0d)));
    */
    // Blocking obstacles
    obs.push_back(Line(Vector2d(30.0d, 15.0d), Vector2d(35.0d, 15.0d)));
    obs.push_back(Line(Vector2d(35.0d, 15.0d), Vector2d(35.0d, -15.0d)));
    obs.push_back(Line(Vector2d(35.0d, -15.0d), Vector2d(30.0d, -15.0d)));
    obs.push_back(Line(Vector2d(30.0d, -15.0d), Vector2d(30.0d, 15.0d)));
    obs.push_back(Line(Vector2d(60.0d, 0.0d), Vector2d(65.0d, 0.0d)));
    obs.push_back(Line(Vector2d(65.0d, 0.0d), Vector2d(65.0d, -35.0d)));
    obs.push_back(Line(Vector2d(65.0d, -35.0d), Vector2d(60.0d, -35.0d)));
    obs.push_back(Line(Vector2d(60.0d, -35.0d), Vector2d(60.0d, 0.0d)));


    log("Initializing Planner");
    // Initialize the planner
    FootstepPlanner planner(_feet);
    vector<Vector2i> mapPlan;
    //vector<FootLocation> plan = planner.getStaticPlan();
    //vector<FootLocation> plan = planner.generatePlan(PLANNER_TYPE_RRT, constraints, currentLoc, goalLoc, obs);
    //vector<FootLocation> plan = planner.generatePlan(PLANNER_TYPE_R_STAR, constraints, currentLoc, goalLoc, obs);
    log("Running planner!");
    vector<FootLocation> plan = planner.runAStarPlanner(_constraints, currentLoc, goalLoc, obs, mapPlan);

    // Initialize the visualizer
    FootstepPlanVisualizer visualizer(_feet);
    //visualizer.visualizePlan2(FootstepPlanner::MIN_POINT, FootstepPlanner::MAX_POINT, FootstepPlanner::DISCRETIZATION_RES, currentLoc, goalLoc, obs, plan, mapPlan);

    log("Done running Planner");
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
