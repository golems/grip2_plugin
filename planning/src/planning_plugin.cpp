#include "planning_plugin.h"
#include <iostream>
#include <fstream>
#include <qplugin.h>
#include <QtGui>

#include <dart/constraint/ConstraintDynamics.h>
#include <dart/planning/PathPlanner.h>
#include <dart/planning/PathShortener.h>
#include <dart/dynamics/Joint.h>

const std::string PlanningPlugin::ROBOT_SKELETON_NAME = "drchubo_v2";
const std::vector<std::string> PlanningPlugin::CONTROLLED_JOINTS_NAMES = {"RSP", "RSR", "RSY", "REP", "RWY", "RWP"};

PlanningPlugin::PlanningPlugin(QWidget *) : ui(new Ui::PlanningPlugin), _controlled_joints_index(CONTROLLED_JOINTS_COUNT), _operator(.1)
{
    ui->setupUi(this);
}

PlanningPlugin::~PlanningPlugin(){}

void PlanningPlugin::Refresh() {}

Q_EXPORT_PLUGIN2(PlanningPlugin, PlanningPlugin)

void PlanningPlugin::on_saveStartButton_clicked()
{
    if (setUp()) {
        _startConf = _skel->getConfig(_controlled_joints_index);
        std::cout << "Start configuration saved" << std::endl;
    } else {
        std::cerr << "Error during set up" << std::endl;
    }
}

void PlanningPlugin::on_saveGoalButton_clicked()
{
    if (setUp()) {
        _goalConf = _skel->getConfig(_controlled_joints_index);
        std::cout << "Goal configuration saved" << std::endl;
    } else {
        std::cerr << "Error during set up" << std::endl;
    }
}

void PlanningPlugin::on_runButton_clicked()
{
    if(!_operator.initialized())
    {
        if(_operator.receive_description(0.1))
        {
            std::cerr << "Operator couldn't connect" << std::endl;
            return;
        }
    }

    _operator.setJointIndices(CONTROLLED_JOINTS_NAMES);
    _operator.addWaypoints(_path);
    //_operator.interpolate();

    std::cout << _operator.getCurrentTrajectory() << std::endl;

    _operator.sendNewTrajectory();
}

bool PlanningPlugin::setUp()
{
    _skel = _world->getSkeleton(ROBOT_SKELETON_NAME);

    if (_skel) {
        std::cout << "Robot skeleton found" << std::endl;
        setUpCollision();
        setUpJointsIndex();

        return true;

    } else {
        std::cerr << "Robot skeleton NOT found" << std::endl;
        return false;
    }
}

void PlanningPlugin::setUpJointsIndex()
{
    for(unsigned int i=0 ; i<CONTROLLED_JOINTS_COUNT ; ++i)
    {
        _controlled_joints_index[i] = _skel->getJoint(CONTROLLED_JOINTS_NAMES[i])->getGenCoord(0)->getSkeletonIndex();
    }
}

void PlanningPlugin::setUpCollision()
{
    // Set non self collidable
    _skel->setSelfCollidable(false);

    std::vector<std::string> arm_nodes = {"Body_RSP","Body_RSR","Body_RSY","Body_REP","Body_RWY","Body_RWP"};
    std::vector<std::string> body_nodes = {"Body_Torso","Body_HNR","Body_HNP","Body_Hip","Body_LHY","Body_RHY"};

    // Set collision between arm and torso
    for (unsigned int i=0 ; i<arm_nodes.size() ; ++i)
    {
        for (unsigned int j=0 ; j<body_nodes.size() ; ++j)
        {
            _world->getConstraintHandler()->getCollisionDetector()->enablePair(_skel->getBodyNode(arm_nodes[i]), _skel->getBodyNode(body_nodes[j]));
        }
    }
}

void PlanningPlugin::on_planAndMoveButton_clicked()
{
    if ((_startConf.cols() != 0) && (_goalConf.cols() != 0))
    {
        _path.clear();

        // Path planner
        dart::planning::PathPlanner<> planner(*_world, true, true, 0.1, 1e6, 0.3);
        bool path_found = planner.planPath(_skel, _controlled_joints_index, _startConf, _goalConf, _path);

        // Path shortener
        dart::planning::PathShortener short_path(_world, _skel, _controlled_joints_index, 0.1);
        short_path.shortenPath(_path);

        if (path_found) {
            std::cout << "Path found. Size: " << _path.size() << std::endl;
        } else {
            std::cout << "Path not found" << std::endl;
            return;
        }

        // Add planning to timeline
        for(std::list<Eigen::VectorXd>::iterator it = _path.begin() ; it != _path.end() ; ++it)
        {
            _skel->setConfig(_controlled_joints_index, *it);
            // Save world to timeline
            _timeline->push_back(GripTimeslice(*_world));
        }
    }
    else
    {
        std::cerr << "Start or goal conf not set" << std::endl;
    }

}
