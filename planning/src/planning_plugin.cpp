#include "planning_plugin.h"
#include <iostream>
#include <qplugin.h>
#include <QtGui>

#include <dart/dynamics/Skeleton.h>
#include <dart/dynamics/BodyNode.h>
#include <dart/dynamics/Joint.h>
#include <dart/dynamics/FreeJoint.h>
#include <dart/dynamics/RevoluteJoint.h>
#include <dart/dynamics/PrismaticJoint.h>
#include <dart/dynamics/GenCoord.h>
#include <dart/constraint/ConstraintDynamics.h>
//#include <dart/collision/bullet/BulletCollisionDetector.h>
#include <dart/simulation/World.h>
#include <dart/planning/PathPlanner.h>

PlanningPlugin::PlanningPlugin(QWidget *parent) : ui(new Ui::PlanningPlugin){
    ui->setupUi(this);
}

PlanningPlugin::~PlanningPlugin(){}

void PlanningPlugin::GRIPEventSimulationBeforeTimestep(){}
void PlanningPlugin::GRIPEventSimulationAfterTimestep(){}
void PlanningPlugin::GRIPEventSimulationStart(){}
void PlanningPlugin::GRIPEventSimulationStop(){}
void PlanningPlugin::GRIPEventTreeViewSelectionChanged() {
    if (!updateIndex()) {
        std::cerr << "No skeleton named GolemHubo" << std::endl;
        _index.empty();
        std::cout << "index empty size: " << _index.size() << std::endl;
    }
}

void PlanningPlugin::Load(TreeViewReturn* ret, ViewerWidget *viewer) {}

void PlanningPlugin::GRIPEventPlaybackBeforeFrame() {}

/**
 * \brief called from the main window whenever the simulation history slider is being played
 * This method is executed after every playback time step
 */
void PlanningPlugin::GRIPEventPlaybackAfterFrame() {}

/**
 * \brief called from the main window whenever the simulation history slider is being played
 * This method is executed at the start of the playback
 */
void PlanningPlugin::GRIPEventPlaybackStart() {}

/**
 * \brief called from the main window whenever the simulation history slider is being played
 * This method is executed at the end of the playback
 */
void PlanningPlugin::GRIPEventPlaybackStop() {}

void PlanningPlugin::Refresh() {}

Q_EXPORT_PLUGIN2(PlanningPlugin, PlanningPlugin)

void PlanningPlugin::on_saveStartButton_clicked()
{
    if (_skel) {
        updateIndex();
        _startConf = _skel->getConfig(_index);
        std::cout << "Start configuration saved" << std::endl;
    } else {
        std::cerr << "No skeleton loaded" << std::endl;
    }
}

void PlanningPlugin::on_saveGoalButton_clicked()
{
    if (_skel) {
        updateIndex();
        _goalConf = _skel->getConfig(_index);
        std::cout << "Goal configuration saved. " << _goalConf.rows() << " rows & " << _goalConf.cols() << " cols." << std::endl;
    } else {
        std::cerr << "No skeleton loaded" << std::endl;
    }
}

void PlanningPlugin::on_PlanMoveButton_clicked()
{
    if ((_startConf.cols() != 0)&&(_goalConf.cols() != 0)) {

        std::list<Eigen::VectorXd> path;
        dart::planning::PathPlanner<> planner(*_world,true,true,0.1,1e5,0.3);
        int i=0;
        std::list<Eigen::VectorXd>::iterator it;

        bool rep = planner.planPath(_skel, _index, _startConf, _goalConf, path);

        if (rep) {
            std::cout << "Path found. Size: " << path.size() << std::endl;
            while (i < path.size())
            {
                it = path.begin();
                std::advance(it, i);
                Eigen::VectorXd vec = (Eigen::VectorXd)(*it);
                // 'it' points to the element at index 'i'
                std::cout << "State " << i << ": " << std::ends;
                for (int j=0;j<vec.rows();j++) {
                    std::cout << vec[j] << " - " << std::ends;
                }
                std::cout << std::endl;
                i++;
            }
        } else {
            std::cout << "Path not found" << std::endl;
        }

        i=0;
        while (i < path.size())
        {
            it = path.begin();
            std::advance(it, i);
            // Move by following the path
            _skel->setConfig(_index, (Eigen::VectorXd)(*it));

            // Save world to timeline
            _timeline->push_back(GripTimeslice(*_world));

            i++;
        }
    } else {
        std::cout << "Start and/or goal configuration have not been saved." << std::endl;
    }
}

bool PlanningPlugin::updateIndex()
{
    _skel = _world->getSkeleton("GolemHubo");
    std::cout << "GolemHubo found" << std::endl;

    if (_skel) {
        // Get index of the right arm
        _index.clear();
        std::cout << "index size: " << _index.size() << std::endl;
        _index.push_back(_skel->getJoint("RSP")->getGenCoord(0)->getSkeletonIndex());
        _index.push_back(_skel->getJoint("RSR")->getGenCoord(0)->getSkeletonIndex());
        _index.push_back(_skel->getJoint("RSY")->getGenCoord(0)->getSkeletonIndex());
        _index.push_back(_skel->getJoint("REP")->getGenCoord(0)->getSkeletonIndex());
        _index.push_back(_skel->getJoint("RWY")->getGenCoord(0)->getSkeletonIndex());
        _index.push_back(_skel->getJoint("RWP")->getGenCoord(0)->getSkeletonIndex());
        std::cout << "index size: " << _index.size() << std::endl;
        return true;
    } else {
        return false;
    }
}
