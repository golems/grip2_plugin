#include "planning_plugin.h"
#include <iostream>
#include <fstream>
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
#include <dart/planning/PathShortener.h>
#include <PCDLoader.h>


PlanningPlugin::PlanningPlugin(QWidget *parent) : ui(new Ui::PlanningPlugin){
    ui->setupUi(this);
}

PlanningPlugin::~PlanningPlugin(){}

//void PlanningPlugin::GRIPEventSimulationBeforeTimestep(){}
//void PlanningPlugin::GRIPEventSimulationAfterTimestep(){}
//void PlanningPlugin::GRIPEventSimulationStart(){}
//void PlanningPlugin::GRIPEventSimulationStop(){}
void PlanningPlugin::GRIPEventTreeViewSelectionChanged() {}

//void PlanningPlugin::Load(TreeViewReturn* ret, ViewerWidget *viewer) {}

//void PlanningPlugin::GRIPEventPlaybackBeforeFrame() {}

/**
 * \brief called from the main window whenever the simulation history slider is being played
 * This method is executed after every playback time step
 */
//void PlanningPlugin::GRIPEventPlaybackAfterFrame() {}

/**
 * \brief called from the main window whenever the simulation history slider is being played
 * This method is executed at the start of the playback
 */
//void PlanningPlugin::GRIPEventPlaybackStart() {}

/**
 * \brief called from the main window whenever the simulation history slider is being played
 * This method is executed at the end of the playback
 */
//void PlanningPlugin::GRIPEventPlaybackStop() {}

void PlanningPlugin::Refresh() {}

Q_EXPORT_PLUGIN2(PlanningPlugin, PlanningPlugin)

void PlanningPlugin::on_saveStartButton_clicked()
{
    updateIndex();
    if (_skel) {
        _startConf = _skel->getConfig(_index);
        std::cout << "Start configuration saved" << std::endl;
    } else {
        std::cerr << "No skeleton loaded" << std::endl;
    }
}

void PlanningPlugin::on_saveGoalButton_clicked()
{
    updateIndex();
    if (_skel) {
        _goalConf = _skel->getConfig(_index);
        std::cout << "Goal configuration saved" << std::endl;
    } else {
        std::cerr << "No skeleton loaded" << std::endl;
    }
}

bool PlanningPlugin::updateIndex()
{
    _skel = _world->getSkeleton("drchubo_v2");//huboplus");
    std::cout << "huboplus found" << std::endl;

    if (_skel) {
        _skel->setSelfCollidable(false);

        std::string arm_nodes[6] = {"Body_RSP","Body_RSR","Body_RSY","Body_REP","Body_RWY","Body_RWP"};
        std::string body_nodes[6] = {"Body_Torso","Body_HNR","Body_HNP","Body_Hip","Body_LHY","Body_RHY"};

        for (int i=0;i<6;i++) {
            for (int j=0;j<6;j++) {
                _world->getConstraintHandler()->getCollisionDetector()->enablePair(_world->getSkeleton(1)->getBodyNode(arm_nodes[i]),_world->getSkeleton(1)->getBodyNode(body_nodes[j]));
            }
        }

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

void PlanningPlugin::interpolate(std::list<Eigen::VectorXd>& path, std::list<Eigen::VectorXd>& interpolation) {
    double max_dx, dx, time, dt, t;
    size_t count = 0;
    std::list<Eigen::VectorXd>::iterator it_start = path.begin();
    std::list<Eigen::VectorXd>::iterator it_last = path.begin();
    for (++it_last;it_last!=path.end();++it_start,++it_last) {
        Eigen::VectorXd& start = *it_start;
        Eigen::VectorXd& last = *it_last;
        max_dx = last[0] - start[0];
        for (int j=1;j<start.size();++j) {
            dx = last[j] - start[j];
            if (dx>max_dx) {
                max_dx = dx;
            }
        }
        time = max_dx/1; // 1rad/sec
        dt = time/200; // 200 Hz
        t=0;
        interpolation.push_back(start);
        while (t<time) {
            t+=dt;
            interpolation.push_back((last-start)*t/time + start);
        }
        count++;
        if (count > path.size()) {
            std::cout << "SOS" << std::endl;
            return;
        }
    }
    interpolation.push_back(path.back());
}

void PlanningPlugin::on_planAndMoveButton_clicked()
{
    if ((_startConf.cols() != 0)&&(_goalConf.cols() != 0)) {

        std::list<Eigen::VectorXd> path;
        std::list<Eigen::VectorXd> interpolation;
        dart::planning::PathPlanner<> planner(*_world,true,true,0.1,1e6,0.3);
        int i=0;
        const int length = 29;
        std::list<Eigen::VectorXd>::iterator it;

        bool rep = planner.planPath(_skel, _index, _startConf, _goalConf, path);

        dart::planning::PathShortener short_path(_world, _skel, _index, 0.1);
        short_path.shortenPath(path);

        interpolate(path, interpolation);
        //interpolation = path;

        if (rep) {
            std::cout << "Path found. Size: " << path.size() << std::endl;
            std::cout << "Interpolation. Size: " << interpolation.size() << std::endl;
        } else {
            std::cout << "Path not found" << std::endl;
            return;
        }

        std::string traj_joints[length] = {"RHY", "RHR", "RHP", "RKP", "RAP", "RAR",
                                      "LHY", "LHR", "LHP", "LKP", "LAP", "LAR",
                                      "RSP", "RSR", "RSY", "REP", "RWY", "RWR", "RWP",
                                      "LSP", "LSR", "LSY", "LEP", "LWY", "LWR", "LWP",
                                      "NKY", "NK1", "NK2", /*"WST",
                                      "RF1", "RF2", "RF3", "RF4", "RF5",
                                      "LF1", "LF2", "LF3", "LF4", "LF5"*/};
        std::vector<int> index;
        for (int k=0;k<length;k++) {
            index.push_back(_skel->getJoint(traj_joints[k])->getGenCoord(0)->getSkeletonIndex());
        }
        Eigen::VectorXd config = _skel->getConfig(index);

        std::ofstream file("test.txt", std::ios::out | std::ios::trunc);
        if(file) {
            i=0;
            while (i < interpolation.size())
            {
                it = interpolation.begin();
                std::advance(it, i);
                // Move by following the path
                _skel->setConfig(_index, (Eigen::VectorXd)(*it));

                // Save world to timeline
                _timeline->push_back(GripTimeslice(*_world));

                // Write the trajectory in output file
                config = _skel->getConfig(index);
                for (int k=0;k<config.rows();k++) {
                    file << config[k] << " ";
                }
                for (int k=config.rows();k<40;k++) {
                    file << 0 << " ";
                }
                file << std::endl;

                i++;
            }
            file.close();
        } else {
            std::cerr << "Error opening the output file" << std::endl;
        }
    } else {
        std::cout << "Start and/or goal configuration have not been saved." << std::endl;
    }
}
