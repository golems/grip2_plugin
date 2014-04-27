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

PlanningPlugin::PlanningPlugin(QWidget *) : ui(new Ui::PlanningPlugin), _controlled_joints_index(CONTROLLED_JOINTS_COUNT){
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

    std::string arm_nodes[6] = {"Body_RSP","Body_RSR","Body_RSY","Body_REP","Body_RWY","Body_RWP"};
    std::string body_nodes[6] = {"Body_Torso","Body_HNR","Body_HNP","Body_Hip","Body_LHY","Body_RHY"};

    // Set collision between arm and torso
    for (unsigned int i=0 ; i<6 ; i++) {
        for (unsigned int j=0 ; j<6 ; j++) {
            _world->getConstraintHandler()->getCollisionDetector()->enablePair(_skel->getBodyNode(arm_nodes[i]), _skel->getBodyNode(body_nodes[j]));
        }
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

        bool path_found = planner.planPath(_skel, _controlled_joints_index, _startConf, _goalConf, path);

        dart::planning::PathShortener short_path(_world, _skel, _controlled_joints_index, 0.1);
        short_path.shortenPath(path);

        interpolate(path, interpolation);
        //interpolation = path;

        if (path_found) {
            std::cout << "Path found. Size: " << path.size() << std::endl;
            std::cout << "Interpolation. Size: " << interpolation.size() << std::endl;
        } else {
            std::cout << "Path not found" << std::endl;
            return;
        }

        const unsigned int length = 29;
        std::string traj_joints[length] = {"RHY", "RHR", "RHP", "RKP", "RAP", "RAR",
                                           "LHY", "LHR", "LHP", "LKP", "LAP", "LAR",
                                           "RSP", "RSR", "RSY", "REP", "RWY", "RWR", "RWP",
                                           "LSP", "LSR", "LSY", "LEP", "LWY", "LWR", "LWP",
                                           "NKY", "NK1", "NK2", /*"WST",
                                                                                                                                                                                  "RF1", "RF2", "RF3", "RF4", "RF5",
                                                                                                                                                                                  "LF1", "LF2", "LF3", "LF4", "LF5"*/};
        std::vector<int> index;
        for (unsigned int k=0 ; k<length ; ++k) {
            index.push_back(_skel->getJoint(traj_joints[k])->getGenCoord(0)->getSkeletonIndex());
        }
        Eigen::VectorXd config = _skel->getConfig(index);

        std::ofstream file("test.txt", std::ios::out | std::ios::trunc);
        if(file) {
            for(std::list<Eigen::VectorXd>::iterator it = interpolation.begin() ; it != interpolation.end() ; ++it)
            {
                _skel->setConfig(_controlled_joints_index, *it);

                // Save world to timeline
                _timeline->push_back(GripTimeslice(*_world));

                // Write the trajectory in output file
                config = _skel->getConfig(index);

                for (int k=0 ; k<config.rows() ; k++) {
                    file << config[k] << " ";
                }

                for (int k=config.rows() ; k<40 ; k++) {
                    file << 0 << " ";
                }
                file << std::endl;

            }
            file.close();
        } else {
            std::cerr << "Error opening the output file" << std::endl;
        }
    } else {
        std::cout << "Start and/or goal configuration have not been saved." << std::endl;
    }
}
