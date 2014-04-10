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
#include <dart/planning/PathShortener.h>
#include <PCDLoader.h>


PlanningPlugin::PlanningPlugin(QWidget *parent) : ui(new Ui::PlanningPlugin){
    ui->setupUi(this);
//    joint_order = {
//              "RHY", "RHR", "RHP", RKN, RAP, RAR,

//              LHY, LHR, LHP, LKN, LAP, LAR,

//              RSP, RSR, RSY, REB, RWY, RWR, RWP,

//              LSP, LSR, LSY, LEB, LWY, LWR, LWP,

//              NKY, NK1, NK2, WST,

//              RF1, RF2, RF3, RF4, RF5,

//              LF1, LF2, LF3, LF4, LF5
//        };
}

PlanningPlugin::~PlanningPlugin(){}

//void PlanningPlugin::GRIPEventSimulationBeforeTimestep(){}
//void PlanningPlugin::GRIPEventSimulationAfterTimestep(){}
//void PlanningPlugin::GRIPEventSimulationStart(){}
//void PlanningPlugin::GRIPEventSimulationStop(){}
void PlanningPlugin::GRIPEventTreeViewSelectionChanged() {/*
    if (!updateIndex()) {
        std::cerr << "No skeleton named GolemHubo" << std::endl;
        _index.empty();
        std::cout << "index empty size: " << _index.size() << std::endl;
    }*/
}

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
    if (_skel) {
        updateIndex();
        std::cout << "num contacts " << _world->getConstraintHandler()->getNumContacts() << std::endl;
        std::cout << "Start conf " << _world->checkCollision(false) << std::endl;
        if (!_world->checkCollision(true)) {
            std::cout << "Validation? OK :-)" << std::endl;
        } else {
            std::cout << "Validation? NO" << std::endl;

            for (int i=1;i<2;i++) {
                for (int j=0;j<_world->getSkeleton(i)->getNumBodyNodes();j++) {
                    dart::dynamics::BodyNode* node1 = _world->getSkeleton(i)->getBodyNode(j);
                    for (int k=0;k<=j;k++) {
                        dart::dynamics::BodyNode* node2 = _world->getSkeleton(i)->getBodyNode(k);
                        std::cout << "1: " << node1->getName() << " - 2: " << node2->getName() << " " << _world->getConstraintHandler()->getCollisionDetector()->detectCollision(node1,node2) << std::endl;
                    }
                }
            }

        }
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
        dart::planning::PathPlanner<> planner(*_world,true,true,0.1,1e6,0.3);
        int i=0;
        std::list<Eigen::VectorXd>::iterator it;

        bool rep = planner.planPath(_skel, _index, _startConf, _goalConf, path);

        dart::planning::PathShortener short_path(_world, _skel, _index, 0.1);
        short_path.shortenPath(path);


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
        _skel->setSelfCollidable(false);

        std::string nodes[6] = {"Body_RSP","Body_RSR","Body_RSY","Body_REP","Body_RWY","Body_RWP"};

        for (int i=0;i<6;i++) {
            _world->getConstraintHandler()->getCollisionDetector()->enablePair(_world->getSkeleton(1)->getBodyNode(nodes[i]),_world->getSkeleton(1)->getBodyNode("Body_Torso"));
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

void PlanningPlugin::on_commandLinkButton_clicked()
{
    QStringList fileNames; //stores the entire path of the file that it attempts to open

    QStringList filters; //setting file filters
    filters << "Point Cloud file (*.pcd)"
            << "Any files (*)";

    //initializing the File dialog box
    //the static QFileDialog does not seem to be working correctly in Ubuntu 12.04 with unity.
    //as per the documentation it may work correctly with gnome
    //the method used below should work correctly on all desktops and is supposedly more powerful
    QFileDialog dialog(this);
    dialog.setNameFilters(filters);
    dialog.setAcceptMode(QFileDialog::AcceptOpen);
    dialog.setFileMode(QFileDialog::ExistingFile);
    if (dialog.exec())
        fileNames = dialog.selectedFiles();

    if (!fileNames.isEmpty())
    {
        std::cerr<<"Attempting to open the following world file: "<<fileNames.front().toStdString() <<std::endl;
        loader = new PCDLoader(fileNames.front().toStdString());
        _viewWidget->addNodeToScene(loader->geode);
    }
}
