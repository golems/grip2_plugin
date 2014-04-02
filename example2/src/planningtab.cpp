/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author: Sungmoon Joo <sungmoon.joo@gmail.com>
 * Date: Feb 2014
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *   * Neither the name of the Humanoid Robotics Lab nor the names of
 *     its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written
 *     permission
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */
 
#include <string>
#include <iostream>
#include <qplugin.h>
#include <QtGui>
#include <dart/dynamics/Skeleton.h>
#include <dart/dynamics/Joint.h>
#include <dart/dynamics/BodyNode.h>
#include <dart/collision/CollisionDetector.h>
#include <dart/constraint/ConstraintDynamics.h>
#include <dart/dynamics/BoxShape.h>
#include <dart/dynamics/GenCoord.h>
#include <dart/planning/PathPlanner.h>
#include <dart/planning/PathShortener.h>
#include <dart/planning/PathFollowingTrajectory.h>
#include "Controller.h"
#include "planningtab.h"

using namespace std;

PlanningTab::PlanningTab(QWidget *parent) : _ui(new Ui::PlanningTabWidget){
    _ui->setupUi(this);

  mPredefStartConf.resize(6);
  mPredefGoalConf.resize(6);
  mPredefStartConf << -0.858702, -0.674395, 0.0, -0.337896, 0.0, 0.0;
  mPredefGoalConf << -0.69115, 0.121475, 0.284977, -1.02486, 0.0, 0.0;
  mStartConf = mPredefStartConf;
  mGoalConf = mPredefGoalConf;

  connect(_ui->setStart,SIGNAL(pressed()),this,SLOT(setStartPressed()));
  connect(_ui->setGoal,SIGNAL(pressed()),this,SLOT(setGoalPressed()));
  connect(_ui->setPredefStart,SIGNAL(pressed()),this,SLOT(setPredefStartPressed()));
  connect(_ui->setPredefGoal,SIGNAL(pressed()),this,SLOT(setPredefGoalPressed()));
  connect(_ui->relocateObjects,SIGNAL(pressed()),this,SLOT(relocateObjectsPressed()));
  connect(_ui->showStart,SIGNAL(pressed()),this,SLOT(showStartPressed()));
  connect(_ui->showGoal,SIGNAL(pressed()),this,SLOT(showGoalPressed()));
  connect(_ui->doPlan,SIGNAL(pressed()),this,SLOT(doPlanPressed()));
}

PlanningTab::~PlanningTab(){}

void PlanningTab::GRIPEventSceneLoaded()
{
  mRobot = _world->getSkeleton("GolemHubo");
  cerr << "Robot loaded1" << endl;

  // Set initial configuration for the legs
  int legDofsArray[] = {19, 20, 23, 24, 27, 28};
  vector<int> legDofs(legDofsArray, legDofsArray + 6);
  Eigen::VectorXd legValues(6);
  legValues << -10.0, -10.0, 20.0, 20.0, -10.0, -10.0;
  legValues *= M_PI / 180.0;

  cerr << "Robot loaded2" << endl;
  mRobot->setConfig(legDofs, legValues);
  cerr << "Robot loaded3" << endl;
  // Define right arm nodes
  const string armNodes[] = {"Body_RSP", "Body_RSR", "Body_RSY", "Body_REP", "Body_RWY", "Body_RWP"};
  mArmDofs.resize(6);
  for(int i = 0; i < mArmDofs.size(); i++) {
    cerr<< "i : " << i <<endl;
    cerr << mRobot->getBodyNode(armNodes[i])->getName() << endl;

//    mArmDofs[i] = mRobot->getJoint(armNodes[i])->getGenCoord(0)->getSkeletonIndex();
    mArmDofs[i] = mRobot->getBodyNode(armNodes[i])->getParentJoint()->getGenCoord(0)->getSkeletonIndex();

  }
  //mController = new Controller;


}

/// Before each simulation step we set the torques the controller applies to the joints
void PlanningTab::GRIPEventSimulationBeforeTimestep()
{

  Eigen::VectorXd torques = mController->getTorques(mRobot->get_q(), mRobot->get_dq(), _world->getTime());
  mRobot->setInternalForces(torques);
  cerr << "control" << endl;
}


/// Set start configuration to the configuration the arm is currently in
void PlanningTab::setStartPressed() {
  if(!_world || _world->getNumSkeletons() < 1) {
    cout << "No world loaded or world does not contain a robot." << endl;
    return;
  }

  mStartConf = mRobot->getConfig(mArmDofs);
  cout << "Start Configuration: " << mStartConf.transpose() << endl;
}


/// Set goal configuration to the configuration the arm is currently in
void PlanningTab::setGoalPressed() {
  if(!_world || _world->getNumSkeletons() < 1) {
    cout << "No world loaded or world does not contain a robot." << endl;
    return;
  }

  mGoalConf = mRobot->getConfig(mArmDofs);
  cout << "Goal Configuration: " << mGoalConf.transpose() << endl;
}


/// Reset start configuration to the predefined one
void PlanningTab::setPredefStartPressed() {
  mStartConf = mPredefStartConf;
}


/// Reset goal configuration to the predefined one
void PlanningTab::setPredefGoalPressed() {
  mGoalConf = mPredefGoalConf;
}


/// Move objects to obstruct the direct path between the predefined start and goal configurations
void PlanningTab::relocateObjectsPressed() {

  dart::dynamics::Skeleton* orangeCube = _world->getSkeleton("orangeCube");
  dart::dynamics::Skeleton* yellowCube = _world->getSkeleton("yellowCube");

  if(!orangeCube || !yellowCube) {
    cout << "Did not find orange or yellow object. Exiting and no moving anything" << endl;
    return;
  }

  Eigen::Matrix<double, 6, 1> pose;
  pose << 0.0, 0.0, 0.0, 0.30, -0.30, 0.83;
  orangeCube->setConfig(pose);
  pose << 0.0, 0.0, 0.0, 0.30, -0.30, 0.935;
  yellowCube->setConfig(pose);

  //viewer->DrawGLScene();
}


/// Show the currently set start configuration
void PlanningTab::showStartPressed() {
  cout << "Showing start conf for right arm: " << mStartConf.transpose() << endl;
  //if(!mArmDofs)
      cerr << "mArmDofs not defined yet" << endl;
  //if(!mStartConf)
      cerr << "mStartConf not defined yet" << endl;
  mRobot->setConfig(mArmDofs, mStartConf);
  //viewer->DrawGLScene();
}



/// Show the currently set goal configuration
void PlanningTab::showGoalPressed() {
  cout << "Showing goal conf for right arm: " << mGoalConf.transpose() << endl;
  mRobot->setConfig(mArmDofs, mGoalConf);
  //viewer->DrawGLScene();
}


/// Set initial dynamic parameters and call planner and controller
void PlanningTab::doPlanPressed() {

  // Store the actuated joints (all except the first 6 which are only a convenience to locate the robot in the world)
  std::vector<int> actuatedDofs(mRobot->getNumGenCoords() - 6);
  for(unsigned int i = 0; i < actuatedDofs.size(); i++) {
    actuatedDofs[i] = i + 6;
  }

  // Deactivate collision checking between the feet and the ground during planning
  dart::dynamics::Skeleton* ground = _world->getSkeleton("ground");
  _world->getConstraintHandler()->getCollisionDetector()->disablePair(mRobot->getBodyNode("Body_LAR"), ground->getRootBodyNode());
//  _world->getCollisionHandle()->getCollisionChecker()->disablePair(mRobot->getBodyNode("Body_LAR"), ground->getRootBodyNode());
  _world->getConstraintHandler()->getCollisionDetector()->disablePair(mRobot->getBodyNode("Body_RAR"), ground->getRootBodyNode());
//  _world->getCollisionHandle()->getCollisionChecker()->disablePair(mRobot->getBodyNode("Body_RAR"), ground->getRootBodyNode());

  // Define PD controller gains
  Eigen::VectorXd kI = 100.0 * Eigen::VectorXd::Ones(mRobot->getNumGenCoords());
  Eigen::VectorXd kP = 3000.0 * Eigen::VectorXd::Ones(mRobot->getNumGenCoords());
  Eigen::VectorXd kD = 500.0 * Eigen::VectorXd::Ones(mRobot->getNumGenCoords());
//  Eigen::VectorXd kI = 100.0 * Eigen::VectorXd::Ones(mRobot->getNumGenCoords());
//  Eigen::VectorXd kP = 500.0 * Eigen::VectorXd::Ones(mRobot->getNumGenCoords());
//  Eigen::VectorXd kD = 100.0 * Eigen::VectorXd::Ones(mRobot->getNumGenCoords());

  // Define gains for the ankle PD
  std::vector<int> ankleDofs(2);
  ankleDofs[0] = 27;
  ankleDofs[1] = 28;
  const Eigen::VectorXd anklePGains = -3000.0 * Eigen::VectorXd::Ones(2);
  const Eigen::VectorXd ankleDGains = -400.0 * Eigen::VectorXd::Ones(2);
//  const Eigen::VectorXd anklePGains = -1000.0 * Eigen::VectorXd::Ones(2);
//  const Eigen::VectorXd ankleDGains = -200.0 * Eigen::VectorXd::Ones(2);

  // Set robot to start configuration
  mRobot->setConfig(mArmDofs, mStartConf);

  // Create controller
  mController = new Controller(mRobot, actuatedDofs, kP, kD, ankleDofs, anklePGains, ankleDGains);

  // Call path planner
  dart::planning::PathPlanner<> pathPlanner(*_world);
  std::list<Eigen::VectorXd> path;
  if(!pathPlanner.planPath(mRobot, mArmDofs, mStartConf, mGoalConf, path)) {
    std::cout << "Path planner could not find a path." << std::endl;
  }
  else {
    // Call path shortener
    dart::planning::PathShortener pathShortener(_world, mRobot, mArmDofs);
    pathShortener.shortenPath(path);

    // Convert path into time-parameterized trajectory satisfying acceleration and velocity constraints
    const Eigen::VectorXd maxVelocity = 0.6 * Eigen::VectorXd::Ones(mArmDofs.size());
    const Eigen::VectorXd maxAcceleration = 0.6 * Eigen::VectorXd::Ones(mArmDofs.size());
    dart::planning::Trajectory* trajectory = new dart::planning::PathFollowingTrajectory(path, maxVelocity, maxAcceleration);
    std::cout << "-- Trajectory duration: " << trajectory->getDuration() << endl;
    mController->setTrajectory(trajectory, 0.0, mArmDofs);
  }

  // Reactivate collision of feet with floor
  _world->getConstraintHandler()->getCollisionDetector()->enablePair(mRobot->getBodyNode("Body_LAR"), ground->getRootBodyNode());
//  _world->getCollisionHandle()->getCollisionChecker()->enablePair(mRobot->getBodyNode("Body_LAR"), ground->getRootBodyNode());
  _world->getConstraintHandler()->getCollisionDetector()->enablePair(mRobot->getBodyNode("Body_RAR"), ground->getRootBodyNode());
//  _world->getCollisionHandle()->getCollisionChecker()->enablePair(mRobot->getBodyNode("Body_RAR"), ground->getRootBodyNode());
}


void PlanningTab::GRIPEventSimulationAfterTimestep(){}
void PlanningTab::GRIPEventSimulationStart(){}
void PlanningTab::GRIPEventSimulationStop(){}
void PlanningTab::GRIPEventTreeViewSelectionChanged(){}

void PlanningTab::GRIPEventPlaybackBeforeFrame() {}

/**
 * \brief called from the main window whenever the simulation history slider is being played
 * This method is executed after every playback time step
 */
void PlanningTab::GRIPEventPlaybackAfterFrame() {}

/**
 * \brief called from the main window whenever the simulation history slider is being played
 * This method is executed at the start of the playback
 */
void PlanningTab::GRIPEventPlaybackStart() {}

/**
 * \brief called from the main window whenever the simulation history slider is being played
 * This method is executed at the end of the playback
 */
void PlanningTab::GRIPEventPlaybackStop() {}

void PlanningTab::Refresh() {}

Q_EXPORT_PLUGIN2(PlanningTab, PlanningTab)
