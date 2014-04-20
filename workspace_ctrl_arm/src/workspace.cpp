/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author: Can Erdogan <cerdogan3@gatech.edu>, Sungmoon Joo <sungmoon.joo@gmail.com>
 * Date: April 2014
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

#include "workspace.h"
#include <qplugin.h>
#include <QtGui>

using namespace std;

/* ********************************************************************************************* */
typedef Eigen::Matrix<double,6,1> Vector6d;
dart::dynamics::Skeleton* hubo;
Vector6d dx = Vector6d::Zero();

/* ********************************************************************************************* */
void WorkSpaceTab::update () {

	if(hubo == NULL) return;

	// Compute the Jacobian
	std::cout << "\n============================================================" << endl;
	std::cout << "dx: " << dx.transpose() << std::endl;
	Eigen::MatrixXd J = hubo->getBodyNode("Body_RWR")->getWorldJacobian().bottomRightCorner<6,6>();
	Eigen::MatrixXd temp = J.topRightCorner<3,6>();
	J.topRightCorner<3,6>() = J.bottomRightCorner<3,6>();
	J.bottomRightCorner<3,6>() = temp;
	for(size_t i = 0; i < 6; i++) J(i,i) += 0.005;
	std::cout << "J: [\n" << J << "]\n";

	// Compute the inverse
	Eigen::Matrix6d JInv = J.inverse();
	//JInv = J;
	//aa_la_inv(6, JInv.data());
	std::cout << "JInv: [\n" << JInv << "]\n";

	// Compute the joint space velocity from workspace velocity
	static const double max_step_size = 0.020;
	Vector6d dq = (JInv * dx);
	double norm = dq.norm();
	if(norm > 1e-4) dq = max_step_size * (dq / norm);
	else dq = Vector6d::Zero();
	std::cout << "dq: " << dq.transpose() << std::endl;

	// Update the joint command
	std::vector <int> rarm_ids;
	for(size_t i = 38; i < 44; i++) rarm_ids.push_back(i);
	Vector6d state = hubo->getConfig(rarm_ids);
	Vector6d next = state + dq;
	cout << "next: " << next.transpose() << endl;
	hubo->setConfig(rarm_ids, next);
	return;
}

/* ********************************************************************************************* */
WorkSpaceTab::WorkSpaceTab(QWidget *parent) : _ui(new Ui::WorkSpaceTabWidget){
	_ui->setupUi(this);
  connect(_ui->setStart,SIGNAL(pressed()),this,SLOT(setStartPressed()));
  connect(_ui->setGoal,SIGNAL(pressed()),this,SLOT(setGoalPressed()));
	connect(&timer, SIGNAL(timeout()), this, SLOT(update()));
	timer.start(100);
}

/* ********************************************************************************************* */
void WorkSpaceTab::GRIPEventSceneLoaded() {

	hubo = _world->getSkeleton("Hubo");
	if(hubo == NULL) return;

	// Set the initial config
	Eigen::VectorXd desiredDofs = hubo->getConfig();
	desiredDofs[RSP] = -M_PI / 6.0;
	desiredDofs[REP] = -2.0*M_PI / 3.0;
	desiredDofs[RWP] = M_PI / 3.0;
	hubo->setConfig(desiredDofs);
}

/* ********************************************************************************************* */
void WorkSpaceTab::Refresh() {
}

/* ********************************************************************************************* */
void WorkSpaceTab::setStartPressed() {
	dx(0) += 0.02;
}

/* ********************************************************************************************* */
void WorkSpaceTab::setGoalPressed() {
	dx(0) -= 0.02;
}

Q_EXPORT_PLUGIN2(WorkSpaceTab, WorkSpaceTab)
