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
#include <dart/dynamics/WeldJoint.h>
#include <dart/dynamics/BodyNode.h>
#include <dart/collision/CollisionDetector.h>
#include <dart/constraint/ConstraintDynamics.h>
#include <dart/dynamics/BoxShape.h>
#include <dart/dynamics/GenCoord.h>
#include <dart/planning/PathPlanner.h>
#include <dart/planning/PathShortener.h>
#include <dart/planning/PathFollowingTrajectory.h>
#include "swingingtab.h"
#include <QVector>

#define pc(x) cout << #x << ": " << x << endl;

using namespace std;
using namespace dart;

dynamics::Skeleton* hubo = NULL;
bool action = false;

/* ******************************************************************************************** */
void SwingTab::act () {
	if(!hubo) return;
	if(action) {
		Eigen::VectorXd desiredDofs = hubo->getConfig();
		desiredDofs[5] = 0.962;
		desiredDofs[19] = 0.0;
		desiredDofs[22] = -M_PI_2 * 1.5;
		desiredDofs[41] = -M_PI_2 * 1.5;
		hubo->setConfig(desiredDofs);
		action = false;
	}
	else action = true;
}

/* ******************************************************************************************** */
void SwingTab::update () {

	// Move hubo around for testing
	if(!hubo) return;
	if(!action) return;
	Eigen::VectorXd qs = hubo->getConfig();
	static int counter = 0;
	counter+=10;
	qs(19) = sin(M_PI/180*counter);
	qs(22) = -(cos(M_PI/180*counter) + 1) * 1.0;
	hubo->setConfig(qs);
}

/* ******************************************************************************************** */
SwingTab::SwingTab(QWidget *parent) : _ui(new Ui::SwingTabWidget) {
	_ui->setupUi(this);
 	connect(&timer, SIGNAL(timeout()), this, SLOT(update()));
  connect(_ui->button,SIGNAL(pressed()),this,SLOT(act()));
	timer.start(100);
}

/* ******************************************************************************************** */
void SwingTab::GRIPEventSceneLoaded() {
	hubo = _world->getSkeleton("Hubo");
}

/* ******************************************************************************************** */
void SwingTab::Refresh() { }

Q_EXPORT_PLUGIN2(SwingTab, SwingTab)
