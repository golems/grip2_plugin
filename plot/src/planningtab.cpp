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
#include "planningtab.h"
#include <QVector>

using namespace std;
using namespace dart;

dynamics::Skeleton* hubo = NULL;
QCustomPlot* plot;


/* ******************************************************************************************** */
void PlotTab::update() {
	
	if(!hubo) return;

	// Some static variables
	static dynamics::BodyNode* node;
	static int lastBodyIdx = -1;
	static int lastOptionIdx = -1;

	// Update the graph if the body or option is changed
	int bodyIdx = _ui->nodeBox->currentIndex();
	int optionIdx = _ui->dofBox->currentIndex();
	if((bodyIdx != lastBodyIdx) || (optionIdx != lastOptionIdx)) {

		// Get the node
		string currName = _ui->nodeBox->itemText(bodyIdx).toStdString();
		node = hubo->getBodyNode(currName);

		// Set the limits based on the option
		if(node != NULL) {

			// Determine the limits
			double maxVal, minVal;
			if(optionIdx == 0) {
				maxVal = node->getParentJoint()->getGenCoord(0)->get_qMax();
				minVal = node->getParentJoint()->getGenCoord(0)->get_qMin();
			}
			else if(optionIdx < 4) {
				maxVal = 2;
				minVal = -2;
			}
			else {
				maxVal = 2*M_PI;
				minVal = -2*M_PI;
			}

			// Set the limits
			plot->yAxis->setRange(minVal, maxVal);
			lastBodyIdx = bodyIdx;
			lastOptionIdx = optionIdx;

			// Reset the value
			for (int i=0; i<100; ++i) y[i] = 0.0;
		}
	}
	
	// Move hubo around for testing
	Eigen::VectorXd qs = hubo->getConfig();
	static int counter = 0;
	counter+=10;
	qs(19) = sin(M_PI/180*counter);
	qs(22) = -(cos(M_PI/180*counter) + 1) * 1.0;
	hubo->setConfig(qs);
	
	// Update the last data based on the option or reset it to zero
	if(node != NULL) {
		double value = 0.0;

		// Determine the value if joint is chosen
		if(optionIdx == 0) value = node->getParentJoint()->getGenCoord(0)->get_q();

		// Determine the value if (x,y,z) or (r,p,y) is chosen
		else {
			Eigen::Matrix<double, 6, 1> pose = Eigen::Matrix<double, 6, 1>::Zero();
			const Eigen::Isometry3d& tf = node->getWorldTransform();
			pose.head<3>() = tf.translation();
			pose.tail<3>() = dart::math::matrixToEulerXYZ(tf.linear());
			value = pose(optionIdx-1);
		}

		// Set the value
		y[100] = value;
	}
	else y[100] = 0.0;

	// Update the rest of the data
  for (int i=0; i<100; ++i) y[i] = y[i+1];
	plot->graph(0)->setData(x, y);
	plot->replot();
}

/* ******************************************************************************************** */
PlotTab::PlotTab(QWidget *parent) : _ui(new Ui::PlotTabWidget) {

	// Setup the ui
	_ui->setupUi(this);
	_ui->nodeBox->addItem("None");

	// Create the timer to visualize the graph as events happen
	connect(&timer, SIGNAL(timeout()), this, SLOT(update()));
	timer.start(100);

	// Add the default graph
  _ui->customPlot->addGraph();
	plot = _ui->customPlot;
  x = QVector<double> (101);
  y = QVector<double> (101);
  for (int i=0; i<101; ++i) {
		x[i] = i;
		y[i] = 0.0;
  }
  plot->xAxis->setLabel("time");
  plot->yAxis->setLabel("y");
  plot->xAxis->setRange(0, 100);
  plot->yAxis->setRange(0, 1);
}

/* ******************************************************************************************** */
void PlotTab::GRIPEventSceneLoaded() {

	// Add the items to the combo box
	hubo = _world->getSkeleton("Hubo");
	if(hubo == NULL) return;
	for(size_t i = 0; i < hubo->getNumBodyNodes(); i++) {
		dynamics::BodyNode* body = hubo->getBodyNode(i);
		_ui->nodeBox->addItem(body->getName().c_str());	
	}
}

/* ******************************************************************************************** */
void PlotTab::Refresh() {

}

Q_EXPORT_PLUGIN2(PlotTab, PlotTab)
