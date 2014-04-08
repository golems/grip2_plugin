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

#define pc(x) cout << #x << ": " << x << endl;

using namespace std;
using namespace dart;

dynamics::Skeleton* hubo = NULL;
QCustomPlot* plot;

dynamics::Skeleton* selectedSkel = NULL;
dynamics::BodyNode* selectedBody = NULL;

vector <DartStream> dartStreams;


/* ******************************************************************************************** */
void PlotTab::drawDartStream (DartStream& stream) {

	// Copy the data, shifting it an index
	QVector <double> curr_vals (NUM_DATA);
	for(size_t i = 0; i < NUM_DATA - 1; i++) curr_vals[i] = stream.vals[i+1];

	// Get the new data if joint type value is requested
	if(stream.type == Q) {
		assert(stream.body != NULL && "The source cannot be NULL in joint type");
		curr_vals[NUM_DATA - 1] = stream.body->getParentJoint()->getGenCoord(0)->get_q();
	}

	// Get the new data if another type is requested
	else {

		// Get the pose of the object (either skeleton or body)
		Eigen::Isometry3d tf;
		Eigen::Matrix<double, 6, 1> pose = Eigen::Matrix<double, 6, 1>::Zero();
		if(stream.skel != NULL) {
			if(stream.com) pose.head<3>() = stream.skel->getWorldCOM();
			else {
				tf = stream.skel->getRootBodyNode()->getWorldTransform();
				pose.head<3>() = tf.translation();
				pose.tail<3>() = dart::math::matrixToEulerXYZ(tf.linear());
			}
		}
		else {
			tf = stream.body->getWorldTransform();
			pose.head<3>() = tf.translation();
			pose.tail<3>() = dart::math::matrixToEulerXYZ(tf.linear());
		}

		// Set the value
		curr_vals[NUM_DATA - 1] = pose(((int) stream.type) - 1);
	}

	// Set the domain 
  QVector<double> domain (NUM_DATA);
  for (int i=0; i< NUM_DATA; ++i) domain[i] = i;

	// Set the new data
	stream.graph->setData(domain, curr_vals);
	stream.vals = curr_vals;
}

/* ******************************************************************************************** */
void PlotTab::update () {

	if(!hubo) return;

	// Move hubo around for testing
	Eigen::VectorXd qs = hubo->getConfig();
	static int counter = 0;
	counter+=10;
	qs(19) = sin(M_PI/180*counter);
	qs(22) = -(cos(M_PI/180*counter) + 1) * 1.0;
	hubo->setConfig(qs);

	// Draw each graph
	for(size_t i = 0; i < dartStreams.size(); i++) drawDartStream(dartStreams[i]);
  _ui->customPlot->replot();

	return;

	// Draw something
	draw();
}

/* ******************************************************************************************** */
void PlotTab::GRIPEventTreeViewSelectionChanged () {

	// Update the selected skeleton or body information
	if(_activeNode->dType == Return_Type_Robot) {
		selectedSkel = ((dart::dynamics::Skeleton*)(_activeNode->object));
		selectedBody = NULL;
	}
	else if(_activeNode->dType == Return_Type_Node) {
		selectedBody = ((dart::dynamics::BodyNode*)(_activeNode->object));
		selectedSkel = NULL;
	}
	else {
		selectedSkel = NULL;
		selectedBody = NULL;
	}
}

/* ******************************************************************************************** *
void PlotTab::draw() {
	
	// Some static variables
	static dynamics::BodyNode* node;
	static int lastBodyIdx = -1;
	static int lastOptionIdx = -1;

	// =================================================================================
	// Change plot ranges

	// Update the graph if the body or option is changed
	static bool updatePlot = false;
	int bodyIdx = _ui->nodeBox->currentIndex();
	int optionIdx = _ui->dofBox->currentIndex();
	int customOptionIdx = _ui->customBox->currentIndex();
	pc(bodyIdx);
	pc(optionIdx);
	pc(customOptionIdx);
	if((customOptionIdx == 0) && ((bodyIdx != lastBodyIdx) || (optionIdx != lastOptionIdx))) {

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
			updatePlot = true;

		}
	}
	
	// Update the graph with custom information
	if(customOptionIdx != 0) {


	}

	// Update the last data point
	if(updatePlot) {

		if(customOptionIdx == 0) {

			// Determine the value if joint is chosen
			if(optionIdx == 0) y[100] = node->getParentJoint()->getGenCoord(0)->get_q();

			// Determine the value if (x,y,z) or (r,p,y) is chosen
			else {
				Eigen::Matrix<double, 6, 1> pose = Eigen::Matrix<double, 6, 1>::Zero();
				const Eigen::Isometry3d& tf = node->getWorldTransform();
				pose.head<3>() = tf.translation();
				pose.tail<3>() = dart::math::matrixToEulerXYZ(tf.linear());
				y[100] = pose(optionIdx-1);
			}
		}
	}

	cout << "updatePlot: " << updatePlot << endl;
	// Update the rest of the data
	if(updatePlot) {
		for (int i=0; i<100; ++i) y[i] = y[i+1];
		plot->graph(0)->setData(x, y);
		plot->replot();
	}
}

/* ******************************************************************************************** */
void PlotTab::addRandomGraph() {
  int n = 50; // number of points in graph
  double xScale = (rand()/(double)RAND_MAX + 0.5)*2;
  double yScale = (rand()/(double)RAND_MAX + 0.5)*2;
  double xOffset = (rand()/(double)RAND_MAX - 0.5)*4;
  double yOffset = (rand()/(double)RAND_MAX - 0.5)*5;
  double r1 = (rand()/(double)RAND_MAX - 0.5)*2;
  double r2 = (rand()/(double)RAND_MAX - 0.5)*2;
  double r3 = (rand()/(double)RAND_MAX - 0.5)*2;
  double r4 = (rand()/(double)RAND_MAX - 0.5)*2;
  QVector<double> x(n), y(n);
  for (int i=0; i<n; i++)
  {
    x[i] = (i/(double)n-0.5)*10.0*xScale + xOffset;
    y[i] = (sin(x[i]*r1*5)*sin(cos(x[i]*r2)*r4*3)+r3*cos(sin(x[i])*r4*2))*yScale + yOffset;
  }
  
  _ui->customPlot->addGraph();
  _ui->customPlot->graph()->setName(QString("New graph %1").arg(_ui->customPlot->graphCount()-1));
  _ui->customPlot->graph()->setData(x, y);
  _ui->customPlot->graph()->setLineStyle((QCPGraph::LineStyle)(rand()%5+1));
  if (rand()%100 > 75)
    _ui->customPlot->graph()->setScatterStyle(QCPScatterStyle((QCPScatterStyle::ScatterShape)(rand()%9+1)));
  QPen graphPen;
  graphPen.setColor(QColor(rand()%245+10, rand()%245+10, rand()%245+10));
  graphPen.setWidthF(rand()/(double)RAND_MAX*2+1);
  _ui->customPlot->graph()->setPen(graphPen);
  _ui->customPlot->replot();
}

/* ******************************************************************************************** */
void PlotTab::contextMenuRequest(QPoint pos) {

  QMenu *menu = new QMenu(this);
  menu->setAttribute(Qt::WA_DeleteOnClose);
	menu->addAction("Add random graph", this, SLOT(addRandomGraph()));
  if (_ui->customPlot->legend->selectTest(pos, false) >= 0) {
    menu->addAction("Move to top left", this, 
			SLOT(moveLegend()))->setData((int)(Qt::AlignTop|Qt::AlignLeft));
    menu->addAction("Move to top right", this, 
			SLOT(moveLegend()))->setData((int)(Qt::AlignTop|Qt::AlignRight));
    menu->addAction("Move to bottom right", this, 
			SLOT(moveLegend()))->setData((int)(Qt::AlignBottom|Qt::AlignRight));
    menu->addAction("Move to bottom left", this, 
			SLOT(moveLegend()))->setData((int)(Qt::AlignBottom|Qt::AlignLeft));
    menu->addAction("Remove legend", this, 
			SLOT(moveLegend()))->setData(1024);
  } 
	else {
		if(selectedSkel != NULL) {
			skelMenu = menu->addMenu(tr("Add skeleton"));
			QAction* action = skelMenu->addAction("Use origin", this, SLOT(selectDartStream()));
			action->setData(0);
			action = skelMenu->addAction("Use COM", this, SLOT(selectDartStream()));
			action->setData(0);
		}
		if(selectedBody != NULL) {
			QAction* action = menu->addAction("Add body", this, SLOT(selectDartStream()));
			action->setData(1);
		}
    if(_ui->customPlot->selectedGraphs().size() > 0) {
			menu->addAction("Remove selected graph", this, SLOT(removeSelectedGraph()));
			changeMenu = menu->addMenu(tr("Change data type"));
			changeMenu->addAction("x-axis", this, SLOT(moveLegend()))->setData(1025);
			changeMenu->addAction("y-axis", this, SLOT(moveLegend()))->setData(1025);
			changeMenu->addAction("z-axis", this, SLOT(moveLegend()))->setData(1025);
			changeMenu->addAction("roll", this, SLOT(moveLegend()))->setData(1025);
			changeMenu->addAction("pitch", this, SLOT(moveLegend()))->setData(1025);
			changeMenu->addAction("yaw", this, SLOT(moveLegend()))->setData(1025);
		}
    if(_ui->customPlot->graphCount() > 0) 
			menu->addAction("Remove all graphs", this, SLOT(removeAllGraphs()));
		if((_ui->customPlot->graphCount()) > 0 && (!_ui->customPlot->legend->visible()))
			menu->addAction("Add legend", this, SLOT(moveLegend()))->setData(1025);
  }

	QAction* action = menu->addAction("Plugin information", this, SLOT(moveLegend()));
	action->setEnabled(false);

  menu->popup(_ui->customPlot->mapToGlobal(pos));
}

/* ******************************************************************************************** */
void PlotTab::selectDartStream() {

	// Get the data type
	QAction* contextAction = qobject_cast<QAction*>(sender());
	if(!contextAction) return;
	bool ok;
	int dataInt = contextAction->data().toInt(&ok);
	if(!ok) return;

	// Create the stream and add it to the list
	QCPGraph* graph = _ui->customPlot->addGraph();
	DartStream stream (graph, selectedSkel, selectedBody, Z, false);

  QVector<double> x (NUM_DATA); 
	stream.vals = QVector <double> (NUM_DATA);
  for (int i=0; i<101; ++i) {
		x[i] = i;
		stream.vals[i] = 0.0;
  }
  _ui->customPlot->xAxis->setRange(0, 100);
  _ui->customPlot->yAxis->setRange(0, 1);
	graph->setData(x,stream.vals);
  _ui->customPlot->replot();
	dartStreams.push_back(stream);

	// Set the range for the stream
}

/* ******************************************************************************************** */
void PlotTab::removeSelectedGraph() {
  if (_ui->customPlot->selectedGraphs().size() > 0) {
    _ui->customPlot->removeGraph(_ui->customPlot->selectedGraphs().first());
    _ui->customPlot->replot();
  }
}

/* ******************************************************************************************** */
void PlotTab::removeAllGraphs() {
  _ui->customPlot->clearGraphs();
  _ui->customPlot->replot();
	dartStreams.clear();
}

/* ******************************************************************************************** */
void PlotTab::selectionChanged() {

  // Make top and bottom axes be selected synchronously, and handle axis and tick labels as one 
	// selectable object:
  if (_ui->customPlot->xAxis->selectedParts().testFlag(QCPAxis::spAxis) || 
			_ui->customPlot->xAxis->selectedParts().testFlag(QCPAxis::spTickLabels) ||
      _ui->customPlot->xAxis2->selectedParts().testFlag(QCPAxis::spAxis) || 
			_ui->customPlot->xAxis2->selectedParts().testFlag(QCPAxis::spTickLabels)) {
    _ui->customPlot->xAxis2->setSelectedParts(QCPAxis::spAxis|QCPAxis::spTickLabels);
    _ui->customPlot->xAxis->setSelectedParts(QCPAxis::spAxis|QCPAxis::spTickLabels);
  }

  // Make left and right axes be selected synchronously, and handle axis and tick labels as one 
	// selectable object:
  if (_ui->customPlot->yAxis->selectedParts().testFlag(QCPAxis::spAxis) || 
			_ui->customPlot->yAxis->selectedParts().testFlag(QCPAxis::spTickLabels) ||
      _ui->customPlot->yAxis2->selectedParts().testFlag(QCPAxis::spAxis) || 
			_ui->customPlot->yAxis2->selectedParts().testFlag(QCPAxis::spTickLabels)) {
    _ui->customPlot->yAxis2->setSelectedParts(QCPAxis::spAxis|QCPAxis::spTickLabels);
    _ui->customPlot->yAxis->setSelectedParts(QCPAxis::spAxis|QCPAxis::spTickLabels);
  }
  
  // Synchronize selection of graphs with selection of corresponding legend items:
  for (int i=0; i<_ui->customPlot->graphCount(); ++i) {
    QCPGraph *graph = _ui->customPlot->graph(i);
    QCPPlottableLegendItem *item = _ui->customPlot->legend->itemWithPlottable(graph);
    if (item->selected() || graph->selected()) {
      item->setSelected(true);
      graph->setSelected(true);
    }
  }
}

/* ******************************************************************************************** */
void PlotTab::mousePress() {

  // If an axis is selected, only allow the direction of that axis to be dragged
  if (_ui->customPlot->xAxis->selectedParts().testFlag(QCPAxis::spAxis))
    _ui->customPlot->axisRect()->setRangeDrag(_ui->customPlot->xAxis->orientation());
  else if (_ui->customPlot->yAxis->selectedParts().testFlag(QCPAxis::spAxis))
    _ui->customPlot->axisRect()->setRangeDrag(_ui->customPlot->yAxis->orientation());

  // If no axis is selected, both directions may be dragged
  else
    _ui->customPlot->axisRect()->setRangeDrag(Qt::Horizontal|Qt::Vertical);
}

/* ******************************************************************************************** */
void PlotTab::mouseWheel() {

  // If an axis is selected, only allow the direction of that axis to be zoomed
  if (_ui->customPlot->xAxis->selectedParts().testFlag(QCPAxis::spAxis))
    _ui->customPlot->axisRect()->setRangeZoom(_ui->customPlot->xAxis->orientation());
  else if (_ui->customPlot->yAxis->selectedParts().testFlag(QCPAxis::spAxis))
    _ui->customPlot->axisRect()->setRangeZoom(_ui->customPlot->yAxis->orientation());

  // If no axis is selected, both directions may be zoomed
  else
    _ui->customPlot->axisRect()->setRangeZoom(Qt::Horizontal|Qt::Vertical);
}

/* ******************************************************************************************** */
void PlotTab::moveLegend() {

  if (QAction* contextAction = qobject_cast<QAction*>(sender())) {
    bool ok;
    int dataInt = contextAction->data().toInt(&ok);
    if(ok && (dataInt < 1024)) {
      _ui->customPlot->axisRect()->insetLayout()->setInsetAlignment(0, (Qt::Alignment)dataInt);
      _ui->customPlot->replot();
    }
		else if(ok && dataInt == 1024) {
			_ui->customPlot->legend->setVisible(false);
      _ui->customPlot->replot();
		}
		else if(ok && dataInt == 1025) {
			_ui->customPlot->legend->setVisible(true);
      _ui->customPlot->replot();
		}
  }
}

/* ******************************************************************************************** */
PlotTab::PlotTab(QWidget *parent) : _ui(new Ui::PlotTabWidget) {

	// Setup the ui
	_ui->setupUi(this);
  _ui->customPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes |
                                  QCP::iSelectLegend | QCP::iSelectPlottables);
  _ui->customPlot->xAxis->setRange(-8, 8);
  _ui->customPlot->yAxis->setRange(-5, 5);
  _ui->customPlot->axisRect()->setupFullAxesBox();

	// Setup the legend
  _ui->customPlot->legend->setVisible(true);
  QFont legendFont = font();
  legendFont.setPointSize(10);
  _ui->customPlot->legend->setFont(legendFont);
  _ui->customPlot->legend->setSelectedFont(legendFont);
  _ui->customPlot->legend->setSelectableParts(QCPLegend::spItems); 
  connect(_ui->customPlot, SIGNAL(legendDoubleClick(QCPLegend*,QCPAbstractLegendItem*,QMouseEvent*)), this, 
		SLOT(legendDoubleClick(QCPLegend*,QCPAbstractLegendItem*)));

	// Set slots for interaction
  _ui->customPlot->setContextMenuPolicy(Qt::CustomContextMenu);
  connect(_ui->customPlot, SIGNAL(customContextMenuRequested(QPoint)), this, SLOT(contextMenuRequest(QPoint)));
  connect(_ui->customPlot, SIGNAL(selectionChangedByUser()), this, SLOT(selectionChanged()));
  connect(_ui->customPlot, SIGNAL(mousePress(QMouseEvent*)), this, SLOT(mousePress()));
  connect(_ui->customPlot, SIGNAL(mouseWheel(QWheelEvent*)), this, SLOT(mouseWheel()));

	// Create the timer to visualize the graph as events happen
	connect(&timer, SIGNAL(timeout()), this, SLOT(update()));
	timer.start(100);



	return;







//	_ui->nodeBox->addItem("None");
//	_ui->customBox->addItem("None");

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
  plot->xAxis->setRange(0, 100);
  plot->yAxis->setRange(0, 1);
}

/* ******************************************************************************************** */
void PlotTab::GRIPEventSceneLoaded() {

	// Add the items to the combo box
	hubo = _world->getSkeleton("Hubo");
/*
	if(hubo == NULL) return;
	for(size_t i = 0; i < hubo->getNumBodyNodes(); i++) {
		dynamics::BodyNode* body = hubo->getBodyNode(i);
		_ui->nodeBox->addItem(body->getName().c_str());	
	}
*/
}

/* ******************************************************************************************** */
void PlotTab::Refresh() {

}

Q_EXPORT_PLUGIN2(PlotTab, PlotTab)
