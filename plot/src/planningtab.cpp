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
#include "planningtab.h"
#include <QVector>

#define pc(x) cout << #x << ": " << x << endl;

using namespace std;
using namespace dart;

dynamics::Skeleton* hubo = NULL;

dynamics::Skeleton* selectedSkel = NULL;
dynamics::BodyNode* selectedBody = NULL;

map <QCPGraph*, DartStream*> dartStreams;

/* ******************************************************************************************** */
void PlotTab::drawDartStream (DartStream& stream) {

	// Copy the data, shifting it an index
	QVector <double> curr_vals (NUM_PLOTTING_POINTS);
	for(int i = 0; i < NUM_PLOTTING_POINTS - 1; i++) {
		curr_vals[i] = stream.vals[(stream.index+1+i)%NUM_PLOTTING_POINTS];
	}

	// Get the new data if joint type value is requested
	if(stream.type == Q) {
		assert(stream.body != NULL && "The source cannot be NULL in joint type");
		dynamics::Joint* joint = stream.body->getParentJoint();
		if(dynamic_cast<dart::dynamics::WeldJoint*>(joint)) return;
		curr_vals[NUM_PLOTTING_POINTS - 1] = joint->getGenCoord(0)->get_q();
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
		curr_vals[NUM_PLOTTING_POINTS - 1] = pose(((int) stream.type) - 1);
	}

	// Set the domain 
  QVector<double> domain (NUM_PLOTTING_POINTS);
  for (int i=0; i< NUM_PLOTTING_POINTS; ++i) domain[i] = i;

	// Set the new data
	stream.graph->setData(domain, curr_vals);
	stream.vals[stream.index] = curr_vals[NUM_PLOTTING_POINTS - 1];
	stream.index = (stream.index + 1) % NUM_PLOTTING_POINTS;
}

/* ******************************************************************************************** */
void PlotTab::drawPluginStream (QCPGraph* graph, PluginStream& stream) {
	cout << "plugin latest: " << stream.vals[(stream.index - 1) % NUM_PLOTTING_POINTS] << endl;

	// Copy the data, shifting it an index
	QVector <double> curr_vals (NUM_PLOTTING_POINTS);
	for(int i = 0; i < NUM_PLOTTING_POINTS; i++) 
		curr_vals[i] = stream.vals[(stream.index+i+1)%NUM_PLOTTING_POINTS];

	// Set the domain 
  QVector<double> domain (NUM_PLOTTING_POINTS);
  for (int i=0; i< NUM_PLOTTING_POINTS; ++i) domain[i] = i;

	// Set the new data
	graph->setData(domain, curr_vals);
	stream.vals[stream.index] = curr_vals[NUM_PLOTTING_POINTS - 1];
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

	// Draw each dart graph
	map <QCPGraph*, DartStream*>::iterator it = dartStreams.begin(); 
	for(; it != dartStreams.end(); it++) drawDartStream(*(it->second));

	// Draw each plugin graph
	pthread_mutex_lock(&plottingMutex);
	map <void*, PluginStream*>::iterator it2 = pluginStreams.begin(); 
	for(; it2 != pluginStreams.end(); ) {

		// Create a new graph and change the map
		if(it2->first == NULL) {

			// Create the new graph
			PluginStream* stream = it2->second;
			QCPGraph* graph = _ui->customPlot->addGraph();
			_ui->customPlot->yAxis->setRange(stream->minVal, stream->maxVal);

			// Set plotting options
			QPen graphPen;
			graphPen.setColor(QColor(rand()%245+10, rand()%245+10, rand()%245+10));
			graphPen.setWidthF(rand()/(double)RAND_MAX*2+1);
			graph->setPen(graphPen);
			graph->setName(QString::fromStdString(stream->label));

			// Update the drawing information and draw
			pluginStreams.erase(it2++);
			pluginStreams[graph] = stream; 	
			drawPluginStream(graph, *stream);
		}

		else {

			drawPluginStream((QCPGraph*) it2->first, *(it2->second));
			++it2;
		}

	}
	pthread_mutex_unlock(&plottingMutex);

	// Update the entire plot
  _ui->customPlot->replot();
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

/* ******************************************************************************************** */
void PlotTab::contextMenuRequest(QPoint pos) {

  QMenu *menu = new QMenu(this);
  menu->setAttribute(Qt::WA_DeleteOnClose);
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
			action->setData(1);
		}
		if(selectedBody != NULL) {
			QAction* action = menu->addAction("Add body", this, SLOT(selectDartStream()));
			action->setData(2);
		}
    if(_ui->customPlot->selectedGraphs().size() > 0) {
			menu->addAction("Remove selected graph", this, SLOT(removeSelectedGraph()));
			changeMenu = menu->addMenu(tr("Change data type"));
			QCPGraph* currGraph = _ui->customPlot->selectedGraphs().front();
			DartStream* stream = dartStreams[currGraph];
			PlotDataType type = stream->type;
			bool weldJoint = (dynamic_cast<dart::dynamics::WeldJoint*>(stream->body->getParentJoint()) == NULL);
			if((type != Q) && (stream->skel == NULL) && weldJoint) 
				changeMenu->addAction("joint", this, SLOT(changeStreamType()))->setData(0);
			if(type != X) changeMenu->addAction("x-axis", this, SLOT(changeStreamType()))->setData(1);
			if(type != Y) changeMenu->addAction("y-axis", this, SLOT(changeStreamType()))->setData(2);
			if(type != Z) changeMenu->addAction("z-axis", this, SLOT(changeStreamType()))->setData(3);
			if((type != R) && !(stream->com)) changeMenu->addAction("roll", this, SLOT(changeStreamType()))->setData(4);
			if((type != P) && !(stream->com)) changeMenu->addAction("pitch", this, SLOT(changeStreamType()))->setData(5);
			if((type != YA) && !(stream->com)) changeMenu->addAction("yaw", this, SLOT(changeStreamType()))->setData(6);
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
void PlotTab::setRange (PlotDataType type, dart::dynamics::BodyNode* node) {

	static bool firstTime = true;

	// Determine the min and maximum limits for the range
	double maxVal, minVal;
	if(type == Q) {
		assert((node != NULL) && "To set the range in joint type, the node should not be NULL");
		dynamics::Joint* joint = node->getParentJoint();
		if(dynamic_cast<dart::dynamics::WeldJoint*>(joint)) return;
		maxVal = joint->getGenCoord(0)->get_qMax();
		minVal = joint->getGenCoord(0)->get_qMin();
	}
	else if((type == X) || (type == Y) || (type == Z)) {
		maxVal = 2;
		minVal = -2;
	}
	else if((type == R) || (type == P) || (type == YA)) {
		maxVal = 2*M_PI;
		minVal = -2*M_PI;
	}

	// Get the min and mix of the range from the last time
	pthread_mutex_lock(&plottingMutex);
	if((!firstTime) || !pluginStreams.empty()) {
		double lastMaxVal = _ui->customPlot->yAxis->range().upper;
		double lastMinVal = _ui->customPlot->yAxis->range().lower;
		maxVal = max(maxVal, lastMaxVal);
		minVal = min(minVal, lastMinVal);
	}
	pthread_mutex_unlock(&plottingMutex);

	// Set the range
	_ui->customPlot->yAxis->setRange(minVal, maxVal);
	firstTime = false;
}

/* ******************************************************************************************** */
const char* PlotTab::getTypeName (PlotDataType type) {
	switch(type) {
		case Q: return "Q";
		case X: return "X";
		case Y: return "Y";
		case Z: return "Z";
		case R: return "R";
		case P: return "P";
		case YA: return "YA";
		default: assert(false && "not possible");
	};
}

/* ******************************************************************************************** */
void PlotTab::changeStreamType () {

	// Get the data type
	QAction* contextAction = qobject_cast<QAction*>(sender());
	if(!contextAction) return;
	bool ok;
	int dataInt = contextAction->data().toInt(&ok);
	if(!ok) return;

	// Set the type
	QCPGraph* currGraph = _ui->customPlot->selectedGraphs().front();
	DartStream* stream = dartStreams[currGraph];
	stream->type = (PlotDataType) dataInt;

	// Set the range
	setRange(stream->type, stream->body);

	// Update the name
	char buf [16];
	if(stream->skel != NULL) 
		sprintf(buf, "%s_%s%s", stream->skel->getName().c_str(), stream->com ? "COM_" : "", 
			getTypeName(stream->type));
	else if(stream->body != NULL)
		sprintf(buf, "%s_%s", stream->body->getName().c_str(), getTypeName(stream->type));
	currGraph->setName(QString::fromStdString(string(buf)));
}

/* ******************************************************************************************** */
void PlotTab::selectDartStream () {

	// Get the data type
	QAction* contextAction = qobject_cast<QAction*>(sender());
	if(!contextAction) return;
	bool ok;
	int dataInt = contextAction->data().toInt(&ok);
	if(!ok) return;

	// Determine the data type for the selection
	PlotDataType type;
	if(dynamic_cast<dart::dynamics::WeldJoint*>(selectedBody->getParentJoint())) type = Z;
	else if(dataInt == 2) type = Q;
	else type = Z;

	// Create a new graph, set its range, set default values
	QCPGraph* graph = _ui->customPlot->addGraph();
	QVector <double> vals (NUM_PLOTTING_POINTS);
  for (int i=0; i < NUM_PLOTTING_POINTS; ++i) vals[i] = 0.0;
	if(dataInt == 2) setRange(type, selectedBody);
	else setRange(type);

	// Set properties to the new graph
  QPen graphPen;
  graphPen.setColor(QColor(rand()%245+10, rand()%245+10, rand()%245+10));
  graphPen.setWidthF(rand()/(double)RAND_MAX*2+1);
	graph->setPen(graphPen);

	// Create the stream and add it to the list
	DartStream* stream = new DartStream (graph, selectedSkel, selectedBody, type, false);
  for (int i=0; i < NUM_PLOTTING_POINTS; ++i) stream->vals.push_back(0.0);
	dartStreams[graph] = stream;

	// Set com option
	if(dataInt == 1) stream->com = true;

	// Set the name
	char buf [16];
	if(stream->skel != NULL) 
		sprintf(buf, "%s_%s%s", stream->skel->getName().c_str(), stream->com ? "COM_" : "", 
			getTypeName(stream->type));
	else if(stream->body != NULL)
		sprintf(buf, "%s_%s", stream->body->getName().c_str(), getTypeName(stream->type));
	graph->setName(QString::fromStdString(string(buf)));
}

/* ******************************************************************************************** */
void PlotTab::removeSelectedGraph() {
  if (_ui->customPlot->selectedGraphs().size() > 0) {
	
		// Remove the stream
		QCPGraph* currGraph = _ui->customPlot->selectedGraphs().front();
		map <QCPGraph*, DartStream*>::iterator it = dartStreams.find(currGraph);
		assert((it != dartStreams.end()) && "A graph without a stream shouldn't be possible");
		dartStreams.erase(it);

		// Remove graph
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
  _ui->customPlot->xAxis->setRange(0, 100);
  _ui->customPlot->yAxis->setRange(0, 1);
  _ui->customPlot->axisRect()->setupFullAxesBox();

	// Setup the legend
  _ui->customPlot->legend->setVisible(true);
  QFont legendFont = font();
  legendFont.setPointSize(10);
  _ui->customPlot->legend->setFont(legendFont);
  _ui->customPlot->legend->setSelectedFont(legendFont);
  _ui->customPlot->legend->setSelectableParts(QCPLegend::spItems); 
	_ui->customPlot->axisRect()->insetLayout()->setInsetAlignment(0, (Qt::Alignment)(Qt::AlignTop|Qt::AlignLeft));

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
