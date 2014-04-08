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
 
#ifndef PLANNINGTAB_H
#define PLANNINGTAB_H

#include "ui_planningtab.h"

// Grip includes
#include <grip/qtWidgets/GripTab.h>
#include <grip/osgGolems/ViewerWidget.h>
#include <grip/qtWidgets/TreeViewReturn.h>

// DART includes
#include <vector>
#include <dart/dynamics/Skeleton.h>
//namespace dart {
//  namespace dynamics { class Skeleton; }
//}

class Controller;

/* ******************************************************************************************** */
// Data to update

#define NUM_DATA 101

enum PlotDataType { Q = 0, X, Y, Z, R, P, YA }; 

/// The definition of source to follow at the update function
struct DartStream {
	dart::dynamics::Skeleton* skel;
	dart::dynamics::BodyNode* body;
	PlotDataType type;	
	bool com;
	QCPGraph* graph;
	QVector <double> vals;
	DartStream(QCPGraph* g, dart::dynamics::Skeleton* s, dart::dynamics::BodyNode* b, PlotDataType t, 
		bool c) : skel(s), body(b), type(t), com(c), graph(g) {}
};

/* ******************************************************************************************** */
class PlotTab : public GripTab
{
    Q_OBJECT
    Q_INTERFACES(GripTab)

public:
		QMenu* skelMenu;
		QMenu* changeMenu;
    PlotTab(QWidget *parent = 0);
    void Refresh();
		void GRIPEventSceneLoaded();
		void GRIPEventTreeViewSelectionChanged();
		QVector<double> x, y;
		QTimer timer;
		void setRange (PlotDataType type, dart::dynamics::BodyNode* node = NULL);
		void draw ();
		void drawDartStream(DartStream& stream);
		const char* getTypeName (PlotDataType type);

protected Q_SLOTS:
		void update ();
		void contextMenuRequest (QPoint pos);

		void removeAllGraphs ();
		void removeSelectedGraph ();

		void changeStreamType();
		void selectDartStream();
		void drawPlugin();

		void selectionChanged();
		void moveLegend ();
		void mouseWheel ();
		void mousePress ();
private:
    Ui::PlotTabWidget *_ui;

};

#endif // PLANNINGTAB_H
