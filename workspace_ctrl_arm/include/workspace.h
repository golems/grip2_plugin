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
 
#ifndef WALKTAB_H
#define WALKTAB_H

#include "ui_workspace.h"

// Grip includes
#include <grip/qtWidgets/GripTab.h>
#include <grip/osgGolems/ViewerWidget.h>
#include <grip/qtWidgets/TreeViewReturn.h>

// DART includes
#include <vector>
#include <dart/dynamics/Skeleton.h>

enum Ids { LHY = 6, LHR = 7, LHP = 8, LKP = 9, LAP = 10, LAR = 11, RHY = 12, RHR = 13, RHP = 14, 
		RKP = 15, RAP = 16, RAR = 17, LSP = 19, LEP = 22, RSP = 38, REP = 41 };

class WorkSpaceTab : public GripTab
{
    Q_OBJECT
    Q_INTERFACES(GripTab)

public:
    WorkSpaceTab(QWidget *parent = 0);
		void GRIPEventSceneLoaded();
		void Refresh();
private slots:
    void setStartPressed();
    void setGoalPressed();
    
private:
    Ui::WorkSpaceTabWidget *_ui;

};

#endif // PLANNINGTAB_H
