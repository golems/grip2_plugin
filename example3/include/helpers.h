/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
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
//#include <amino.h>
#include <dart/constraint/ConstraintDynamics.h>
#include <dart/dynamics/Skeleton.h>
#include <dart/dynamics/BodyNode.h>
#include <dart/dynamics/Joint.h>
#include <dart/dynamics/RevoluteJoint.h>
#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <dart/simulation/World.h>

#define pv(x) cout << #x << ": " << (x).transpose() << endl;
#define pc(x) cout << #x << ": " << x << endl;
#define D2R(x) (((x) / 180.0) * M_PI)
#define otherFoot(x) ((x == LEFT) ? RIGHT : LEFT)
using namespace std;

/* ********************************************************************************************* */
enum Category { Prepare, Walk, Climb, Descend, Stop };
enum Mode { Forward, Up, Backward, Down, ToPos, Shift, PushDown, Shrink, NONE	};
enum Foot { LEFT = 0, RIGHT = 1 };
enum Ids { LHY = 6, LHR = 7, LHP = 8, LKP = 9, LAP = 10, LAR = 11, RHY = 12, RHR = 13, RHP = 14, 
		RKP = 15, RAP = 16, RAR = 17, LSP = 19, LEP = 22, RSP = 38, REP = 41 };

static const double boxHeight = 0.20;

extern double stepSize; 
extern int lastId;
extern dart::dynamics::Skeleton* hubo;
extern Eigen::Vector3d nextpos;
extern double motionLimit;
extern Eigen::VectorXd desiredDofs;
extern vector <Eigen::Vector2d> wayPoints;
extern Mode mode;
extern Foot foot;
extern double currentHeight;
extern double lastHeight;

using namespace dart::dynamics;

/* ********************************************************************************************* */

/// The finite automata for descending
bool descend (size_t step);

/// The finite automata for climbing
bool climb (size_t step);

/// The finite automata for walking
bool walk (size_t step);

/// The finite automata for preparing for walking
bool prepare (size_t step);

/// Prints the contact forces
void printContactForces ();

/// Prints the mode
std::string modeStr(Mode mode);
