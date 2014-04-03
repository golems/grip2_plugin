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
#include "helpers.h"
#include "walktab.h"
#include <qplugin.h>
#include <QtGui>

/* ********************************************************************************************* */
Eigen::MatrixXd mKp, mKd;
size_t numDofs;
dart::dynamics::Skeleton* hubo;
Eigen::VectorXd dx (6);
Eigen::VectorXd mTorques;

bool start = false;
bool moveDirectionSet = false;

int stepCounter = 0;
int lastId;
Eigen::Vector3d nextpos;
double stepSize;
double forwardStart;
double motionLimit;
Eigen::VectorXd desiredDofs;
Mode mode = NONE;
Foot foot;
Category cat = Prepare;
vector <Eigen::Vector2d> wayPoints;
double currentHeight = 0.0;
double lastHeight = 0.0;

/* ********************************************************************************************* */
WalkTab::WalkTab(QWidget *parent) : _ui(new Ui::WalkTabWidget){
    _ui->setupUi(this);
  connect(_ui->setStart,SIGNAL(pressed()),this,SLOT(setStartPressed()));
  connect(_ui->setGoal,SIGNAL(pressed()),this,SLOT(setGoalPressed()));
}

/* ********************************************************************************************* */
void WalkTab::GRIPEventSceneLoaded() {

	hubo = _world->getSkeleton("Hubo");
	_world->getSkeleton("Obstacle")->setMobile(false);
	numDofs = hubo->getNumGenCoords();
	stepSize = 0.20;

	// Set the initial config
	desiredDofs = hubo->getConfig();
	desiredDofs[LEP] = -M_PI_2 * 1.5;
	desiredDofs[REP] = -M_PI_2 * 1.5;
	hubo->setConfig(desiredDofs);
	// viewer->DrawGLScene();

	// Set the way points
	wayPoints.push_back(Eigen::Vector2d(0.4, 0.2));
	wayPoints.push_back(Eigen::Vector2d(0.8, 0.0));

	// Set the position and derivative gains
  mKp = 3000.0 * Eigen::MatrixXd::Identity(numDofs, numDofs);
  mKd = 400.0 * Eigen::MatrixXd::Identity(numDofs, numDofs);
	for (int i = 0; i < 6; i++) {
		mKp(i, i) = 0.0;
		mKd(i, i) = 0.0;
  }
}

/* ********************************************************************************************* */
void WalkTab::setTorques(const Eigen::VectorXd& desiredDofs) {

	static const double kI = 0.0;
	static int counter = 0;
	static const int errorsSize = 10;
	static vector <Eigen::VectorXd> errors;
	if(counter == 0) {
		for(size_t i = 0; i < errorsSize; i++) {
			Eigen::VectorXd zero = Eigen::VectorXd::Zero(numDofs);
			errors.push_back(zero);
		}
	}

    // Get the timestep: Modified to remove amino dependencies
    static double time_last = _world->getTime();
    double time_now = _world->getTime();
    double time_delta = 0.001;
    //Original version with amino
//    static double time_last = aa_tm_timespec2sec(aa_tm_now());
//    double time_now = aa_tm_timespec2sec(aa_tm_now());
//    double time_delta = time_now - time_last;

	time_last = time_now;

	// Get the external forces
	Eigen::VectorXd mConstrForces = _world->getConstraintHandler()->getTotalConstraintForce(1);
    //cout << "id 1: " << _world->getSkeleton(1)->getName().c_str() << endl;
	if(mConstrForces.rows() == 0) {
		cout << "No constr forces!!!!!! " << endl;
		return;
	}
	

	// SPD tracking             
	Eigen::VectorXd dof = hubo->getConfig();
	Eigen::VectorXd dofVel = hubo->get_dq();
	Eigen::MatrixXd invM = (hubo->getMassMatrix() + mKd * time_delta).inverse();
	Eigen::VectorXd error = (dof + dofVel * time_delta - desiredDofs);
	errors[counter++ % errorsSize] = error;
	Eigen::VectorXd p = -mKp * error;
	Eigen::VectorXd d = -mKd * dofVel;
	Eigen::VectorXd qddot = invM * (-hubo->getCombinedVector() + p + d + mConstrForces); 
	mTorques = p + d - mKd * qddot * time_delta;

	// Add integral term to the torques
	Eigen::VectorXd totalError = Eigen::VectorXd::Zero(numDofs);
	for(size_t i = 0; i < errorsSize; i++) totalError += errors[i];
	totalError /= errorsSize;
	for(size_t i = 0; i < numDofs; i++) {
		mTorques[i] += totalError[i] * kI;
	}

	// Just to make sure no illegal torque is used    
	for (int i = 0; i < 6; i++) mTorques[i] = 0.0;

	// Set the torques
	for(size_t i = LHY; i < LAR; i++) {
		if(fabs(mTorques(i)) > 10000.0) {
			mTorques(i) = 10000.0 * (mTorques(i) / fabs(mTorques(i)));
			cout << "Limiting joint " << i - LHY << " on left leg" << endl;
		}
	}
	for(size_t i = RHY; i < RAR; i++) {
		if(fabs(mTorques(i)) > 10000.0) {
			mTorques(i) = 10000.0 * (mTorques(i) / fabs(mTorques(i)));
			cout << "Limiting joint " << i - RHY << " on right leg" << endl;
		}
	}
	hubo->setInternalForces(mTorques);
}

/* ********************************************************************************************* */
void WalkTab::moveFoot(const Eigen::VectorXd& dx, bool left, size_t period, bool sameFrame) {

	static int counter = 0;
	if(!sameFrame) counter++;
	if((counter % period != 0)) return;

	// Get the jacobian
	Eigen::MatrixXd J = hubo->getBodyNode(left ? "leftFoot" : "rightFoot")
		->getWorldJacobian().bottomRightCorner<6,6>();
	Eigen::MatrixXd temp = J.topRightCorner<3,6>();
	J.topRightCorner<3,6>() = J.bottomRightCorner<3,6>();
	J.bottomRightCorner<3,6>() = temp;
	for(size_t i = 0; i < 6; i++) J(i,i) += 0.005;

	// Compute the inverse
	Eigen::MatrixXd JInv;
	JInv = J;
    //Modified to remove amino dependencies
    JInv = JInv.inverse();
    //Original with amino
    //aa_la_inv(6, JInv.data());

	// Compute joint space velocity
	Eigen::VectorXd dq = (JInv * dx);
	if(dq.norm() > 0.5) {
		cout << "WARNING. Wanted too fast!" << endl;
		return;
	}

	// Apply velocity
	for(size_t i = 0; i < 6; i++) {
		if(isnan(dq(i))) return;
		desiredDofs[(left ? 6 : 12) + i] += dq(i);
	}
}

/* ********************************************************************************************* */
void WalkTab::GRIPEventSimulationBeforeTimestep() { 
	
	static Mode lastMode = mode;
	if(lastMode != mode) cout << "new mode: " << modeStr(mode).c_str() << endl;
	lastMode = mode;

	// Check if the robot fell down
	Eigen::Vector3d com = hubo->getWorldCOM();
	if(com(2) < 0.60) {
		cout << "fell down" << endl;
		// wxCommandEvent evt;
		// frame->OnSimulateStop(evt);
	}

	// Get the current pose of the main foot
	BodyNode* leftFoot = hubo->getBodyNode("leftFoot"), *rightFoot = hubo->getBodyNode("rightFoot");
	Eigen::Vector3d prim = ((foot == LEFT) ? leftFoot : rightFoot)->getWorldTransform().translation();
	Eigen::Vector3d sec = ((foot == LEFT) ? rightFoot : leftFoot) ->getWorldTransform().translation();

	// Set the desired workspace velocities and decide on end cases based on the mode
	bool reachedGoal = false;
	switch(mode) {

		// =========================================================
		case Up: {

			// Speed up once contact is broken with ground
			double effectiveHeight = (cat == Climb) ? 0.0 : currentHeight;
			if(cat == Descend) effectiveHeight = lastHeight;
			if((foot == LEFT) && (prim(2) > (effectiveHeight + 0.01))) dx << -0.003, 0.0, 0.010, 0.0, 0.0, 0.0;
			else dx << 0.0, 0.0, 0.0005, 0.0, 0.0, 0.0;

			// Avoid collision with the obstacle
			if((foot == RIGHT) && (prim(2) > (effectiveHeight + 0.02))) dx << -0.003, 0.0, 0.010, 0.0, 0.0, 0.0;
			else if((foot == RIGHT)) dx << 0.000, 0.0, 0.0005, 0.0, 0.0, 0.0;

			// Check for end case
			reachedGoal = (prim(2) > motionLimit);
		} break;

		// =========================================================
		case Shift: {

			// Set the move direction
			if(!moveDirectionSet) {
				Eigen::Vector3d diff = prim - com;
				dx << diff(0), diff(1), 0.0, 0.0, 0.0, 0.0;
				dx = -dx.normalized() * 0.005;
				moveDirectionSet = true;
			}

			// Check for end case
			reachedGoal = (com(0) > (prim(0) - 0.01)) && 
										((foot == LEFT) ? (com(1) > (prim(1) - 0.01)) : (com(1) < (prim(1) + 0.01)));
			if(reachedGoal)	moveDirectionSet = false;

		} break;

		// =========================================================
		case ToPos: {

			// Set the velocity
			Eigen::Vector3d err = (nextpos - prim);
			Eigen::Vector3d dir = err.normalized() * 0.010;
			dx << dir(0), dir(1), dir(2), 0.0, 0.0, 0.0;
			
			// Check for end case
			reachedGoal = (err.norm() < 0.02);

		} break;

		// =========================================================
		case Forward: {
			dx << 0.020, 0.0, 0.0, 0.0, 0.0, 0.0;
			if(fabs(prim(1) - sec(1)) < 0.14) 	
				dx << 0.010, (foot == RIGHT) ? -0.001 : 0.001, 0.0, 0.0, 0.0, 0.0;
			reachedGoal = (prim(0) > motionLimit);
		} break;

		// =========================================================
		case Backward: {
			dx << -0.001, 0.0, 0.0, 0.0, 0.0, 0.0;
			reachedGoal = (com(0) > motionLimit);
		} break;

		// =========================================================
		case Down: {
			dx << 0.0, 0.0, -0.0100, 0.0, 0.0, 0.0;
			if(fabs(prim(1) - sec(1)) < 0.14) dx(1) = (foot == RIGHT) ? -0.001 : 0.001;
			reachedGoal = (prim(2) < motionLimit);
		} break;

		// =========================================================
		case PushDown: {
			dx << 0.0, 0.0, -0.0020, 0.0, 0.0, 0.0;
			reachedGoal = (com(2) > motionLimit);
		} break;

		// =========================================================
		case Shrink: {
			dx << 0.0, 0.0, 0.0020, 0.0, 0.0, 0.0;
			reachedGoal = (com(2) < motionLimit);
		} break;

		// =========================================================
		case NONE: {
			if(lastId < 1000) 
				reachedGoal = ((desiredDofs - hubo->getConfig()).cwiseAbs().maxCoeff() < 0.2);
			else reachedGoal = false;
		} break;
	}

	// Set the desired DOFs if necessary
	if(!reachedGoal) {
		moveFoot(dx, foot == LEFT);
		if(mode == Shift)	moveFoot(dx, foot != LEFT, 50, true);
	}

	// Set the torques for the given DOFs
	if(start) setTorques(desiredDofs);

	// Change the mode
	if(reachedGoal) {

		// Set the next id and call the mode based on the category
		static int repeatCounter = 0;
		int nextId;
		// pc(stepCounter);
		if((stepCounter++ % 500) == 0) {
			bool catDone = false;
			nextId = lastId + 1;
			pc(cat);
			switch(cat) {
				case Prepare:	catDone = prepare(nextId); break;
				case Walk: catDone = walk(nextId % 4); break;
				case Climb: catDone = climb(nextId); break;
				case Descend: catDone = descend(nextId); break;
			};
			if(catDone) {
				if(cat == Prepare) {
					cat = Walk;
					walk(0);
				}
				else if(cat == Walk) {
					if(currentHeight > lastHeight) cat = Climb;
					if(currentHeight < lastHeight) {
						cat = Descend;
						descend(0);
					}
				}
				else if(cat == Climb) {
					cat = Walk;
					foot = otherFoot(foot);
					walk(0);
				}
				else if(cat == Descend) {
					cat = Walk;
					walk(0);
				}
				lastId = 0;
				cout << "new cat: " << cat << endl;
			}
		}
	}
}

/* ********************************************************************************************* */
void WalkTab::Refresh() {
}

/* ********************************************************************************************* */
void WalkTab::setStartPressed() {
	start = true;
	lastId = 1233;
}

/* ********************************************************************************************* */
void WalkTab::setGoalPressed() {
	prepare(0);
	cat = Prepare;
	stepCounter = 0;
	stepSize = 0.20;
}

Q_EXPORT_PLUGIN2(WalkTab, WalkTab)
