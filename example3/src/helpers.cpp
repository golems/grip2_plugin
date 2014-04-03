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

double obstacleLoc = 0.2;

/* ********************************************************************************************* */
void lift () {
	vector <int> z_idx;
	z_idx.push_back(5);
	Eigen::VectorXd z = hubo->getConfig(z_idx);
	z(0) += 1e-3;
	hubo->setConfig(z_idx, z);
}

/* ********************************************************************************************* */
bool descend (size_t step) {

	cout << "descend step: " << step << endl;
	cout << "descend foot: " << ((foot == LEFT) ? "LEFT" : "RIGHT") << endl;

	// Get the feet
	Eigen::Vector3d com = hubo->getWorldCOM();
	BodyNode* leftFoot = hubo->getBodyNode("leftFoot"), *rightFoot = hubo->getBodyNode("rightFoot");
	BodyNode* lead = (foot == LEFT) ? leftFoot : rightFoot, *stable= (foot != LEFT) ? leftFoot : rightFoot;

	switch(step) {

		case 0: {
			nextpos = Eigen::Vector3d(obstacleLoc + 0.10, ((foot == LEFT) ? 0.09 : -0.09), (lastHeight + 0.03));
			mode = ToPos;
		} break;
		case 1: {
			foot = otherFoot(foot);
			mode = Shrink;
			motionLimit = com(2) - 0.2;
		} break;
		case 2: {
			foot = otherFoot(foot);
			mode = Down;
			motionLimit = currentHeight + 0.010;
		} break;
		case 3: {
			mode = Shift;
		} break;
		case 4: {
			lift();
			foot = otherFoot(foot);
			mode = Up;
			motionLimit = lastHeight + 0.03;
		} break;
		case 5: {
			mode = Forward;
			Eigen::Vector3d temp = stable->getWorldTransform().translation();
			motionLimit = temp(0) - 0.03;
		} break;
		case 6: {
			mode = Down;
			motionLimit = currentHeight + 0.05;
		} break;
		case 7: {
			return true;
		} break;
	};

	lastId = step;
	return false;
}

/* ********************************************************************************************* */
bool climb (size_t step) {

	cout << "climb foot: " << ((foot == LEFT) ? "LEFT" : "RIGHT") << endl;
	Eigen::Vector3d com = hubo->getWorldCOM();

	// Get the feet
	BodyNode* leftFoot = hubo->getBodyNode("leftFoot"), *rightFoot = hubo->getBodyNode("rightFoot");
	BodyNode* lead = (foot == LEFT) ? leftFoot : rightFoot, *stable= (foot != LEFT) ? leftFoot : rightFoot;

	// Decide on the actions
	pc(step);
	switch(step) {

		// Bring left more forward - match height of the obstacle 
		case 0: {
			mode = Up;
			motionLimit = boxHeight + 0.03;
		} break;
		case 1: {
			Eigen::Vector3d temp = stable->getWorldTransform().translation();
			nextpos = Eigen::Vector3d(temp(0), ((foot == LEFT) ? 0.09 : -0.09), (boxHeight + 0.03));
			mode = ToPos;
		} break;
		case 2: {
			nextpos = Eigen::Vector3d(obstacleLoc + 0.05, ((foot == LEFT) ? 0.09 : -0.09), (boxHeight + 0.03));
			mode = ToPos;
		} break;
		case 3: {
			mode = Backward;
			foot = otherFoot(foot);
			motionLimit = com(0) + 0.05;
		} break;
		case 4: {
			foot = otherFoot(foot);
			mode = Down;
			motionLimit = boxHeight + 0.005;
		} break;
		case 5: {
			mode = Shift;
		} break;
		case 6: {
			lift();
			foot = otherFoot(foot);
			mode = Up;
			motionLimit = boxHeight + 0.03;
		} break;
		case 7: {
			Eigen::Vector3d temp = stable->getWorldTransform().translation();
			nextpos = Eigen::Vector3d(temp(0), ((foot == LEFT) ? 0.09 : -0.09), (boxHeight + 0.03));
			mode = ToPos;
		} break;
		case 8: {
			foot = otherFoot(foot);
			mode = PushDown;
			motionLimit = 0.73 + currentHeight;
		} break;
		case 9: {
			return true;
		} break;
	};

	lastId = step;
	return false;
}

/* ********************************************************************************************* */
bool walk (size_t step) {

	static const double footLength = 0.16;
	cout << "walk step: " << step << endl;
	cout << "walk foot: " << ((foot == LEFT) ? "LEFT" : "RIGHT") << endl;

	// Declare the walk done if step was already shortened and repeat is asked
	static size_t wayPointIdx = 0;
	static bool shortenedStep = false;
	static bool liftedFoot = false;
	if((step == 0) && shortenedStep && liftedFoot) {
		shortenedStep = liftedFoot = false;
		lastHeight = currentHeight;
		currentHeight = wayPoints[wayPointIdx](1); 
		cout << "currentHeight: " << currentHeight << ", lastHeight: " << lastHeight << endl;
		return true;
	} 

	// Get the feet
	BodyNode* leftFoot = hubo->getBodyNode("leftFoot"), *rightFoot = hubo->getBodyNode("rightFoot");
	BodyNode* lead = (foot == LEFT) ? leftFoot : rightFoot, *stable= (foot != LEFT) ? leftFoot : rightFoot;

	// Decide on the actions
	switch(step) {

		// Move lead foot forward
		case 0: {

			// Decide on the step size
			Eigen::Vector3d temp = stable->getWorldTransform().translation();
			double nextWayPoint;
			for(size_t i = 0; i < wayPoints.size(); i++) {
				double dist = wayPoints[i](0) - (temp(0) + footLength);
				if(dist < 0.0) continue;
				if(dist < 0.25) {
					wayPointIdx = i;
					obstacleLoc = wayPoints[i](0);
					stepSize = dist - 0.06;	
					shortenedStep = true;
					pc(stepSize);
					break;
				}
			}

			// Set the inputs
			mode = Forward;
			motionLimit = temp(0) + stepSize;
		} break;

		// Put lead down
		case 1: {
			mode = Down;
			motionLimit = 0.020 + currentHeight;
		} break;

		// Shift mass
		case 2: {
			lift();
			mode = Shift;
		} break;

		// Raise back 
		case 3: {
			lift();
			mode = Up;
			motionLimit = 0.05 + currentHeight;
			foot = otherFoot(foot);
			liftedFoot = true;
		} break;
	};

	lastId = step;
	return false;
}

/* ********************************************************************************************* */
bool prepare (size_t step) {

	switch(step) {

		case 0: {
			static const double angle = (-25.0 / 180.0) * M_PI;
			desiredDofs[RHP] = angle;
			desiredDofs[RKP] = -2*angle;
			desiredDofs[RAP] = angle;
			desiredDofs[LHP] = angle;
			desiredDofs[LKP] = -2*angle;
			desiredDofs[LAP] = angle;
			mode = NONE;
		} break;
		case 3:
		case 2:
		case 1: {
			static const double angle = (-3.3 / 180.0) * M_PI;
			desiredDofs[RHR] += -angle;
			desiredDofs[RAR] += angle;
			desiredDofs[LHR] += -angle;
			desiredDofs[LAR] += angle;
			mode = NONE;
		} break;
		case 4: {
			mode = Up;
			foot = LEFT;
			motionLimit = 0.05 + currentHeight;
		} break;
		case 5: {
			cout << "Prepare returning true" << endl;
			return true;
		} break;
	};

	lastId = step;
	return false;
}

/* ********************************************************************************************* *
void printContactForces () {

	// Print the average forces under the feet
	static size_t counter = 0;
	Eigen::Vector3d leftAve = Eigen::Vector3d::Zero(), rightAve = Eigen::Vector3d::Zero();
	int nContacts = mWorld->getConstraintHandler()->getCollisionDetector()->getNumContacts();
	int numLeftContacts = 0, numRightContacts = 0;
	BodyNode* leftAnkle = hubo->getBodyNode("Body_LAR"), *rightAnkle = hubo->getBodyNode("Body_RAR");
	if(counter % 100 == 0) printf("\n");
	for (int k = 0; k < nContacts; k++) {
		dart::collision::Contact contact = mWorld->getConstraintHandler()->getCollisionDetector()->getContact(k);
		if(0 && (counter % 100 == 0)) {
			printf("'%s' vs. '%s': ", contact.collisionNode1->getBodyNode()->getName().c_str(), 
				contact.collisionNode2->getBodyNode()->getName().c_str());
			cout << contact.force.transpose() << endl;
		}
		if(contact.collisionNode1->getBodyNode() == leftAnkle || contact.collisionNode2->getBodyNode() == leftAnkle) {
			if(contact.force.norm() > 0.1) {
				leftAve += contact.force;
				numLeftContacts++;
			}
		}   
		if(contact.collisionNode1->getBodyNode() == rightAnkle || contact.collisionNode2->getBodyNode() == rightAnkle) {
			if(contact.force.norm() > 0.1) {
				numRightContacts++;
				rightAve += contact.force;
			}
		}   
	}  
	if(numLeftContacts != 0) leftAve /= numLeftContacts;
	if(numRightContacts != 0) rightAve /= numRightContacts;
	if(counter++ % 100 == 0) {
		printf("\n# contacts: %d, # left: %d, # right: %d\n", nContacts, numLeftContacts, numRightContacts);
		pv(leftAve);
		pv(rightAve);
		cout << "average z: " << (leftAve(2) + rightAve(2)) << endl;
	}
	
	// Clear the average force if it is negative
	if(leftAve(2) < 0.0) {
		leftAnkle->clearContactForces();
		leftAnkle->clearExternalForces();
	}
	if(rightAve(2) < 0.0) {
		rightAnkle->clearContactForces();
		rightAnkle->clearExternalForces();
	}
}

/* ********************************************************************************************* */
std::string modeStr(Mode mode) {
	switch(mode) {
		case Forward: return "Forward";
		case Up: return "Up";
		case Backward: return "Backward";
		case Down: return "Down";
		case ToPos: return "ToPos";
		case Shift: return "Shift";
		case PushDown: return "PushDown";
		case Shrink: return "Shrink";
		case NONE: return "NONE";
	}; 
}
