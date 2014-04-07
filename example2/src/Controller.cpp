/**
 * @file Controller.cpp
 * @author T. Kunz
 */

#include <iostream>
#include <vector>
#include <fstream>
#include <Eigen/Dense>

#include "Controller.h"
#include <dart/dynamics/Skeleton.h>
#include <dart/planning/Trajectory.h>
#include <dart/dynamics/GenCoord.h>
#include <dart/dynamics/BodyNode.h>
#include <grip/qtWidgets/GripTab.h>
#include <dart/simulation/World.h>
#include <dart/constraint/ConstraintDynamics.h>


using namespace std;
using namespace Eigen;

Controller::Controller(dart::dynamics::Skeleton* _skel, const vector<int> &_actuatedDofs,
                       const VectorXd &_kP, const VectorXd &_kD, const VectorXd &_kI,const vector<int> &_ankleDofs, const VectorXd &_anklePGains, const VectorXd &_ankleDGains) :
    mSkel(_skel),
    mKp(_kP.asDiagonal()),
    mKd(_kD.asDiagonal()),
    mKi(_kI.asDiagonal()),
    mAnkleDofs(_ankleDofs),
    mAnklePGains(_anklePGains),
    mAnkleDGains(_ankleDGains),
    mTrajectory(NULL)
{
    const int nDof = mSkel->getNumGenCoords();

    mSelectionMatrix = MatrixXd::Zero(nDof, nDof);
    for (int i = 0; i < _actuatedDofs.size(); i++) {
        mSelectionMatrix(_actuatedDofs[i], _actuatedDofs[i]) = 1.0;
    }

    mDesiredDofs.resize(nDof);
    for (int i = 0; i < nDof; i++) {
        mDesiredDofs[i] = mSkel->getGenCoord(i)->get_q();
    }

    Vector3d com = mSkel->getWorldCOM();
    double cop = 0.0;
    mPreOffset = com[0] - cop;
}


void Controller::setTrajectory(const dart::planning::Trajectory* _trajectory, double _startTime, const std::vector<int> &_dofs) {
    mTrajectoryDofs = _dofs;
    mTrajectory = _trajectory;
    mStartTime = _startTime;
}


//Version 2
VectorXd Controller::getTorques(const VectorXd& _dof, const VectorXd& _dofVel, double _time) {
    Eigen::VectorXd desiredDofVels = VectorXd::Zero(mSkel->getNumGenCoords());

    if(mTrajectory && _time - mStartTime >= 0.0 & _time - mStartTime <= mTrajectory->getDuration()) {
        for(unsigned int i = 0; i < mTrajectoryDofs.size(); i++) {
            mDesiredDofs[mTrajectoryDofs[i]] = mTrajectory->getPosition(_time - mStartTime)[i];
            desiredDofVels[mTrajectoryDofs[i]] = mTrajectory->getVelocity(_time - mStartTime)[i];
        }
    }
    
    VectorXd torques;
    const double mTimestep = 0.001;

    // Get the external forces
//    Eigen::VectorXd mConstrForces = _world->getConstraintHandler()->getTotalConstraintForce(1);
//    cout << "id 1: " << mSkel->getName().c_str() << endl;
//    if(mConstrForces.rows() == 0) {
//        cout << "No constr forces!!!!!! " << endl;
//        return;
//    }

    const int nDof = mSkel->getNumGenCoords();
    // SPD controller
    // J. Tan, K. Liu, G. Turk. Stable Proportional-Derivative Controllers. IEEE Computer Graphics and Applications, Vol. 31, No. 4, pp 34-44, 2011.
    MatrixXd M = mSkel->getMassMatrix() + mKd * mTimestep;
    VectorXd p = -mKp * (_dof - mDesiredDofs + _dofVel * mTimestep);
    VectorXd d = -mKd * (_dofVel - desiredDofVels);
    VectorXd qddot = M.ldlt().solve(-mSkel->getCombinedVector() + p + d);// + mConstrForces);
    torques = p + d - mKd * qddot * mTimestep;

//    static int counter = 0;
//    static const int errorsSize = 50;
//    static vector <Eigen::VectorXd> errors;
//    if(counter == 0) {
//        for(size_t i = 0; i < errorsSize; i++) {
//            Eigen::VectorXd zero = Eigen::VectorXd::Zero(nDof);
//            errors.push_back(zero);
//        }
//    }
//    Eigen::VectorXd error = (_dof + _dofVel * mTimestep - mDesiredDofs);
//    errors[counter++ % errorsSize] = error;

//    // Add integral term to the torques

//    Eigen::VectorXd totalError = Eigen::VectorXd::Zero(nDof);
//    for(size_t i = 0; i < errorsSize; i++) totalError += errors[i];
//    //totalError /= errorsSize;
//    for(size_t i = 0; i < nDof; i++) {
//        std::cerr << "error: " << totalError[i] * mKi(i) << std::endl;
//        torques(i) += totalError[i] * mKi(i);
//    }

    double maxTorque = 100.0;

    for (int i = 0;i<nDof;i++){
        if (fabs(torques(i)) > maxTorque) {
            //torques(i) = maxTorque*(torques(i)/fabs(torques(i)));
            std::cerr << "Torque limit" << std::endl;
        }
    }


    // ankle strategy for sagital plane
    Vector3d com = mSkel->getWorldCOM();
    double cop = 0.0;
    double offset = com[0] - cop;

//    for(unsigned int i = 0; i < mAnkleDofs.size(); i++) {
//        torques[mAnkleDofs[i]] = - mAnklePGains[i] * offset - mAnkleDGains[i] * (offset - mPreOffset) / mTimestep;
//    }

    mPreOffset = offset;

    return mSelectionMatrix * torques;
}

//Verstion 1
VectorXd Controller::setTorques(const Eigen::VectorXd& _dof, const Eigen::VectorXd& _dofVel, double _time, dart::simulation::World *mWorld)
{
    Eigen::VectorXd desiredDofVels = VectorXd::Zero(mSkel->getNumGenCoords());

    static const double kI = 0.0;
    static int counter = 0;
    static const int errorsSize = 10;
    static vector <Eigen::VectorXd> errors;
    int numDofs = mSkel->getNumGenCoords();

    if(counter == 0) {
        for(size_t i = 0; i < errorsSize; i++) {
            Eigen::VectorXd zero = Eigen::VectorXd::Zero(numDofs);
            errors.push_back(zero);
        }
    }

    double time_delta = 0.001;


    // Get the external forces
    Eigen::VectorXd mConstrForces = mWorld->getConstraintHandler()->getTotalConstraintForce(1);
    //cout << "id 1: " << _world->getSkeleton(1)->getName().c_str() << endl;
    if(mConstrForces.rows() == 0) {
        cout << "No constr forces!!!!!! " << endl;
        return VectorXd::Zero(mSkel->getNumGenCoords());
    }


    // SPD tracking
    Eigen::VectorXd mTorques;

    if(mTrajectory && _time - mStartTime >= 0.0 & _time - mStartTime <= mTrajectory->getDuration()) {
        for(unsigned int i = 0; i < mTrajectoryDofs.size(); i++) {
            mDesiredDofs[mTrajectoryDofs[i]] = mTrajectory->getPosition(_time - mStartTime)[i];
            desiredDofVels[mTrajectoryDofs[i]] = mTrajectory->getVelocity(_time - mStartTime)[i];
        }
    }

    Eigen::MatrixXd invM = (mSkel->getMassMatrix() + mKd * time_delta).inverse();
    Eigen::VectorXd error = (_dof + _dofVel * time_delta - mDesiredDofs);
    errors[counter++ % errorsSize] = error;
    Eigen::VectorXd p = -mKp * error;
    Eigen::VectorXd d = -mKd * (_dofVel - desiredDofVels);
    Eigen::VectorXd qddot = invM * (-mSkel->getCombinedVector() + p + d + mConstrForces);
    mTorques = p + d - mKd * qddot * time_delta;

    // Add integral term to the torques
    Eigen::VectorXd totalError = Eigen::VectorXd::Zero(numDofs);
    for(size_t i = 0; i < errorsSize; i++) totalError += errors[i];
    totalError /= errorsSize;
    for(size_t i = 0; i < numDofs; i++) {
        mTorques[i] += totalError[i] * kI;
    }

    // Just to make sure no illegal torque is used
//    for (int i = 0; i < 6; i++) mTorques[i] = 0.0;

    // Set the torques
    for(size_t i = mSkel->getBodyNode("Body_LHY")->getSkeletonIndex(); i < mSkel->getBodyNode("Body_LAR")->getSkeletonIndex(); i++) {
        if(fabs(mTorques(i)) > 10000.0) {
            mTorques(i) = 10000.0 * (mTorques(i) / fabs(mTorques(i)));
            cout << "Limiting joint " << i - mSkel->getBodyNode("Body_LHY")->getSkeletonIndex() << " on left leg" << endl;
        }
    }
    for(size_t i = mSkel->getBodyNode("Body_RHY")->getSkeletonIndex(); i < mSkel->getBodyNode("Body_RAR")->getSkeletonIndex(); i++) {
        if(fabs(mTorques(i)) > 10000.0) {
            mTorques(i) = 10000.0 * (mTorques(i) / fabs(mTorques(i)));
            cout << "Limiting joint " << i - mSkel->getBodyNode("Body_RHY")->getSkeletonIndex() << " on right leg" << endl;
        }
    }

    return mTorques;
}
