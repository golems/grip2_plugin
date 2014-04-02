/**
 * @file helpers.h
 * @author Can Erdogan
 * @date Mar 25, 2014
 * @brief Helper functions.
 */
#include <amino.h>
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
