/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author: Mohit Khatri <mkhatri7@gatech.edu>
 * Date: Mar 2014
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

#ifndef COMPARATOR_H
#define COMPARATOR_H

#include <math.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <vector>


namespace fsp {

/*!
  * \class FootLocationComparator
  * \brief Comparator used by the priority queue. Compares the given nodes based on their distance to the goal location. 
  *
  */
class FootLocationComparator
{
    public:
        FootLocationComparator(Eigen::Vector2d value, int weight) {
                _goal = value;
                _weight  = weight;
        }

        bool operator() (const FootLocationNode* lhs, const FootLocationNode* rhs) const {
                bool result = false;
                //Find the lhs distance to the goal.
                double lhs_distance = sqrt(pow((_goal[0] - (lhs->getLocation())[0]), 2.0) + pow((_goal[1] - (lhs->getLocation())[1]), 2.0));
                double rhs_distance = sqrt(pow((_goal[0] - (rhs->getLocation())[0]), 2.0) + pow((_goal[1] - (rhs->getLocation())[1]), 2.0));
                double lhs_cost = 0;
                double rhs_cost = 0;
                if (_weight == 1) {
                    lhs_cost = lhs_distance;
                    rhs_cost = rhs_distance;
                } else {
                    lhs_cost = lhs->getCost() + _weight*lhs_distance;
                    rhs_cost = rhs->getCost() + _weight*rhs_distance;
                }

                if (lhs_cost >rhs_cost) {
                        result = true;
                }
                return result;
        }

        private:
        Eigen::Vector2d _goal;
        int _weight;
    };
} // namespace fsp
#endif
