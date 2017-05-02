// Copyright 2017 kvedder@umass.edu, joydeepb@cs.umass.edu
//
// College of Information and Computer Sciences,
// University of Massachusetts Amherst
//
// Constants for UMass RoboCup SSL robots.
//
//========================================================================
// This software is free: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License Version 3,
// as published by the Free Software Foundation.
//
// This software is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// Version 3 in the file COPYING that came with this distribution.
// If not, see <http://www.gnu.org/licenses/>.
//========================================================================
#ifndef SRC_CONSTANTS_CONSTANTS_H_
#define SRC_CONSTANTS_CONSTANTS_H_

#include <string>
#include <vector>

// Used to define if debug behavior should be allowed.
// Example of debug behavior: LOG(FATAL) and quitting upon error versus allow
// undefined behavior.
const bool kProduction = false;

#define DATA_STREAM_VISION_IP ("224.5.23.2")
#define DATA_STREAM_VISION_PORT (10006)

#define DATA_STREAM_CMD_IP ("224.5.23.3")
#define DATA_STREAM_CMD_PORT (10007)

#define DATA_STREAM_REF_IP ("224.5.23.1")
#define DATA_STREAM_REF_PORT (10003)

#define DATA_STREAM_DEBUG_IP ("224.5.10.1")
#define DATA_STREAM_DEBUG_PORT (10008)

#define DATA_STREAM_FEEDBACK_IP ("127.0.0.1")
#define DATA_STREAM_FEEDBACK_PORT (10009)

#define STANDARD_USINGS using Eigen::Vector2f;\
                        using std::endl;\
                        using std::string;\
                        using std::vector;

const unsigned int kMaxTeamRobots = 6;
const unsigned int kNumTeams = 2;

const float kRobotRadius = 90;
const float kBallRadius = 21;

const float kFieldXMax = 4525;
const float kFieldYMax = 2900;

// Max velocity of the robots in mm/s.
const float kMaxVelocity = 4000;
// Max acceleration of the robots in mm/s^2
const float kMaxAcceleration = 7000;

const float kDefaultSafetyMargin = kRobotRadius + 20;


const unsigned int kNumRulesObstacles = 0;
const unsigned int kNumStaticObstacles = 0;
const unsigned int kNumObstacles = kMaxTeamRobots*kNumTeams + 1
                                   + kNumRulesObstacles + kNumStaticObstacles;

const Eigen::Vector2f kOurGoalL(-kFieldXMax, -500);
const Eigen::Vector2f kOurGoalR(-kFieldXMax, 500);
const Eigen::Vector2f kOpponentGoalL(kFieldXMax, 500);
const Eigen::Vector2f kOpponentGoalR(kFieldXMax, -500);

const float kImprobabilityRejectionThreshold = 0.25f;
const float kConfidenceThreshold = 0.5f;

namespace field_dimensions {
const float kFieldLength = 9000.0;
const float kFieldWidth = 6000.0;
const float kHalfFieldLength = 0.5 * kFieldLength;
const float kHalfFieldWidth = 0.5 * kFieldWidth;
const float kGoalWidth = 1000.0;
const float kGoalDepth = 200.0;
const float kBoundaryWidth = 250.0;
const float kFieldLineWidth = 20.0;
const float kDefenseStretch = 500.0;
const float kDefenseRadius = 1000.0;
const float kCenterCircleRadius = 500.0;
const float kFieldBoundary = 300.0;
}  // namespace field_dimensions

#endif  // SRC_CONSTANTS_CONSTANTS_H_
