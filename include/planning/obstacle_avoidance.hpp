/********************************************************************************
 * Copyright (c) 2026 Contributors to the Eclipse Foundation
 *
 * SPDX-License-Identifier: EPL-2.0
 ********************************************************************************/

#pragma once

#include "adore_map/route.hpp"
#include "dynamics/traffic_participant.hpp"
#include "dynamics/trajectory.hpp"
#include "planning/trajectory_planner.hpp"

#include <string>

namespace adore
{
namespace planner
{

struct ObstacleAvoidanceParams
{
  double max_object_ahead             = 45.0;
  double max_static_object_speed      = 0.5;
  double side_clearance               = 0.6;
  double front_clearance              = 8.0;
  double rear_clearance               = 8.0;
  double max_lateral_shift            = 3.2;
  double min_lateral_shift            = 0.5;
  double in_lane_shift_limit          = 1.2;
  double min_obstacle_route_overlap   = 0.5;
  double oncoming_lookahead_after_obj = 35.0;
  double min_oncoming_heading_diff    = 2.35; // rad, about 135 deg
  double stop_time_step               = 0.3;
};

enum class ObstacleAvoidanceMode
{
  None,
  InLaneShift,
  OvertakeLeft,
  WaitForOncoming
};

struct ObstacleAvoidanceResult
{
  bool success = false;
  std::string reason;
  ObstacleAvoidanceMode mode = ObstacleAvoidanceMode::None;

  map::Route modified_route;
  dynamics::Trajectory trajectory;
};

/**
 * Obstacle avoidance for the current eclipse-adore/bugfix/revert_to_make API.
 *
 * This intentionally does not import Sanath's map/dynamics API changes. It uses the
 * existing TrajectoryPlanner and creates a modified route when a static obstacle
 * overlaps the current route corridor.
 */
ObstacleAvoidanceResult
try_plan_obstacle_avoidance( TrajectoryPlanner& planner,
                             const map::Route& route,
                             const dynamics::VehicleStateDynamic& ego,
                             const dynamics::TrafficParticipantSet& traffic_participants,
                             const ObstacleAvoidanceParams& params = {} );

} // namespace planner
} // namespace adore
