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

  // Safety distances around the detected raw obstacle footprint.
  double side_clearance               = 0.2;
  double front_clearance              = 2.0;
  double rear_clearance               = 2.0;


  // Detection corridor: obstacle is relevant only if its raw footprint intersects
  // [-0.5 * ego_width - ego_corridor_safety_margin, +0.5 * ego_width + ego_corridor_safety_margin].
  double ego_corridor_safety_margin       = 0.2;

  // Plausibility filter for bad route projections / unrelated objects.
  double max_object_lateral_distance  = 20.0;

  double min_obstacle_route_overlap   = 0.5;
  double oncoming_lookahead_after_obj = 35.0;
  double min_oncoming_heading_diff    = 2.35; // rad, about 135 deg
  double stop_time_step               = 0.1;

  double stop_distance_before_obstacle = 5.0;

  // The route points are modified directly. These distances shape the x^5 transition.
  double entry_extra_distance          = 8.0;
  double return_extra_distance         = 14.0;

  // 0.0 disables speed capping
  double max_speed_during_avoidance    = 2.78;  // 10 km/h

  // If left and right are equally good, prefer left. This matches right-hand traffic overtaking behavior.
  bool prefer_left_shift               = true;

  bool overtaking_allowed              = true;

  // If enabled, a lateral-shift candidate is accepted only if the ego footprint
  // remains inside the current route lane or, if overtaking is allowed, inside a
  // connected corridor of driving lanes on the intended shift side.
  bool enforce_drivable_area           = true;

  // Additional lateral safety margin between ego footprint and lane boundary.
  double drivable_area_margin          = 0.10;

  // Small tolerance for numerically merging touching lane intervals.
  double lane_boundary_join_slack      = 0.25;

  // Longitudinal tolerance for considering a neighbouring lane available at the
  // same lane-local s position. Prevents using a lane outside its actual s range.
  double lane_s_overlap_slack          = 0.50;

  // If false, overtaking may only use driving lanes with the same direction as
  // the route lane. If true, opposite-direction driving lanes of the same road
  // may also be used, subject to the oncoming-traffic check.
  bool allow_opposite_direction_lanes  = true;

  double max_projection_distance_from_route = 5.0;

  // Static obstacles with a longitudinal gap below this threshold are treated as
  // one common obstacle envelope. This prevents returning to the original route
  // between multiple closely spaced parked vehicles.
  double obstacle_cluster_join_gap_s = 8.0;


};

enum class ObstacleAvoidanceMode
{
  None,
  InLaneShift,
  OvertakeLeft,
  OvertakeRight,
  WaitForOncoming,
  StopBeforeObstacle
};

struct ObstacleAvoidanceResult
{
  bool success = false;
  std::string reason;
  ObstacleAvoidanceMode mode = ObstacleAvoidanceMode::None;

  map::Route modified_route;
  dynamics::Trajectory trajectory;

  bool has_maneuver_bounds = false;

  int obstacle_id = -1;

  double shift_start_s = 0.0;
  double plateau_start_s = 0.0;
  double plateau_end_s = 0.0;
  double shift_end_s = 0.0;

  double lateral_shift = 0.0;
  bool in_lane = false;
};

/**
 * Obstacle avoidance for the current eclipse-adore/bugfix/revert_to_make API.
 *
 * The planner does not publish or create a separate shifted path here. It modifies
 * the points of a copy of the original route and then calls TrajectoryPlanner on
 * that modified route.
 */
ObstacleAvoidanceResult
try_plan_obstacle_avoidance( TrajectoryPlanner& planner,
                             const map::Route& route,
                             const dynamics::VehicleStateDynamic& ego,
                             const dynamics::TrafficParticipantSet& traffic_participants,
                             const ObstacleAvoidanceParams& params = {} );

} // namespace planner
} // namespace adore