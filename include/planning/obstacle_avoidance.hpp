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

#include <limits>
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

  // ============================================================================
  // Oncoming traffic gap-acceptance parameters (improved time-based check)
  // ============================================================================

  // Minimum time margin before an oncoming vehicle arrives at the conflict area.
  // If the computed oncoming arrival time is <= ego_clear_time + oncoming_time_margin,
  // the maneuver is rejected.
  double oncoming_time_margin = 2.0;

  // Minimum speed for ego vehicle when computing clear time. Prevents division by
  // very small numbers; uses max(actual_ego_speed, min_ego_speed_for_gap_check).
  double min_ego_speed_for_gap_check = 1.0;

  // Minimum speed used when computing oncoming arrival time. This prevents
  // division by tiny route-aligned speeds; static filtering still uses
  // max_static_object_speed.
  double min_oncoming_speed_for_gap_check = 1.0;

  // Minimum route-aligned speed for an oncoming participant to be considered
  // as moving in the opposite direction.
  double min_oncoming_route_speed = 1.0;

  // Time horizon for predicting participant trajectories. Limits lookahead.
  double prediction_time_horizon = 12.0;

  // Time step for sampling predicted trajectories (if available).
  double prediction_time_step = 0.2;

  // Safety distance from ego footprint front to oncoming vehicle rear during conflict.
  double oncoming_safety_distance_front = 10.0;

  // Safety distance from ego footprint rear to oncoming vehicle front during conflict.
  double oncoming_safety_distance_rear = 5.0;

  // Enable detailed debug logging for oncoming conflict checks.
  bool debug_oncoming_check = false;

  // ============================================================================
  // Ego-lane oncoming stop behavior
  // ============================================================================

  // If enabled, dynamic participants that move against the ego route direction
  // and overlap the current ego lane/corridor cause a defensive stop behavior.
  // This covers the case where another vehicle temporarily uses the ego lane,
  // for example while avoiding a parked vehicle on its own side.
  bool ego_lane_oncoming_stop_enabled = true;

  // Maximum longitudinal distance ahead of ego for considering an oncoming
  // participant on the ego lane relevant.
  double ego_lane_oncoming_max_distance = 80.0;

  // Maximum time-to-conflict for triggering the defensive stop. A value <= 0.0
  // disables the time filter and uses distance only.
  double ego_lane_oncoming_time_horizon = 12.0;

  // Required route-aligned speed against the ego route direction.
  double ego_lane_oncoming_min_route_speed = 1.0;

  // Additional lateral tolerance around the current ego lane when deciding
  // whether the participant footprint overlaps the ego lane.
  double ego_lane_oncoming_lateral_margin = 0.20;

  // Desired distance between the ego front and the nearest footprint point of
  // the oncoming participant when the ego vehicle comes to rest.
  double ego_lane_oncoming_stop_distance = 10.0;
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

/**
 * Detailed result of an oncoming traffic gap-acceptance check.
 *
 * This structure provides comprehensive diagnostics explaining whether a candidate
 * obstacle-avoidance maneuver can be safely executed with respect to oncoming traffic
 * in the opposite-direction lane.
 *
 * Assumptions:
 * - "Conflict interval" refers to the longitudinal range [conflict_start_s, conflict_end_s]
 *   where the ego vehicle's footprint occupies an opposite-direction lane during avoidance.
 * - Time calculations assume constant-velocity prediction for traffic participants.
 * - Times are measured from now (current timestamp).
 * - Route-aligned velocity is computed as: v_route = speed * cos(yaw_delta_from_route_yaw)
 *   Positive v_route means moving in route direction; negative means oncoming.
 * - The maneuver is rejected if: oncoming_arrival_time <= ego_clear_time + oncoming_time_margin
 *   to provide a safety buffer.
 */
struct OncomingConflictResult
{
  // True if the maneuver must be rejected due to oncoming traffic conflict.
  bool conflict = false;

  // The traffic participant ID causing the conflict (or -1 if no conflict).
  int participant_id = -1;

  // Longitudinal range [conflict_start_s, conflict_end_s] where ego is in opposite lane.
  // Typically derived from the maneuver's shift_start_s and shift_end_s or from the
  // obstacle's geometry.
  double conflict_start_s = 0.0;
  double conflict_end_s = 0.0;

  // Estimated time for ego to safely clear the conflict interval,
  // including the maneuver's return phase, measured from now.
  double ego_clear_time = 0.0;

  // Estimated arrival time of the oncoming vehicle at the rear of the conflict interval,
  // measured from now. Set to +infinity if no conflict or if the vehicle is not moving
  // oncoming.
  double oncoming_arrival_time = std::numeric_limits<double>::infinity();

  // Human-readable explanation of the decision (e.g., "participant id=5 arrival_time=2.5s <= ego_clear_time=3.0s + margin=2.0s").
  std::string reason;

  // Diagnostic information (only populated if debug_oncoming_check is enabled).
  struct Diagnostics
  {
    int participant_id = -1;
    double participant_s = 0.0;
    double participant_vx = 0.0;
    double participant_yaw = 0.0;
    double route_yaw_at_participant_s = 0.0;
    double yaw_diff = 0.0;
    double v_route = 0.0;  // Route-aligned velocity (positive = route direction, negative = oncoming)
    double heading_diff_rad = 0.0;
  } diagnostics;
};

struct EgoLaneOncomingStopResult
{
  bool success = false;
  std::string reason;

  int participant_id = -1;
  double participant_s = 0.0;
  double participant_distance_s = 0.0;
  double participant_route_speed = 0.0;
  double time_to_conflict = std::numeric_limits<double>::infinity();
  double stop_s = 0.0;

  map::Route modified_route;
  dynamics::Trajectory trajectory;
};

struct ObstacleAvoidanceManeuver
{
  bool active = false;

  ObstacleAvoidanceMode mode = ObstacleAvoidanceMode::None;

  int obstacle_id = -1;

  double shift_start_s = 0.0;
  double plateau_start_s = 0.0;
  double plateau_end_s = 0.0;
  double shift_end_s = 0.0;
  double release_s = 0.0;

  double lateral_shift = 0.0;
  bool in_lane = false;

  // True if the maneuver occupies a lane whose direction is opposite to the
  // current route direction. These fields make active maneuver monitoring
  // possible from the decision maker without recomputing private obstacle
  // envelopes.
  bool uses_opposite_lane = false;
  bool has_opposite_lane_conflict_interval = false;
  double opposite_lane_conflict_start_s = 0.0;
  double opposite_lane_conflict_end_s = 0.0;

  // Before this point, a new oncoming conflict can still abort the maneuver to a
  // controlled stop before the static obstacle. After this point, the safer
  // fallback is usually to slow down and finish returning to the lane.
  double commitment_s = 0.0;
};

struct ObstacleAvoidanceMonitorResult
{
  bool safe_to_continue = true;
  bool should_abort_before_commitment = false;
  bool already_committed = false;

  OncomingConflictResult oncoming;
  std::string reason;
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

  ObstacleAvoidanceManeuver maneuver;
};

/**
 * Obstacle avoidance for the current eclipse-adore/bugfix/revert_to_make API.
 *
 * The planner does not publish or create a separate shifted path here. It modifies
 * the points of a copy of the original route and then calls TrajectoryPlanner on
 * that modified route.
 */
EgoLaneOncomingStopResult
try_plan_ego_lane_oncoming_stop( TrajectoryPlanner& planner,
                                 const map::Route& route,
                                 const dynamics::VehicleStateDynamic& ego,
                                 const dynamics::TrafficParticipantSet& traffic_participants,
                                 const ObstacleAvoidanceParams& params = {} );

ObstacleAvoidanceMonitorResult
monitor_active_obstacle_avoidance_maneuver(
  const map::Route& route,
  const dynamics::VehicleStateDynamic& ego,
  const dynamics::TrafficParticipantSet& traffic_participants,
  const ObstacleAvoidanceManeuver& maneuver,
  const ObstacleAvoidanceParams& params = {} );

ObstacleAvoidanceResult
try_plan_obstacle_avoidance( TrajectoryPlanner& planner,
                             const map::Route& route,
                             const dynamics::VehicleStateDynamic& ego,
                             const dynamics::TrafficParticipantSet& traffic_participants,
                             const ObstacleAvoidanceParams& params = {} );

} // namespace planner
} // namespace adore
