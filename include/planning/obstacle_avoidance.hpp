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

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <vector>
#include <array>

namespace adore
{
namespace planner
{

struct ObstacleAvoidanceParams
{
  // --------------------------------------------------------------------------
  // Public baseline parameters
  // --------------------------------------------------------------------------

  // Master enable/disable for obstacle avoidance.
  bool enabled = true;

  // Maximum distance ahead of ego in which static obstacles are considered.
  double max_object_ahead             = 60.0;
  double max_static_object_speed      = 0.5;

  // Hysteresis for obstacles that are already part of an active avoidance
  // maneuver (ignored/expected ids): they stay ignored until their measured
  // speed exceeds this value. Prevents tracker speed noise on a parked vehicle
  // from turning it into a conflict right next to ego. Must be >=
  // max_static_object_speed to have any effect.
  double ignored_obstacle_release_speed = 1.5;

  // Detection corridor: obstacle is relevant only if its raw footprint intersects
  // [-0.5 * ego_width - ego_corridor_safety_margin, +0.5 * ego_width + ego_corridor_safety_margin].
  double ego_corridor_safety_margin       = 0.2;

  // Required lateral distance from the ego outer edge to the real obstacle
  // outer edge. This is the only object clearance requirement.
  double side_clearance               = 0.5;

  // Longitudinal planning distances for stop/shift timing. These do not
  // inflate stored obstacle geometry.
  double front_clearance              = 4.0;
  double rear_clearance               = 4.0;
  double stop_before_obstacle         = 7.0;

  // Allow lateral shifts within the current lane (without changing lanes).
  bool in_lane_shift_enabled = true;

  // Allow use of adjacent driving lanes in the same direction as the route.
  bool adjacent_lane_enabled = true;

  // Allow use of opposite-direction lanes (oncoming lanes). Requires special
  // oncoming traffic checks. Should remain false for baseline tests.
  bool opposite_lane_enabled = true; //false;

  // Enable multi-obstacle clustering and hull-bridge behavior. If false, only
  // the nearest ego-corridor obstacle is used for OA, with no cluster/hull link.
  // If true, nearby ego-corridor obstacles form one maneuver while preserving
  // their individual obstacle hulls.
  bool clustering_enabled = true; //false;

  // If enabled, a lateral-shift candidate is accepted only if the ego footprint
  // remains inside the current route lane or, if the relevant mode switch is
  // enabled, inside a connected corridor of driving lanes on the intended shift
  // side.
  bool enforce_drivable_area           = true;

  // 0.0 disables speed capping
  double max_speed_during_avoidance    = 2.7; // ~10 km/h

  // Candidate generation.
  bool validate_shifted_trajectory = true;
  int lateral_candidate_extra_steps = 2;
  double lateral_candidate_extra_step = 0.30;

  // If enabled, every active avoidance state is monitored against the route that ego is
  // actually following.
  bool modified_route_safety_check_enabled = true;
  double modified_route_max_check_distance = 60.0;
  double modified_route_time_horizon = 12.0;

  // --------------------------------------------------------------------------
  // Internal/advanced parameters
  // --------------------------------------------------------------------------

  // Plausibility filter for bad route projections / unrelated objects.
  double max_object_lateral_distance  = 8.0;

  // Despite the name, this is the minimum s-distance ahead of ego at which the
  // static-obstacle search window starts (not an overlap requirement).
  double min_obstacle_route_overlap   = 0.5;
  double min_oncoming_heading_diff    = 2.35; // rad, about 135 deg
  // Time step for the synthetic hold trajectories built in the decision maker;
  // not used by the planner library itself.
  double stop_time_step               = 0.1;

  // If left and right are equally good, prefer left. This matches right-hand traffic overtaking behavior.
  bool prefer_left_shift               = true;

  // Longitudinal tolerance for considering a neighbouring lane available at the
  // same lane-local s position. Prevents using a lane outside its actual s range.
  double lane_s_overlap_slack          = 0.50;

  // Lateral tolerance for joining adjacent lane-border intervals into one
  // connected drivable area. Small map gaps below this value are ignored.
  double lane_boundary_join_slack      = 0.25;

  double max_projection_distance_from_route = 5.0;

  // Advanced/internal multi-obstacle grouping. Used only when clustering_enabled
  // is true.
  //
  // Gap <= cluster_hold_gap_s:
  //   keep individual obstacle hulls and connect their per-object lateral
  //   shift targets directly. This is not a geometric rectangle merge.
  //
  // cluster_hold_gap_s < gap <= shift_hull_gap_s:
  //   keep the raw obstacle envelopes separate, but create one AvoidanceGroup
  //   and connect the individual shift profiles with a smooth hull bridge.
  //
  // gap > shift_hull_gap_s:
  //   treat the obstacles as separate maneuvers.
  double cluster_hold_gap_s = 10.0;
  double shift_hull_gap_s = 35.0;

  // Minimum shift fraction between hull-linked obstacles.
  // 0.0 = allow full return, 1.0 = keep full shift. 0.5 makes the hull visible
  // and avoids returning completely before the next obstacle.
  double min_alpha_between_hull_obstacles = 0.5;

  // Internal switch for evaluating extra lateral shift variants.
  bool enable_multi_candidate_route_shift = true;

  // ============================================================================
  // Internal/advanced oncoming traffic gap-acceptance parameters.
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
  double prediction_time_horizon = 15.0;

  // Time step for sampling predicted trajectories (if available).
  //double prediction_time_step = 0.2;

  // Safety distance from ego footprint front to oncoming vehicle rear during conflict.
  double oncoming_safety_distance_front = 10.0;

  // Safety distance from ego footprint rear to oncoming vehicle front during conflict.
  double oncoming_safety_distance_rear = 5.0;

  // Enable detailed debug logging for oncoming conflict checks.
  bool debug_oncoming_check = false;

  // ============================================================================
  // Internal/advanced ego-lane oncoming stop behavior.
  // ============================================================================

  // If enabled, dynamic participants that move against the ego route direction
  // and overlap the current ego lane/corridor cause a defensive stop behavior.
  // This covers the case where another vehicle temporarily uses the ego lane,
  // for example while avoiding a parked vehicle on its own side.
  bool ego_lane_oncoming_stop_enabled = true;

  // Maximum longitudinal distance ahead of ego for considering an oncoming
  // participant on the ego lane relevant.
  double ego_lane_oncoming_max_distance = 100.0;

  // Maximum time-to-conflict for triggering the defensive stop. A value <= 0.0
  // disables the time filter and uses distance only.
  double ego_lane_oncoming_time_horizon = 15.0;

  // Required route-aligned speed against the ego route direction.
  double ego_lane_oncoming_min_route_speed = 1.0;

  // Additional lateral tolerance around the current ego lane when deciding
  // whether the participant footprint overlaps the ego lane.
  double ego_lane_oncoming_lateral_margin = 0.20;

  // Desired distance between the ego front and the nearest footprint point of
  // the oncoming participant when the ego vehicle comes to rest.
  double ego_lane_oncoming_stop_distance = 15.0;

  // ============================================================================
  // Internal/advanced active modified-route safety monitor parameters.
  // ============================================================================

  double modified_route_ttc_margin = 2.0;
  double modified_route_stop_ttc_threshold = 5.0;
  double modified_route_braking_safety_margin = 5.0;
  double min_valid_stop_margin = 1.0;

  // Ghost memory for obstacles that were relevant during an active avoidance state.
  double ghost_obstacle_hold_time = 2.0;
  double ghost_obstacle_release_extra_s = 5.0;
  double ghost_obstacle_match_s_margin = 3.0;
  double ghost_obstacle_match_l_margin = 1.0;
  double ghost_obstacle_max_lifetime = 10.0;
  int ghost_dynamic_max_missing_cycles = 5;

  // ============================================================================
  // Internal/advanced trajectory and geometry parameters.
  // ============================================================================

  // Minimum vehicle dimension fallback when physical parameters are unavailable
  // or invalid (e.g., body_length or body_width < this value). This prevents
  // numerical issues in footprint calculations.
  double min_vehicle_dimension = 0.1;

  // Minimum search window for route projection. Prevents degenerate route segments.
  double route_window_min = 20.0;

  // Trajectory sampling step size for stop/hold trajectories. Controls granularity
  // of state predictions during emergency deceleration.
  double trajectory_step_size = 0.05;

  // Minimum speed threshold for participant motion detection. Speeds below this
  // are treated as static or negligible motion.
  double min_motion_speed = 0.05;

  // Lower bound for the braking deceleration used in stop-distance and
  // stop-profile calculations. Guards against an unset/implausible
  // vehicle acceleration_min producing near-infinite braking distances.
  double min_braking_deceleration = 0.5;

  // Adjustment offset added to front_clearance when stop_before_obstacle < front_clearance.
  // Ensures minimum safety distance to obstacles during emergency stop.
  double stop_adjustment_offset = 2.0;

  // Lateral shift penalty score for non-in-lane candidates. Higher values penalize
  // lane-change maneuvers, encouraging in-lane solutions when available.
  double lateral_shift_penalty_score = 20.0;
  double opposite_lane_penalty_score = 40.0;

};

class RouteSpeedPolicy
{
public:
  static map::Route apply_avoidance_speed_profile(
    const map::Route& route,
    double ego_s,
    double shift_start_s,
    double maneuver_end_s,
    const dynamics::PhysicalVehicleParameters& vehicle_params,
    const ObstacleAvoidanceParams& params );

  static map::Route apply_stop_profile(
    const map::Route& route,
    double ego_s,
    double ego_v,
    double desired_stop_s,
    const dynamics::PhysicalVehicleParameters& vehicle_params,
    const ObstacleAvoidanceParams& params );
};

enum class StopReason
{
  StaticObstacleAhead,
  OncomingOnEgoLane,
  ModifiedRouteConflict,
  ReplanFailed,
  InvalidProjection,
  EmergencyFallback
};

struct StopPlan
{
  bool valid = false;
  map::Route route;
  double stop_s = 0.0;
  StopReason reason = StopReason::EmergencyFallback;
};

class RouteStopPolicy
{
public:
  static StopPlan plan_stop_on_route(
    const map::Route& route,
    double ego_s,
    double ego_v,
    double desired_stop_s,
    StopReason reason,
    const dynamics::PhysicalVehicleParameters& vehicle_params,
    const ObstacleAvoidanceParams& params );
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

enum class RouteCorridorObjectClass
{
  StaticOrSlow,
  Oncoming,
  SameDirection,
  CrossingOrUnknown
};

struct RouteCorridorConflict
{
  int participant_id = -1;
  RouteCorridorObjectClass object_class = RouteCorridorObjectClass::CrossingOrUnknown;

  double object_s_min = std::numeric_limits<double>::infinity();
  double object_s_max = -std::numeric_limits<double>::infinity();
  double object_l_min = std::numeric_limits<double>::infinity();
  double object_l_max = -std::numeric_limits<double>::infinity();

  double inflated_s_min = std::numeric_limits<double>::infinity();
  double inflated_s_max = -std::numeric_limits<double>::infinity();
  double inflated_l_min = std::numeric_limits<double>::infinity();
  double inflated_l_max = -std::numeric_limits<double>::infinity();

  double distance_s = std::numeric_limits<double>::infinity();
  double time_to_conflict = std::numeric_limits<double>::infinity();

  bool currently_overlaps_route_corridor = false;
  bool currently_overlaps_ego_footprint = false;
  bool predicted_spatiotemporal_conflict = false;
  bool requires_stop = false;
  bool allows_replan = true;

  std::string ttc_source;

  double object_center_x = std::numeric_limits<double>::quiet_NaN();
  double object_center_y = std::numeric_limits<double>::quiet_NaN();
  double object_yaw = 0.0;
  double object_length = 0.1;
  double object_width = 0.1;
  std::array<double, 4> footprint_x{};
  std::array<double, 4> footprint_y{};
  bool has_world_footprint = false;

  bool currently_inside_corridor = false;

  std::string reason;
};

struct RouteCorridorCheckResult
{
  bool safe = true;
  bool has_conflict = false;
  double ego_s = std::numeric_limits<double>::quiet_NaN();
  RouteCorridorConflict conflict;
  // All conflicts found this cycle (conflict above is the most relevant one).
  // Consumers that maintain per-obstacle memory must see every detection, not
  // only the best one, so simultaneously visible obstacles do not age out.
  std::vector<RouteCorridorConflict> conflicts;
  std::string reason;
};

struct ObstacleGhostEnvelope
{
  double object_s_min = std::numeric_limits<double>::infinity();
  double object_s_max = -std::numeric_limits<double>::infinity();
  double object_l_min = std::numeric_limits<double>::infinity();
  double object_l_max = -std::numeric_limits<double>::infinity();

  double inflated_s_min = std::numeric_limits<double>::infinity();
  double inflated_s_max = -std::numeric_limits<double>::infinity();
  double inflated_l_min = std::numeric_limits<double>::infinity();
  double inflated_l_max = -std::numeric_limits<double>::infinity();

  RouteCorridorObjectClass object_class = RouteCorridorObjectClass::CrossingOrUnknown;

  double first_seen_time = std::numeric_limits<double>::quiet_NaN();
  double last_seen_time = std::numeric_limits<double>::quiet_NaN();
  int seen_count = 0;
  double hold_until_s = std::numeric_limits<double>::infinity();
  int consecutive_missing_cycles = 0;
  bool hold_until_passed = false;
  bool created_from_original_avoidance_obstacle = false;
  bool created_from_active_route_conflict = false;
  bool is_ghost = false;
  int last_participant_id = -1;

  double object_center_x = std::numeric_limits<double>::quiet_NaN();
  double object_center_y = std::numeric_limits<double>::quiet_NaN();
  double object_yaw = 0.0;
  double object_length = 0.1;
  double object_width = 0.1;
  std::array<double, 4> footprint_x{};
  std::array<double, 4> footprint_y{};
  bool has_world_footprint = false;
};

struct ObstacleAvoidanceManeuver
{
  bool active = false;

  ObstacleAvoidanceMode mode = ObstacleAvoidanceMode::None;

  int obstacle_id = -1;
  std::vector<int> obstacle_ids;
  double obstacle_s_min = std::numeric_limits<double>::infinity();
  double obstacle_s_max = -std::numeric_limits<double>::infinity();

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
  std::vector<int> obstacle_ids;
  double obstacle_s_min = std::numeric_limits<double>::infinity();
  double obstacle_s_max = -std::numeric_limits<double>::infinity();

  double shift_start_s = 0.0;
  double plateau_start_s = 0.0;
  double plateau_end_s = 0.0;
  double shift_end_s = 0.0;

  double lateral_shift = 0.0;
  bool in_lane = false;

  ObstacleAvoidanceManeuver maneuver;
};

// ----------------------------------------------------------------------------
// Shared helpers used by both the planner library and the decision-maker node.
// Kept here (instead of duplicated in each translation unit) so the projection
// and oncoming-clearance logic stays single-sourced.
// ----------------------------------------------------------------------------

// Project an ego/participant state onto the (possibly laterally modified) route
// reference line via segment projection. hint_s seeds the coarse search; the
// search window defaults to the full route span and the projection distance is
// unbounded by default.
template<typename State>
double
project_s_on_reference_line(
  const map::Route& route,
  const State& state,
  double hint_s = std::numeric_limits<double>::quiet_NaN(),
  double search_window = std::numeric_limits<double>::infinity(),
  double max_projection_distance = std::numeric_limits<double>::infinity() )
{
  if( route.reference_line.size() < 2 )
  {
    return std::numeric_limits<double>::infinity();
  }

  const double first_s = route.reference_line.begin()->first;
  const double last_s = route.reference_line.rbegin()->first;
  const double route_window =
    std::max( ObstacleAvoidanceParams{}.route_window_min, last_s - first_s );
  const double coarse_s =
    std::isfinite( hint_s ) ? hint_s : 0.5 * ( first_s + last_s );
  const double window =
    std::isfinite( search_window ) ? search_window : route_window;

  return adore::map::get_s_on_reference_line_segments(
    route, state, coarse_s, window, max_projection_distance );
}

// Lateral clearance between a route-centered ego footprint of half-width
// ego_half_width and an object whose route-frame lateral extent is
// [object_l_min, object_l_max]. +l is left of the route tangent.
inline double
actual_lateral_clearance_to_centered_ego(
  double object_l_min,
  double object_l_max,
  double ego_half_width )
{
  const double left_clearance = -ego_half_width - object_l_max;
  const double right_clearance = object_l_min - ego_half_width;
  return std::max( left_clearance, right_clearance );
}

// True if an oncoming object in the other lane keeps at least side_clearance to a
// route-centered ego footprint, i.e. it does not actually obstruct the ego lane.
inline bool
is_oncoming_other_lane_conflict(
  const RouteCorridorConflict& conflict,
  const dynamics::PhysicalVehicleParameters& ego_params,
  const ObstacleAvoidanceParams& params )
{
  if( conflict.object_class != RouteCorridorObjectClass::Oncoming ||
      conflict.currently_overlaps_ego_footprint )
  {
    return false;
  }

  const double ego_half_width =
    0.5 * std::max( params.min_vehicle_dimension, ego_params.body_width );
  const double actual_clearance =
    actual_lateral_clearance_to_centered_ego(
      conflict.object_l_min,
      conflict.object_l_max,
      ego_half_width );

  return actual_clearance >= std::max( 0.0, params.side_clearance );
}

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

RouteCorridorCheckResult
check_route_corridor_safety(
  const map::Route& route_to_check,
  const dynamics::VehicleStateDynamic& ego,
  const dynamics::TrafficParticipantSet& traffic_participants,
  const dynamics::PhysicalVehicleParameters& ego_params,
  const ObstacleAvoidanceParams& params = {},
  const dynamics::Trajectory* ego_trajectory = nullptr,
  const std::vector<int>* ignored_participant_ids = nullptr );

ObstacleAvoidanceResult
plan_stop_on_route_before_corridor_conflict(
  TrajectoryPlanner& planner,
  const map::Route& active_route,
  const dynamics::VehicleStateDynamic& ego,
  const dynamics::TrafficParticipantSet& traffic_participants,
  const RouteCorridorConflict& conflict,
  const ObstacleAvoidanceParams& params = {} );

bool
trajectory_stops_before_conflict(
  const dynamics::Trajectory& trajectory,
  const map::Route& route,
  const RouteCorridorConflict& conflict,
  const dynamics::PhysicalVehicleParameters& ego_params,
  const ObstacleAvoidanceParams& params = {} );

ObstacleAvoidanceMonitorResult
monitor_active_obstacle_avoidance_maneuver(
  const map::Route& route,
  const dynamics::VehicleStateDynamic& ego,
  const dynamics::TrafficParticipantSet& traffic_participants,
  const ObstacleAvoidanceManeuver& maneuver,
  const ObstacleAvoidanceParams& params = {},
  const dynamics::Trajectory* candidate_ego_trajectory = nullptr );

ObstacleAvoidanceResult
try_plan_obstacle_avoidance( TrajectoryPlanner& planner,
                             const map::Route& route,
                             const dynamics::VehicleStateDynamic& ego,
                             const dynamics::TrafficParticipantSet& traffic_participants,
                             const ObstacleAvoidanceParams& params = {},
                             const std::vector<int>* additional_ignored_participant_ids = nullptr );

} // namespace planner
} // namespace adore
