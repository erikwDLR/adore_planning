/********************************************************************************
 * Copyright (c) 2026 Contributors to the Eclipse Foundation
 *
 * SPDX-License-Identifier: EPL-2.0
 ********************************************************************************/

#pragma once

#include <limits>
#include <optional>
#include <string>

#include <adore_map/route.hpp>

#include "dynamics/traffic_participant.hpp"
#include "dynamics/vehicle_state.hpp"

#include "planning/active_avoidance_state.hpp"
#include "planning/obstacle_avoidance.hpp"

namespace adore
{
namespace planner
{

// Operations on the obstacle-avoidance maneuver state and its ghost memory.
// These are pure (no ROS) so the maneuver lifecycle can be unit-tested; the
// decision-maker node owns the ActiveAvoidanceState instance and calls these.

// Build a hold-able ghost envelope for an original avoidance obstacle by
// projecting the participant footprint onto the route. Returns nullopt if no
// footprint corner projects onto the route.
std::optional<ObstacleGhostEnvelope>
make_original_obstacle_envelope_from_participant(
  const map::Route& route,
  const dynamics::TrafficParticipant& participant,
  double now,
  const ObstacleAvoidanceParams& params );

// Age, match, refresh and release the ghost memory against this cycle's
// route-corridor safety result.
void
update_obstacle_ghost_memory(
  ActiveAvoidanceState& state,
  const RouteCorridorCheckResult& safety,
  double ego_s,
  double now,
  const ObstacleAvoidanceParams& params );

// Initialise the active maneuver state from a planned avoidance result and seed
// the ghost memory from the original obstacles (optionally preserving the
// existing ghost memory across a dynamic replan).
void
start_active_avoidance_state(
  ActiveAvoidanceState& state,
  const ObstacleAvoidanceResult& oa_result,
  const map::Route& original_route,
  const dynamics::TrafficParticipantSet& traffic_participants,
  const dynamics::VehicleStateDynamic& ego,
  const ObstacleAvoidanceParams& params,
  bool preserve_existing_ghost_memory = false );

// Most relevant (nearest, unpassed) ghosted conflict to stop for, or nullopt.
std::optional<RouteCorridorConflict>
most_relevant_ghost_conflict(
  const ActiveAvoidanceState& state,
  double ego_s );

// An original avoidance obstacle that ego has not yet passed, or nullopt.
std::optional<ObstacleGhostEnvelope>
find_unpassed_original_ghost(
  const ActiveAvoidanceState& state,
  double ego_s );

// Monotonic-progression plausibility for the ego projection onto the active
// modified route. Enforces no-backward motion and rejects implausible forward
// jumps (projection artifacts, e.g. matches at the search-window edge) by
// advancing via odometry instead of latching the jump. Updates the state's
// last_modified_s / last_modified_time. Returns nullopt if the result is not
// finite (projection lost with no prior value).
std::optional<double>
compute_monotonic_ego_s_modified(
  double ego_s_modified_raw,
  const dynamics::VehicleStateDynamic& vehicle_state_dynamic,
  ActiveAvoidanceState& state );

// ----------------------------------------------------------------------------
// Conflict assessment and stop geometry for a route ego is following.
// ----------------------------------------------------------------------------

// Longitudinal braking geometry to stop before a conflict on a given route.
struct RouteStopPlan
{
  bool valid = false;
  double ego_s = std::numeric_limits<double>::quiet_NaN();
  double stop_s = std::numeric_limits<double>::quiet_NaN();
  double brake_start_s = std::numeric_limits<double>::quiet_NaN();
  double required_braking_distance = 0.0;
  double braking_deceleration = 0.0;
  std::string reason;
};

// Build a RouteCorridorConflict from an active opposite-lane monitor result so
// the route-based stop policy can consume it. reference_s is the ego s on the
// route used to derive the longitudinal distance to the conflict.
RouteCorridorConflict
make_oncoming_monitor_conflict(
  const ObstacleAvoidanceMonitorResult& monitor_result,
  double reference_s,
  const char* ttc_source );

// True if a static/slow conflict keeps at least side_clearance to a route-
// centered ego footprint, so ego can keep going without stopping.
bool
static_or_slow_conflict_has_side_clearance(
  const RouteCorridorConflict& conflict,
  const dynamics::PhysicalVehicleParameters& vehicle_params,
  const ObstacleAvoidanceParams& params );

// Longitudinal braking geometry to stop before a conflict on the active route.
RouteStopPlan
compute_route_stop_plan(
  const map::Route& active_route,
  const dynamics::VehicleStateDynamic& ego,
  const dynamics::PhysicalVehicleParameters& vehicle_params,
  const RouteCorridorConflict& conflict,
  const ObstacleAvoidanceParams& params );

// Whether ego must brake now for the conflict on the active route.
bool
should_stop_for_active_conflict(
  const map::Route& active_route,
  const dynamics::VehicleStateDynamic& ego,
  const dynamics::PhysicalVehicleParameters& vehicle_params,
  const RouteCorridorConflict& conflict,
  const ObstacleAvoidanceParams& params );

} // namespace planner
} // namespace adore
