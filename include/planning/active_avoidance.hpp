/********************************************************************************
 * Copyright (c) 2026 Contributors to the Eclipse Foundation
 *
 * SPDX-License-Identifier: EPL-2.0
 ********************************************************************************/

#pragma once

#include <limits>
#include <optional>

#include <adore_map/route.hpp>

#include "dynamics/traffic_participant.hpp"
#include "dynamics/vehicle_state.hpp"

#include "planning/active_avoidance_state.hpp"
#include "planning/obstacle_avoidance.hpp"

namespace adore
{
namespace planner
{

// Operations on the obstacle-avoidance maneuver state. These are pure (no ROS)
// so the maneuver lifecycle can be unit-tested; the decision-maker node owns the
// ActiveAvoidanceState instance and calls these.

// Initialise or refresh the persistent active maneuver state from a planned
// avoidance result.
void
start_active_avoidance_state(
  ActiveAvoidanceState& state,
  const ObstacleAvoidanceResult& oa_result,
  const map::Route& mission_route_baseline );

// Compare only route geometry, not speed overlays. Active obstacle hulls use
// route-s coordinates and therefore remain valid only while this geometric
// reference frame stays compatible.
bool
routes_have_compatible_geometry(
  const map::Route& baseline,
  const map::Route& candidate,
  double position_tolerance = 0.5,
  double length_tolerance = 1.0 );

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
};

// Build a RouteCorridorConflict from an active opposite-lane monitor result so
// the route-based stop policy can consume it. reference_s is the ego s on the
// route used to derive the longitudinal distance to the conflict.
RouteCorridorConflict
make_oncoming_monitor_conflict(
  const ObstacleAvoidanceMonitorResult& monitor_result,
  double reference_s );

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
