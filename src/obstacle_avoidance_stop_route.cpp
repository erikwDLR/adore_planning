/********************************************************************************
 * Copyright (c) 2026 Contributors to the Eclipse Foundation
 *
 * SPDX-License-Identifier: EPL-2.0
 ********************************************************************************/

// Stop-route construction: route-speed stop profiles, zeroing route speeds from
// a given s, inserting a zero-speed stop point, and building a route-based
// braking profile that stops before an obstacle.

#include "obstacle_avoidance_internal.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

namespace adore
{
namespace planner
{
namespace oa_detail
{

double
normalized_stop_before_obstacle( const ObstacleAvoidanceParams& params )
{
  if( params.stop_before_obstacle > params.front_clearance )
  {
    return params.stop_before_obstacle;
  }

  const double adjusted = params.front_clearance + params.stop_adjustment_offset;
  return adjusted;
}

// set_route_points_from_s_to_zero moved to the public API (defined in
// obstacle_avoidance.cpp, declared in planning/obstacle_avoidance.hpp). Callers
// here resolve it via the enclosing planner namespace.

void
insert_zero_speed_stop_point( map::Route& route, double stop_s )
{
  if( route.reference_line.empty() || !std::isfinite( stop_s ) )
  {
    return;
  }

  const double first_s = route.reference_line.begin()->first;
  const double last_s = route.reference_line.rbegin()->first;
  if( stop_s < first_s || stop_s > last_s )
  {
    return;
  }

  auto stop_point = route.interpolate_at_s<map::MapPoint>( stop_s );
  stop_point.s = stop_s;
  stop_point.max_speed = 0.0;
  route.reference_line[stop_s] = stop_point;
}

map::Route
build_stop_route_before_obstacle(
  const map::Route& route,
  const ObstacleEnvelope& obstacle,
  const dynamics::VehicleStateDynamic& ego,
  const dynamics::PhysicalVehicleParameters& vehicle_params,
  double stop_before_obstacle,
  const ObstacleAvoidanceParams& params )
{
  map::Route stop_route = route;

  const double ego_s = project_s_on_reference_line( route, ego );

  if( !std::isfinite( ego_s ) )
  {

    return stop_route;
  }

  const double ego_front_offset =
    vehicle_params.wheelbase + vehicle_params.front_axle_to_front_border;
  const double front_stop_s =
    obstacle.object_s_min
    - stop_before_obstacle;
  const double stop_s =
    front_stop_s
    - ego_front_offset;

  if( !std::isfinite( stop_s ) )
  {

    return stop_route;
  }

  // Unified braking: the envelope itself decides between gentle (comfort) braking
  // to stop_s and maximum braking (|acceleration_min|) when stop_s is too close,
  // so no separate reachability check or zero-from-ego fallback is needed here.
  return RouteSpeedPolicy::apply_brake_envelope(
    stop_route,
    ego_s,
    ego.vx,
    stop_s,
    vehicle_params,
    params );
}

} // namespace oa_detail
} // namespace planner
} // namespace adore
