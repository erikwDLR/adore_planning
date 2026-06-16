/********************************************************************************
 * Copyright (c) 2026 Contributors to the Eclipse Foundation
 *
 * SPDX-License-Identifier: EPL-2.0
 ********************************************************************************/

// Stop / hold route and trajectory construction: emergency stop and hard-hold
// fallback trajectories, zeroing route speeds from a given s, inserting a
// zero-speed stop point, and building a route-based braking profile that stops
// before an obstacle (delegating to the public RouteSpeedPolicy).

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
  std::fprintf(
    stderr,
    "[OA][CONFIG] stop_before_obstacle must be greater than front_clearance; adjusted from %.2f to %.2f\n",
    params.stop_before_obstacle,
    adjusted );
  std::fflush( stderr );
  return adjusted;
}

dynamics::Trajectory
make_stop_trajectory( const dynamics::VehicleStateDynamic& ego, double dt )
{
  dynamics::Trajectory trajectory;
  trajectory.label = "obstacle avoidance: stop fallback";

  auto stop_state = ego;
  stop_state.vx = 0.0;
  stop_state.time = ego.time + dt;

  trajectory.states.push_back( ego );
  trajectory.states.push_back( stop_state );

  return trajectory;
}

dynamics::Trajectory
make_hard_stop_trajectory( const dynamics::VehicleStateDynamic& ego,
                           const dynamics::PhysicalVehicleParameters& vehicle_params,
                           double dt,
                           const ObstacleAvoidanceParams& params )
{
  dynamics::Trajectory trajectory;
  trajectory.label = "obstacle avoidance: immediate hold";

  const double step = std::max( params.trajectory_step_size, dt );
  const double start_speed = std::max( 0.0, ego.vx );
  const int samples = 6;

  for( int i = 0; i < samples; ++i )
  {
    auto state = ego;
    state.time = ego.time + i * step;
    state.vx =
      i == 0
        ? ego.vx
        : start_speed * std::max( 0.0, 1.0 - static_cast<double>( i ) / ( samples - 1 ) );
    trajectory.states.push_back( state );
  }

  trajectory.states.back().vx = 0.0;

  (void)vehicle_params;

  return trajectory;
}

void
set_route_points_from_s_to_zero( map::Route& route, double ego_s )
{
  if( route.reference_line.empty() )
  {
    return;
  }

  if( !std::isfinite( ego_s ) )
  {
    ego_s = route.reference_line.begin()->first;
  }

  for( auto& [s, point] : route.reference_line )
  {
    if( s >= ego_s )
    {
      point.max_speed = 0.0;
    }
  }
}

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
  double safety_margin,
  const ObstacleAvoidanceParams& params )
{
  map::Route stop_route = route;

  const double ego_s = project_s_on_reference_line( route, ego );

  if( !std::isfinite( ego_s ) )
  {
    std::fprintf(
      stderr,
      "[OA] stop route: invalid ego_s\n" );
    std::fflush( stderr );

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
    std::fprintf(
      stderr,
      "[OA] stop route: invalid reference stop_s, object_s_min=%.2f stop_distance=%.2f front_offset=%.2f\n",
      obstacle.object_s_min,
      stop_before_obstacle,
      vehicle_params.wheelbase + vehicle_params.front_axle_to_front_border );
    std::fflush( stderr );

    return stop_route;
  }

  /*
   * acceleration_min is defined in the vehicle parameter file, e.g. NGC.json.
   * It is expected to be negative, for example -1.5 m/s².
   */
  const double braking_deceleration =
    std::max( params.min_braking_deceleration, std::fabs( vehicle_params.acceleration_min ) );

  const double current_speed =
    std::max( 0.0, ego.vx );

  const double available_distance =
    stop_s - ego_s;

  const double required_braking_distance =
    current_speed * current_speed / ( 2.0 * braking_deceleration );

  /*
   * If the desired stop point is already behind or at the ego reference point,
   * command zero speed from the current ego position onward.
   */
  const double reachability_margin = std::max( 0.0, safety_margin );
  const double close_stop_hold_margin =
    std::max( 0.25, std::min( 1.0, std::max( 0.0, safety_margin ) ) );
  const double hard_reachability_margin =
    current_speed > 0.5
      ? reachability_margin
      : close_stop_hold_margin;

  if( available_distance <= 0.0 ||
      available_distance < required_braking_distance + hard_reachability_margin )
  {
    std::fprintf(
      stderr,
      "[OA][STOP_ROUTE] requested stop unreachable; applying maximum route-based braking ego_s=%.2f reference_stop_s=%.2f front_stop_s=%.2f object_s_min=%.2f v=%.2f available=%.2f required=%.2f safety_margin=%.2f reachability_margin=%.2f\n",
      ego_s,
      stop_s,
      front_stop_s,
      obstacle.object_s_min,
      current_speed,
      available_distance,
      required_braking_distance,
      reachability_margin,
      hard_reachability_margin );
    std::fflush( stderr );
    set_route_points_from_s_to_zero( stop_route, ego_s );

    return stop_route;
  }

  return RouteSpeedPolicy::apply_stop_profile(
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
