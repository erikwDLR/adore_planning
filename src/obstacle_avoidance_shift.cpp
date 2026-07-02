/********************************************************************************
 * Copyright (c) 2026 Contributors to the Eclipse Foundation
 *
 * SPDX-License-Identifier: EPL-2.0
 ********************************************************************************/

// Lateral-shift profile math: per-obstacle and per-group shift alpha/offset
// curves, hard/soft hull bridges between clustered obstacles, and construction
// of the laterally modified route (plus its avoidance speed profile). Depends on
// the geometry primitives and the public RouteSpeedPolicy.

#include "obstacle_avoidance_internal.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace adore
{
namespace planner
{
namespace oa_detail
{

void
apply_avoidance_speed_profile( map::Route& route,
                               double ego_s,
                               double shift_start_s,
                               double maneuver_end_s,
                               const dynamics::PhysicalVehicleParameters& vehicle_params,
                               const ObstacleAvoidanceParams& params )
{
  route =
    RouteSpeedPolicy::apply_avoidance_speed_profile(
      route,
      ego_s,
      shift_start_s,
      maneuver_end_s,
      vehicle_params,
      params );
}

double
avoidance_shift_alpha_at_s( double s,
                            const ObstacleEnvelope& obstacle,
                            const ObstacleAvoidanceParams& params )
{
  // Shift geometry keyed directly to the obstacle: the lateral shift ramps up over
  // front_clearance ahead of the obstacle, holds full shift alongside it
  // [object_s_min, object_s_max], and ramps back down over rear_clearance behind
  // it. With equal clearances this is symmetric about the obstacle. front_clearance
  // and rear_clearance are the only longitudinal knobs (no implicit ego offset).
  const double shift_start_s =
    std::max(
      0.0,
      obstacle.object_s_min - std::max( 0.0, params.front_clearance ) );
  const double shift_end_s =
    obstacle.object_s_max + std::max( 0.0, params.rear_clearance );

  if( s < shift_start_s || s > shift_end_s )
  {
    return 0.0;
  }

  if( s < obstacle.object_s_min )
  {
    return smoothstep01(
      ( s - shift_start_s ) /
      std::max( 0.1, obstacle.object_s_min - shift_start_s ) );
  }

  if( s <= obstacle.object_s_max )
  {
    return 1.0;
  }

  return 1.0 - smoothstep01(
    ( s - obstacle.object_s_max ) /
    std::max( 0.1, shift_end_s - obstacle.object_s_max ) );
}

double
required_signed_shift_for_obstacle(
  const ObstacleEnvelope& obstacle,
  double nominal_lateral_shift,
  const dynamics::PhysicalVehicleParameters& ego_params,
  const ObstacleAvoidanceParams& params )
{
  if( !std::isfinite( nominal_lateral_shift ) ||
      std::fabs( nominal_lateral_shift ) < 1e-6 )
  {
    return 0.0;
  }

  const double ego_half_width =
    0.5 * std::max( params.min_vehicle_dimension, ego_params.body_width );
  const double side_clearance =
    std::max( 0.0, params.side_clearance );

  if( nominal_lateral_shift > 0.0 )
  {
    const double required =
      obstacle.object_l_max + side_clearance + ego_half_width;
    return std::clamp( required, 0.0, nominal_lateral_shift );
  }

  const double required =
    obstacle.object_l_min - side_clearance - ego_half_width;
  return std::clamp( required, nominal_lateral_shift, 0.0 );
}

double
choose_larger_magnitude_shift( double a, double b )
{
  return std::fabs( b ) > std::fabs( a ) ? b : a;
}

double
hard_bridge_alpha_at_s( double s,
                        const ObstacleEnvelope& previous,
                        const ObstacleEnvelope& next,
                        const ObstacleAvoidanceParams& params )
{
  // Clearance-based cluster bounds (see avoidance_shift_alpha_at_s): ramp up over
  // front_clearance before the first obstacle, hold across the cluster, ramp down
  // over rear_clearance after the last obstacle. No implicit ego offset.
  const double bridge_start_s =
    std::max(
      0.0,
      previous.object_s_min - std::max( 0.0, params.front_clearance ) );
  const double bridge_end_s =
    next.object_s_max + std::max( 0.0, params.rear_clearance );

  if( s < bridge_start_s || s > bridge_end_s )
  {
    return 0.0;
  }

  if( s < previous.object_s_min )
  {
    return smoothstep01(
      ( s - bridge_start_s ) /
      std::max( 0.1, previous.object_s_min - bridge_start_s ) );
  }

  if( s > next.object_s_max )
  {
    return 1.0 - smoothstep01(
      ( s - next.object_s_max ) /
      std::max( 0.1, bridge_end_s - next.object_s_max ) );
  }

  return 1.0;
}

double
avoidance_shift_offset_at_s(
  double s,
  const AvoidanceGroup& group,
  double nominal_lateral_shift,
  const dynamics::PhysicalVehicleParameters& ego_params,
  const ObstacleAvoidanceParams& params )
{
  double offset = 0.0;

  for( const auto& obstacle : group.obstacles )
  {
    const double alpha =
      avoidance_shift_alpha_at_s( s, obstacle, params );
    if( alpha <= 0.0 )
    {
      continue;
    }

    const double required_shift =
      required_signed_shift_for_obstacle(
        obstacle,
        nominal_lateral_shift,
        ego_params,
        params );
    offset = choose_larger_magnitude_shift( offset, required_shift * alpha );
  }

  if( ( group.uses_hull_curve || group.hard_merged ) &&
      group.obstacles.size() >= 2 )
  {
    const double hard_merge_gap_s =
      std::max( 0.0, params.cluster_hold_gap_s );
    const double shift_hull_gap_s =
      std::max( hard_merge_gap_s, params.shift_hull_gap_s );

    for( std::size_t i = 1; i < group.obstacles.size(); ++i )
    {
      const auto& previous = group.obstacles[i - 1];
      const auto& next = group.obstacles[i];

      const double raw_gap_s = next.object_s_min - previous.object_s_max;
      if( raw_gap_s > shift_hull_gap_s )
      {
        continue;
      }

      const double previous_shift =
        required_signed_shift_for_obstacle(
          previous,
          nominal_lateral_shift,
          ego_params,
          params );
      const double next_shift =
        required_signed_shift_for_obstacle(
          next,
          nominal_lateral_shift,
          ego_params,
          params );

      if( raw_gap_s <= hard_merge_gap_s )
      {
        const double bridge_alpha =
          hard_bridge_alpha_at_s( s, previous, next, params );
        if( bridge_alpha <= 0.0 )
        {
          continue;
        }

        // Hard clusters keep the larger required per-object shift through the
        // bridge. If the next object needs more shift, ego is already in place;
        // if it needs less, ego avoids an unnecessary return steering input.
        const double bridge_offset =
          choose_larger_magnitude_shift( previous_shift, next_shift ) *
          bridge_alpha;
        offset = choose_larger_magnitude_shift( offset, bridge_offset );
        continue;
      }

      // Span the bridge over the full physical gap between the obstacles. The
      // endpoints then coincide with the full per-obstacle shift plateaus
      // (alpha=1, shift=prev/next) at object_s_max / object_s_min, so the
      // bridge hands off to them seamlessly (C1). Insetting by ego geometry
      // moved the endpoints into the per-obstacle ramp-down/ramp-up regions,
      // where the max() source switches and creates a slope kink -> a curvature
      // spike that is barely visible on a straight but obvious through a curve.
      const double bridge_start_s = previous.object_s_max;
      const double bridge_end_s = next.object_s_min;

      if( s < bridge_start_s || s > bridge_end_s ||
          bridge_end_s <= bridge_start_s + 0.1 )
      {
        continue;
      }

      const double t =
        std::clamp(
          ( s - bridge_start_s ) / ( bridge_end_s - bridge_start_s ),
          0.0,
          1.0 );
      const double bridge_floor =
        std::clamp( params.min_alpha_between_hull_obstacles, 0.0, 1.0 );
      double bridge_alpha = 1.0;

      if( t <= 0.5 )
      {
        bridge_alpha =
          1.0 - ( 1.0 - bridge_floor ) * smoothstep01( 2.0 * t );
      }
      else
      {
        bridge_alpha =
          bridge_floor +
          ( 1.0 - bridge_floor ) * smoothstep01( 2.0 * t - 1.0 );
      }

      const double bridge_shift =
        previous_shift +
        ( next_shift - previous_shift ) * smoothstep01( t );
      const double bridge_offset = bridge_shift * bridge_alpha;
      offset = choose_larger_magnitude_shift( offset, bridge_offset );
    }
  }

  return offset;
}

map::Route
build_modified_avoidance_route( const map::Route& route,
                                const AvoidanceGroup& group,
                                double lateral_shift,
                                double ego_s,
                                const dynamics::PhysicalVehicleParameters& ego_params,
                                const ObstacleAvoidanceParams& params )
{
  map::Route modified_route = route;

  for( auto& [s, point] : modified_route.reference_line )
  {
    const double offset =
      avoidance_shift_offset_at_s(
        s,
        group,
        lateral_shift,
        ego_params,
        params );

    if( std::fabs( offset ) <= 1e-6 )
    {
      continue;
    }

    // Important: use the original route as reference frame.
    const auto frame = make_route_frame( route, s );
    const auto shifted_point_xy = shifted_point( frame, offset );

    point.x = shifted_point_xy.x;
    point.y = shifted_point_xy.y;
  }

  // Clearance-based shift window (see avoidance_shift_alpha_at_s): matches the
  // lateral shift window so the speed profile ramps over the same span.
  const double shift_start_s =
    std::max(
      0.0,
      group.envelope.object_s_min - std::max( 0.0, params.front_clearance ) );
  const double shift_end_s =
    group.envelope.object_s_max + std::max( 0.0, params.rear_clearance );
  apply_avoidance_speed_profile(
    modified_route,
    ego_s,
    shift_start_s,
    shift_end_s,
    ego_params,
    params );

  return modified_route;
}

} // namespace oa_detail
} // namespace planner
} // namespace adore
