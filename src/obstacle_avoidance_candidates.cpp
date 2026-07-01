/********************************************************************************
 * Copyright (c) 2026 Contributors to the Eclipse Foundation
 *
 * SPDX-License-Identifier: EPL-2.0
 ********************************************************************************/

// Lateral-shift candidate generation, drivable-area / opposite-lane validation,
// planned-trajectory validation against lanes and obstacles, and candidate
// scoring. Depends on the geometry, projection, grouping, shift, drivable-area
// and oncoming helpers in oa_detail.

#include "obstacle_avoidance_internal.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace adore
{
namespace planner
{
namespace oa_detail
{

// Returns true if the candidate maneuver enters a driving lane that goes in the
// opposite direction relative to the current route lane.
//
// Preferred behavior: derived from map data via `left_of_reference`.
// Fallback: if no map information is available for the shifted region, this
// helper uses the right-hand-traffic convention that a left shift leaving the
// current lane (`lateral_shift > 0 && !in_lane`) is a probable opposite-lane
// use. In left-hand-traffic environments this convention does not hold and
// the helper would need to be adapted.
bool
candidate_uses_opposite_direction_lane(
  const map::Route& route,
  const AvoidanceGroup& group,
  double lateral_shift,
  bool in_lane,
  AvoidanceCandidateType candidate_type,
  const dynamics::PhysicalVehicleParameters& ego_params,
  const ObstacleAvoidanceParams& params )
{
  if( !params.opposite_lane_enabled )
  {
    return false;
  }

  if( candidate_type == AvoidanceCandidateType::OppositeDirection )
  {
    return true;
  }

  if( candidate_type == AvoidanceCandidateType::InLane || in_lane )
  {
    return false;
  }

  if( !std::isfinite( lateral_shift ) || std::fabs( lateral_shift ) < 1e-6 )
  {
    return false;
  }

  const auto conflict_interval =
    compute_opposite_lane_conflict_interval(
      route,
      group,
      lateral_shift,
      ego_params,
      params );

  if( conflict_interval.valid )
  {
    return conflict_interval.occupies_opposite_lane;
  }

  return false;
}

bool
candidate_respects_drivable_area( const map::Route& route,
                                  const AvoidanceGroup& group,
                                  double lateral_shift,
                                  const dynamics::PhysicalVehicleParameters& ego_params,
                                  const ObstacleAvoidanceParams& params,
                                  bool allow_adjacent_driving_lanes )
{
  if( route.reference_line.empty() )
  {
    return false;
  }

  const double ego_half_width =
    0.5 * std::max( params.min_vehicle_dimension, ego_params.body_width );

  for( const auto& [route_s, route_point] : route.reference_line )
  {
    const double center_l =
      avoidance_shift_offset_at_s(
        route_s,
        group,
        lateral_shift,
        ego_params,
        params );
    if( std::fabs( center_l ) <= 1e-6 )
    {
      continue;
    }

    const double ego_min_l = center_l - ego_half_width;
    const double ego_max_l = center_l + ego_half_width;

    const auto allowed_interval = get_allowed_lateral_interval_at_route_point(
      route,
      route_point,
      route_s,
      lateral_shift,
      allow_adjacent_driving_lanes,
      params );

    if( !allowed_interval.has_value() )
    {
      return false;
    }

    if( ego_min_l < allowed_interval->min_l || ego_max_l > allowed_interval->max_l )
    {
      return false;
    }
  }

  return true;
}

ShiftCandidate
make_candidate_from_obstacle_hulls(
  ShiftDirection direction,
  const AvoidanceGroup& group,
  const dynamics::PhysicalVehicleParameters& ego_params,
  const ObstacleAvoidanceParams& params )
{
  const double ego_half_width =
    0.5 * std::max( params.min_vehicle_dimension, ego_params.body_width );
  const bool shift_left = direction == ShiftDirection::Left;
  double required_shift = 0.0;
  std::size_t hull_count = 0;

  for( const auto& obs : group.obstacles )
  {
    if( !obs.overlaps_ego_corridor )
    {
      continue;
    }

    const double required_for_obs =
      shift_left
        ? obs.object_l_max + params.side_clearance + ego_half_width
        : obs.object_l_min - params.side_clearance - ego_half_width;

    required_shift =
      shift_left
        ? std::max( required_shift, required_for_obs )
        : std::min( required_shift, required_for_obs );
    ++hull_count;
  }

  return ShiftCandidate{
    required_shift,
    std::isfinite( required_shift ) &&
      ( shift_left ? required_shift > 0.0 : required_shift < 0.0 ) };
}

bool
candidate_respects_opposite_direction_area(
  const map::Route& route,
  const AvoidanceGroup& group,
  double lateral_shift,
  const dynamics::PhysicalVehicleParameters& ego_params,
  const ObstacleAvoidanceParams& params )
{
  if( route.reference_line.empty() )
  {
    return false;
  }

  if( !params.opposite_lane_enabled )
  {
    return false;
  }

  const double ego_half_width =
    0.5 * std::max( params.min_vehicle_dimension, ego_params.body_width );
  const bool shift_left = lateral_shift > 0.0;

  for( const auto& [route_s, route_point] : route.reference_line )
  {
    const double center_l =
      avoidance_shift_offset_at_s(
        route_s,
        group,
        lateral_shift,
        ego_params,
        params );
    if( std::fabs( center_l ) <= 1e-6 )
    {
      continue;
    }

    const double ego_min_l = center_l - ego_half_width;
    const double ego_max_l = center_l + ego_half_width;

    const auto opposite_query =
      query_opposite_direction_lateral_interval_at_route_point(
        route,
        route_point,
        route_s,
        shift_left,
        params );

    if( !opposite_query.map_usable || !opposite_query.has_opposite_lane )
    {
      return false;
    }

    const auto current_interval =
      get_allowed_lateral_interval_at_route_point(
        route,
        route_point,
        route_s,
        lateral_shift,
        false,
        params );

    const double allowed_min_l = current_interval.has_value()
      ? std::min( current_interval->min_l, opposite_query.interval.min_l )
      : opposite_query.interval.min_l;
    const double allowed_max_l = current_interval.has_value()
      ? std::max( current_interval->max_l, opposite_query.interval.max_l )
      : opposite_query.interval.max_l;

    if( ego_min_l < allowed_min_l || ego_max_l > allowed_max_l )
    {
      return false;
    }
  }

  return true;
}

void
evaluate_shift_candidate( ShiftCandidate& candidate,
                          const map::Route& route,
                          const AvoidanceGroup& group,
                          const dynamics::PhysicalVehicleParameters& ego_params,
                          const ObstacleAvoidanceParams& params )
{
  if( !candidate.valid )
  {
    return;
  }

  if( !params.enforce_drivable_area )
  {
    candidate.in_lane = false;

    if( ( candidate.type == AvoidanceCandidateType::AdjacentSameDirection &&
          params.adjacent_lane_enabled ) ||
        ( candidate.type == AvoidanceCandidateType::OppositeDirection &&
          params.opposite_lane_enabled ) )
    {
      return;
    }

    candidate.valid = false;
    return;
  }

  if( candidate.type == AvoidanceCandidateType::OppositeDirection )
  {
    candidate.in_lane = false;

    if( !params.opposite_lane_enabled )
    {
      candidate.valid = false;
      return;
    }

    if( !candidate_respects_opposite_direction_area(
          route,
          group,
          candidate.shift,
          ego_params,
          params ) )
    {
      candidate.valid = false;
    }

    return;
  }

  const bool fits_current_lane = candidate_respects_drivable_area(
    route,
    group,
    candidate.shift,
    ego_params,
    params,
    false );

  if( fits_current_lane )
  {
    candidate.in_lane = true;
    return;
  }

  candidate.in_lane = false;

  if( candidate.type == AvoidanceCandidateType::InLane )
  {
    candidate.valid = false;
    return;
  }

  if( !params.adjacent_lane_enabled )
  {
    candidate.valid = false;
    return;
  }

  auto same_direction_params = params;
  same_direction_params.opposite_lane_enabled = false;

  if( !candidate_respects_drivable_area(
        route,
        group,
        candidate.shift,
        ego_params,
        same_direction_params,
        true ) )
  {
    candidate.valid = false;
  }
}

std::vector<ShiftCandidate>
generate_shift_candidate_variants(
  const AvoidanceGroup& group,
  const dynamics::PhysicalVehicleParameters& ego_params,
  const ObstacleAvoidanceParams& params )
{
  std::vector<ShiftCandidate> candidates;

  const auto left_base =
    make_candidate_from_obstacle_hulls(
      ShiftDirection::Left,
      group,
      ego_params,
      params );
  const auto right_base =
    make_candidate_from_obstacle_hulls(
      ShiftDirection::Right,
      group,
      ego_params,
      params );

  const int extra_steps =
    params.enable_multi_candidate_route_shift
      ? std::max( 0, params.lateral_candidate_extra_steps )
      : 0;

  const double extra_step =
    std::max( 0.0, params.lateral_candidate_extra_step );

  auto append_side =
    [&]( const ShiftCandidate& base, double sign )
    {
      for( int i = 0; i <= extra_steps; ++i )
      {
        ShiftCandidate candidate;
        candidate.shift = base.shift + sign * extra_step * static_cast<double>( i );
        candidate.valid =
          std::isfinite( candidate.shift ) &&
          ( sign > 0.0 ? candidate.shift > 0.0 : candidate.shift < 0.0 );

        if( !candidate.valid )
        {
          continue;
        }

        const bool duplicate =
          std::any_of(
            candidates.begin(),
            candidates.end(),
            [&]( const ShiftCandidate& existing )
            {
              return std::fabs( existing.shift - candidate.shift ) < 1e-6;
            } );

        if( !duplicate )
        {
          candidates.push_back( candidate );
        }
      }
    };

  append_side( left_base, 1.0 );
  append_side( right_base, -1.0 );

  return candidates;
}


std::vector<ShiftCandidate>
generate_opposite_lane_candidate_variants(
  const map::Route& route,
  const AvoidanceGroup& group,
  const dynamics::PhysicalVehicleParameters& ego_params,
  const ObstacleAvoidanceParams& params )
{
  std::vector<ShiftCandidate> candidates;

  if( !params.opposite_lane_enabled )
  {
    return candidates;
  }

  const double ego_half_width =
    0.5 * std::max( params.min_vehicle_dimension, ego_params.body_width );
  const double required_lane_margin = 0.0;

  const std::array<double, 3> sample_s_values = {{
    group.envelope.object_s_min,
    0.5 * ( group.envelope.object_s_min + group.envelope.object_s_max ),
    group.envelope.object_s_max
  }};

  auto append_unique_candidate =
    [&]( double shift )
    {
      if( !std::isfinite( shift ) || std::fabs( shift ) < 1e-6 )
      {
        return;
      }

      const bool duplicate =
        std::any_of(
          candidates.begin(),
          candidates.end(),
          [&]( const ShiftCandidate& existing )
          {
            return std::fabs( existing.shift - shift ) < 1e-6;
          } );

      if( duplicate )
      {
        return;
      }

      ShiftCandidate candidate;
      candidate.shift = shift;
      candidate.valid = true;
      candidate.type = AvoidanceCandidateType::OppositeDirection;
      candidate.in_lane = false;
      candidates.push_back( candidate );
    };

  for( const bool shift_left : { true, false } )
  {
    LateralInterval opposite_intersection;
    bool initialized = false;
    bool all_samples_usable = true;

    for( const double sample_s : sample_s_values )
    {
      const auto route_point = find_reference_point_near_s( route, sample_s );
      if( !route_point.has_value() )
      {
        all_samples_usable = false;
        break;
      }

      const auto opposite_query =
        query_opposite_direction_lateral_interval_at_route_point(
          route,
          route_point->second,
          route_point->first,
          shift_left,
          params );

      if( !opposite_query.map_usable || !opposite_query.has_opposite_lane )
      {
        all_samples_usable = false;
        break;
      }

      if( !initialized )
      {
        opposite_intersection = opposite_query.interval;
        initialized = true;
      }
      else
      {
        opposite_intersection.min_l =
          std::max( opposite_intersection.min_l, opposite_query.interval.min_l );
        opposite_intersection.max_l =
          std::min( opposite_intersection.max_l, opposite_query.interval.max_l );
      }
    }

    if( !all_samples_usable || !initialized )
    {
      continue;
    }

    const double center_l_min =
      opposite_intersection.min_l + ego_half_width + required_lane_margin;
    const double center_l_max =
      opposite_intersection.max_l - ego_half_width - required_lane_margin;

    if( center_l_min > center_l_max )
    {
      continue;
    }

    double obstacle_clear_center_l = 0.0;
    if( shift_left )
    {
      const auto hull_shift =
        make_candidate_from_obstacle_hulls(
          ShiftDirection::Left,
          group,
          ego_params,
          params );
      obstacle_clear_center_l = hull_shift.shift;
    }
    else
    {
      const auto hull_shift =
        make_candidate_from_obstacle_hulls(
          ShiftDirection::Right,
          group,
          ego_params,
          params );
      obstacle_clear_center_l = hull_shift.shift;
    }

    const double opposite_center_l =
      0.5 * ( opposite_intersection.min_l + opposite_intersection.max_l );

    std::vector<double> requested_centers;
    requested_centers.push_back( obstacle_clear_center_l );
    requested_centers.push_back(
      std::clamp( opposite_center_l, center_l_min, center_l_max ) );

    std::size_t generated_for_side = 0;
    for( double requested_center_l : requested_centers )
    {
      const double shift = std::clamp( requested_center_l, center_l_min, center_l_max );

      if( shift_left && shift <= 0.0 )
      {
        continue;
      }
      if( !shift_left && shift >= 0.0 )
      {
        continue;
      }

      append_unique_candidate( shift );
      ++generated_for_side;

    }

  }

  return candidates;
}

TrajectoryValidationResult
validate_planned_shift_trajectory(
  const map::Route& route,
  const dynamics::Trajectory& trajectory,
  const AvoidanceGroup& group,
  double lateral_shift,
  bool in_lane,
  AvoidanceCandidateType candidate_type,
  const dynamics::PhysicalVehicleParameters& ego_params,
  const ObstacleAvoidanceParams& params,
  double initial_s_hint )
{
  TrajectoryValidationResult result;
  result.reason = "trajectory valid";

  if( !params.validate_shifted_trajectory )
  {
    return result;
  }

  if( trajectory.states.empty() )
  {
    result.valid = false;
    result.reason = "trajectory validation failed: empty trajectory";
    return result;
  }

  const double ego_half_width =
    0.5 * std::max( params.min_vehicle_dimension, ego_params.body_width );
  const double ego_front_offset =
    ego_params.wheelbase + ego_params.front_axle_to_front_border;
  const double ego_rear_offset =
    ego_params.rear_border_to_rear_axle;
  // Route points are shifted in the original route frame and the resulting
  // trajectory is then interpolated and projected back onto that frame. On
  // curved/discretized map segments this round trip introduces millimetre- to
  // centimetre-scale lateral error. Keep the configured clearance in route
  // construction exact, but do not reject that route for numerical noise.
  constexpr double geometry_validation_tolerance = 0.02;
  double previous_s = initial_s_hint;

  for( const auto& state : trajectory.states )
  {
    double state_s =
      adore::map::get_s_on_reference_line_segments(
        route,
        state,
        std::isfinite( previous_s ) ? previous_s : group.envelope.center_s,
        30.0 );

    if( !std::isfinite( state_s ) )
    {
      state_s = project_s_on_reference_line( route, state, previous_s );
    }

    if( !std::isfinite( state_s ) )
    {
      result.valid = false;
      result.reason = "trajectory validation failed: state does not project to route";
      return result;
    }

    previous_s = state_s;

    // Only validate the region where the avoidance route can matter, plus a
    // small buffer. This avoids rejecting future optimizer samples after the
    // maneuver has already returned to the original lane.
    const double planned_offset =
      avoidance_shift_offset_at_s(
        state_s,
        group,
        lateral_shift,
        ego_params,
        params );
    const double group_timing_s_min =
      group.envelope.object_s_min - std::max( 0.0, params.front_clearance );
    const double group_timing_s_max =
      group.envelope.object_s_max + std::max( 0.0, params.rear_clearance );
    const bool near_obstacle =
      state_s >= group_timing_s_min &&
      state_s <= group_timing_s_max;

    if( std::fabs( planned_offset ) <= 1e-6 && !near_obstacle )
    {
      continue;
    }

    const auto route_point = find_reference_point_near_s( route, state_s );
    if( !route_point.has_value() )
    {
      result.valid = false;
      result.reason = "trajectory validation failed: no route point near state";
      return result;
    }

    const auto frame = make_route_frame( route, state_s );
    const math::Point2d state_xy{ state.x, state.y };
    const double state_l = signed_lateral_offset( frame, state_xy );

    std::optional<LateralInterval> allowed_interval;

    if( candidate_type == AvoidanceCandidateType::OppositeDirection )
    {
      const auto opposite_query =
        query_opposite_direction_lateral_interval_at_route_point(
          route,
          route_point->second,
          route_point->first,
          lateral_shift > 0.0,
          params );

      if( opposite_query.map_usable && opposite_query.has_opposite_lane )
      {
        const auto current_interval =
          get_allowed_lateral_interval_at_route_point(
            route,
            route_point->second,
            route_point->first,
            lateral_shift,
            false,
            params );

        allowed_interval = opposite_query.interval;
        if( current_interval.has_value() )
        {
          allowed_interval->min_l =
            std::min( allowed_interval->min_l, current_interval->min_l );
          allowed_interval->max_l =
            std::max( allowed_interval->max_l, current_interval->max_l );
        }
      }
      else
      {
        result.valid = false;
        result.reason =
          "trajectory validation failed: no usable opposite-direction lane interval: " +
          opposite_query.reason;
        return result;
      }
    }
    else
    {
      auto validation_params = params;
      if( candidate_type == AvoidanceCandidateType::AdjacentSameDirection )
      {
        validation_params.opposite_lane_enabled = false;
      }

      allowed_interval =
        get_allowed_lateral_interval_at_route_point(
          route,
          route_point->second,
          route_point->first,
          lateral_shift,
          !in_lane,
          validation_params );
    }

    if( !allowed_interval.has_value() )
    {
      result.valid = false;
      result.reason = "trajectory validation failed: no usable lane interval";
      return result;
    }

    const double lane_ego_min_l = state_l - ego_half_width;
    const double lane_ego_max_l = state_l + ego_half_width;

    const double lane_margin =
      std::min( lane_ego_min_l - allowed_interval->min_l,
                allowed_interval->max_l - lane_ego_max_l );
    result.min_lane_margin =
      std::min( result.min_lane_margin, lane_margin );

    if( lane_margin < -geometry_validation_tolerance )
    {
      char buf[256];
      std::snprintf(
        buf,
        sizeof( buf ),
        "trajectory validation failed: ego footprint leaves drivable area at s=%.2f margin=%.2f",
        state_s,
        lane_margin );
      result.valid = false;
      result.reason = buf;
      return result;
    }

    const double ego_s_min = state_s - ego_rear_offset;
    const double ego_s_max = state_s + ego_front_offset;
    const double ego_min_l = state_l - ego_half_width;
    const double ego_max_l = state_l + ego_half_width;

    for( const auto& obstacle : group.obstacles )
    {
      const bool longitudinal_overlap =
        ego_s_max >= obstacle.object_s_min &&
        ego_s_min <= obstacle.object_s_max;

      if( longitudinal_overlap )
      {
        const double left_clearance = ego_min_l - obstacle.object_l_max;
        const double right_clearance = obstacle.object_l_min - ego_max_l;
        const double actual_clearance =
          std::max( left_clearance, right_clearance );
        const double required_clearance =
          std::max( 0.0, params.side_clearance );
        const double obstacle_lateral_margin =
          actual_clearance - required_clearance;

        result.min_obstacle_lateral_margin =
          std::min( result.min_obstacle_lateral_margin, obstacle_lateral_margin );

        if( obstacle_lateral_margin < -geometry_validation_tolerance )
        {

          char buf[256];
          std::snprintf(
            buf,
            sizeof( buf ),
            "trajectory validation failed: insufficient obstacle side clearance at s=%.2f actual_clearance=%.2f required=%.2f",
            state_s,
            actual_clearance,
            required_clearance );
          result.valid = false;
          result.reason = buf;
          return result;
        }
      }
    }
  }

  if( !std::isfinite( result.min_lane_margin ) )
  {
    result.min_lane_margin = 0.0;
  }

  if( !std::isfinite( result.min_obstacle_lateral_margin ) )
  {
    result.min_obstacle_lateral_margin = 0.0;
  }

  return result;
}

double
score_route_shift_candidate( const RouteShiftPlanCandidate& candidate,
                             const ObstacleAvoidanceParams& params )
{
  double score = 0.0;

  // Hard safety checks have already accepted the candidate; scoring is for
  // choosing the least intrusive accepted maneuver.
  score += std::fabs( candidate.shift_candidate.shift );
  score +=
    candidate.shift_candidate.in_lane
      ? 0.0
      : params.lateral_shift_penalty_score;
  score += candidate.uses_opposite_lane ? params.opposite_lane_penalty_score : 0.0;
  score -= 0.5 * std::max( 0.0, candidate.validation.min_lane_margin );
  score -= 0.5 * std::max( 0.0, candidate.validation.min_obstacle_lateral_margin );

  if( candidate.uses_opposite_lane &&
      std::isfinite( candidate.oncoming.oncoming_arrival_time ) )
  {
    const double gap_margin =
      candidate.oncoming.oncoming_arrival_time
      - candidate.oncoming.ego_clear_time;
    score -= 0.1 * std::max( 0.0, gap_margin );
  }

  return score;
}

} // namespace oa_detail
} // namespace planner
} // namespace adore
