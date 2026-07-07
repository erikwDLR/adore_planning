/********************************************************************************
 * Copyright (c) 2026 Contributors to the Eclipse Foundation
 *
 * SPDX-License-Identifier: EPL-2.0
 ********************************************************************************/

// Static-obstacle detection: participant classification (static / slow-oncoming /
// opposite-heading), per-obstacle route-frame envelopes, and selection of the
// single nearest static obstacle reaching into the ego corridor ahead. No
// clustering -- a further obstacle is handled cyclically from the driven route by
// the active-maneuver corridor check. Depends on the geometry and projection
// helpers in oa_detail.

#include "obstacle_avoidance_internal.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

namespace adore
{
namespace planner
{
namespace oa_detail
{

bool
participant_has_future_motion_prediction(
  const dynamics::TrafficParticipant& participant,
  double min_motion_speed,
  double min_motion_distance )
{
  if( !participant.trajectory.has_value() ||
      participant.trajectory->states.size() < 2 )
  {
    return false;
  }

  const double now_time = participant.state.time;
  const double motion_speed_threshold =
    std::max( 0.0, min_motion_speed );
  const double motion_distance_threshold =
    std::max( 0.5, min_motion_distance );

  bool saw_usable_state = false;

  for( const auto& state : participant.trajectory->states )
  {
    if( std::isfinite( now_time ) &&
        std::isfinite( state.time ) &&
        state.time + 0.5 < now_time )
    {
      continue;
    }

    saw_usable_state = true;

    if( std::fabs( state.vx ) > motion_speed_threshold )
    {
      return true;
    }

    const double distance_from_current =
      std::hypot(
        state.x - participant.state.x,
        state.y - participant.state.y );

    if( distance_from_current > motion_distance_threshold )
    {
      return true;
    }
  }

  return saw_usable_state && std::fabs( participant.state.vx ) > motion_speed_threshold;
}

bool
participant_is_slow_opposite_direction_traffic(
  const map::Route& route,
  const dynamics::TrafficParticipant& participant,
  const ObstacleAvoidanceParams& params )
{
  const double speed = std::fabs( participant.state.vx );

  if( speed > params.max_static_object_speed )
  {
    return false;
  }

  const double participant_s =
    project_s_on_reference_line( route, participant.state );
  if( !std::isfinite( participant_s ) )
  {
    return false;
  }

  const auto route_pose = route.get_pose_at_s( participant_s );
  const double yaw_diff =
    normalize_angle( participant.state.yaw_angle - route_pose.yaw );

  return std::fabs( yaw_diff ) >= params.min_oncoming_heading_diff;
}

bool
participant_heading_is_opposite_to_route(
  const map::Route& route,
  const dynamics::TrafficParticipant& participant,
  double participant_s,
  const ObstacleAvoidanceParams& params )
{
  if( !std::isfinite( participant_s ) )
  {
    return false;
  }

  const auto route_pose = route.get_pose_at_s( participant_s );
  const double yaw_diff =
    normalize_angle( participant.state.yaw_angle - route_pose.yaw );

  return std::fabs( yaw_diff ) >= params.min_oncoming_heading_diff;
}

// Carry several simultaneously-detected obstacles as one avoidance group. The
// obstacles keep their individual hulls (the shift is composed per obstacle in
// avoidance_shift_offset_at_s, so there is no lateral union that would inflate the
// shift); the envelope is their union, used only for the group s-window, the
// participant id list and coarse hints. This is NOT s-gap clustering: it does not
// decide what belongs together by longitudinal gap, it simply carries everything
// the caller already selected (all ego-corridor intrusions this cycle).
AvoidanceGroup
make_avoidance_group_from_obstacles( std::vector<ObstacleEnvelope> obstacles )
{
  AvoidanceGroup group;
  if( obstacles.empty() )
  {
    return group;
  }

  std::sort(
    obstacles.begin(),
    obstacles.end(),
    []( const ObstacleEnvelope& a, const ObstacleEnvelope& b )
    {
      return a.object_s_min < b.object_s_min;
    } );

  ObstacleEnvelope envelope;
  // Prefer a real participant id for the group id: a synthetic "hold" obstacle carries
  // id -1 and may sort first, but the group should still be identified by a real object.
  envelope.id = obstacles.front().id;
  for( const auto& obstacle : obstacles )
  {
    if( obstacle.id >= 0 )
    {
      envelope.id = obstacle.id;
      break;
    }
  }
  for( const auto& obstacle : obstacles )
  {
    envelope.object_s_min = std::min( envelope.object_s_min, obstacle.object_s_min );
    envelope.object_s_max = std::max( envelope.object_s_max, obstacle.object_s_max );
    envelope.object_l_min = std::min( envelope.object_l_min, obstacle.object_l_min );
    envelope.object_l_max = std::max( envelope.object_l_max, obstacle.object_l_max );
    envelope.participant_ids.insert(
      envelope.participant_ids.end(),
      obstacle.participant_ids.begin(),
      obstacle.participant_ids.end() );
    envelope.overlaps_ego_corridor =
      envelope.overlaps_ego_corridor || obstacle.overlaps_ego_corridor;
  }
  envelope.s_min = envelope.object_s_min;
  envelope.s_max = envelope.object_s_max;
  envelope.l_min = envelope.object_l_min;
  envelope.l_max = envelope.object_l_max;
  envelope.center_s = 0.5 * ( envelope.s_min + envelope.s_max );
  envelope.center_l = 0.5 * ( envelope.l_min + envelope.l_max );

  group.obstacles = std::move( obstacles );
  group.envelope = envelope;
  return group;
}

bool
avoidance_group_contains_participant_id( const AvoidanceGroup& group, int id )
{
  return std::any_of(
    group.obstacles.begin(),
    group.obstacles.end(),
    [id]( const ObstacleEnvelope& obstacle )
    {
      if( obstacle.id == id )
      {
        return true;
      }

      return std::find(
               obstacle.participant_ids.begin(),
               obstacle.participant_ids.end(),
               id ) != obstacle.participant_ids.end();
    } );
}

std::optional<AvoidanceGroup>
find_static_obstacle_group_on_route(
  const map::Route& route,
  const dynamics::VehicleStateDynamic& ego,
  const dynamics::TrafficParticipantSet& traffic_participants,
  const dynamics::PhysicalVehicleParameters& ego_params,
  const ObstacleAvoidanceParams& params,
  const std::vector<int>* ignored_participant_ids,
  double held_shift_s_min,
  double held_shift_s_max,
  double held_lateral_shift )
{
  const double ego_s = project_s_on_reference_line( route, ego );
  if( !std::isfinite( ego_s ) )
  {
    return std::nullopt;
  }

  const double ego_half_width =
    0.5 * std::max( params.min_vehicle_dimension, ego_params.body_width );

  const double search_start_s = ego_s + params.min_object_ahead;
  const double search_end_s   = ego_s + params.max_object_ahead;

  std::vector<ObstacleEnvelope> obstacles;

  for( const auto& [id, participant] : traffic_participants.participants )
  {
    // Skip obstacles already handled by an active maneuver (e.g. the one ego is
    // currently passing): detection must lock onto the genuinely new obstacle,
    // not re-target a participant the caller is already avoiding.
    if( ignored_participant_ids != nullptr &&
        contains_participant_id( *ignored_participant_ids, static_cast<int>( id ) ) )
    {
      continue;
    }

    if( std::fabs( participant.state.vx ) > params.max_static_object_speed )
    {
      continue;
    }

    if( participant_has_future_motion_prediction(
          participant,
          params.max_static_object_speed,
          1.0 ) ||
        ( std::fabs( participant.state.vx ) > params.min_motion_speed &&
          participant_is_slow_opposite_direction_traffic(
            route,
            participant,
            params ) ) )
    {
      continue;
    }

    ObstacleEnvelope env;
    env.id = static_cast<int>( id );
    env.participant_ids.push_back( env.id );

    if( !project_obstacle_to_route_analytic(
          route,
          participant,
          params,
          ego_s,
          ego_half_width,
          env ) )
    {
      continue;
    }

    const bool opposite_heading =
      participant_heading_is_opposite_to_route(
        route,
        participant,
        0.5 * ( env.object_s_min + env.object_s_max ),
        params );

    // Opposite-heading obstacles are only avoidance obstacles when they reach
    // into the actual ego (trigger) corridor; otherwise they are oncoming /
    // other-lane traffic handled by the oncoming logic, not the static shift.
    // Secondary (non-trigger) obstacles are therefore never opposite-heading.
    if( opposite_heading && !env.overlaps_ego_corridor )
    {
      continue;
    }

    const double obstacle_timing_s_min =
      env.object_s_min - std::max( 0.0, params.front_clearance );
    const double obstacle_timing_s_max =
      env.object_s_max + std::max( 0.0, params.rear_clearance );

    if( obstacle_timing_s_max < search_start_s ||
        obstacle_timing_s_min > search_end_s )
    {
      continue;
    }

    // Only obstacles that actually reach into the ego corridor drive a maneuver.
    // Every one of them is carried (see below); objects outside the corridor are
    // not collected now that clustering / secondary inclusion are gone.
    if( !env.overlaps_ego_corridor )
    {
      continue;
    }

    obstacles.push_back( env );
  }

  const bool have_committed_hold =
    std::isfinite( held_shift_s_min ) && std::isfinite( held_shift_s_max ) &&
    held_shift_s_max > held_shift_s_min &&
    std::fabs( held_lateral_shift ) > 1e-3;

  // Mark every real obstacle overlapping the committed hold span as belonging to the
  // maneuver ego is already executing, so its clearance is not re-validated from ego's
  // transient turn-in pose (where the object ego is currently passing spuriously fails
  // even though the committed route clears it). Geometric and id-independent: it is the
  // committed span, not an obstacle id, that decides what ego is currently passing.
  if( have_committed_hold )
  {
    for( auto& env : obstacles )
    {
      if( env.object_s_min <= held_shift_s_max &&
          env.object_s_max >= held_shift_s_min )
      {
        env.committed_hold = true;
      }
    }
  }

  // Reconstruct the committed shift as a synthetic "hold" obstacle so a mid-maneuver
  // replan keeps ego at (at least) its committed offset over the committed span, and
  // the per-object bridge extends that shift smoothly into any newly detected object
  // instead of dipping back toward the lane between them. This is the maneuver's own
  // geometry (its s-span + lateral_shift, passed in by the caller from the active
  // state), NOT an obstacle-id memory: it holds even when perception drops the real
  // object mid-shift, and it is id-independent -- detection is purely geometric.
  if( have_committed_hold )
  {
    const double side_clearance = std::max( 0.0, params.side_clearance );

    ObstacleEnvelope held;
    held.object_s_min = held_shift_s_min;
    held.object_s_max = held_shift_s_max;
    // Back-compute the obstacle edge so required_signed_shift_for_obstacle reproduces
    // exactly held_lateral_shift (a thin hull on the far side of that edge).
    if( held_lateral_shift > 0.0 )
    {
      held.object_l_max = held_lateral_shift - side_clearance - ego_half_width;
      held.object_l_min = held.object_l_max - 0.1;
    }
    else
    {
      held.object_l_min = held_lateral_shift + side_clearance + ego_half_width;
      held.object_l_max = held.object_l_min + 0.1;
    }
    held.s_min = held.object_s_min;
    held.s_max = held.object_s_max;
    held.l_min = held.object_l_min;
    held.l_max = held.object_l_max;
    held.center_s = 0.5 * ( held.s_min + held.s_max );
    held.center_l = 0.5 * ( held.l_min + held.l_max );
    held.overlaps_ego_corridor = true;
    held.committed_hold = true;
    obstacles.push_back( held );
  }

  if( obstacles.empty() )
  {
    // Nothing intrudes into the ego corridor and no committed shift to hold.
    return std::nullopt;
  }

  // Carry EVERY static object currently intruding the ego corridor in one group,
  // so the modified route is (re)planned to clear all of them at once. The shift is
  // composed per obstacle (each gets its own clearance over its own ramp, max where
  // ramps overlap, back toward the lane where they do not), so there is no lateral
  // union that inflates the shift and no s-gap clustering that pre-groups far or
  // occluded obstacles. Detection runs every cycle, so it does not matter when an
  // object enters the corridor -- a fragment of the same object, a late or
  // previously occluded one is picked up as soon as it intrudes and the route
  // adapts on the next replan.
  return make_avoidance_group_from_obstacles( std::move( obstacles ) );
}

} // namespace oa_detail
} // namespace planner
} // namespace adore
