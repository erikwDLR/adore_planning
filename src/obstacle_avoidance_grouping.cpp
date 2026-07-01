/********************************************************************************
 * Copyright (c) 2026 Contributors to the Eclipse Foundation
 *
 * SPDX-License-Identifier: EPL-2.0
 ********************************************************************************/

// Static-obstacle detection and clustering: participant classification
// (static / slow-oncoming / opposite-heading), per-obstacle envelopes grouped
// into avoidance groups, and selection of the relevant static group ahead of
// ego. Depends on the geometry and projection helpers in oa_detail.

#include "obstacle_avoidance_internal.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
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

AvoidanceGroup
make_avoidance_group_from_obstacle( const ObstacleEnvelope& obstacle )
{
  AvoidanceGroup group;
  group.obstacles.push_back( obstacle );
  group.envelope = obstacle;
  return group;
}

void
append_obstacle_to_avoidance_group( AvoidanceGroup& group,
                                   const ObstacleEnvelope& obstacle,
                                   const ObstacleAvoidanceParams& params,
                                   bool hard_merge )
{
  if( group.obstacles.empty() )
  {
    group = make_avoidance_group_from_obstacle( obstacle );
    return;
  }

  if( hard_merge )
  {
    // "Hard merge" is a maneuver-profile decision only: keep the shift held
    // between close obstacles. Do not merge lateral geometry here, because
    // candidate generation and validation must use the individual hulls.
    group.obstacles.push_back( obstacle );
    group.hard_merged = true;
  }
  else
  {
    group.obstacles.push_back( obstacle );
    group.uses_hull_curve = true;
  }

  group.envelope.object_s_min =
    std::min( group.envelope.object_s_min, obstacle.object_s_min );
  group.envelope.object_s_max =
    std::max( group.envelope.object_s_max, obstacle.object_s_max );
  group.envelope.object_l_min =
    std::min( group.envelope.object_l_min, obstacle.object_l_min );
  group.envelope.object_l_max =
    std::max( group.envelope.object_l_max, obstacle.object_l_max );
  group.envelope.participant_ids.insert(
    group.envelope.participant_ids.end(),
    obstacle.participant_ids.begin(),
    obstacle.participant_ids.end() );
  group.envelope.overlaps_ego_corridor =
    group.envelope.overlaps_ego_corridor || obstacle.overlaps_ego_corridor;

  refresh_obstacle_envelope_derived_values( group.envelope, params );
}

std::vector<AvoidanceGroup>
make_avoidance_groups_from_clusters(
  const std::vector<ObstacleEnvelope>& obstacle_hulls,
  const ObstacleAvoidanceParams& params )
{
  std::vector<AvoidanceGroup> groups;
  groups.reserve( obstacle_hulls.size() );

  const double hard_merge_gap_s =
    std::max( 0.0, params.cluster_hold_gap_s );
  const double shift_hull_gap_s =
    std::max( hard_merge_gap_s, params.shift_hull_gap_s );

  for( const auto& obstacle : obstacle_hulls )
  {
    if( groups.empty() )
    {
      groups.push_back( make_avoidance_group_from_obstacle( obstacle ) );
      continue;
    }

    const double gap_s =
      obstacle.object_s_min - groups.back().envelope.object_s_max;

    if( gap_s <= hard_merge_gap_s )
    {

      append_obstacle_to_avoidance_group( groups.back(), obstacle, params, true );
    }
    else if( gap_s <= shift_hull_gap_s )
    {

      append_obstacle_to_avoidance_group( groups.back(), obstacle, params, false );
    }
    else
    {
      groups.push_back( make_avoidance_group_from_obstacle( obstacle ) );
    }
  }

  return groups;
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
  const std::vector<int>* ignored_participant_ids )
{
  const double ego_s = project_s_on_reference_line( route, ego );
  if( !std::isfinite( ego_s ) )
  {
    return std::nullopt;
  }

  const double ego_half_width =
    0.5 * std::max( params.min_vehicle_dimension, ego_params.body_width );
  const double side_clearance = std::max( 0.0, params.side_clearance );

  const double search_start_s = ego_s + params.min_object_ahead;
  const double search_end_s   = ego_s + params.max_object_ahead;

  const double cluster_search_end_s =
    search_end_s +
    std::max(
      std::max( 0.0, params.cluster_hold_gap_s ),
      std::max( 0.0, params.shift_hull_gap_s ) ) +
    params.front_clearance +
    params.rear_clearance;

  std::vector<ObstacleEnvelope> obstacles;
  bool saw_trigger_obstacle = false;

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
        obstacle_timing_s_min > cluster_search_end_s )
    {
      continue;
    }

    // env.overlaps_ego_corridor marks a trigger obstacle (reaches into the ego
    // corridor); obstacles only within the wider band are secondary and do not
    // start a maneuver on their own.
    saw_trigger_obstacle = saw_trigger_obstacle || env.overlaps_ego_corridor;

    obstacles.push_back( env );
  }

  if( obstacles.empty() || !saw_trigger_obstacle )
  {
    // Nothing actually intrudes into the ego corridor: no maneuver. Obstacles
    // found only in the wider secondary band must never start a maneuver alone.
    return std::nullopt;
  }

  // Size the secondary-inclusion band to the avoidance shift this trigger set
  // will actually apply. An obstacle matters when the shifted ego corridor
  // (half-width ego_half_width + side_clearance, centered on the shifted route)
  // can reach it, i.e. when it lies within
  //   ego_half_width + side_clearance + (planned lateral shift)
  // of the original route centerline. Deriving the reach from the trigger
  // geometry (the same clearance shift candidate generation computes, plus the
  // candidate exploration steps) keeps this self-tuning: a small in-lane nudge
  // includes only near obstacles, a large lane-change shift reaches further. A
  // secondary obstacle is then validated for side clearance and carried as a
  // known maneuver obstacle, instead of only surfacing on the already-shifted
  // route and forcing a late stop. It never drives the shift itself (see the
  // overlaps_ego_corridor guard in make_candidate_from_obstacle_hulls).
  double max_trigger_shift = 0.0;
  for( const auto& env : obstacles )
  {
    if( !env.overlaps_ego_corridor )
    {
      continue;
    }

    const double shift_to_pass_left =
      env.object_l_max + side_clearance + ego_half_width;
    const double shift_to_pass_right =
      ego_half_width + side_clearance - env.object_l_min;
    max_trigger_shift =
      std::max( { max_trigger_shift, shift_to_pass_left, shift_to_pass_right } );
  }
  max_trigger_shift +=
    std::max( 0, params.lateral_candidate_extra_steps ) *
    std::max( 0.0, params.lateral_candidate_extra_step );

  const double secondary_inclusion_half_width =
    ego_half_width + side_clearance + max_trigger_shift;

  obstacles.erase(
    std::remove_if(
      obstacles.begin(),
      obstacles.end(),
      [&]( const ObstacleEnvelope& env )
      {
        if( env.overlaps_ego_corridor )
        {
          return false; // always keep trigger obstacles
        }
        return !( env.object_l_min <= secondary_inclusion_half_width &&
                  env.object_l_max >= -secondary_inclusion_half_width );
      } ),
    obstacles.end() );

  std::sort(
    obstacles.begin(),
    obstacles.end(),
    []( const ObstacleEnvelope& a, const ObstacleEnvelope& b )
    {
      if( std::fabs( a.object_s_min - b.object_s_min ) < 1e-6 )
      {
        return a.object_s_max < b.object_s_max;
      }
      return a.object_s_min < b.object_s_min;
    } );

  if( !params.clustering_enabled )
  {
    // Without clustering only the nearest trigger obstacle drives the maneuver;
    // secondary-band obstacles are not linked in this mode. obstacles is sorted
    // by s, so the first trigger is the nearest one.
    for( const auto& obstacle : obstacles )
    {
      if( obstacle.overlaps_ego_corridor )
      {
        return make_avoidance_group_from_obstacle( obstacle );
      }
    }

    return std::nullopt;
  }

  auto groups = make_avoidance_groups_from_clusters( obstacles, params );

  std::optional<AvoidanceGroup> best_group;
  double best_distance_s = std::numeric_limits<double>::infinity();

  for( const auto& group : groups )
  {
    if( !group.envelope.overlaps_ego_corridor )
    {
      continue;
    }

    const double group_timing_s_min =
      group.envelope.object_s_min - std::max( 0.0, params.front_clearance );
    const double group_timing_s_max =
      group.envelope.object_s_max + std::max( 0.0, params.rear_clearance );

    if( group_timing_s_max < search_start_s || group_timing_s_min > search_end_s )
    {
      continue;
    }

    const double nearest_relevant_s = std::max( group_timing_s_min, ego_s );
    const double distance_s = nearest_relevant_s - ego_s;

    if( !best_group.has_value() || distance_s < best_distance_s )
    {
      best_group = group;
      best_distance_s = distance_s;
    }
  }

  return best_group;
}

} // namespace oa_detail
} // namespace planner
} // namespace adore
