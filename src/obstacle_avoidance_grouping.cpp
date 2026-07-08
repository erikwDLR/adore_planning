/********************************************************************************
 * Copyright (c) 2026 Contributors to the Eclipse Foundation
 *
 * SPDX-License-Identifier: EPL-2.0
 ********************************************************************************/

// Static-obstacle detection: participant classification (static / slow-oncoming /
// opposite-heading), per-obstacle route-frame envelopes, and collection of every
// relevant corridor intrusion for this planning cycle. Further or newly revealed
// objects are added cyclically from the driven-route corridor check without
// distance-based clustering. Depends on the geometry and projection helpers in
// oa_detail.

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
  // Prefer a real participant id for the group id. A frozen hull can be
  // id-independent, but the group should still use a real id when available.
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
  const std::vector<AvoidanceShiftContribution>* committed_contributions,
  const std::vector<int>* forced_participant_ids )
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

  // Start with the frozen hulls of the active maneuver. They are already in the
  // mission-route frame, so rebuilding from them cannot stack a shift on top of
  // the previously modified route. A later observation may only expand one of
  // these hulls; disappearance or tracker jitter never shrinks it.
  if( committed_contributions != nullptr )
  {
    obstacles.reserve(
      committed_contributions->size() +
      traffic_participants.participants.size() );

    for( const auto& contribution : *committed_contributions )
    {
      if( !std::isfinite( contribution.object_s_min ) ||
          !std::isfinite( contribution.object_s_max ) ||
          !std::isfinite( contribution.object_l_min ) ||
          !std::isfinite( contribution.object_l_max ) ||
          contribution.object_s_max < contribution.object_s_min ||
          contribution.object_l_max < contribution.object_l_min )
      {
        continue;
      }

      ObstacleEnvelope frozen;
      frozen.id =
        contribution.participant_ids.empty()
          ? -1
          : contribution.participant_ids.front();
      frozen.participant_ids = contribution.participant_ids;
      frozen.object_s_min = contribution.object_s_min;
      frozen.object_s_max = contribution.object_s_max;
      frozen.object_l_min = contribution.object_l_min;
      frozen.object_l_max = contribution.object_l_max;
      frozen.s_min = frozen.object_s_min;
      frozen.s_max = frozen.object_s_max;
      frozen.l_min = frozen.object_l_min;
      frozen.l_max = frozen.object_l_max;
      frozen.center_s = 0.5 * ( frozen.s_min + frozen.s_max );
      frozen.center_l = 0.5 * ( frozen.l_min + frozen.l_max );
      frozen.overlaps_ego_corridor = true;
      frozen.committed_hold = true;

      const bool valid_profile =
        contribution.has_persistent_profile &&
        std::isfinite( contribution.signed_shift ) &&
        std::isfinite( contribution.ramp_start_s ) &&
        std::isfinite( contribution.full_shift_start_s ) &&
        std::isfinite( contribution.full_shift_end_s ) &&
        std::isfinite( contribution.ramp_end_s ) &&
        contribution.ramp_start_s <= contribution.full_shift_start_s &&
        contribution.full_shift_start_s <= contribution.full_shift_end_s &&
        contribution.full_shift_end_s <= contribution.ramp_end_s;
      if( valid_profile )
      {
        frozen.has_persistent_profile = true;
        frozen.persistent_signed_shift = contribution.signed_shift;
        frozen.persistent_ramp_start_s = contribution.ramp_start_s;
        frozen.persistent_full_shift_start_s =
          contribution.full_shift_start_s;
        frozen.persistent_full_shift_end_s =
          contribution.full_shift_end_s;
        frozen.persistent_ramp_end_s = contribution.ramp_end_s;
      }

      obstacles.push_back( std::move( frozen ) );
    }
  }

  const auto merge_observation =
    [&]( ObstacleEnvelope observation, bool forced_from_driven_corridor )
    {
      auto existing_it =
        std::find_if(
          obstacles.begin(),
          obstacles.end(),
          [&]( const ObstacleEnvelope& existing )
          {
            return std::any_of(
              observation.participant_ids.begin(),
              observation.participant_ids.end(),
              [&]( int id )
              {
                return contains_participant_id(
                  existing.participant_ids, id );
              } );
          } );

      if( existing_it == obstacles.end() )
      {
        obstacles.push_back( std::move( observation ) );
        return;
      }

      existing_it->object_s_min =
        std::min( existing_it->object_s_min, observation.object_s_min );
      existing_it->object_s_max =
        std::max( existing_it->object_s_max, observation.object_s_max );
      existing_it->object_l_min =
        std::min( existing_it->object_l_min, observation.object_l_min );
      existing_it->object_l_max =
        std::max( existing_it->object_l_max, observation.object_l_max );
      for( const int id : observation.participant_ids )
      {
        if( !contains_participant_id( existing_it->participant_ids, id ) )
        {
          existing_it->participant_ids.push_back( id );
        }
      }
      existing_it->s_min = existing_it->object_s_min;
      existing_it->s_max = existing_it->object_s_max;
      existing_it->l_min = existing_it->object_l_min;
      existing_it->l_max = existing_it->object_l_max;
      existing_it->center_s =
        0.5 * ( existing_it->s_min + existing_it->s_max );
      existing_it->center_l =
        0.5 * ( existing_it->l_min + existing_it->l_max );
      existing_it->overlaps_ego_corridor = true;

      if( existing_it->has_persistent_profile )
      {
        const double ego_front_offset =
          ego_params.wheelbase + ego_params.front_axle_to_front_border;
        const double ego_rear_offset =
          ego_params.rear_border_to_rear_axle;
        const double expanded_full_start =
          existing_it->object_s_min - ego_front_offset;
        const double expanded_full_end =
          existing_it->object_s_max + ego_rear_offset;
        const double expanded_ramp_start =
          std::max(
            0.0,
            expanded_full_start -
              std::max( 0.0, params.front_clearance ) );
        const double expanded_ramp_end =
          expanded_full_end + std::max( 0.0, params.rear_clearance );

        existing_it->persistent_full_shift_start_s =
          std::min(
            existing_it->persistent_full_shift_start_s,
            expanded_full_start );
        existing_it->persistent_full_shift_end_s =
          std::max(
            existing_it->persistent_full_shift_end_s,
            expanded_full_end );
        existing_it->persistent_ramp_start_s =
          std::min(
            existing_it->persistent_ramp_start_s,
            expanded_ramp_start );
        existing_it->persistent_ramp_end_s =
          std::max(
            existing_it->persistent_ramp_end_s,
            expanded_ramp_end );
      }

      if( forced_from_driven_corridor )
      {
        // This hull invalidated the currently driven route and therefore must
        // be validated again from ego's live pose.
        existing_it->committed_hold = false;
      }
    };

  for( const auto& [id, participant] : traffic_participants.participants )
  {
    const int participant_id = static_cast<int>( id );
    const bool forced_from_driven_corridor =
      forced_participant_ids != nullptr &&
      contains_participant_id(
        *forced_participant_ids, participant_id );
    const bool updates_committed_hull =
      std::any_of(
        obstacles.begin(),
        obstacles.end(),
        [&]( const ObstacleEnvelope& obstacle )
        {
          return contains_participant_id(
            obstacle.participant_ids, participant_id );
        } );

    // During an active maneuver, membership comes exclusively from the corridor
    // of the route ego actually drives. Other mission-corridor objects are not
    // added merely because a different object triggered this replan.
    if( committed_contributions != nullptr &&
        !forced_from_driven_corridor &&
        !updates_committed_hull )
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
    env.id = participant_id;
    env.participant_ids.push_back( env.id );

    auto projection_params = params;
    if( forced_from_driven_corridor )
    {
      // The driven route may legitimately be several metres away from the
      // mission centerline. The modified-route corridor check already proved
      // relevance, so mission-frame plausibility limits must not discard this
      // object during the coordinate conversion.
      projection_params.max_projection_distance_from_route = 0.0;
      projection_params.max_object_lateral_distance =
        std::numeric_limits<double>::infinity();
    }

    if( !project_obstacle_to_route_analytic(
          route,
          participant,
          projection_params,
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
    if( opposite_heading &&
        !env.overlaps_ego_corridor &&
        !forced_from_driven_corridor )
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
    if( !env.overlaps_ego_corridor &&
        !forced_from_driven_corridor )
    {
      continue;
    }

    // A forced object was selected against the driven modified route. In the
    // mission frame it can sit outside the original trigger corridor; marking it
    // relevant makes candidate generation compute the absolute shift needed to
    // clear that hull from the fixed mission baseline.
    env.overlaps_ego_corridor = true;
    merge_observation(
      std::move( env ), forced_from_driven_corridor );
  }

  if( obstacles.empty() )
  {
    // Nothing intrudes into the ego corridor and no frozen hull remains.
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
