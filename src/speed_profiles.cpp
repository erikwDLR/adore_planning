/********************************************************************************
 * Copyright (c) 2025 Contributors to the Eclipse Foundation
 *
 * See the NOTICE file(s) distributed with this work for additional
 * information regarding copyright ownership.
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0
 *
 * SPDX-License-Identifier: EPL-2.0
 ********************************************************************************/

#include "planning/speed_profiles.hpp"

namespace adore
{
namespace planner
{

double
SpeedProfile::get_speed_at_s( double s ) const
{
  auto it = s_to_speed.lower_bound( s );
  if( it == s_to_speed.end() )
  {
    auto   it_last = s_to_speed.rbegin();
    auto   it_prev = std::next( it_last );
    double slope   = ( it_last->second - it_prev->second ) / ( it_last->first - it_prev->first );
    return it_last->second + slope * ( s - it_last->first );
  }
  else if( it == s_to_speed.begin() )
  {
    auto   it_next = std::next( it );
    double slope   = ( it_next->second - it->second ) / ( it_next->first - it->first );
    return it->second + slope * ( s - it->first );
  }
  else
  {
    auto   it_prev = std::prev( it );
    double ratio   = ( s - it_prev->first ) / ( it->first - it_prev->first );
    return it_prev->second + ratio * ( it->second - it_prev->second );
  }
}

double
SpeedProfile::get_acc_at_s( double s ) const
{
  auto it = s_to_acc.lower_bound( s );

  if( it == s_to_acc.end() )
    return s_to_acc.rbegin()->second;

  if( it == s_to_acc.begin() )
    return it->second;

  auto it_prev = std::prev( it );

  double ratio = ( s - it_prev->first ) / ( it->first - it_prev->first );

  return it_prev->second + ratio * ( it->second - it_prev->second );
}

void
SpeedProfile::generate_from_route_and_participants( const map::Route& route, const dynamics::TrafficParticipantSet& traffic_participants,
                                                    double initial_speed, double initial_s, double initial_time, double length )
{
  s_to_speed.clear();
  s_to_acc.clear();

  std::map<double, double> s_to_curvature = calculate_curvature_speeds( route, initial_s, length );

  auto it = route.reference_line.lower_bound( initial_s );

  if( it == route.reference_line.end() )
    return;

  double s_curr      = it->first;
  s_to_speed[s_curr] = std::max( 0.0, initial_speed );

  auto end_it = std::prev( route.reference_line.lower_bound( initial_s + length ) );

  auto prev_it = it;
  ++it;

  safety_distance = comfort_settings.distance_headway + vehicle_params.wheelbase + vehicle_params.front_axle_to_front_border;
  max_acc         = comfort_settings.max_acceleration;
  max_decel       = -comfort_settings.min_acceleration;

  forward_pass( it, end_it, prev_it, s_to_curvature, route, traffic_participants, initial_time );

  auto current_it  = end_it;
  auto previous_it = std::prev( current_it );

  backward_pass( previous_it, route, initial_s, current_it, length );
}

void
SpeedProfile::forward_pass( MapPointIter& it, MapPointIter& end_it, MapPointIter& prev_it, std::map<double, double>& s_to_curvature,
                            const adore::map::Route& route, const dynamics::TrafficParticipantSet& traffic_participants,
                            double initial_time )
{
  bool   stop = false;
  double time = initial_time;

  auto get_nearest_object_info_at_time = [&]( double s_curr, double time ) {
    double object_distance = std::numeric_limits<double>::max();
    double object_speed    = 0.0;

    for( const auto& [id, participant] : traffic_participants.participants )
    {
      auto state = participant.state;

      if( participant.trajectory.has_value() )
      {
        const auto& traj = participant.trajectory.value();
        state            = traj.get_state_at_time( time );
      }

      double obj_s = route.get_s( state );

      double offset = adore::math::distance_2d( state, route.get_pose_at_s( obj_s ) );

      // Width-aware lateral gate: the object is longitudinally relevant if the
      // two bodies could overlap laterally (center-to-centerline distance below
      // half ego width + half object width + margin). A fixed gate ignored wide
      // vehicles whose center is offset but whose body reaches into the lane.
      const double lateral_gate =
        0.5 * vehicle_params.body_width +
        0.5 * std::max( 0.1, participant.physical_parameters.body_width ) +
        0.2;

      if( offset > lateral_gate )
        continue;

      if( obj_s > s_curr )
      {
        double distance = obj_s - s_curr;

        if( distance < object_distance )
        {
          object_distance = distance;
          object_speed    = state.vx;
        }
      }
    }

    return std::make_pair( object_distance, object_speed );
  };


  for( ; it != end_it; ++it, ++prev_it )
  {
    double s_prev = prev_it->first;
    double s_curr = it->first;

    double delta_s = s_curr - s_prev;

    auto [object_distance, object_speed] = get_nearest_object_info_at_time( s_curr, time );

    if( object_distance < safety_distance )
      stop = true;

    if( stop )
    {
      s_to_speed[s_curr] = 0.0;
      continue;
    }

    const auto curvature_it = s_to_curvature.lower_bound( s_curr );
    double max_curvature_speed =
      curvature_it != s_to_curvature.end()
        ? curvature_it->second
        : comfort_settings.max_speed;

    // comfort_settings.max_speed is an absolute cap: where the map limit is higher
    // it is clamped down, so a per-vehicle max_speed actually limits cruising speed.
    double max_legal_speed = it->second.max_speed ? std::min( *it->second.max_speed, comfort_settings.max_speed )
                                                  : comfort_settings.max_speed;

    max_legal_speed *= comfort_settings.speed_fraction_of_limit;

    double desired_speed = std::min( { max_curvature_speed, max_legal_speed } );
    if( desired_speed <= 0.0 )
    {
      s_to_speed[s_curr] = 0.0;
      s_to_acc[s_curr] = -max_decel;
      stop = true;
      continue;
    }

    double idm_acc = idm::calculate_idm_acc( route.get_length() - s_curr, object_distance, desired_speed, comfort_settings.time_headway,
                                             safety_distance, s_to_speed[s_prev], max_acc, object_speed );

    idm_acc = std::clamp( idm_acc, -max_decel, max_acc );

    double idm_speed = std::sqrt( s_to_speed[s_prev] * s_to_speed[s_prev] + 2 * idm_acc * delta_s );

    if( !std::isfinite( idm_speed ) )
      idm_speed = 0.0;

    // IMPORTANT FIX
    idm_speed = std::min( idm_speed, desired_speed );

    s_to_speed[s_curr] = idm_speed;
    s_to_acc[s_curr]   = idm_acc;

    time += delta_s / std::max( idm_speed, 0.1 );

    if( idm_speed == 0.0 )
      stop = true;
  }
}

void
SpeedProfile::backward_pass( MapPointIter& previous_it, const adore::map::Route& route, double initial_s, MapPointIter& current_it,
                             double length )
{
  while( previous_it != route.reference_line.lower_bound( initial_s ) )
  {
    double s_prev = previous_it->first;
    double s_curr = current_it->first;

    double delta_s = s_curr - s_prev;

    double braking_speed = std::sqrt( s_to_speed[s_curr] * s_to_speed[s_curr] + 2.0 * max_decel * delta_s );

    if( s_to_speed[s_prev] > braking_speed )
    {
      s_to_speed[s_prev] = braking_speed;
      s_to_acc[s_prev]   = -max_decel;
    }

    if( previous_it == route.reference_line.begin() )
      break;

    --current_it;
    --previous_it;
  }
}

std::map<double, double>
SpeedProfile::calculate_curvature_speeds( const adore::map::Route& route, double initial_s, double length, double max_curvature )
{
  std::map<double, double> s_to_curvature;

  if( route.reference_line.size() < 3 )
    return s_to_curvature;

  auto begin_it = std::next( route.reference_line.lower_bound( initial_s ) );

  auto end_it = route.reference_line.upper_bound( initial_s + length );

  for( auto it = begin_it; std::next( it ) != end_it; ++it )
  {
    double s = it->first;

    double curvature = route.get_curvature_at_s( s );

    curvature = std::min( std::fabs( curvature ), max_curvature );

    double vmax = std::sqrt( comfort_settings.max_lateral_acceleration / std::max( curvature, 1e-6 ) );

    // IMPORTANT SAFETY MARGIN
    vmax *= 0.7;

    s_to_curvature[s] = vmax;
  }

  return s_to_curvature;
}

} // namespace planner
} // namespace adore
