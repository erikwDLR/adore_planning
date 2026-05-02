/********************************************************************************
 * Copyright (c) 2026 Contributors to the Eclipse Foundation
 *
 * SPDX-License-Identifier: EPL-2.0
 ********************************************************************************/

#include "planning/obstacle_avoidance.hpp"

#include "adore_math/point.h"
#include "adore_math/pose.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

namespace adore
{
namespace planner
{
namespace
{

struct RouteFrame
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

struct ObstacleEnvelope
{
  int id = -1;
  double s_min = std::numeric_limits<double>::infinity();
  double s_max = -std::numeric_limits<double>::infinity();
  double l_min = std::numeric_limits<double>::infinity();
  double l_max = -std::numeric_limits<double>::infinity();
  double center_s = std::numeric_limits<double>::infinity();
  double center_l = 0.0;
};

double
normalize_angle( double angle )
{
  while( angle > M_PI ) angle -= 2.0 * M_PI;
  while( angle < -M_PI ) angle += 2.0 * M_PI;
  return angle;
}

double
smoothstep01( double t )
{
  t = std::clamp( t, 0.0, 1.0 );
  return t * t * ( 3.0 - 2.0 * t );
}

RouteFrame
make_route_frame( const map::Route& route, double s )
{
  const auto pose = route.get_pose_at_s( s );
  return RouteFrame{ pose.x, pose.y, pose.yaw };
}

double
signed_lateral_offset( const RouteFrame& frame, const math::Point2d& point )
{
  const double dx = point.x - frame.x;
  const double dy = point.y - frame.y;

  // +l = left of route tangent, -l = right of route tangent
  return -dx * std::sin( frame.yaw ) + dy * std::cos( frame.yaw );
}

math::Point2d
shifted_point( const RouteFrame& frame, double lateral_offset )
{
  return math::Point2d{
    frame.x - std::sin( frame.yaw ) * lateral_offset,
    frame.y + std::cos( frame.yaw ) * lateral_offset
  };
}

dynamics::Trajectory
make_stop_trajectory( const dynamics::VehicleStateDynamic& ego, double dt )
{
  dynamics::Trajectory trajectory;
  trajectory.label = "obstacle avoidance: waiting for oncoming traffic";

  auto stop_state = ego;
  stop_state.vx = 0.0;
  stop_state.time = ego.time + dt;

  trajectory.states.push_back( ego );
  trajectory.states.push_back( stop_state );

  return trajectory;
}

bool
project_obstacle_to_route( const map::Route& route,
                           const dynamics::TrafficParticipant& participant,
                           const ObstacleAvoidanceParams& params,
                           double ego_half_width,
                           ObstacleEnvelope& envelope )
{
  const auto corners = participant.get_corners();

  if( corners.points.size() < 3 )
  {
    return false;
  }

  for( const auto& corner : corners.points )
  {
    const double corner_s = route.get_s( corner );

    if( !std::isfinite( corner_s ) )
    {
      continue;
    }

    const auto frame = make_route_frame( route, corner_s );
    const double corner_l = signed_lateral_offset( frame, corner );

    envelope.s_min = std::min( envelope.s_min, corner_s );
    envelope.s_max = std::max( envelope.s_max, corner_s );
    envelope.l_min = std::min( envelope.l_min, corner_l );
    envelope.l_max = std::max( envelope.l_max, corner_l );
  }

  if( !std::isfinite( envelope.s_min ) || !std::isfinite( envelope.s_max ) )
  {
    return false;
  }

  envelope.s_min -= params.front_clearance;
  envelope.s_max += params.rear_clearance;

  envelope.l_min -= params.side_clearance;
  envelope.l_max += params.side_clearance;

  envelope.center_s = 0.5 * ( envelope.s_min + envelope.s_max );
  envelope.center_l = 0.5 * ( envelope.l_min + envelope.l_max );

  const bool overlaps_route_center_corridor =
    envelope.l_min <= ego_half_width + params.side_clearance &&
    envelope.l_max >= -ego_half_width - params.side_clearance;

  return overlaps_route_center_corridor;
}

std::optional<ObstacleEnvelope>
find_nearest_static_obstacle_on_route( const map::Route& route,
                                       const dynamics::VehicleStateDynamic& ego,
                                       const dynamics::TrafficParticipantSet& traffic_participants,
                                       const dynamics::PhysicalVehicleParameters& ego_params,
                                       const ObstacleAvoidanceParams& params )
{
  const double ego_s = route.get_s( ego );
  if( !std::isfinite( ego_s ) )
  {
    return std::nullopt;
  }

  const double ego_half_width = 0.5 * ego_params.body_width;

  std::optional<ObstacleEnvelope> best;

  for( const auto& [id, participant] : traffic_participants.participants )
  {
    if( std::fabs( participant.state.vx ) > params.max_static_object_speed )
    {
      continue;
    }

    const double participant_s = route.get_s( participant.state );
    if( !std::isfinite( participant_s ) )
    {
      continue;
    }

    const double ahead = participant_s - ego_s;
    if( ahead < params.min_obstacle_route_overlap || ahead > params.max_object_ahead )
    {
      continue;
    }

    ObstacleEnvelope env;
    env.id = id;

    if( !project_obstacle_to_route( route, participant, params, ego_half_width, env ) )
    {
      continue;
    }

    if( !best.has_value() || env.center_s < best->center_s )
    {
      best = env;
    }
  }

  return best;
}

bool
has_oncoming_traffic_conflict( const map::Route& route,
                               const dynamics::VehicleStateDynamic& ego,
                               const dynamics::TrafficParticipantSet& traffic_participants,
                               const ObstacleEnvelope& obstacle,
                               const ObstacleAvoidanceParams& params )
{
  const double ego_s = route.get_s( ego );
  if( !std::isfinite( ego_s ) )
  {
    return false;
  }

  const double conflict_start_s = std::max( ego_s, obstacle.s_min );
  const double conflict_end_s = obstacle.s_max + params.oncoming_lookahead_after_obj;

  for( const auto& [id, participant] : traffic_participants.participants )
  {
    (void)id;

    if( std::fabs( participant.state.vx ) <= params.max_static_object_speed )
    {
      continue;
    }

    const double other_s = route.get_s( participant.state );
    if( !std::isfinite( other_s ) )
    {
      continue;
    }

    if( other_s < conflict_start_s || other_s > conflict_end_s )
    {
      continue;
    }

    const auto route_pose = route.get_pose_at_s( other_s );
    const double heading_diff = std::fabs( normalize_angle( participant.state.yaw_angle - route_pose.yaw ) );

    if( heading_diff > params.min_oncoming_heading_diff )
    {
      return true;
    }
  }

  return false;
}

double
compute_left_shift_target( const ObstacleEnvelope& obstacle,
                           const dynamics::PhysicalVehicleParameters& ego_params,
                           const ObstacleAvoidanceParams& params )
{
  const double ego_half_width = 0.5 * ego_params.body_width;

  // Shift ego centerline left so that ego right side clears the inflated obstacle envelope.
  double target_shift = obstacle.l_max + ego_half_width + params.side_clearance;

  if( target_shift < params.min_lateral_shift )
  {
    target_shift = params.min_lateral_shift;
  }

  return std::clamp( target_shift, params.min_lateral_shift, params.max_lateral_shift );
}

map::Route
build_shifted_route( const map::Route& route,
                     const ObstacleEnvelope& obstacle,
                     double lateral_shift )
{
  map::Route shifted_route = route;

  const double entry_start_s = obstacle.s_min;
  const double entry_end_s   = obstacle.center_s;
  const double exit_start_s  = obstacle.center_s;
  const double exit_end_s    = obstacle.s_max;

  for( auto& [s, point] : shifted_route.reference_line )
  {
    if( s < entry_start_s || s > exit_end_s )
    {
      continue;
    }

    double alpha = 0.0;

    if( s < entry_end_s )
    {
      alpha = smoothstep01( ( s - entry_start_s ) / std::max( 0.1, entry_end_s - entry_start_s ) );
    }
    else if( s <= exit_start_s )
    {
      alpha = 1.0;
    }
    else
    {
      alpha = 1.0 - smoothstep01( ( s - exit_start_s ) / std::max( 0.1, exit_end_s - exit_start_s ) );
    }

    const double offset = lateral_shift * alpha;
    const auto frame = make_route_frame( route, s );
    const auto shifted_point_xy = shifted_point( frame, offset );

    point.x = shifted_point_xy.x;
    point.y = shifted_point_xy.y;
  }

  return shifted_route;
}

} // namespace

ObstacleAvoidanceResult
try_plan_obstacle_avoidance( TrajectoryPlanner& planner,
                             const map::Route& route,
                             const dynamics::VehicleStateDynamic& ego,
                             const dynamics::TrafficParticipantSet& traffic_participants,
                             const ObstacleAvoidanceParams& params )
{
  ObstacleAvoidanceResult result;
  result.modified_route = route;

  const auto obstacle = find_nearest_static_obstacle_on_route(
    route,
    ego,
    traffic_participants,
    planner.get_physical_vehicle_parameters(),
    params );

  if( !obstacle.has_value() )
  {
    result.reason = "no static obstacle on route";
    return result;
  }

  const auto vehicle_params = planner.get_physical_vehicle_parameters();
  const double target_shift = compute_left_shift_target( obstacle.value(), vehicle_params, params );

  result.mode = target_shift <= params.in_lane_shift_limit
                  ? ObstacleAvoidanceMode::InLaneShift
                  : ObstacleAvoidanceMode::OvertakeLeft;

  if( result.mode == ObstacleAvoidanceMode::OvertakeLeft &&
      has_oncoming_traffic_conflict( route, ego, traffic_participants, obstacle.value(), params ) )
  {
    result.mode = ObstacleAvoidanceMode::WaitForOncoming;
    result.trajectory = make_stop_trajectory( ego, params.stop_time_step );
    result.success = true;
    result.reason = "waiting for oncoming traffic";
    return result;
  }

  result.modified_route = build_shifted_route( route, obstacle.value(), target_shift );

  result.trajectory = planner.plan_route_trajectory(
    result.modified_route,
    ego,
    traffic_participants );

  if( result.trajectory.states.empty() )
  {
    result.success = false;
    result.reason = "trajectory planner returned empty trajectory for modified route";
    return result;
  }

  result.trajectory.label =
    result.mode == ObstacleAvoidanceMode::InLaneShift
      ? "driving mission (in-lane obstacle avoidance)"
      : "driving mission (obstacle overtaking)";

  result.success = true;
  result.reason = "planned Obstacle avoidance route";

  return result;
}

} // namespace planner
} // namespace adore
