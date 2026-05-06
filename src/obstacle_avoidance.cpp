/********************************************************************************
 * Copyright (c) 2026 Contributors to the Eclipse Foundation
 *
 * SPDX-License-Identifier: EPL-2.0
 ********************************************************************************/

#include "planning/obstacle_avoidance.hpp"

#include "adore_math/point.h"
#include "adore_math/pose.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <limits>
#include <optional>
#include <string>

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

  // Raw object extent in route/Frenet-like coordinates before adding safety distances.
  double object_s_min = std::numeric_limits<double>::infinity();
  double object_s_max = -std::numeric_limits<double>::infinity();
  double object_l_min = std::numeric_limits<double>::infinity();
  double object_l_max = -std::numeric_limits<double>::infinity();

  // Inflated envelope used for planning and stop/oncoming checks.
  double s_min = std::numeric_limits<double>::infinity();
  double s_max = -std::numeric_limits<double>::infinity();
  double l_min = std::numeric_limits<double>::infinity();
  double l_max = -std::numeric_limits<double>::infinity();

  double center_s = std::numeric_limits<double>::infinity();
  double center_l = 0.0;
};

struct ShiftCandidate
{
  double shift = 0.0; // +left, -right
  bool valid = false;
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
  // Quintic smootherstep: zero slope and zero curvature at both ends.
  t = std::clamp( t, 0.0, 1.0 );
  return t * t * t * ( t * ( t * 6.0 - 15.0 ) + 10.0 );
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

  // +l = left of route tangent, -l = right of route tangent.
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
  trajectory.label = "obstacle avoidance: stop fallback";

  auto stop_state = ego;
  stop_state.vx = 0.0;
  stop_state.time = ego.time + dt;

  trajectory.states.push_back( ego );
  trajectory.states.push_back( stop_state );

  return trajectory;
}

map::Route
build_stop_route_before_obstacle( const map::Route& route,
                                  const ObstacleEnvelope& obstacle,
                                  const dynamics::VehicleStateDynamic& ego,
                                  double stop_distance_before_obstacle )
{
  map::Route stop_route = route;

  const double ego_s = route.get_s( ego );
  if( !std::isfinite( ego_s ) )
  {
    return stop_route;
  }

  const double stop_s = std::max(
    ego_s,
    obstacle.object_s_min - stop_distance_before_obstacle );

  for( auto& [s, point] : stop_route.reference_line )
  {
    if( s >= stop_s )
    {
      point.max_speed = 0.0;
    }
  }

  return stop_route;
}

bool
project_obstacle_to_route_analytic( const map::Route& route,
                                    const dynamics::TrafficParticipant& participant,
                                    const ObstacleAvoidanceParams& params,
                                    double ego_s,
                                    double ego_half_width,
                                    ObstacleEnvelope& envelope )
{
  const double center_s = route.get_s( participant.state );
  if( !std::isfinite( center_s ) )
  {
    return false;
  }

  const double ahead = center_s - ego_s;
  if( ahead < -params.rear_clearance || ahead > params.max_object_ahead + params.front_clearance )
  {
    return false;
  }

  const auto center_frame = make_route_frame( route, center_s );
  const math::Point2d center_point{ participant.state.x, participant.state.y };
  const double center_l = signed_lateral_offset( center_frame, center_point );

  const double rel_yaw = normalize_angle( participant.state.yaw_angle - center_frame.yaw );
  const double cos_yaw = std::cos( rel_yaw );
  const double sin_yaw = std::sin( rel_yaw );

  const double half_length = 0.5 * std::max( 0.1, participant.physical_parameters.body_length );
  const double half_width  = 0.5 * std::max( 0.1, participant.physical_parameters.body_width );

  const std::array<std::pair<double, double>, 4> local_corners = {{
    { -half_length, -half_width },
    { -half_length,  half_width },
    {  half_length,  half_width },
    {  half_length, -half_width }
  }};

  // Sanath-like idea, but kept local: build the rotated obstacle footprint directly
  // in route-relative s/l coordinates. This avoids route.get_s(corner) for each corner.
  for( const auto& [local_x, local_y] : local_corners )
  {
    const double corner_s = center_s + local_x * cos_yaw - local_y * sin_yaw;
    const double corner_l = center_l + local_x * sin_yaw + local_y * cos_yaw;

    envelope.object_s_min = std::min( envelope.object_s_min, corner_s );
    envelope.object_s_max = std::max( envelope.object_s_max, corner_s );
    envelope.object_l_min = std::min( envelope.object_l_min, corner_l );
    envelope.object_l_max = std::max( envelope.object_l_max, corner_l );
  }

  if( !std::isfinite( envelope.object_s_min ) ||
      !std::isfinite( envelope.object_s_max ) ||
      !std::isfinite( envelope.object_l_min ) ||
      !std::isfinite( envelope.object_l_max ) )
  {
    return false;
  }

  if( std::max( std::fabs( envelope.object_l_min ), std::fabs( envelope.object_l_max ) ) >
      params.max_object_lateral_distance )
  {
    std::fprintf(
      stderr,
      "[OA] reject obstacle id=%d: implausible raw_l=[%.2f, %.2f], max_object_lateral_distance=%.2f\n",
      envelope.id,
      envelope.object_l_min,
      envelope.object_l_max,
      params.max_object_lateral_distance );
    std::fflush( stderr );
    return false;
  }

  const double corridor_half_width = ego_half_width + params.ego_corridor_safety_margin;
  const bool overlaps_ego_corridor =
    envelope.object_l_min <= corridor_half_width &&
    envelope.object_l_max >= -corridor_half_width;

  const double distance_to_corridor =
    envelope.object_l_min > corridor_half_width
      ? envelope.object_l_min - corridor_half_width
      : ( envelope.object_l_max < -corridor_half_width
            ? -corridor_half_width - envelope.object_l_max
            : 0.0 );

  std::fprintf(
    stderr,
    "[OA] obstacle id=%d: center_s=%.2f center_l=%.2f rel_yaw=%.2f "
    "raw_s=[%.2f, %.2f] raw_l=[%.2f, %.2f] corridor=[%.2f, %.2f] "
    "distance_to_corridor=%.2f overlap=%s\n",
    envelope.id,
    center_s,
    center_l,
    rel_yaw,
    envelope.object_s_min,
    envelope.object_s_max,
    envelope.object_l_min,
    envelope.object_l_max,
    -corridor_half_width,
    corridor_half_width,
    distance_to_corridor,
    overlaps_ego_corridor ? "true" : "false" );
  std::fflush( stderr );

  if( !overlaps_ego_corridor )
  {
    return false;
  }

  // Planning envelope. Inflate only after relevance was established on raw geometry.
  envelope.s_min = envelope.object_s_min - params.front_clearance;
  envelope.s_max = envelope.object_s_max + params.rear_clearance;
  envelope.l_min = envelope.object_l_min - params.side_clearance;
  envelope.l_max = envelope.object_l_max + params.side_clearance;

  envelope.center_s = 0.5 * ( envelope.s_min + envelope.s_max );
  envelope.center_l = 0.5 * ( envelope.l_min + envelope.l_max );

  return true;
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

  const double search_start_s = ego_s + params.min_obstacle_route_overlap;
  const double search_end_s   = ego_s + params.max_object_ahead;

  std::optional<ObstacleEnvelope> best;
  double best_distance_s = std::numeric_limits<double>::infinity();

  for( const auto& [id, participant] : traffic_participants.participants )
  {
    if( std::fabs( participant.state.vx ) > params.max_static_object_speed )
    {
      continue;
    }

    ObstacleEnvelope env;
    env.id = static_cast<int>( id );

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

    if( env.s_max < search_start_s || env.s_min > search_end_s )
    {
      std::fprintf(
        stderr,
        "[OA] ignore obstacle id=%d: inflated_s=[%.2f, %.2f] outside search=[%.2f, %.2f]\n",
        env.id,
        env.s_min,
        env.s_max,
        search_start_s,
        search_end_s );
      std::fflush( stderr );
      continue;
    }

    const double nearest_relevant_s = std::max( env.s_min, ego_s );
    const double distance_s = nearest_relevant_s - ego_s;

    if( !best.has_value() || distance_s < best_distance_s )
    {
      best = env;
      best_distance_s = distance_s;
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

ShiftCandidate
make_left_candidate( const ObstacleEnvelope& obstacle,
                     const dynamics::PhysicalVehicleParameters& ego_params,
                     const ObstacleAvoidanceParams& params )
{
  const double ego_half_width = 0.5 * ego_params.body_width;
  const double raw_shift = obstacle.object_l_max + ego_half_width + params.side_clearance;
  const double shift = std::max( raw_shift, params.min_lateral_shift );

  return ShiftCandidate{ shift, shift <= params.max_lateral_shift };
}

ShiftCandidate
make_right_candidate( const ObstacleEnvelope& obstacle,
                      const dynamics::PhysicalVehicleParameters& ego_params,
                      const ObstacleAvoidanceParams& params )
{
  const double ego_half_width = 0.5 * ego_params.body_width;
  const double raw_shift = obstacle.object_l_min - ego_half_width - params.side_clearance;
  const double shift = std::min( raw_shift, -params.min_lateral_shift );

  return ShiftCandidate{ shift, std::fabs( shift ) <= params.max_lateral_shift };
}

std::optional<ShiftCandidate>
choose_shift_candidate( const ObstacleEnvelope& obstacle,
                        const dynamics::PhysicalVehicleParameters& ego_params,
                        const ObstacleAvoidanceParams& params )
{
  const auto left = make_left_candidate( obstacle, ego_params, params );
  const auto right = make_right_candidate( obstacle, ego_params, params );

  if( left.valid && !right.valid ) return left;
  if( right.valid && !left.valid ) return right;
  if( !left.valid && !right.valid ) return std::nullopt;

  const double abs_left = std::fabs( left.shift );
  const double abs_right = std::fabs( right.shift );

  if( std::fabs( abs_left - abs_right ) < 1e-6 )
  {
    return params.prefer_left_shift ? left : right;
  }

  return abs_left < abs_right ? left : right;
}

map::Route
build_modified_avoidance_route( const map::Route& route,
                                const ObstacleEnvelope& obstacle,
                                double lateral_shift,
                                const ObstacleAvoidanceParams& params )
{
  map::Route modified_route = route;

  // Full lateral shift is reached before the raw obstacle begins and held until
  // the raw obstacle plus rear clearance is passed. Entry/return are x^5 blends.
  const double plateau_start_s = std::max( 0.0, obstacle.object_s_min - params.front_clearance );
  const double plateau_end_s   = obstacle.object_s_max + params.rear_clearance;

  const double shift_start_s = std::max( 0.0, plateau_start_s - params.entry_extra_distance );
  const double shift_end_s   = plateau_end_s + params.return_extra_distance;

  for( auto& [s, point] : modified_route.reference_line )
  {
    if( s < shift_start_s || s > shift_end_s )
    {
      continue;
    }

    double alpha = 0.0;

    if( s < plateau_start_s )
    {
      alpha = smoothstep01(
        ( s - shift_start_s ) /
        std::max( 0.1, plateau_start_s - shift_start_s ) );
    }
    else if( s <= plateau_end_s )
    {
      alpha = 1.0;
    }
    else
    {
      alpha = 1.0 - smoothstep01(
        ( s - plateau_end_s ) /
        std::max( 0.1, shift_end_s - plateau_end_s ) );
    }

    const double offset = lateral_shift * alpha;
    const auto frame = make_route_frame( route, s );
    const auto shifted_point_xy = shifted_point( frame, offset );

    point.x = shifted_point_xy.x;
    point.y = shifted_point_xy.y;

    if( params.max_speed_during_avoidance > 0.0 )
    {
      point.max_speed = std::min(
        point.max_speed.value_or( params.max_speed_during_avoidance ),
        params.max_speed_during_avoidance );
    }
  }

  return modified_route;
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
    result.reason = "no static obstacle in ego corridor";
    return result;
  }

  const auto vehicle_params = planner.get_physical_vehicle_parameters();

  auto plan_stop_before_obstacle = [&]( ObstacleAvoidanceMode mode,
                                        const std::string& reason ) -> ObstacleAvoidanceResult
  {
    ObstacleAvoidanceResult stop_result;
    stop_result.mode = mode;
    stop_result.reason = reason;
    stop_result.modified_route = build_stop_route_before_obstacle(
      route,
      obstacle.value(),
      ego,
      params.stop_distance_before_obstacle );

    stop_result.trajectory = planner.plan_route_trajectory(
      stop_result.modified_route,
      ego,
      traffic_participants );

    if( stop_result.trajectory.states.empty() )
    {
      stop_result.trajectory = make_stop_trajectory( ego, params.stop_time_step );
    }

    stop_result.trajectory.label = reason;
    stop_result.success = true;

    return stop_result;
  };

  const auto shift_candidate = choose_shift_candidate( obstacle.value(), vehicle_params, params );
  if( !shift_candidate.has_value() )
  {
    return plan_stop_before_obstacle(
      ObstacleAvoidanceMode::StopBeforeObstacle,
      "driving mission (stop before obstacle: no feasible lateral shift)" );
  }

  const double lateral_shift = shift_candidate->shift;
  const double abs_shift = std::fabs( lateral_shift );

  if( abs_shift <= params.in_lane_shift_limit )
  {
    result.mode = ObstacleAvoidanceMode::InLaneShift;
  }
  else
  {
    result.mode = lateral_shift >= 0.0
                    ? ObstacleAvoidanceMode::OvertakeLeft
                    : ObstacleAvoidanceMode::OvertakeRight;
  }

  // For right-hand traffic, only left-side crossing/overtaking can conflict with oncoming traffic here.
  if( result.mode == ObstacleAvoidanceMode::OvertakeLeft &&
      has_oncoming_traffic_conflict( route, ego, traffic_participants, obstacle.value(), params ) )
  {
    return plan_stop_before_obstacle(
      ObstacleAvoidanceMode::WaitForOncoming,
      "driving mission (waiting before obstacle: oncoming traffic conflict)" );
  }

  result.modified_route = build_modified_avoidance_route(
    route,
    obstacle.value(),
    lateral_shift,
    params );

  result.trajectory = planner.plan_route_trajectory(
    result.modified_route,
    ego,
    traffic_participants );

  if( result.trajectory.states.empty() )
  {
    return plan_stop_before_obstacle(
      ObstacleAvoidanceMode::StopBeforeObstacle,
      "driving mission (stop before obstacle: modified route planning failed)" );
  }

  if( result.mode == ObstacleAvoidanceMode::InLaneShift )
  {
    result.trajectory.label = "driving mission (in-lane obstacle avoidance)";
  }
  else if( result.mode == ObstacleAvoidanceMode::OvertakeLeft )
  {
    result.trajectory.label = "driving mission (obstacle avoidance left)";
  }
  else
  {
    result.trajectory.label = "driving mission (obstacle avoidance right)";
  }

  result.success = true;
  result.reason = "planned obstacle avoidance by modifying route points";

  return result;
}

} // namespace planner
} // namespace adore
