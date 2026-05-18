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
#include <exception>
#include <limits>
#include <optional>
#include <string>
#include <vector>
#include <cstddef>

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

  double object_s_min = std::numeric_limits<double>::infinity();
  double object_s_max = -std::numeric_limits<double>::infinity();
  double object_l_min = std::numeric_limits<double>::infinity();
  double object_l_max = -std::numeric_limits<double>::infinity();

  double s_min = std::numeric_limits<double>::infinity();
  double s_max = -std::numeric_limits<double>::infinity();
  double l_min = std::numeric_limits<double>::infinity();
  double l_max = -std::numeric_limits<double>::infinity();

  double center_s = std::numeric_limits<double>::infinity();
  double center_l = 0.0;

  std::size_t object_count = 1;

  bool overlaps_ego_corridor = false;
};

struct ShiftCandidate
{
  double shift = 0.0; // +left, -right
  bool valid = false;

  // True if the complete ego footprint stays inside the current route lane.
  // False means the shift uses adjacent driving-lane space and is therefore an
  // overtaking / lane-crossing maneuver.
  bool in_lane = false;
};

struct LateralInterval
{
  double min_l = std::numeric_limits<double>::infinity();
  double max_l = -std::numeric_limits<double>::infinity();
};

struct OppositeDirectionLaneQuery
{
  // true: map topology and the current lane/road could be evaluated reliably.
  // false: map/topology is incomplete; callers may use a conservative fallback.
  bool map_usable = false;

  // true only if at least one adjacent driving lane with opposite direction was
  // found on the candidate shift side. If map_usable=true and this is false,
  // the map explicitly says that the candidate does not enter oncoming traffic.
  bool has_opposite_lane = false;

  LateralInterval interval;
  std::string reason;
};

struct RouteDifferenceBounds
{
  bool has_difference = false;

  double first_different_s = 0.0;
  double last_different_s = 0.0;

  // First route point after the shifted section where modified_route and
  // original_route are equal again.
  double first_equal_s_after_last_difference = 0.0;
  bool has_equal_point_after_last_difference = false;
};

struct OppositeLaneConflictInterval
{
  bool valid = false;
  bool occupies_opposite_lane = false;

  double start_s = 0.0;
  double end_s = 0.0;

  std::string reason;
};

struct ParticipantFootprintOnRoute
{
  bool valid = false;

  double s_min = std::numeric_limits<double>::infinity();
  double s_max = -std::numeric_limits<double>::infinity();
  double l_min = std::numeric_limits<double>::infinity();
  double l_max = -std::numeric_limits<double>::infinity();

  double center_s = std::numeric_limits<double>::infinity();
  double center_l = 0.0;
};

struct EgoLaneOncomingThreat
{
  bool valid = false;

  int participant_id = -1;
  double participant_near_s = 0.0;
  double participant_center_s = 0.0;
  double distance_s = 0.0;
  double v_route = 0.0;
  double time_to_conflict = std::numeric_limits<double>::infinity();
  double stop_s = 0.0;

  std::string reason;
};

// Forward declarations for helpers used by the oncoming-gap check.
double
avoidance_shift_alpha_at_s( double s,
                            const ObstacleEnvelope& obstacle,
                            const ObstacleAvoidanceParams& params );

std::optional<LateralInterval>
get_allowed_lateral_interval_at_route_point(
  const map::Route& route,
  const map::MapPoint& route_point,
  double route_s,
  double lateral_shift,
  bool allow_adjacent_driving_lanes,
  const ObstacleAvoidanceParams& params );

// Query adjacent driving lanes that go in the opposite direction relative to
// the current route lane and lie on the indicated side. This distinguishes
// "map not usable" from "map usable, but no opposite-direction lane present".
OppositeDirectionLaneQuery
query_opposite_direction_lateral_interval_at_route_point(
  const map::Route& route,
  const map::MapPoint& route_point,
  double route_s,
  bool shift_left,
  const ObstacleAvoidanceParams& params );

// Earliest time (relative to `now_time`) at which a participant's predicted
// trajectory enters the longitudinal interval [conflict_start_s, conflict_end_s]
// along the given ego route. Returns nullopt if the trajectory is unusable
// (empty / too short / no state projects onto the route / no state enters the
// interval within its horizon).
std::optional<double>
compute_arrival_time_from_trajectory(
  const map::Route& route,
  const dynamics::Trajectory& trajectory,
  double now_time,
  double conflict_start_s,
  double conflict_end_s );

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

bool
map_points_differ_xy(
  const adore::map::MapPoint& a,
  const adore::map::MapPoint& b,
  const double xy_tolerance )
{
  return std::hypot( a.x - b.x, a.y - b.y ) > xy_tolerance;
}

std::optional<RouteDifferenceBounds>
find_route_difference_bounds(
  const adore::map::Route& original_route,
  const adore::map::Route& modified_route,
  const double xy_tolerance )
{
  RouteDifferenceBounds bounds;

  for( const auto& [s, original_point] : original_route.reference_line )
  {
    const auto modified_it = modified_route.reference_line.find( s );

    if( modified_it == modified_route.reference_line.end() )
    {
      // If the keys differ, this simple comparison is not valid.
      // For the current OA implementation this should usually not happen,
      // because modified_route is a copy of original_route.
      continue;
    }

    const bool different =
      map_points_differ_xy(
        original_point,
        modified_it->second,
        xy_tolerance );

    if( different )
    {
      if( !bounds.has_difference )
      {
        bounds.first_different_s = s;
        bounds.has_difference = true;
      }

      bounds.last_different_s = s;

      // Reset this, because we found a later changed point.
      bounds.has_equal_point_after_last_difference = false;
      bounds.first_equal_s_after_last_difference = 0.0;
    }
    else if( bounds.has_difference &&
             !bounds.has_equal_point_after_last_difference )
    {
      // This is the first equal point after the currently last changed point.
      bounds.first_equal_s_after_last_difference = s;
      bounds.has_equal_point_after_last_difference = true;
    }
  }

  if( !bounds.has_difference )
  {
    return std::nullopt;
  }

  return bounds;
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
build_stop_route_before_obstacle(
  const map::Route& route,
  const ObstacleEnvelope& obstacle,
  const dynamics::VehicleStateDynamic& ego,
  const dynamics::PhysicalVehicleParameters& vehicle_params,
  double stop_distance_before_obstacle )
{
  map::Route stop_route = route;

  const double ego_s = route.get_s( ego );

  if( !std::isfinite( ego_s ) )
  {
    std::fprintf(
      stderr,
      "[OA] stop route: invalid ego_s\n" );
    std::fflush( stderr );

    return stop_route;
  }

  /*
   * The route/trajectory reference point is assumed to be the vehicle reference
   * point used by the kinematic model, not the vehicle front.
   * Therefore, if the vehicle FRONT shall stop
   *   stop_distance_before_obstacle
   * before the obstacle, the vehicle reference point has to stop earlier by the
   * distance from reference point to vehicle front.
   */
  const double ego_front_offset =
    vehicle_params.wheelbase + vehicle_params.front_axle_to_front_border;

  const double stop_s =
    obstacle.object_s_min
    - stop_distance_before_obstacle
    - ego_front_offset;

  if( !std::isfinite( stop_s ) )
  {
    std::fprintf(
      stderr,
      "[OA] stop route: invalid stop_s, object_s_min=%.2f stop_distance=%.2f front_offset=%.2f\n",
      obstacle.object_s_min,
      stop_distance_before_obstacle,
      ego_front_offset );
    std::fflush( stderr );

    return stop_route;
  }

  /*
   * acceleration_min is defined in the vehicle parameter file, e.g. NGC.json.
   * It is expected to be negative, for example -1.5 m/s².
   */
  const double braking_deceleration =
    std::max( 0.1, std::fabs( vehicle_params.acceleration_min ) );

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
  if( available_distance <= 0.0 )
  {
    for( auto& [s, point] : stop_route.reference_line )
    {
      if( s >= ego_s )
      {
        point.max_speed = 0.0;
      }
    }

    return stop_route;
  }
  
  /*
   * Start braking as late as physically possible under the selected deceleration.
   * The speed limit along the route is then:
   *   v_allowed(s) = sqrt( 2 * a_brake * (stop_s - s) )
   * This makes v_allowed become zero exactly at stop_s.
   * If the required braking distance is greater than or equal to the available
   * distance, braking has to start immediately.
   * Otherwise, braking starts at the latest possible point that still allows the
   * vehicle to stop at stop_s under the selected deceleration.
  */
  const double brake_start_s =
    ( required_braking_distance >= available_distance )
      ? ego_s
      : stop_s - required_braking_distance;

  for( auto& [s, point] : stop_route.reference_line )
  {
    if( s >= brake_start_s )
    {
      const double dist_to_stop = stop_s - s;
      const double allowed_speed = std::sqrt( 2.0 * braking_deceleration * std::max( 0.0, dist_to_stop ) );
      point.max_speed = std::min( point.max_speed.value_or( std::numeric_limits<double>::infinity() ), allowed_speed );
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
  if( route.reference_line.size() < 2 )
  {
    return false;
  }

  envelope.object_s_min =  std::numeric_limits<double>::infinity();
  envelope.object_s_max = -std::numeric_limits<double>::infinity();
  envelope.object_l_min =  std::numeric_limits<double>::infinity();
  envelope.object_l_max = -std::numeric_limits<double>::infinity();

  envelope.s_min =  std::numeric_limits<double>::infinity();
  envelope.s_max = -std::numeric_limits<double>::infinity();
  envelope.l_min =  std::numeric_limits<double>::infinity();
  envelope.l_max = -std::numeric_limits<double>::infinity();

  envelope.object_count = 1;

  struct Projection
  {
    double s = std::numeric_limits<double>::quiet_NaN();
    double l = std::numeric_limits<double>::quiet_NaN();
    double distance = std::numeric_limits<double>::infinity();
  };

  auto project_xy_to_route =
    [&]( double x, double y ) -> std::optional<Projection>
    {
      Projection best;

      auto it_prev = route.reference_line.begin();
      auto it      = std::next( it_prev );

      for( ; it != route.reference_line.end(); ++it, ++it_prev )
      {
        const double s0 = it_prev->first;
        const double s1 = it->first;

        const auto& p0 = it_prev->second;
        const auto& p1 = it->second;

        const double dx = p1.x - p0.x;
        const double dy = p1.y - p0.y;

        const double len2 = dx * dx + dy * dy;
        if( len2 < 1e-9 )
        {
          continue;
        }

        const double t_raw =
          ( ( x - p0.x ) * dx + ( y - p0.y ) * dy ) / len2;

        const double t = std::max( 0.0, std::min( 1.0, t_raw ) );

        const double px = p0.x + t * dx;
        const double py = p0.y + t * dy;

        const double ex = x - px;
        const double ey = y - py;

        const double distance2 = ex * ex + ey * ey;

        if( distance2 < best.distance * best.distance )
        {
          const double len = std::sqrt( len2 );

          const double nx = -dy / len;
          const double ny =  dx / len;

          best.s = s0 + t * ( s1 - s0 );
          best.l = ex * nx + ey * ny;
          best.distance = std::sqrt( distance2 );
        }
      }

      if( !std::isfinite( best.s ) || !std::isfinite( best.l ) )
      {
        return std::nullopt;
      }

      return best;
    };

  const double center_x = participant.state.x;
  const double center_y = participant.state.y;
  const double yaw      = participant.state.yaw_angle;

  const double cos_yaw = std::cos( yaw );
  const double sin_yaw = std::sin( yaw );

  const double half_length =
    0.5 * std::max( 0.1, participant.physical_parameters.body_length );

  const double half_width =
    0.5 * std::max( 0.1, participant.physical_parameters.body_width );

  const std::array<std::pair<double, double>, 4> local_corners = {{
    { -half_length, -half_width },
    { -half_length,  half_width },
    {  half_length,  half_width },
    {  half_length, -half_width }
  }};

  bool any_valid_projection = false;
  double min_corner_distance = std::numeric_limits<double>::infinity();

  for( const auto& [local_x, local_y] : local_corners )
  {
    const double corner_x =
      center_x + local_x * cos_yaw - local_y * sin_yaw;

    const double corner_y =
      center_y + local_x * sin_yaw + local_y * cos_yaw;

    const auto projection = project_xy_to_route( corner_x, corner_y );

    if( !projection.has_value() )
    {
      continue;
    }

    min_corner_distance =
      std::min( min_corner_distance, projection->distance );

    any_valid_projection = true;

    envelope.object_s_min =
      std::min( envelope.object_s_min, projection->s );

    envelope.object_s_max =
      std::max( envelope.object_s_max, projection->s );

    envelope.object_l_min =
      std::min( envelope.object_l_min, projection->l );

    envelope.object_l_max =
      std::max( envelope.object_l_max, projection->l );
  }

  if( !any_valid_projection )
  {

    return false;
  }

  if( params.max_projection_distance_from_route > 0.0 && min_corner_distance > params.max_projection_distance_from_route )
  {
    return false;
  }

  if( !std::isfinite( envelope.object_s_min ) ||
      !std::isfinite( envelope.object_s_max ) ||
      !std::isfinite( envelope.object_l_min ) ||
      !std::isfinite( envelope.object_l_max ) )
  {
    return false;
  }

  if( envelope.object_s_max < ego_s - params.rear_clearance )
  {
    return false;
  }

  if( std::max( std::fabs( envelope.object_l_min ),
                std::fabs( envelope.object_l_max ) ) >
      params.max_object_lateral_distance )
  {
    return false;
  }

  const double center_s =
    0.5 * ( envelope.object_s_min + envelope.object_s_max );

  const double center_l =
    0.5 * ( envelope.object_l_min + envelope.object_l_max );

  const auto center_frame = make_route_frame( route, center_s );

  const double rel_yaw =
    normalize_angle( participant.state.yaw_angle - center_frame.yaw );

  const double corridor_half_width =
    ego_half_width + params.ego_corridor_safety_margin;

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
    "distance_to_corridor=%.2f overlap=%s min_corner_distance=%.2f\n",
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
    overlaps_ego_corridor ? "true" : "false",
    min_corner_distance );
  std::fflush( stderr );

  envelope.overlaps_ego_corridor = overlaps_ego_corridor;

  envelope.s_min = envelope.object_s_min - params.front_clearance;
  envelope.s_max = envelope.object_s_max + params.rear_clearance;

  envelope.l_min = envelope.object_l_min - params.side_clearance;
  envelope.l_max = envelope.object_l_max + params.side_clearance;

  envelope.center_s = 0.5 * ( envelope.s_min + envelope.s_max );
  envelope.center_l = 0.5 * ( envelope.l_min + envelope.l_max );

  return true;
}

void
refresh_obstacle_envelope_derived_values( ObstacleEnvelope& envelope,
                                         const ObstacleAvoidanceParams& params )
{
  envelope.s_min = envelope.object_s_min - params.front_clearance;
  envelope.s_max = envelope.object_s_max + params.rear_clearance;
  envelope.l_min = envelope.object_l_min - params.side_clearance;
  envelope.l_max = envelope.object_l_max + params.side_clearance;

  envelope.center_s = 0.5 * ( envelope.s_min + envelope.s_max );
  envelope.center_l = 0.5 * ( envelope.l_min + envelope.l_max );
}

void
merge_obstacle_into_cluster( ObstacleEnvelope& cluster,
                             const ObstacleEnvelope& next,
                             const ObstacleAvoidanceParams& params )
{
  cluster.object_s_min = std::min( cluster.object_s_min, next.object_s_min );
  cluster.object_s_max = std::max( cluster.object_s_max, next.object_s_max );
  cluster.object_l_min = std::min( cluster.object_l_min, next.object_l_min );
  cluster.object_l_max = std::max( cluster.object_l_max, next.object_l_max );
  cluster.object_count += next.object_count;

  cluster.overlaps_ego_corridor = cluster.overlaps_ego_corridor || next.overlaps_ego_corridor;

  refresh_obstacle_envelope_derived_values( cluster, params );
}

std::optional<ObstacleEnvelope>
find_static_obstacle_cluster_on_route(
  const map::Route& route,
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

  const double cluster_search_end_s =
    search_end_s +
    params.obstacle_cluster_join_gap_s +
    params.front_clearance +
    params.rear_clearance;

  std::vector<ObstacleEnvelope> obstacles;

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

    if( env.s_max < search_start_s || env.s_min > cluster_search_end_s )
    {
      continue;
    }

    obstacles.push_back( env );
  }

  if( obstacles.empty() )
  {
    return std::nullopt;
  }

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

  std::vector<ObstacleEnvelope> clusters;

  for( const auto& obstacle : obstacles )
  {
    if( clusters.empty() ||
        obstacle.object_s_min > clusters.back().object_s_max + params.obstacle_cluster_join_gap_s )
    {
      clusters.push_back( obstacle );
    }
    else
    {
      merge_obstacle_into_cluster( clusters.back(), obstacle, params );
    }
  }

  std::optional<ObstacleEnvelope> best_cluster;
  double best_distance_s = std::numeric_limits<double>::infinity();

  for( const auto& cluster : clusters )
  {
    if( !cluster.overlaps_ego_corridor )
    {
      continue;
    }

    if( cluster.s_max < search_start_s || cluster.s_min > search_end_s )
    {
      continue;
    }

    const double nearest_relevant_s = std::max( cluster.s_min, ego_s );
    const double distance_s = nearest_relevant_s - ego_s;

    if( !best_cluster.has_value() || distance_s < best_distance_s )
    {
      best_cluster = cluster;
      best_distance_s = distance_s;
    }
  }

  if( best_cluster.has_value() )
  {
    std::fprintf(
      stderr,
      "[OA][cluster] selected count=%zu first_id=%d raw_s=[%.2f, %.2f] raw_l=[%.2f, %.2f] inflated_s=[%.2f, %.2f] join_gap=%.2f\n",
      best_cluster->object_count,
      best_cluster->id,
      best_cluster->object_s_min,
      best_cluster->object_s_max,
      best_cluster->object_l_min,
      best_cluster->object_l_max,
      best_cluster->s_min,
      best_cluster->s_max,
      params.obstacle_cluster_join_gap_s );
    std::fflush( stderr );
  }

  return best_cluster;
}

std::optional<std::pair<double, map::MapPoint>>
find_reference_point_near_s( const map::Route& route, double query_s )
{
  if( route.reference_line.empty() || !std::isfinite( query_s ) )
  {
    return std::nullopt;
  }

  auto it_upper = route.reference_line.lower_bound( query_s );

  if( it_upper == route.reference_line.begin() )
  {
    return std::make_pair( it_upper->first, it_upper->second );
  }

  if( it_upper == route.reference_line.end() )
  {
    auto it_last = std::prev( route.reference_line.end() );
    return std::make_pair( it_last->first, it_last->second );
  }

  auto it_lower = std::prev( it_upper );

  if( std::fabs( it_upper->first - query_s ) <
      std::fabs( query_s - it_lower->first ) )
  {
    return std::make_pair( it_upper->first, it_upper->second );
  }

  return std::make_pair( it_lower->first, it_lower->second );
}

std::optional<ParticipantFootprintOnRoute>
project_participant_footprint_to_route(
  const map::Route& route,
  const dynamics::TrafficParticipant& participant )
{
  if( route.reference_line.size() < 2 )
  {
    return std::nullopt;
  }

  struct Projection
  {
    double s = std::numeric_limits<double>::quiet_NaN();
    double l = std::numeric_limits<double>::quiet_NaN();
    double distance = std::numeric_limits<double>::infinity();
  };

  auto project_xy_to_route =
    [&]( double x, double y ) -> std::optional<Projection>
    {
      Projection best;

      auto it_prev = route.reference_line.begin();
      auto it      = std::next( it_prev );

      for( ; it != route.reference_line.end(); ++it, ++it_prev )
      {
        const double s0 = it_prev->first;
        const double s1 = it->first;

        const auto& p0 = it_prev->second;
        const auto& p1 = it->second;

        const double dx = p1.x - p0.x;
        const double dy = p1.y - p0.y;

        const double len2 = dx * dx + dy * dy;
        if( len2 < 1e-9 )
        {
          continue;
        }

        const double t_raw =
          ( ( x - p0.x ) * dx + ( y - p0.y ) * dy ) / len2;

        const double t = std::clamp( t_raw, 0.0, 1.0 );

        const double px = p0.x + t * dx;
        const double py = p0.y + t * dy;

        const double ex = x - px;
        const double ey = y - py;

        const double distance2 = ex * ex + ey * ey;

        if( distance2 < best.distance * best.distance )
        {
          const double len = std::sqrt( len2 );
          const double nx = -dy / len;
          const double ny =  dx / len;

          best.s = s0 + t * ( s1 - s0 );
          best.l = ex * nx + ey * ny;
          best.distance = std::sqrt( distance2 );
        }
      }

      if( !std::isfinite( best.s ) || !std::isfinite( best.l ) )
      {
        return std::nullopt;
      }

      return best;
    };

  const double center_x = participant.state.x;
  const double center_y = participant.state.y;
  const double yaw      = participant.state.yaw_angle;

  const double cos_yaw = std::cos( yaw );
  const double sin_yaw = std::sin( yaw );

  const double half_length =
    0.5 * std::max( 0.1, participant.physical_parameters.body_length );

  const double half_width =
    0.5 * std::max( 0.1, participant.physical_parameters.body_width );

  const std::array<std::pair<double, double>, 4> local_corners = {{
    { -half_length, -half_width },
    { -half_length,  half_width },
    {  half_length,  half_width },
    {  half_length, -half_width }
  }};

  ParticipantFootprintOnRoute footprint;
  bool any_valid_projection = false;

  for( const auto& [local_x, local_y] : local_corners )
  {
    const double corner_x =
      center_x + local_x * cos_yaw - local_y * sin_yaw;

    const double corner_y =
      center_y + local_x * sin_yaw + local_y * cos_yaw;

    const auto projection = project_xy_to_route( corner_x, corner_y );
    if( !projection.has_value() )
    {
      continue;
    }

    any_valid_projection = true;

    footprint.s_min = std::min( footprint.s_min, projection->s );
    footprint.s_max = std::max( footprint.s_max, projection->s );
    footprint.l_min = std::min( footprint.l_min, projection->l );
    footprint.l_max = std::max( footprint.l_max, projection->l );
  }

  const auto center_projection = project_xy_to_route( center_x, center_y );
  if( center_projection.has_value() )
  {
    footprint.center_s = center_projection->s;
    footprint.center_l = center_projection->l;
  }
  else
  {
    footprint.center_s = 0.5 * ( footprint.s_min + footprint.s_max );
    footprint.center_l = 0.5 * ( footprint.l_min + footprint.l_max );
  }

  if( !any_valid_projection ||
      !std::isfinite( footprint.s_min ) ||
      !std::isfinite( footprint.s_max ) ||
      !std::isfinite( footprint.l_min ) ||
      !std::isfinite( footprint.l_max ) ||
      !std::isfinite( footprint.center_s ) )
  {
    return std::nullopt;
  }

  footprint.valid = true;
  return footprint;
}

bool
participant_footprint_overlaps_ego_lane(
  const map::Route& route,
  const ParticipantFootprintOnRoute& footprint,
  const dynamics::PhysicalVehicleParameters& ego_params,
  const ObstacleAvoidanceParams& params )
{
  const auto route_point = find_reference_point_near_s( route, footprint.center_s );

  if( route_point.has_value() )
  {
    const auto current_lane_interval =
      get_allowed_lateral_interval_at_route_point(
        route,
        route_point->second,
        route_point->first,
        0.0,
        false,
        params );

    if( current_lane_interval.has_value() )
    {
      return footprint.l_max >= current_lane_interval->min_l - params.ego_lane_oncoming_lateral_margin &&
             footprint.l_min <= current_lane_interval->max_l + params.ego_lane_oncoming_lateral_margin;
    }
  }

  // Fallback if map/lane boundaries are unavailable: use a narrow ego corridor
  // around the reference line. This still prevents normal oncoming traffic in a
  // clearly separated opposite lane from triggering unless it overlaps the ego
  // corridor.
  const double corridor_half_width =
    0.5 * std::max( 0.1, ego_params.body_width ) +
    params.ego_corridor_safety_margin +
    params.ego_lane_oncoming_lateral_margin;

  return footprint.l_min <= corridor_half_width &&
         footprint.l_max >= -corridor_half_width;
}

std::optional<EgoLaneOncomingThreat>
find_ego_lane_oncoming_threat(
  const map::Route& route,
  const dynamics::VehicleStateDynamic& ego,
  const dynamics::TrafficParticipantSet& traffic_participants,
  const dynamics::PhysicalVehicleParameters& ego_params,
  const ObstacleAvoidanceParams& params )
{
  if( !params.ego_lane_oncoming_stop_enabled )
  {
    return std::nullopt;
  }

  const double ego_s = route.get_s( ego );
  if( !std::isfinite( ego_s ) )
  {
    return std::nullopt;
  }

  const double ego_front_offset =
    ego_params.wheelbase + ego_params.front_axle_to_front_border;

  std::optional<EgoLaneOncomingThreat> best_threat;

  for( const auto& [id, participant] : traffic_participants.participants )
  {
    const double participant_speed = std::fabs( participant.state.vx );
    if( participant_speed <= params.max_static_object_speed )
    {
      continue;
    }

    const auto footprint = project_participant_footprint_to_route( route, participant );
    if( !footprint.has_value() )
    {
      continue;
    }

    if( footprint->s_max < ego_s - params.rear_clearance )
    {
      continue;
    }

    const double participant_near_s = footprint->s_min;
    const double distance_s = participant_near_s - ego_s - ego_front_offset;

    if( distance_s > params.ego_lane_oncoming_max_distance )
    {
      continue;
    }

    // Participant has already passed the ego reference point and moves further
    // toward decreasing s. It no longer creates a head-on threat ahead.
    if( footprint->s_max < ego_s )
    {
      continue;
    }

    const auto route_pose = route.get_pose_at_s( footprint->center_s );
    const double yaw_diff = normalize_angle( participant.state.yaw_angle - route_pose.yaw );
    const double v_route = participant_speed * std::cos( yaw_diff );

    if( v_route >= -params.ego_lane_oncoming_min_route_speed )
    {
      continue;
    }

    if( !participant_footprint_overlaps_ego_lane(
          route,
          footprint.value(),
          ego_params,
          params ) )
    {
      continue;
    }

    const double closing_speed =
      std::max( 0.0, ego.vx ) + std::fabs( v_route );

    const double time_to_conflict =
      std::max( 0.0, distance_s ) /
      std::max( 0.1, closing_speed );

    if( params.ego_lane_oncoming_time_horizon > 0.0 &&
        time_to_conflict > params.ego_lane_oncoming_time_horizon )
    {
      continue;
    }

    EgoLaneOncomingThreat threat;
    threat.valid = true;
    threat.participant_id = static_cast<int>( id );
    threat.participant_near_s = participant_near_s;
    threat.participant_center_s = footprint->center_s;
    threat.distance_s = std::max( 0.0, distance_s );
    threat.v_route = v_route;
    threat.time_to_conflict = time_to_conflict;
    threat.stop_s =
      participant_near_s
      - params.ego_lane_oncoming_stop_distance
      - ego_front_offset;

    char buf[256];
    std::snprintf(
      buf,
      sizeof( buf ),
      "ego-lane oncoming participant id=%d s=%.2f distance=%.2f v_route=%.2f ttc=%.2f stop_s=%.2f",
      threat.participant_id,
      threat.participant_center_s,
      threat.distance_s,
      threat.v_route,
      threat.time_to_conflict,
      threat.stop_s );
    threat.reason = buf;

    if( !best_threat.has_value() ||
        threat.time_to_conflict < best_threat->time_to_conflict )
    {
      best_threat = threat;
    }
  }

  return best_threat;
}

OppositeLaneConflictInterval
compute_opposite_lane_conflict_interval(
  const map::Route& route,
  const ObstacleEnvelope& obstacle,
  double lateral_shift,
  const dynamics::PhysicalVehicleParameters& ego_params,
  const ObstacleAvoidanceParams& params )
{
  OppositeLaneConflictInterval interval;

  if( route.reference_line.empty() )
  {
    interval.reason = "route reference line is empty";
    return interval;
  }

  if( !std::isfinite( lateral_shift ) || std::fabs( lateral_shift ) < 1e-6 )
  {
    interval.valid = true;
    interval.occupies_opposite_lane = false;
    interval.reason = "lateral shift is zero";
    return interval;
  }

  const double ego_half_width =
    0.5 * std::max( 0.1, ego_params.body_width );

  const bool shift_left = lateral_shift > 0.0;

  bool found_conflict_sample = false;
  bool found_active_sample = false;
  bool map_usable_at_least_once = false;
  bool opposite_lane_present_at_least_once = false;
  bool fallback_used_at_least_once = false;
  std::size_t invalid_lane_samples = 0;

  double first_conflict_s = std::numeric_limits<double>::infinity();
  double last_conflict_s = -std::numeric_limits<double>::infinity();

  for( const auto& [route_s, route_point] : route.reference_line )
  {
    const double alpha = avoidance_shift_alpha_at_s( route_s, obstacle, params );
    if( alpha <= 0.0 )
    {
      continue;
    }

    found_active_sample = true;

    const double center_l = lateral_shift * alpha;

    // Use the outer ego edge, not only the reference point. The drivable-area
    // margin makes the interval slightly conservative and avoids flickering at
    // the lane boundary.
    const double ego_left_l =
      center_l + ego_half_width + params.drivable_area_margin;

    const double ego_right_l =
      center_l - ego_half_width - params.drivable_area_margin;

    const auto opposite_query =
      query_opposite_direction_lateral_interval_at_route_point(
        route,
        route_point,
        route_s,
        shift_left,
        params );

    bool conflict_here = false;

    if( opposite_query.map_usable )
    {
      map_usable_at_least_once = true;

      if( opposite_query.has_opposite_lane )
      {
        opposite_lane_present_at_least_once = true;

        // Overlap of two 1-D lateral intervals. This is the preferred conflict
        // definition: ego footprint physically overlaps an opposite-direction
        // driving lane.
        const bool ego_overlaps_opposite =
          ego_left_l > opposite_query.interval.min_l &&
          ego_right_l < opposite_query.interval.max_l;

        conflict_here = ego_overlaps_opposite;
      }
      else
      {
        // Important distinction: the map was usable and explicitly found no
        // opposite-direction lane on the shift side. This covers multi-lane
        // same-direction roads. Do NOT fall back to "left lane departure ==
        // oncoming" in this case.
        conflict_here = false;
      }
    }
    else
    {
      // Only when topology is not usable do we use the conservative right-hand-
      // traffic fallback: leaving the current lane on the shift side may mean
      // entering an oncoming lane.
      fallback_used_at_least_once = true;

      const auto current_lane_interval =
        get_allowed_lateral_interval_at_route_point(
          route,
          route_point,
          route_s,
          lateral_shift,
          false,  // current route lane only
          params );

      if( !current_lane_interval.has_value() )
      {
        ++invalid_lane_samples;
        continue;
      }

      const bool crosses_left_border =
        shift_left && ego_left_l > current_lane_interval->max_l;

      const bool crosses_right_border =
        !shift_left && ego_right_l < current_lane_interval->min_l;

      conflict_here = crosses_left_border || crosses_right_border;
    }

    if( conflict_here )
    {
      found_conflict_sample = true;
      first_conflict_s = std::min( first_conflict_s, route_s );
      last_conflict_s = std::max( last_conflict_s, route_s );
    }
  }

  if( found_conflict_sample )
  {
    interval.valid = true;
    interval.occupies_opposite_lane = true;
    interval.start_s =
      std::max( 0.0, first_conflict_s - params.oncoming_safety_distance_rear );
    interval.end_s =
      last_conflict_s + params.oncoming_safety_distance_front;
    interval.reason =
      opposite_lane_present_at_least_once
        ? "ego footprint enters opposite-direction lane"
        : "ego footprint crosses current lane border (map unavailable fallback)";
    return interval;
  }

  if( found_active_sample )
  {
    interval.valid = true;
    interval.occupies_opposite_lane = false;

    if( map_usable_at_least_once && !opposite_lane_present_at_least_once )
    {
      interval.reason = "map usable: no opposite-direction lane on candidate shift side";
    }
    else if( opposite_lane_present_at_least_once )
    {
      interval.reason = "ego footprint stays clear of opposite-direction lane";
    }
    else if( fallback_used_at_least_once )
    {
      interval.reason = "fallback evaluated: ego footprint stays inside current lane";
    }
    else
    {
      interval.reason = "shifted samples evaluated without opposite-lane occupancy";
    }

    return interval;
  }

  interval.valid = false;
  interval.occupies_opposite_lane = false;
  interval.reason =
    invalid_lane_samples > 0
      ? "could not evaluate lane intervals on shifted samples"
      : "no active shifted samples available";
  return interval;
}

std::optional<double>
compute_arrival_time_from_trajectory(
  const map::Route& route,
  const dynamics::Trajectory& trajectory,
  double now_time,
  double conflict_start_s,
  double conflict_end_s )
{
  if( trajectory.states.size() < 2 )
  {
    return std::nullopt;
  }

  if( !std::isfinite( now_time ) )
  {
    return std::nullopt;
  }

  // Walk the trajectory in time order; remember the previous state's s and t so
  // we can linearly interpolate the crossing into the conflict interval.
  double s_prev = std::numeric_limits<double>::quiet_NaN();
  double t_prev = std::numeric_limits<double>::quiet_NaN();
  bool   has_prev = false;
  bool   any_state_in_future = false;

  for( const auto& state : trajectory.states )
  {
    const double t_now = state.time - now_time;
    if( t_now < 0.0 )
    {
      // Trajectory state already in the past; skip but treat it as a valid
      // anchor for interpolation if a future state is seen later.
      const double s_now_past = route.get_s( state );
      if( std::isfinite( s_now_past ) )
      {
        s_prev   = s_now_past;
        t_prev   = t_now;
        has_prev = true;
      }
      continue;
    }

    any_state_in_future = true;

    const double s_now = route.get_s( state );
    if( !std::isfinite( s_now ) )
    {
      has_prev = false;
      continue;
    }

    const bool inside_now =
      s_now >= conflict_start_s && s_now <= conflict_end_s;

    if( inside_now )
    {
      if( has_prev && std::isfinite( s_prev ) && std::isfinite( t_prev ) )
      {
        // Refine entry by linear interpolation between the previous (outside)
        // and current (inside) sample. Use the boundary that the previous
        // sample approached: if s_prev was past the end, interpolate at
        // conflict_end_s; otherwise interpolate at conflict_start_s.
        const double boundary = s_prev > conflict_end_s
                                  ? conflict_end_s
                                  : conflict_start_s;

        const double ds = s_now - s_prev;
        if( std::fabs( ds ) > 1e-9 )
        {
          const double alpha = std::clamp( ( boundary - s_prev ) / ds, 0.0, 1.0 );
          return std::max( 0.0, t_prev + alpha * ( t_now - t_prev ) );
        }
      }

      return std::max( 0.0, t_now );
    }

    s_prev   = s_now;
    t_prev   = t_now;
    has_prev = true;
  }

  if( !any_state_in_future )
  {
    return std::nullopt;
  }

  // Trajectory horizon did not cover the conflict interval.
  return std::nullopt;
}

std::optional<double>
compute_ego_clear_time_from_trajectory(
  const map::Route& route,
  const dynamics::Trajectory& trajectory,
  double now_time,
  double conflict_end_s,
  double initial_s_hint )
{
  if( trajectory.states.empty() || !std::isfinite( now_time ) ||
      !std::isfinite( conflict_end_s ) )
  {
    return std::nullopt;
  }

  double previous_s = initial_s_hint;

  for( const auto& state : trajectory.states )
  {
    const double t_now = state.time - now_time;
    if( t_now < 0.0 )
    {
      continue;
    }

    double state_s =
      adore::map::get_s_on_reference_line_segments(
        route,
        state,
        std::isfinite( previous_s ) ? previous_s : conflict_end_s,
        30.0 );

    if( !std::isfinite( state_s ) )
    {
      state_s = route.get_s( state );
    }

    if( !std::isfinite( state_s ) )
    {
      continue;
    }

    previous_s = state_s;

    if( state_s >= conflict_end_s )
    {
      return std::max( 0.0, t_now );
    }
  }

  return std::nullopt;
}

/**
 * Time-based oncoming traffic gap-acceptance check.
 *
 * Determines whether an obstacle-avoidance maneuver can be safely executed
 * with respect to oncoming traffic in an opposite-direction lane.
 *
 * Conflict-zone definition (from compute_opposite_lane_conflict_interval):
 * - Preferred: the longitudinal range of route s values at which the ego
 *   footprint overlaps the lateral interval of an adjacent opposite-direction
 *   driving lane. The first such s becomes conflict_start_s, the last becomes
 *   conflict_end_s, then both are expanded by oncoming_safety_distance_rear /
 *   _front to account for prediction uncertainty.
 * - If adjacent-lane direction cannot be determined from map data, a right-
 *   hand-traffic fallback flags the same interval whenever the ego footprint
 *   leaves the current lane on the shift side.
 * - If no shifted samples can be evaluated at all, a conservative obstacle-
 *   envelope fallback is used.
 *
 * Route-aligned velocity convention:
 * - v_route = speed * cos(participant_yaw - route_yaw)
 * - v_route > 0: participant moves in the ego route direction.
 * - v_route < 0: participant moves against the ego route direction / oncoming.
 *
 * Arrival-time estimation:
 * - If TrafficParticipant.trajectory is available, the participant's predicted
 *   trajectory is walked in time order and projected onto the ego route. The
 *   earliest state that lands in [conflict_start_s, conflict_end_s] defines the
 *   arrival time (linearly interpolated between consecutive samples).
 * - Otherwise, a constant-velocity prediction is used with the route-aligned
 *   speed |v_route| floored by min_oncoming_speed_for_gap_check.
 *
 * Decision rule:
 *   arrival_time <= ego_clear_time + oncoming_time_margin -> reject maneuver
 *   arrival_time >  ego_clear_time + oncoming_time_margin -> accept maneuver
 */
planner::OncomingConflictResult
check_oncoming_gap( const map::Route& route,
                    const dynamics::VehicleStateDynamic& ego,
                    const dynamics::TrafficParticipantSet& traffic_participants,
                    const ObstacleEnvelope& obstacle,
                    double lateral_shift,
                    const dynamics::PhysicalVehicleParameters& ego_params,
                    const dynamics::Trajectory* candidate_ego_trajectory,
                    const ObstacleAvoidanceParams& params )
{
  planner::OncomingConflictResult result;
  result.conflict = false;
  result.participant_id = -1;
  result.oncoming_arrival_time = std::numeric_limits<double>::infinity();

  const double ego_s = route.get_s( ego );
  if( !std::isfinite( ego_s ) )
  {
    result.reason = "ego position invalid on route";
    return result;
  }

  const auto conflict_interval =
    compute_opposite_lane_conflict_interval(
      route,
      obstacle,
      lateral_shift,
      ego_params,
      params );

  if( conflict_interval.valid && !conflict_interval.occupies_opposite_lane )
  {
    result.reason = conflict_interval.reason;

    if( params.debug_oncoming_check )
    {
      std::fprintf(
        stderr,
        "[OA][oncoming] ACCEPT: no opposite-lane occupancy (%s)\n",
        result.reason.c_str() );
      std::fflush( stderr );
    }

    return result;
  }

  if( conflict_interval.valid )
  {
    result.conflict_start_s = conflict_interval.start_s;
    result.conflict_end_s = conflict_interval.end_s;
  }
  else
  {
    // Conservative fallback if lane boundaries cannot be evaluated. This is not
    // the preferred conflict-zone definition, but it avoids accepting an
    // opposite-lane maneuver blindly when map data is incomplete.
    result.conflict_start_s =
      std::max(
        0.0,
        obstacle.object_s_min
        - params.front_clearance
        - params.oncoming_safety_distance_rear );

    result.conflict_end_s =
      obstacle.object_s_max
      + params.rear_clearance
      + params.oncoming_safety_distance_front;

    if( params.debug_oncoming_check )
    {
      std::fprintf(
        stderr,
        "[OA][oncoming] warning: using obstacle-envelope fallback conflict interval: %s\n",
        conflict_interval.reason.c_str() );
      std::fflush( stderr );
    }
  }

  if( candidate_ego_trajectory != nullptr )
  {
    const auto trajectory_clear_time =
      compute_ego_clear_time_from_trajectory(
        route,
        *candidate_ego_trajectory,
        ego.time,
        result.conflict_end_s,
        ego_s );

    if( trajectory_clear_time.has_value() )
    {
      result.ego_clear_time = trajectory_clear_time.value();
    }
  }

  if( result.ego_clear_time <= 0.0 )
  {
    const double conflict_distance =
      std::max( 0.0, result.conflict_end_s - ego_s );

    double ego_speed = std::max( 0.0, ego.vx );
    if( params.max_speed_during_avoidance > 0.0 )
    {
      ego_speed = std::min( ego_speed, params.max_speed_during_avoidance );
    }

    ego_speed =
      std::max( params.min_ego_speed_for_gap_check, ego_speed );

    result.ego_clear_time = conflict_distance / ego_speed;
  }

  if( params.debug_oncoming_check )
  {
    std::fprintf(
      stderr,
      "[OA][oncoming] ego_s=%.2f conflict=[%.2f, %.2f] ego_clear_time=%.2f source=%s clear_source=%s\n",
      ego_s,
      result.conflict_start_s,
      result.conflict_end_s,
      result.ego_clear_time,
      conflict_interval.valid ? conflict_interval.reason.c_str() : "fallback",
      candidate_ego_trajectory != nullptr ? "trajectory_or_fallback" : "kinematic_fallback" );
    std::fflush( stderr );
  }

  for( const auto& [id, participant] : traffic_participants.participants )
  {
    const double participant_speed = std::fabs( participant.state.vx );
    if( participant_speed <= params.max_static_object_speed )
    {
      continue;
    }

    const double participant_s = route.get_s( participant.state );
    if( !std::isfinite( participant_s ) )
    {
      continue;
    }

    const auto route_pose = route.get_pose_at_s( participant_s );
    const double route_yaw = route_pose.yaw;

    const double yaw_diff =
      normalize_angle( participant.state.yaw_angle - route_yaw );

    const double v_route = participant_speed * std::cos( yaw_diff );

    if( v_route >= -params.min_oncoming_route_speed )
    {
      if( params.debug_oncoming_check &&
          participant_speed > params.max_static_object_speed )
      {
        std::fprintf(
          stderr,
          "[OA][oncoming] participant id=%zu s=%.2f v_route=%.2f (not oncoming)\n",
          id,
          participant_s,
          v_route );
        std::fflush( stderr );
      }
      continue;
    }

    const double oncoming_speed =
      std::max(
        params.min_oncoming_speed_for_gap_check,
        std::fabs( v_route ) );

    double arrival_time = std::numeric_limits<double>::infinity();
    double distance_to_conflict = std::numeric_limits<double>::infinity();
    const char* arrival_source = "constant_velocity";

    bool used_trajectory = false;

    if( participant.trajectory.has_value() &&
        participant.trajectory->states.size() >= 2 )
    {
      const auto trajectory_arrival =
        compute_arrival_time_from_trajectory(
          route,
          participant.trajectory.value(),
          ego.time,
          result.conflict_start_s,
          result.conflict_end_s );

      if( trajectory_arrival.has_value() )
      {
        arrival_time = trajectory_arrival.value();
        distance_to_conflict = arrival_time * oncoming_speed;
        arrival_source = "trajectory";
        used_trajectory = true;
      }
      else if( params.debug_oncoming_check )
      {
        std::fprintf(
          stderr,
          "[OA][oncoming] participant id=%zu trajectory has %zu states but does not enter conflict interval; "
          "falling back to constant-velocity prediction\n",
          id,
          participant.trajectory->states.size() );
        std::fflush( stderr );
      }
    }

    if( !used_trajectory )
    {
      if( participant_s >= result.conflict_start_s &&
          participant_s <= result.conflict_end_s )
      {
        // The oncoming vehicle is already in the longitudinal interval where ego
        // would occupy the opposite lane.
        arrival_time = 0.0;
        distance_to_conflict = 0.0;
      }
      else if( participant_s > result.conflict_end_s )
      {
        // Oncoming traffic is ahead on the route and moves toward decreasing s.
        // Therefore the distance to the conflict interval is participant_s - end.
        distance_to_conflict = participant_s - result.conflict_end_s;
        arrival_time = distance_to_conflict / oncoming_speed;
      }
      else
      {
        // participant_s < conflict_start_s and v_route < 0: the participant has
        // already passed the conflict interval and continues moving away from it.
        if( params.debug_oncoming_check )
        {
          std::fprintf(
            stderr,
            "[OA][oncoming] participant id=%zu s=%.2f already passed conflict interval [%.2f, %.2f]\n",
            id,
            participant_s,
            result.conflict_start_s,
            result.conflict_end_s );
          std::fflush( stderr );
        }
        continue;
      }
    }

    if( params.prediction_time_horizon > 0.0 &&
        arrival_time > params.prediction_time_horizon &&
        result.ego_clear_time + params.oncoming_time_margin <= params.prediction_time_horizon )
    {
      if( params.debug_oncoming_check )
      {
        std::fprintf(
          stderr,
          "[OA][oncoming] participant id=%zu arrival_time=%.2f beyond prediction_horizon=%.2f\n",
          id,
          arrival_time,
          params.prediction_time_horizon );
        std::fflush( stderr );
      }
      continue;
    }

    if( params.debug_oncoming_check )
    {
      std::fprintf(
        stderr,
        "[OA][oncoming] participant id=%zu s=%.2f yaw_diff=%.2f v_route=%.2f (oncoming) "
        "source=%s distance_to_conflict=%.2f arrival_time=%.2f ego_clear_time=%.2f margin=%.2f\n",
        id,
        participant_s,
        yaw_diff,
        v_route,
        arrival_source,
        distance_to_conflict,
        arrival_time,
        result.ego_clear_time,
        params.oncoming_time_margin );
      std::fflush( stderr );
    }

    const double required_arrival_time =
      result.ego_clear_time + params.oncoming_time_margin;

    if( arrival_time <= required_arrival_time )
    {
      result.conflict = true;
      result.participant_id = static_cast<int>( id );
      result.oncoming_arrival_time = arrival_time;

      if( params.debug_oncoming_check )
      {
        result.diagnostics.participant_id = static_cast<int>( id );
        result.diagnostics.participant_s = participant_s;
        result.diagnostics.participant_vx = participant.state.vx;
        result.diagnostics.participant_yaw = participant.state.yaw_angle;
        result.diagnostics.route_yaw_at_participant_s = route_yaw;
        result.diagnostics.yaw_diff = yaw_diff;
        result.diagnostics.v_route = v_route;
        result.diagnostics.heading_diff_rad = std::fabs( yaw_diff );
      }

      char buf[384];
      std::snprintf(
        buf,
        sizeof( buf ),
        "participant id=%d arrival_time=%.2f (source=%s) <= ego_clear_time=%.2f + margin=%.2f, conflict=[%.2f, %.2f]",
        static_cast<int>( id ),
        arrival_time,
        arrival_source,
        result.ego_clear_time,
        params.oncoming_time_margin,
        result.conflict_start_s,
        result.conflict_end_s );
      result.reason = buf;

      if( params.debug_oncoming_check )
      {
        std::fprintf(
          stderr,
          "[OA][oncoming] REJECT: %s\n",
          result.reason.c_str() );
        std::fflush( stderr );
      }

      return result;
    }
  }

  result.conflict = false;
  result.reason = "no oncoming conflict detected";

  if( params.debug_oncoming_check )
  {
    std::fprintf(
      stderr,
      "[OA][oncoming] ACCEPT: %s\n",
      result.reason.c_str() );
    std::fflush( stderr );
  }

  return result;
}

double
avoidance_shift_alpha_at_s( double s,
                            const ObstacleEnvelope& obstacle,
                            const ObstacleAvoidanceParams& params )
{
  // Full lateral shift is reached before the raw obstacle begins and held until
  // the raw obstacle plus rear clearance is passed. Entry/return are x^5 blends.
  const double plateau_start_s = std::max( 0.0, obstacle.object_s_min - params.front_clearance );
  const double plateau_end_s   = obstacle.object_s_max + params.rear_clearance;

  const double shift_start_s = std::max( 0.0, plateau_start_s - params.entry_extra_distance );
  const double shift_end_s   = plateau_end_s + params.return_extra_distance;

  if( s < shift_start_s || s > shift_end_s )
  {
    return 0.0;
  }

  if( s < plateau_start_s )
  {
    return smoothstep01(
      ( s - shift_start_s ) /
      std::max( 0.1, plateau_start_s - shift_start_s ) );
  }

  if( s <= plateau_end_s )
  {
    return 1.0;
  }

  return 1.0 - smoothstep01(
    ( s - plateau_end_s ) /
    std::max( 0.1, shift_end_s - plateau_end_s ) );
}

bool
lane_has_usable_borders( const map::Lane& lane )
{
  const bool inner_ok =
    !lane.borders.inner.points.empty() ||
    !lane.borders.inner.interpolated_points.empty();

  const bool outer_ok =
    !lane.borders.outer.points.empty() ||
    !lane.borders.outer.interpolated_points.empty();

  return inner_ok && outer_ok;
}

bool
border_contains_s( const map::Border& border, double s, double slack )
{
  if( !border.interpolated_points.empty() )
  {
    const double min_s = std::min( border.interpolated_points.front().s,
                                   border.interpolated_points.back().s );
    const double max_s = std::max( border.interpolated_points.front().s,
                                   border.interpolated_points.back().s );
    return s >= min_s - slack && s <= max_s + slack;
  }

  if( !border.points.empty() )
  {
    const double min_s = std::min( border.points.front().s, border.points.back().s );
    const double max_s = std::max( border.points.front().s, border.points.back().s );
    return s >= min_s - slack && s <= max_s + slack;
  }

  if( border.length > 0.0 )
  {
    return s >= -slack && s <= border.length + slack;
  }

  return false;
}

bool
lane_contains_s( const map::Lane& lane, double s, double slack )
{
  return border_contains_s( lane.borders.inner, s, slack ) &&
         border_contains_s( lane.borders.outer, s, slack );
}

bool
add_lane_lateral_interval( const RouteFrame& frame,
                           const map::Lane& lane,
                           double lane_s,
                           double lane_s_overlap_slack,
                           LateralInterval& interval )
{
  if( !lane_has_usable_borders( lane ) )
  {
    return false;
  }

  if( !lane_contains_s( lane, lane_s, lane_s_overlap_slack ) )
  {
    return false;
  }

  try
  {
    const auto inner = lane.borders.inner.get_interpolated_point( lane_s );
    const auto outer = lane.borders.outer.get_interpolated_point( lane_s );

    const math::Point2d inner_xy{ inner.x, inner.y };
    const math::Point2d outer_xy{ outer.x, outer.y };

    const double inner_l = signed_lateral_offset( frame, inner_xy );
    const double outer_l = signed_lateral_offset( frame, outer_xy );

    const double lane_min_l = std::min( inner_l, outer_l );
    const double lane_max_l = std::max( inner_l, outer_l );

    if( !std::isfinite( lane_min_l ) || !std::isfinite( lane_max_l ) ||
        lane_max_l <= lane_min_l )
    {
      return false;
    }

    interval.min_l = std::min( interval.min_l, lane_min_l );
    interval.max_l = std::max( interval.max_l, lane_max_l );
  }
  catch( const std::exception& )
  {
    return false;
  }

  return true;
}

std::optional<LateralInterval>
get_allowed_lateral_interval_at_route_point(
  const map::Route& route,
  const map::MapPoint& route_point,
  double route_s,
  double lateral_shift,
  bool allow_adjacent_driving_lanes,
  const ObstacleAvoidanceParams& params )
{
  if( !route.map )
  {
    return std::nullopt;
  }

  const auto current_lane_it = route.map->lanes.find( route_point.parent_id );
  if( current_lane_it == route.map->lanes.end() || !current_lane_it->second )
  {
    return std::nullopt;
  }

  const auto& current_lane = *current_lane_it->second;
  const auto frame = make_route_frame( route, route_s );

  std::vector<LateralInterval> intervals;
  LateralInterval current_interval;
  if( !add_lane_lateral_interval(
        frame,
        current_lane,
        route_point.s,
        params.lane_s_overlap_slack,
        current_interval ) )
  {
    return std::nullopt;
  }

  intervals.push_back( current_interval );

  if( allow_adjacent_driving_lanes )
  {
    const auto road_it = route.map->roads.find( current_lane.road_id );
    if( road_it != route.map->roads.end() )
    {
      const bool shift_left = lateral_shift >= 0.0;

      for( const auto& lane_ptr : road_it->second.lanes )
      {
        if( !lane_ptr || lane_ptr->id == current_lane.id )
        {
          continue;
        }

        // The obstacle-avoidance drivable area is limited to actual driving
        // lanes. Parking, shoulders, sidewalks, bus lanes, bike lanes, etc. are
        // not accepted as avoidance area here.
        if( lane_ptr->type != map::driving )
        {
          continue;
        }

        if( !params.allow_opposite_direction_lanes &&
            lane_ptr->left_of_reference != current_lane.left_of_reference )
        {
          continue;
        }

        LateralInterval lane_interval;
        if( !add_lane_lateral_interval(
              frame,
              *lane_ptr,
              route_point.s,
              params.lane_s_overlap_slack,
              lane_interval ) )
        {
          continue;
        }

        const double lane_center_l = 0.5 * ( lane_interval.min_l + lane_interval.max_l );

        // Only include lanes on the side on which the candidate shifts. This
        // prevents a left-shift candidate from gaining artificial drivable area
        // on the right, and vice versa.
        if( shift_left && lane_center_l <= 0.0 )
        {
          continue;
        }

        if( !shift_left && lane_center_l >= 0.0 )
        {
          continue;
        }

        intervals.push_back( lane_interval );
      }
    }
  }

  std::sort(
    intervals.begin(),
    intervals.end(),
    []( const LateralInterval& a, const LateralInterval& b )
    {
      return a.min_l < b.min_l;
    } );

  std::vector<LateralInterval> merged;
  for( const auto& interval : intervals )
  {
    if( merged.empty() ||
        interval.min_l > merged.back().max_l + params.lane_boundary_join_slack )
    {
      merged.push_back( interval );
    }
    else
    {
      merged.back().max_l = std::max( merged.back().max_l, interval.max_l );
    }
  }

  // Use only the connected drivable interval containing the route centerline.
  // This rejects candidates that would jump over a median, shoulder, sidewalk,
  // or any non-driving gap.
  for( const auto& interval : merged )
  {
    if( interval.min_l <= 0.0 && interval.max_l >= 0.0 )
    {
      return interval;
    }
  }

  return current_interval;
}

OppositeDirectionLaneQuery
query_opposite_direction_lateral_interval_at_route_point(
  const map::Route& route,
  const map::MapPoint& route_point,
  double route_s,
  bool shift_left,
  const ObstacleAvoidanceParams& params )
{
  OppositeDirectionLaneQuery query;

  if( !route.map )
  {
    query.reason = "route has no map";
    return query;
  }

  const auto current_lane_it = route.map->lanes.find( route_point.parent_id );
  if( current_lane_it == route.map->lanes.end() || !current_lane_it->second )
  {
    query.reason = "current route lane not found in map";
    return query;
  }

  const auto& current_lane = *current_lane_it->second;

  const auto road_it = route.map->roads.find( current_lane.road_id );
  if( road_it == route.map->roads.end() )
  {
    query.reason = "current lane road not found in map";
    return query;
  }

  query.map_usable = true;
  query.reason = "map usable; no opposite-direction driving lane on candidate shift side";

  const auto frame = make_route_frame( route, route_s );

  LateralInterval merged_opposite;
  bool found = false;
  bool found_opposite_wrong_side = false;
  bool found_opposite_side_without_usable_borders = false;

  for( const auto& lane_ptr : road_it->second.lanes )
  {
    if( !lane_ptr || lane_ptr->id == current_lane.id )
    {
      continue;
    }

    if( lane_ptr->type != map::driving )
    {
      continue;
    }

    // Same direction lanes are deliberately ignored here. This is essential for
    // multi-lane roads: leaving the current lane into a same-direction lane must
    // not trigger a head-on traffic check.
    if( lane_ptr->left_of_reference == current_lane.left_of_reference )
    {
      continue;
    }

    LateralInterval lane_interval;
    if( !add_lane_lateral_interval(
          frame,
          *lane_ptr,
          route_point.s,
          params.lane_s_overlap_slack,
          lane_interval ) )
    {
      found_opposite_side_without_usable_borders = true;
      continue;
    }

    const double lane_center_l =
      0.5 * ( lane_interval.min_l + lane_interval.max_l );

    // Only count lanes on the side toward which the candidate shifts.
    if( shift_left && lane_center_l <= 0.0 )
    {
      found_opposite_wrong_side = true;
      continue;
    }

    if( !shift_left && lane_center_l >= 0.0 )
    {
      found_opposite_wrong_side = true;
      continue;
    }

    if( !found )
    {
      merged_opposite = lane_interval;
      found = true;
    }
    else
    {
      merged_opposite.min_l = std::min( merged_opposite.min_l, lane_interval.min_l );
      merged_opposite.max_l = std::max( merged_opposite.max_l, lane_interval.max_l );
    }
  }

  if( found )
  {
    query.has_opposite_lane = true;
    query.interval = merged_opposite;
    query.reason = "opposite-direction driving lane found on candidate shift side";
    return query;
  }

  if( found_opposite_side_without_usable_borders )
  {
    query.reason = "opposite-direction lane exists but its borders are not usable at this s";
  }
  else if( found_opposite_wrong_side )
  {
    query.reason = "opposite-direction driving lane exists only on the other side";
  }

  return query;
}

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
  const ObstacleEnvelope& obstacle,
  double lateral_shift,
  bool in_lane,
  const dynamics::PhysicalVehicleParameters& ego_params,
  const ObstacleAvoidanceParams& params )
{
  if( in_lane )
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
      obstacle,
      lateral_shift,
      ego_params,
      params );

  if( conflict_interval.valid )
  {
    return conflict_interval.occupies_opposite_lane;
  }

  // Map data not usable -> use the documented RHT assumption.
  return lateral_shift > 0.0;
}

bool
candidate_respects_drivable_area( const map::Route& route,
                                  const ObstacleEnvelope& obstacle,
                                  double lateral_shift,
                                  const dynamics::PhysicalVehicleParameters& ego_params,
                                  const ObstacleAvoidanceParams& params,
                                  bool allow_adjacent_driving_lanes )
{
  if( route.reference_line.empty() )
  {
    return false;
  }

  const double ego_half_width = 0.5 * std::max( 0.1, ego_params.body_width );

  for( const auto& [route_s, route_point] : route.reference_line )
  {
    const double alpha = avoidance_shift_alpha_at_s( route_s, obstacle, params );
    if( alpha <= 0.0 )
    {
      continue;
    }

    const double center_l = lateral_shift * alpha;
    const double ego_min_l = center_l - ego_half_width - params.drivable_area_margin;
    const double ego_max_l = center_l + ego_half_width + params.drivable_area_margin;

    const auto allowed_interval = get_allowed_lateral_interval_at_route_point(
      route,
      route_point,
      route_s,
      lateral_shift,
      allow_adjacent_driving_lanes,
      params );

    if( !allowed_interval.has_value() )
    {
      std::fprintf(
        stderr,
        "[OA] reject shift=%.2f at route_s=%.2f: no usable lane boundaries for lane_id=%zu\n",
        lateral_shift,
        route_s,
        route_point.parent_id );
      std::fflush( stderr );
      return false;
    }

    if( ego_min_l < allowed_interval->min_l || ego_max_l > allowed_interval->max_l )
    {
      std::fprintf(
        stderr,
        "[OA] reject shift=%.2f at route_s=%.2f: ego_l=[%.2f, %.2f] outside allowed_l=[%.2f, %.2f], adjacent_allowed=%s\n",
        lateral_shift,
        route_s,
        ego_min_l,
        ego_max_l,
        allowed_interval->min_l,
        allowed_interval->max_l,
        allow_adjacent_driving_lanes ? "true" : "false" );
      std::fflush( stderr );
      return false;
    }
  }

  return true;
}

ShiftCandidate
make_left_candidate( const ObstacleEnvelope& obstacle,
                     const dynamics::PhysicalVehicleParameters& ego_params,
                     const ObstacleAvoidanceParams& params )
{
  const double ego_half_width = 0.5 * std::max( 0.1, ego_params.body_width );

  // Minimum shift that places the ego right edge to the left of the obstacle
  // envelope plus side clearance. No artificial min/max shift is applied here;
  // validity is decided by the lane-boundary/drivable-area check below.
  const double shift = obstacle.object_l_max + ego_half_width + params.side_clearance;

  return ShiftCandidate{ shift, std::isfinite( shift ) && shift > 0.0 };
}

ShiftCandidate
make_right_candidate( const ObstacleEnvelope& obstacle,
                      const dynamics::PhysicalVehicleParameters& ego_params,
                      const ObstacleAvoidanceParams& params )
{
  const double ego_half_width = 0.5 * std::max( 0.1, ego_params.body_width );

  // Minimum shift that places the ego left edge to the right of the obstacle
  // envelope minus side clearance. No artificial min/max shift is applied here;
  // validity is decided by the lane-boundary/drivable-area check below.
  const double shift = obstacle.object_l_min - ego_half_width - params.side_clearance;

  return ShiftCandidate{ shift, std::isfinite( shift ) && shift < 0.0 };
}

std::optional<ShiftCandidate>
choose_shift_candidate( const map::Route& route,
                        const ObstacleEnvelope& obstacle,
                        const dynamics::PhysicalVehicleParameters& ego_params,
                        const ObstacleAvoidanceParams& params )
{
  auto left = make_left_candidate( obstacle, ego_params, params );
  auto right = make_right_candidate( obstacle, ego_params, params );

  auto evaluate_candidate = [&]( ShiftCandidate& candidate )
  {
    if( !candidate.valid )
    {
      return;
    }

    if( !params.enforce_drivable_area )
    {
      // Without lane boundaries there is no reliable way to distinguish an
      // in-lane shift from a lane-crossing maneuver. Keep the candidate only if
      // overtaking/lane-crossing is explicitly allowed.
      candidate.in_lane = false;

      if( !params.overtaking_allowed )
      {
        std::fprintf(
          stderr,
          "[OA] reject shift=%.2f: drivable-area enforcement is disabled and overtaking_allowed=false\n",
          candidate.shift );
        std::fflush( stderr );
        candidate.valid = false;
      }

      return;
    }

    const bool fits_current_lane = candidate_respects_drivable_area(
      route,
      obstacle,
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

    if( !params.overtaking_allowed )
    {
      std::fprintf(
        stderr,
        "[OA] reject shift=%.2f: would leave current lane, but overtaking_allowed=false\n",
        candidate.shift );
      std::fflush( stderr );
      candidate.valid = false;
      return;
    }

    if( !candidate_respects_drivable_area(
          route,
          obstacle,
          candidate.shift,
          ego_params,
          params,
          true ) )
    {
      std::fprintf(
        stderr,
        "[OA] reject shift=%.2f: outside connected driving-lane drivable area\n",
        candidate.shift );
      std::fflush( stderr );
      candidate.valid = false;
    }
  };

  evaluate_candidate( left );
  evaluate_candidate( right );

  if( left.valid && !right.valid ) return left;
  if( right.valid && !left.valid ) return right;
  if( !left.valid && !right.valid ) return std::nullopt;

  // Prefer an in-lane solution over an overtaking / lane-crossing solution even
  // if the absolute shift is slightly larger. It is less intrusive and avoids
  // unnecessary oncoming-lane checks.
  if( left.in_lane && !right.in_lane ) return left;
  if( right.in_lane && !left.in_lane ) return right;

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

  for( auto& [s, point] : modified_route.reference_line )
  {
    const double alpha = avoidance_shift_alpha_at_s( s, obstacle, params );

    if( alpha <= 0.0 )
    {
      continue;
    }

    const double offset = lateral_shift * alpha;

    // Important: use the original route as reference frame.
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

  // Find nearest relevant static obstacle on route. This is the only place where the raw obstacle geometry is used.
  const auto obstacle = find_static_obstacle_cluster_on_route(
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

  // The obstacle is relevant and represented by the ObstacleEnvelope. 
  // Now check if a lateral shift is possible, if it would conflict with oncoming traffic, and plan the modified route and trajectory accordingly. 
  // If any of these steps fail, fall back to a stop trajectory before the obstacle.
  auto plan_stop_before_obstacle = [&]( ObstacleAvoidanceMode mode,
                                        const std::string& reason ) -> ObstacleAvoidanceResult
  {
    ObstacleAvoidanceResult stop_result;
    stop_result.mode = mode;
    stop_result.reason = reason;


  stop_result.modified_route =
    build_stop_route_before_obstacle(
      route,
      obstacle.value(),
      ego,
      vehicle_params,
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

  const auto shift_candidate = choose_shift_candidate(
    route,
    obstacle.value(),
    vehicle_params,
    params );
  if( !shift_candidate.has_value() )
  {
    return plan_stop_before_obstacle(
      ObstacleAvoidanceMode::StopBeforeObstacle,
      "driving mission (stop before obstacle: no feasible lateral shift)" );
  }

  const double lateral_shift = shift_candidate->shift;

  if( shift_candidate->in_lane )
  {
    result.mode = ObstacleAvoidanceMode::InLaneShift;
  }
  else
  {
    result.mode = lateral_shift >= 0.0
                    ? ObstacleAvoidanceMode::OvertakeLeft
                    : ObstacleAvoidanceMode::OvertakeRight;
  }

  std::fprintf(
    stderr,
    "[OA] selected shift=%.2f, in_lane=%s, overtaking_allowed=%s, enforce_drivable_area=%s\n",
    lateral_shift,
    shift_candidate->in_lane ? "true" : "false",
    params.overtaking_allowed ? "true" : "false",
    params.enforce_drivable_area ? "true" : "false" );
  std::fflush( stderr );

  const bool uses_opposite_lane =
    candidate_uses_opposite_direction_lane(
      route,
      obstacle.value(),
      lateral_shift,
      shift_candidate->in_lane,
      vehicle_params,
      params );

  const auto opposite_conflict_interval =
    compute_opposite_lane_conflict_interval(
      route,
      obstacle.value(),
      lateral_shift,
      vehicle_params,
      params );

  result.modified_route = build_modified_avoidance_route(
    route,
    obstacle.value(),
    lateral_shift,
    params );

  const double ego_s_original = route.get_s( ego );

  double ego_s_modified =
      adore::map::get_s_on_reference_line_segments(
          result.modified_route,
          ego,
          ego_s_original,
          20.0 );

  if( !std::isfinite( ego_s_modified ) )
  {
      ego_s_modified = ego_s_original;
  }

  if( !std::isfinite( ego_s_modified ) )
  {
      return plan_stop_before_obstacle(
          ObstacleAvoidanceMode::StopBeforeObstacle,
          "driving mission (stop before obstacle: invalid modified-route projection)" );
  }

  result.trajectory = planner.plan_route_trajectory_from_s(
      result.modified_route,
      ego,
      traffic_participants,
      ego_s_modified );



  if( result.trajectory.states.empty() )
  {
    return plan_stop_before_obstacle(
      ObstacleAvoidanceMode::StopBeforeObstacle,
      "driving mission (stop before obstacle: modified route planning failed)" );
  }

  // The oncoming-traffic check applies whenever the candidate enters an opposite-
  // direction driving lane. It is deliberately run after the candidate trajectory
  // is planned so the clear-time estimate can use the actual planned motion
  // instead of only distance / current speed.
  if( uses_opposite_lane )
  {
    const auto oncoming_check = check_oncoming_gap(
      route,
      ego,
      traffic_participants,
      obstacle.value(),
      lateral_shift,
      vehicle_params,
      &result.trajectory,
      params );

    if( oncoming_check.conflict )
    {
      if( params.debug_oncoming_check )
      {
        std::fprintf(
          stderr,
          "[OA][oncoming] rejecting opposite-lane maneuver (mode=%d): %s\n",
          static_cast<int>( result.mode ),
          oncoming_check.reason.c_str() );
        std::fflush( stderr );
      }

      return plan_stop_before_obstacle(
        ObstacleAvoidanceMode::WaitForOncoming,
        "driving mission (waiting before obstacle: oncoming traffic conflict)" );
    }
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


    const auto route_diff_bounds =
    find_route_difference_bounds(
      route,
      result.modified_route,
      0.02 ); // 2cm point distance threshold for difference detection

    if( route_diff_bounds.has_value() )
    {
      const double route_equal_again_s =
        route_diff_bounds->has_equal_point_after_last_difference
          ? route_diff_bounds->first_equal_s_after_last_difference
          : route_diff_bounds->last_different_s;

      result.has_maneuver_bounds = true;
      result.shift_start_s = route_diff_bounds->first_different_s;
      result.shift_end_s = route_equal_again_s;

      // Optional: keep analytical values for debug
      result.plateau_start_s =
        std::max( 0.0, obstacle.value().object_s_min - params.front_clearance );

      result.plateau_end_s =
        obstacle.value().object_s_max + params.rear_clearance;

      result.obstacle_id = obstacle.value().id;
      result.lateral_shift = lateral_shift;
      result.in_lane = shift_candidate->in_lane;

      result.maneuver.active = true;
      result.maneuver.mode = result.mode;
      result.maneuver.obstacle_id = result.obstacle_id;
      result.maneuver.shift_start_s = result.shift_start_s;
      result.maneuver.plateau_start_s = result.plateau_start_s;
      result.maneuver.plateau_end_s = result.plateau_end_s;
      result.maneuver.shift_end_s = result.shift_end_s;
      result.maneuver.release_s = result.shift_end_s;
      result.maneuver.lateral_shift = result.lateral_shift;
      result.maneuver.in_lane = result.in_lane;
      result.maneuver.uses_opposite_lane = uses_opposite_lane;
      result.maneuver.has_opposite_lane_conflict_interval =
        opposite_conflict_interval.valid &&
        opposite_conflict_interval.occupies_opposite_lane;
      result.maneuver.opposite_lane_conflict_start_s =
        opposite_conflict_interval.start_s;
      result.maneuver.opposite_lane_conflict_end_s =
        opposite_conflict_interval.end_s;
      result.maneuver.commitment_s =
        result.maneuver.has_opposite_lane_conflict_interval
          ? result.maneuver.opposite_lane_conflict_start_s
          : result.plateau_start_s;

      std::fprintf(
        stderr,
        "[OA][route_diff] first_diff_s=%.2f last_diff_s=%.2f equal_again_s=%.2f has_equal_after=%s\n",
        route_diff_bounds->first_different_s,
        route_diff_bounds->last_different_s,
        route_equal_again_s,
        route_diff_bounds->has_equal_point_after_last_difference ? "true" : "false" );
      std::fflush( stderr );
    }
    else
    {
      std::fprintf(
        stderr,
        "[OA][route_diff] warning: modified route has no detectable difference to original route\n" );
      std::fflush( stderr );
    }



  return result;
}


EgoLaneOncomingStopResult
try_plan_ego_lane_oncoming_stop( TrajectoryPlanner& planner,
                                 const map::Route& route,
                                 const dynamics::VehicleStateDynamic& ego,
                                 const dynamics::TrafficParticipantSet& traffic_participants,
                                 const ObstacleAvoidanceParams& params )
{
  EgoLaneOncomingStopResult result;
  result.modified_route = route;

  const auto vehicle_params = planner.get_physical_vehicle_parameters();

  const auto threat = find_ego_lane_oncoming_threat(
    route,
    ego,
    traffic_participants,
    vehicle_params,
    params );

  if( !threat.has_value() )
  {
    result.reason = "no ego-lane oncoming participant";
    return result;
  }

  ObstacleEnvelope pseudo_obstacle;
  pseudo_obstacle.id = threat->participant_id;
  pseudo_obstacle.object_s_min = threat->participant_near_s;
  pseudo_obstacle.object_s_max = threat->participant_near_s;
  pseudo_obstacle.s_min = threat->participant_near_s;
  pseudo_obstacle.s_max = threat->participant_near_s;

  result.modified_route = build_stop_route_before_obstacle(
    route,
    pseudo_obstacle,
    ego,
    vehicle_params,
    params.ego_lane_oncoming_stop_distance );

  result.trajectory = planner.plan_route_trajectory(
    result.modified_route,
    ego,
    traffic_participants );

  if( result.trajectory.states.empty() )
  {
    result.trajectory = make_stop_trajectory( ego, params.stop_time_step );
  }

  result.trajectory.label =
    "driving mission (waiting: oncoming vehicle on ego lane)";

  result.success = true;
  result.reason = threat->reason;
  result.participant_id = threat->participant_id;
  result.participant_s = threat->participant_center_s;
  result.participant_distance_s = threat->distance_s;
  result.participant_route_speed = threat->v_route;
  result.time_to_conflict = threat->time_to_conflict;
  result.stop_s = threat->stop_s;

  if( params.debug_oncoming_check )
  {
    std::fprintf(
      stderr,
      "[OA][ego_lane_oncoming] STOP: %s\n",
      result.reason.c_str() );
    std::fflush( stderr );
  }

  return result;
}

ObstacleAvoidanceMonitorResult
monitor_active_obstacle_avoidance_maneuver(
  const map::Route& route,
  const dynamics::VehicleStateDynamic& ego,
  const dynamics::TrafficParticipantSet& traffic_participants,
  const ObstacleAvoidanceManeuver& maneuver,
  const ObstacleAvoidanceParams& params )
{
  ObstacleAvoidanceMonitorResult result;
  result.safe_to_continue = true;
  result.should_abort_before_commitment = false;
  result.reason = "active OA maneuver does not use an opposite-direction lane";

  if( !maneuver.active )
  {
    result.reason = "no active OA maneuver";
    return result;
  }

  if( !maneuver.uses_opposite_lane )
  {
    return result;
  }

  const double ego_s = route.get_s( ego );
  if( !std::isfinite( ego_s ) )
  {
    result.safe_to_continue = false;
    result.should_abort_before_commitment = true;
    result.reason = "cannot monitor active OA maneuver: invalid ego route projection";
    result.oncoming.conflict = true;
    result.oncoming.reason = result.reason;
    return result;
  }

  result.already_committed =
    std::isfinite( maneuver.commitment_s ) &&
    ego_s >= maneuver.commitment_s;

  if( !maneuver.has_opposite_lane_conflict_interval )
  {
    result.safe_to_continue = result.already_committed;
    result.should_abort_before_commitment = !result.already_committed;
    result.reason =
      result.already_committed
        ? "opposite-lane maneuver is committed but has no monitorable conflict interval"
        : "opposite-lane maneuver has no monitorable conflict interval";
    result.oncoming.conflict = !result.already_committed;
    result.oncoming.reason = result.reason;
    return result;
  }

  result.oncoming.conflict_start_s =
    maneuver.opposite_lane_conflict_start_s;
  result.oncoming.conflict_end_s =
    maneuver.opposite_lane_conflict_end_s;

  double ego_speed = std::max( 0.0, ego.vx );
  if( params.max_speed_during_avoidance > 0.0 )
  {
    ego_speed = std::min( ego_speed, params.max_speed_during_avoidance );
  }
  ego_speed = std::max( params.min_ego_speed_for_gap_check, ego_speed );

  result.oncoming.ego_clear_time =
    std::max( 0.0, result.oncoming.conflict_end_s - ego_s ) /
    ego_speed;

  for( const auto& [id, participant] : traffic_participants.participants )
  {
    const double participant_speed = std::fabs( participant.state.vx );
    if( participant_speed <= params.max_static_object_speed )
    {
      continue;
    }

    const double participant_s = route.get_s( participant.state );
    if( !std::isfinite( participant_s ) )
    {
      continue;
    }

    const auto route_pose = route.get_pose_at_s( participant_s );
    const double yaw_diff =
      normalize_angle( participant.state.yaw_angle - route_pose.yaw );
    const double v_route = participant_speed * std::cos( yaw_diff );

    if( v_route >= -params.min_oncoming_route_speed )
    {
      continue;
    }

    const double oncoming_speed =
      std::max( params.min_oncoming_speed_for_gap_check,
                std::fabs( v_route ) );

    double arrival_time = std::numeric_limits<double>::infinity();
    const char* arrival_source = "constant_velocity";

    if( participant.trajectory.has_value() &&
        participant.trajectory->states.size() >= 2 )
    {
      const auto trajectory_arrival =
        compute_arrival_time_from_trajectory(
          route,
          participant.trajectory.value(),
          ego.time,
          result.oncoming.conflict_start_s,
          result.oncoming.conflict_end_s );

      if( trajectory_arrival.has_value() )
      {
        arrival_time = trajectory_arrival.value();
        arrival_source = "trajectory";
      }
    }

    if( !std::isfinite( arrival_time ) )
    {
      if( participant_s >= result.oncoming.conflict_start_s &&
          participant_s <= result.oncoming.conflict_end_s )
      {
        arrival_time = 0.0;
      }
      else if( participant_s > result.oncoming.conflict_end_s )
      {
        arrival_time =
          ( participant_s - result.oncoming.conflict_end_s ) /
          oncoming_speed;
      }
      else
      {
        continue;
      }
    }

    const double required_arrival_time =
      result.oncoming.ego_clear_time + params.oncoming_time_margin;

    if( arrival_time <= required_arrival_time )
    {
      result.oncoming.conflict = true;
      result.oncoming.participant_id = static_cast<int>( id );
      result.oncoming.oncoming_arrival_time = arrival_time;

      char buf[384];
      std::snprintf(
        buf,
        sizeof( buf ),
        "active OA monitor: participant id=%d arrival_time=%.2f (source=%s) <= ego_clear_time=%.2f + margin=%.2f, committed=%s",
        static_cast<int>( id ),
        arrival_time,
        arrival_source,
        result.oncoming.ego_clear_time,
        params.oncoming_time_margin,
        result.already_committed ? "true" : "false" );
      result.reason = buf;
      result.oncoming.reason = buf;
      result.safe_to_continue = false;
      result.should_abort_before_commitment = !result.already_committed;

      return result;
    }
  }

  result.oncoming.conflict = false;
  result.oncoming.reason = "active OA monitor: no oncoming conflict detected";
  result.reason = result.oncoming.reason;
  return result;
}

} // namespace planner
} // namespace adore
