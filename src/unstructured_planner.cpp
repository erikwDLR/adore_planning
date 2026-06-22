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
#include "planning/unstructured_planner.hpp"

#include <algorithm>
#include <deque>

using namespace adore::math;

namespace adore
{
namespace planner
{

HybridAStarPlanner::HybridAStarPlanner() {}

void
HybridAStarPlanner::set_parameters( const std::map<std::string, double>& )
{}

void
HybridAStarPlanner::set_comfort_settings( const std::shared_ptr<dynamics::ComfortSettings>& settings )
{
  comfort_settings = settings;
}

void
HybridAStarPlanner::set_vehicle_parameters( const dynamics::PhysicalVehicleParameters& params )
{
  vehicle_params = params;
}

void
HybridAStarPlanner::set_goal( const map::Route& route, const math::Polygon2d& drivable_area, const dynamics::VehicleStateDynamic& ego )
{
  constexpr double STEP_SIZE      = 0.5; // meters
  constexpr double GOAL_LOOKAHEAD = 5.0; // meters outside polygon

  const double ego_s        = route.get_s( ego );
  const double route_length = route.get_length();

  bool   found_exit = false;
  double exit_s     = route_length;

  auto ego_mp = route.get_map_point_at_s( ego_s );

  if( !drivable_area.point_inside( ego_mp ) )
  {
    auto goal_mp = route.get_map_point_at_s( std::min( route_length, ego_s + 3.0 ) );

    goal_x = goal_mp.x;
    goal_y = goal_mp.y;

    return;
  }

  bool prev_inside = drivable_area.point_inside( ego_mp );

  for( double s = ego_s + STEP_SIZE; s <= route_length; s += STEP_SIZE )
  {
    auto mp = route.get_map_point_at_s( s );

    bool inside = drivable_area.point_inside( mp );

    // Route leaves drivable area here
    if( prev_inside && !inside )
    {
      exit_s     = s;
      found_exit = true;
      break;
    }

    prev_inside = inside;
  }

  // If no exit found, use route end
  if( !found_exit )
  {
    auto goal_mp = route.get_map_point_at_s( route_length );

    goal_x = goal_mp.x;
    goal_y = goal_mp.y;

    return;
  }

  // Move goal further along route so vehicle exits polygon
  double goal_s = std::min( route_length, exit_s + GOAL_LOOKAHEAD );

  auto goal_mp = route.get_map_point_at_s( goal_s );

  goal_x = goal_mp.x;
  goal_y = goal_mp.y;
}

std::tuple<int, int, int>
HybridAStarPlanner::Node::grid_index() const
{
  int ix   = static_cast<int>( x / XY_RES );
  int iy   = static_cast<int>( y / XY_RES );
  int iyaw = static_cast<int>( yaw / YAW_RES );

  return { ix, iy, iyaw };
}

size_t
HybridAStarPlanner::GridHash::operator()( const std::tuple<int, int, int>& k ) const
{
  auto [x, y, yaw] = k;

  size_t h1 = std::hash<int>()( x );
  size_t h2 = std::hash<int>()( y );
  size_t h3 = std::hash<int>()( yaw );

  return h1 ^ ( h2 << 1 ) ^ ( h3 << 2 );
}

// ======================================================
// CONFIG
// ======================================================

static constexpr double LOCAL_GOAL_MAX_DIST = 20.0;
static constexpr double GOAL_REACHED_RADIUS = 0.5;

static constexpr double VEHICLE_LENGTH = 4.5;
static constexpr double VEHICLE_WIDTH  = 2.0;

// ======================================================
// LOCAL GOAL STORAGE
// ======================================================

math::Point2d current_local_goal;
bool          has_local_goal = false;

// ======================================================
// DRIVABLE AREA CHECK
// ======================================================

bool
HybridAStarPlanner::inside_drivable_area( double x, double y, double yaw, const math::Polygon2d& drivable_area )
{
  // ----------------------------------------------------
  // IMPORTANT:
  //
  // Use CENTER POINT checking only.
  //
  // Full polygon footprint checks become too strict
  // in narrow zig-zag corridors.
  // ----------------------------------------------------

  math::Point2d p;
  p.x = x;
  p.y = y;

  return drivable_area.point_inside( p );
}

bool
HybridAStarPlanner::inside_search_region( double x, double y, double yaw, const math::Polygon2d& drivable_area )
{
  if( inside_drivable_area( x, y, yaw, drivable_area ) )
  {
    return true;
  }

  constexpr double GOAL_REGION_RADIUS = 8.0;

  double dist_to_goal = std::hypot( x - goal_x, y - goal_y );

  return dist_to_goal < GOAL_REGION_RADIUS;
}

// ======================================================
// COLLISION
// ======================================================

bool
HybridAStarPlanner::collision( double x, double y, const dynamics::TrafficParticipantSet& participants )
{
  for( const auto& [id, p] : participants.participants )
  {
    double dx = x - p.state.x;
    double dy = y - p.state.y;

    if( std::hypot( dx, dy ) < VEHICLE_RADIUS )
      return true;
  }

  return false;
}

double
HybridAStarPlanner::distance_to_polygon_boundary( const math::Point2d& p, const math::Polygon2d& polygon )
{
  double best = std::numeric_limits<double>::max();

  for( size_t i = 0; i < polygon.points.size(); i++ )
  {
    auto a = polygon.points[i];
    auto b = polygon.points[( i + 1 ) % polygon.points.size()];

    double d = distance_point_to_segment( p, a, b );

    best = std::min( best, d );
  }

  return best;
}

// ======================================================
// MOTION SIMULATION
// ======================================================

bool
HybridAStarPlanner::simulate_motion( double& x, double& y, double& yaw, double steer, const dynamics::TrafficParticipantSet& participants,
                                     const math::Polygon2d& drivable_area )
{
  double distance = 0.0;

  double start_x   = x;
  double start_y   = y;
  double start_yaw = yaw;

  while( distance < STEP )
  {
    x += MOTION_RESOLUTION * cos( yaw );
    y += MOTION_RESOLUTION * sin( yaw );

    yaw += MOTION_RESOLUTION / WHEEL_BASE * tan( steer );

    // ---------------------------------------------
    // collision checking
    // ---------------------------------------------

    if( collision( x, y, participants ) )
    {
      x   = start_x;
      y   = start_y;
      yaw = start_yaw;

      return false;
    }

    distance += MOTION_RESOLUTION;
  }

  // ---------------------------------------------
  // ONLY CHECK END STATE
  // ---------------------------------------------

  if( !inside_search_region( x, y, yaw, drivable_area ) )
  {
    x   = start_x;
    y   = start_y;
    yaw = start_yaw;

    return false;
  }

  return true;
}

// ======================================================
// LOCAL GOAL GENERATION
// ======================================================

math::Point2d
HybridAStarPlanner::compute_local_goal( const dynamics::VehicleStateDynamic& ego, const math::Polygon2d& drivable_area )
{
  //--------------------------------------------------
  // Directly use final goal when close enough
  //--------------------------------------------------

  double ego_to_goal = std::hypot( goal_x - ego.x, goal_y - ego.y );

  if( ego_to_goal < 15.0 )
  {
    math::Point2d p;
    p.x = goal_x;
    p.y = goal_y;

    current_local_goal = p;
    has_local_goal     = true;

    return p;
  }

  math::Point2d best_point;

  double best_score = -std::numeric_limits<double>::max();

  //--------------------------------------------------
  // Goal direction
  //--------------------------------------------------

  double goal_dx = goal_x - ego.x;
  double goal_dy = goal_y - ego.y;

  double goal_norm = std::hypot( goal_dx, goal_dy );

  if( goal_norm > 1e-3 )
  {
    goal_dx /= goal_norm;
    goal_dy /= goal_norm;
  }

  //--------------------------------------------------
  // Blend ego heading and goal heading
  //--------------------------------------------------

  double goal_heading = std::atan2( goal_y - ego.y, goal_x - ego.x );

  double search_heading = 0.7 * ego.yaw_angle + 0.3 * goal_heading;

  //--------------------------------------------------
  // Search frontier
  //--------------------------------------------------

  for( double r = 10.0; r <= LOCAL_GOAL_MAX_DIST; r += 0.5 )
  {
    for( double angle = -1.3; angle <= 1.3; angle += 0.1 )
    {
      double theta = search_heading + angle;

      math::Point2d p;

      p.x = ego.x + r * std::cos( theta );
      p.y = ego.y + r * std::sin( theta );

      //--------------------------------------------------
      // Keep candidates inside polygon
      //--------------------------------------------------

      bool inside = drivable_area.point_inside( p );

      double dist_to_goal = std::hypot( p.x - goal_x, p.y - goal_y );

      if( !inside && dist_to_goal > 8.0 )
      {
        continue;
      }

      //--------------------------------------------------
      // Candidate direction
      //--------------------------------------------------

      double dir_x = p.x - ego.x;
      double dir_y = p.y - ego.y;

      double dir_norm = std::hypot( dir_x, dir_y );

      if( dir_norm < 1e-3 )
        continue;

      dir_x /= dir_norm;
      dir_y /= dir_norm;

      //--------------------------------------------------
      // Heading alignment
      //--------------------------------------------------

      double heading_alignment = dir_x * std::cos( ego.yaw_angle ) + dir_y * std::sin( ego.yaw_angle );

      //--------------------------------------------------
      // Goal alignment
      //--------------------------------------------------

      double goal_alignment = dir_x * goal_dx + dir_y * goal_dy;

      //--------------------------------------------------
      // Actual progress toward goal
      //--------------------------------------------------

      double distance_to_goal = std::hypot( goal_x - p.x, goal_y - p.y );

      //--------------------------------------------------
      // Clearance
      //--------------------------------------------------

      double clearance = distance_to_polygon_boundary( p, drivable_area );

      //--------------------------------------------------
      // Reduce clearance importance
      //--------------------------------------------------

      double clearance_weight = 0.5;

      if( distance_to_goal < 10.0 )
      {
        clearance_weight = 0.0;
      }

      //--------------------------------------------------
      // Turn penalty
      //--------------------------------------------------

      double turning_penalty = std::abs( angle );

      //--------------------------------------------------
      // Score
      //--------------------------------------------------

      double score = 0.0;

      score += 20.0 * goal_alignment;
      score += 5.0 * heading_alignment;
      score += 1.0 * r;

      score += clearance_weight * clearance;

      score -= 1.0 * distance_to_goal;

      score -= 2.0 * turning_penalty;

      if( score > best_score )
      {
        best_score = score;
        best_point = p;
      }
    }
  }

  //--------------------------------------------------
  // Fallback
  //--------------------------------------------------

  if( best_score < -std::numeric_limits<double>::max() / 2.0 )
  {
    best_point.x = ego.x + 5.0 * std::cos( ego.yaw_angle );

    best_point.y = ego.y + 5.0 * std::sin( ego.yaw_angle );
  }

  //--------------------------------------------------
  // Hysteresis
  //--------------------------------------------------

  if( has_local_goal )
  {
    double d = std::hypot( best_point.x - current_local_goal.x, best_point.y - current_local_goal.y );

    if( d < 2.0 )
    {
      return current_local_goal;
    }
  }

  current_local_goal = best_point;
  has_local_goal     = true;

  return best_point;
}

// ======================================================
// HEURISTIC
// ======================================================

double
HybridAStarPlanner::heuristic( double x, double y, double yaw, const math::Point2d& local_goal )
{
  double dx = local_goal.x - x;
  double dy = local_goal.y - y;

  double dist = hypot( dx, dy );

  double target_heading = atan2( dy, dx );

  double heading_error = fabs( target_heading - yaw );

  return dist + 2.0 * heading_error;
}

// ======================================================
// GOAL CONNECTION
// ======================================================

bool
HybridAStarPlanner::try_goal_connection( Node* node, const math::Point2d& local_goal, const dynamics::TrafficParticipantSet& participants,
                                         const math::Polygon2d& drivable_area, std::vector<std::pair<double, double>>& path )
{
  double x   = node->x;
  double y   = node->y;
  double yaw = node->yaw;

  for( int i = 0; i < 80; i++ )
  {
    double dx = local_goal.x - x;
    double dy = local_goal.y - y;

    double dist = hypot( dx, dy );

    if( dist < GOAL_REACHED_RADIUS )
    {
      path.push_back( { local_goal.x, local_goal.y } );

      return true;
    }

    double target = atan2( dy, dx );

    double steer = std::max( -MAX_STEER, std::min( MAX_STEER, target - yaw ) );

    x += MOTION_RESOLUTION * cos( yaw );
    y += MOTION_RESOLUTION * sin( yaw );

    yaw += MOTION_RESOLUTION / WHEEL_BASE * tan( steer );

    if( collision( x, y, participants ) )
      return false;

    if( !inside_search_region( x, y, yaw, drivable_area ) )
    {
      return false;
    }

    path.push_back( { x, y } );
  }

  return false;
}

// ======================================================
// RECONSTRUCT
// ======================================================

std::vector<HybridAStarPlanner::Node*>
HybridAStarPlanner::reconstruct( Node* node )
{
  std::vector<Node*> path;

  while( node )
  {
    path.push_back( node );
    node = node->parent;
  }

  std::reverse( path.begin(), path.end() );

  return path;
}

// ======================================================
// PREVIOUS ROUTE UTILITIES
// ======================================================

double
HybridAStarPlanner::distance_to_previous_route( double x, double y )
{
  if( !has_previous_route || previous_route.reference_line.empty() )
  {
    return 0.0;
  }

  double best = std::numeric_limits<double>::max();

  for( const auto& [s, p] : previous_route.reference_line )
  {
    double dx = x - p.x;
    double dy = y - p.y;

    best = std::min( best, std::hypot( dx, dy ) );
  }

  return best;
}

double
HybridAStarPlanner::route_difference( const map::Route& r1, const map::Route& r2 )
{
  if( r1.reference_line.empty() || r2.reference_line.empty() )
  {
    return std::numeric_limits<double>::max();
  }

  double max_diff = 0.0;

  for( const auto& [s, p] : r1.reference_line )
  {
    auto pose = r2.get_pose_at_s( s );

    double dx = p.x - pose.x;
    double dy = p.y - pose.y;

    max_diff = std::max( max_diff, std::hypot( dx, dy ) );
  }

  return max_diff;
}

double
HybridAStarPlanner::find_closest_s_on_route( const map::Route& route, const dynamics::VehicleStateDynamic& ego )
{
  double best_s    = 0.0;
  double best_dist = std::numeric_limits<double>::max();

  for( const auto& [s, p] : route.reference_line )
  {
    double dx = ego.x - p.x;
    double dy = ego.y - p.y;

    double d = std::hypot( dx, dy );

    if( d < best_dist )
    {
      best_dist = d;
      best_s    = s;
    }
  }

  return best_s;
}

map::Route
HybridAStarPlanner::trim_route_from_ego( const map::Route& route, const dynamics::VehicleStateDynamic& ego )
{
  map::Route trimmed;

  if( route.reference_line.empty() )
    return trimmed;

  double s_ego = find_closest_s_on_route( route, ego );

  for( const auto& [s, p] : route.reference_line )
  {
    if( s < s_ego )
      continue;

    map::MapPoint mp;

    mp.x = p.x;
    mp.y = p.y;

    trimmed.reference_line[s - s_ego] = mp;
  }

  return trimmed;
}

// ======================================================
// MAIN PLANNER
// ======================================================

map::Route
HybridAStarPlanner::plan( const dynamics::VehicleStateDynamic& ego, const dynamics::TrafficParticipantSet& participants,
                          const math::Polygon2d& drivable_area )
{
  std::priority_queue<QueueNode> open_set;

  std::unordered_map<std::tuple<int, int, int>, double, GridHash> visited;

  std::deque<Node> nodes;

  // ====================================================
  // START VALIDATION
  // ====================================================

  double dist_to_goal = std::hypot( ego.x - goal_x, ego.y - goal_y );

  if( !inside_drivable_area( ego.x, ego.y, ego.yaw_angle, drivable_area ) )
  {
    if( dist_to_goal > 10.0 )
    {
      std::cerr << "Start outside search region" << std::endl;

      return map::Route();
    }
  }

  // ====================================================
  // COMPUTE LOCAL GOAL
  // ====================================================

  math::Point2d local_goal = compute_local_goal( ego, drivable_area );

  // ====================================================
  // INIT
  // ====================================================

  nodes.emplace_back( ego.x, ego.y, ego.yaw_angle, 0.0, ego.steering_angle, nullptr );

  Node* start = &nodes.back();

  int counter = 0;

  open_set.push( { 0.0, counter++, start } );

  static const std::vector<double> steering_set = { -MAX_STEER,       -0.66 * MAX_STEER, -0.33 * MAX_STEER, 0.0,
                                                    0.33 * MAX_STEER, 0.66 * MAX_STEER,  MAX_STEER };

  // ====================================================
  // BEST NODE TRACKING
  // ====================================================

  Node* best_node = start;

  double best_goal_dist = std::hypot( local_goal.x - ego.x, local_goal.y - ego.y );

  // ====================================================
  // SEARCH
  // ====================================================

  while( !open_set.empty() )
  {
    Node* current = open_set.top().node;

    open_set.pop();

    auto grid = current->grid_index();

    if( visited.count( grid ) && visited[grid] <= current->g )
    {
      continue;
    }

    visited[grid] = current->g;

    // --------------------------------------------------
    // Track best frontier node
    // --------------------------------------------------

    double local_goal_dist = std::hypot( local_goal.x - current->x, local_goal.y - current->y );

    if( local_goal_dist < best_goal_dist )
    {
      best_goal_dist = local_goal_dist;
      best_node      = current;
    }

    // --------------------------------------------------
    // Goal reached
    // --------------------------------------------------

    if( local_goal_dist < GOAL_REACHED_RADIUS )
    {
      std::vector<std::pair<double, double>> connection;

      if( try_goal_connection( current, local_goal, participants, drivable_area, connection ) )
      {
        best_node = current;
        break;
      }
    }

    // ==================================================
    // NODE EXPANSION
    // ==================================================

    for( double steer : steering_set )
    {
      double nx   = current->x;
      double ny   = current->y;
      double nyaw = current->yaw;

      bool valid = simulate_motion( nx, ny, nyaw, steer, participants, drivable_area );

      if( !valid )
        continue;

      // -----------------------------------------------
      // LOCAL HORIZON LIMIT
      // -----------------------------------------------

      double dist_from_ego = std::hypot( nx - ego.x, ny - ego.y );

      if( dist_from_ego > LOCAL_GOAL_MAX_DIST )
      {
        continue;
      }

      // -----------------------------------------------
      // COSTS
      // -----------------------------------------------

      double steer_change = std::abs( steer - current->steer );

      double steer_cost = 3.0 * std::abs( steer );

      double smooth_cost = 5.0 * steer_change;

      double prev_dist = distance_to_previous_route( nx, ny );

      double continuity_cost = 0.3 * prev_dist * prev_dist;

      double g = current->g + STEP + steer_cost + smooth_cost + continuity_cost;

      nodes.emplace_back( nx, ny, nyaw, g, steer, current );

      Node* node = &nodes.back();

      double h = heuristic( nx, ny, nyaw, local_goal );

      open_set.push( { g + h, counter++, node } );
    }
  }

  // ====================================================
  // RECONSTRUCT BEST PATH
  // ====================================================

  if( best_node )
  {
    auto path_nodes = reconstruct( best_node );

    map::Route route;

    double s = 0.0;

    for( size_t i = 0; i < path_nodes.size(); i++ )
    {
      map::MapPoint p;

      p.x = path_nodes[i]->x;
      p.y = path_nodes[i]->y;

      if( i > 0 )
      {
        double dx = path_nodes[i]->x - path_nodes[i - 1]->x;

        double dy = path_nodes[i]->y - path_nodes[i - 1]->y;

        s += std::hypot( dx, dy );
      }

      route.reference_line[s] = p;
    }

    // ====================================================
    // PATH HYSTERESIS
    // ====================================================

    if( has_previous_route )
    {
      double diff = route_difference( route, previous_route );

      bool blocked = false;

      for( const auto& [s, p] : previous_route.reference_line )
      {
        if( collision( p.x, p.y, participants ) )
        {
          blocked = true;
          break;
        }
      }

      // ------------------------------------------------
      // Reuse previous route if still valid
      // ------------------------------------------------

      if( !blocked && diff < 4.0 )
      {
        return trim_route_from_ego( previous_route, ego );
      }
    }

    previous_route     = route;
    has_previous_route = true;

    return route;
  }

  return map::Route();
}

double
compute_idm_velocity( const adore::map::Route& route, const adore::dynamics::VehicleStateDynamic& ego,
                      const adore::dynamics::TrafficParticipantSet& participants, double goal_distance, double dt = 0.1 )
{
  // IDM parameters
  const double v0    = 3.0;
  const double a     = 1.5;
  const double b     = 1.5;
  const double T     = 1.2;
  double       s0    = 2.0;
  const double delta = 4.0;

  // Route corridor parameters
  const double lane_half_width = 1.75; // ~3.5m lane
  const double margin          = 0.5;

  double ego_v = ego.vx;

  double min_gap = goal_distance;
  double lead_v  = 0.0;

  // ---------------------------------------------------
  // Find closest blocking participant
  // ---------------------------------------------------
  for( const auto& [id, p] : participants.participants )
  {
    double best_dist = std::numeric_limits<double>::infinity();
    double best_s    = -1.0;
    double best_dx   = 0.0;
    double best_dy   = 0.0;

    // project participant onto route
    for( const auto& [s, pt] : route.reference_line )
    {
      double dx = p.state.x - pt.x;
      double dy = p.state.y - pt.y;

      double dist = dx * dx + dy * dy;

      if( dist < best_dist )
      {
        best_dist = dist;
        best_s    = s;
        best_dx   = dx;
        best_dy   = dy;
      }
    }

    if( best_s <= 0.0 )
      continue;

    // lateral distance from route
    double lateral_dist = std::sqrt( best_dx * best_dx + best_dy * best_dy );

    // check if object blocks route
    if( lateral_dist > lane_half_width + margin )
      continue;

    // update closest obstacle
    if( best_s < min_gap )
    {
      min_gap = best_s;
      lead_v  = p.state.vx;
    }
  }
  if( goal_distance < 4 )
    s0 = 0.5;

  // ---------------------------------------------------
  // IDM calculation
  // ---------------------------------------------------

  double delta_v = ego_v - lead_v;

  double s_star = s0 + ego_v * T + ( ego_v * delta_v ) / ( 2 * std::sqrt( a * b ) );

  double accel = a * ( 1 - std::pow( ego_v / v0, delta ) - std::pow( s_star / std::max( min_gap, 0.1 ), 2 ) );

  double v_next = ego_v + accel * 0.1;

  return std::clamp( v_next, 0.0, v0 );
}

mas::MotionModel
HybridAStarPlanner::get_planning_model( const dynamics::PhysicalVehicleParameters& params )
{
  return [params]( const mas::State& x, const mas::Control& u ) -> mas::StateDerivative {
    mas::StateDerivative dxdt;
    dxdt.setZero( 6 );
    dxdt( 0 ) = x( 3 ) * std::cos( x( 2 ) );                    // x
    dxdt( 1 ) = x( 3 ) * std::sin( x( 2 ) );                    // y
    dxdt( 2 ) = x( 3 ) * std::tan( x( 5 ) ) / params.wheelbase; // yaw_angle
    dxdt( 3 ) = u( 1 );                                         // v
    dxdt( 4 ) = x( 3 );                                         // s
    dxdt( 5 ) = u( 0 );                                         // delta
    return dxdt;
  };
}

mas::StageCostFunction
HybridAStarPlanner::make_trajectory_cost( const map::Route& ref_route )
{
  return [start_state = start_state, ref_route = ref_route, ref_velocity = ref_velocity, weights = weights,
          dt = dt]( const mas::State& x, const mas::Control& u, std::size_t k ) -> double {
    double cost = 0.0;

    const double t   = k * dt;
    const auto   ref = ref_route.get_pose_at_s( x( 4 ) );

    const double dx = x( 0 ) - ref.x;
    const double dy = x( 1 ) - ref.y;

    const double c = math::fast_cos( ref.yaw );
    const double s = math::fast_sin( ref.yaw );

    const double lon_err = dx * c + dy * s;
    const double lat_err = -dx * s + dy * c;
    const double hdg_err = math::normalize_angle( x( 2 ) - ref.yaw );
    const double spd_err = x( 3 ) - ref_velocity;

    cost += weights.lane_error * lat_err * lat_err;
    cost += weights.long_error * lon_err * lon_err;
    cost += weights.heading_error * hdg_err * hdg_err;
    cost += weights.speed_error * spd_err * spd_err;
    cost += weights.steering_angle * u( 0 ) * u( 0 ) + weights.acceleration * u( 1 ) * u( 1 );
    return cost;
  };
}

PlannerResult
HybridAStarPlanner::plan_trajectory( const dynamics::VehicleStateDynamic&   current_state,
                                     const dynamics::TrafficParticipantSet& participants, const math::Polygon2d& drivable_area,
                                     const map::Route& route )
{
  PlannerResult planner_output;
  all_participants     = participants;
  map::Route ref_route = plan( current_state, participants, drivable_area );
  if( ref_route.reference_line.size() < 2 )
  {
    std::cerr << "no route found to goal" << std::endl;
    return planner_output;
  }
  double goal_distance = ref_route.reference_line.rbegin()->first - 1.0;
  ref_velocity         = compute_idm_velocity( ref_route, current_state, all_participants, goal_distance );
  if( !drivable_area.point_inside( current_state ) )
  {
    ref_velocity = std::max( 0.0, current_state.vx - 0.5 * 0.1 );
  }
  planner_output.modified_route = ref_route;
  planner_output.trajectory     = optimize_trajectory( current_state, ref_route );
  return planner_output;
}

dynamics::Trajectory
HybridAStarPlanner::optimize_trajectory( const dynamics::VehicleStateDynamic& current_state, const map::Route& ref_route )
{
  start_state     = current_state;
  reference_route = ref_route;
  setup_problem();
  solve_problem();
  auto out_trajectory = extract_trajectory();
  return out_trajectory;
}

void
HybridAStarPlanner::solve_problem()
{
  mas::SolverParams params;
  params["max_iterations"] = solver_params.max_iterations;
  params["tolerance"]      = solver_params.tolerance;
  params["max_ms"]         = solver_params.max_ms;
  params["debug"]          = solver_params.debug;

  auto solve_with = [&]( auto&& solver, double max_ms ) {
    params["max_ms"] = max_ms;
    solver.set_params( params );
    solver.solve( *problem );
    problem->update_initial_with_best();
  };

  // first pass with collocation
  solve_with( mas::OSQPCollocation{}, 60 );
}

dynamics::Trajectory
HybridAStarPlanner::extract_trajectory()
{
  dynamics::Trajectory trajectory;
  trajectory.states.reserve( problem->horizon_steps );
  for( size_t i = 0; i < problem->horizon_steps; ++i )
  {
    dynamics::VehicleStateDynamic state;
    auto                          x = problem->best_states.col( i );
    auto                          u = problem->best_controls.col( i );

    state.x              = x( 0 );
    state.y              = x( 1 );
    state.yaw_angle      = math::normalize_angle( x( 2 ) );
    state.vx             = x( 3 );
    state.time           = start_state.time + i * dt;
    state.steering_angle = math::normalize_angle( x( 5 ) );
    state.ax             = u( 1 );

    trajectory.states.push_back( state );
  }

  return trajectory;
}

void
HybridAStarPlanner::setup_problem()
{
  problem = std::make_shared<mas::OCP>();

  problem->state_dim     = 6;
  problem->control_dim   = 2;
  problem->horizon_steps = horizon_steps;
  problem->dt            = dt;
  problem->initial_state = Eigen::VectorXd( 6 );
  problem->dynamics      = get_planning_model( vehicle_params );


  Eigen::VectorXd lower_bounds( problem->control_dim ), upper_bounds( problem->control_dim );
  lower_bounds << -vehicle_params.steering_angle_max, vehicle_params.acceleration_min;
  upper_bounds << vehicle_params.steering_angle_max, vehicle_params.acceleration_max;
  problem->input_lower_bounds = lower_bounds;
  problem->input_upper_bounds = upper_bounds;
  problem->stage_cost         = make_trajectory_cost( reference_route );

  problem->initial_state << start_state.x, start_state.y, start_state.yaw_angle, start_state.vx, 0.0, start_state.steering_angle;

  // initialize best guess controls from guess trajectory
  problem->initial_controls = mas::ControlTrajectory::Zero( problem->control_dim, problem->horizon_steps );
  for( size_t i = 0; i < problem->horizon_steps; ++i )
  {
    double t                          = i * dt;
    problem->initial_controls( 1, i ) = start_state.ax; // steering
    problem->initial_controls( 0, i ) = 0.0;
  }

  problem->initialize_problem();
  problem->verify_problem();
}

} // namespace planner
} // namespace adore