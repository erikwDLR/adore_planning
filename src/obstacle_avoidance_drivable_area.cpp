/********************************************************************************
 * Copyright (c) 2026 Contributors to the Eclipse Foundation
 *
 * SPDX-License-Identifier: EPL-2.0
 ********************************************************************************/

// Drivable-area / lane-interval queries: whether a lane has usable borders and
// covers a given s, and the allowed lateral interval at a route point for the
// current lane, adjacent same-direction lanes and opposite-direction lanes.
// Depends on the geometry primitives in oa_detail.

#include "obstacle_avoidance_internal.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <optional>

namespace adore
{
namespace planner
{
namespace oa_detail
{

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

        if( !params.opposite_lane_enabled &&
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
        interval.min_l >
          merged.back().max_l + std::max( 0.0, params.lane_boundary_join_slack ) )
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

} // namespace oa_detail
} // namespace planner
} // namespace adore
