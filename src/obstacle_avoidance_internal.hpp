/********************************************************************************
 * Copyright (c) 2026 Contributors to the Eclipse Foundation
 *
 * SPDX-License-Identifier: EPL-2.0
 ********************************************************************************/

#pragma once

// Private implementation detail of the adore_planning obstacle-avoidance module.
// This header intentionally lives in src/ (not include/) so it is never installed
// and is NOT part of the public API. It holds the shared internal types and helper
// declarations used to split the formerly monolithic obstacle_avoidance.cpp into
// cohesive translation units. All symbols live in adore::planner::oa_detail.

#include "planning/obstacle_avoidance.hpp"

#include "adore_math/point.h"
#include "adore_math/pose.h"

#include <limits>
#include <optional>
#include <utility>

namespace adore
{
namespace planner
{
namespace oa_detail
{

// ---------------------------------------------------------------------------
// Geometry primitives (obstacle_avoidance_geometry.cpp)
// ---------------------------------------------------------------------------

struct RouteFrame
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
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

struct RouteProjectionSample
{
  double s = std::numeric_limits<double>::quiet_NaN();
  double l = std::numeric_limits<double>::quiet_NaN();
  double distance = std::numeric_limits<double>::infinity();
};

double
normalize_angle( double angle );

double
smoothstep01( double t );

bool
map_points_differ_xy(
  const adore::map::MapPoint& a,
  const adore::map::MapPoint& b,
  const double xy_tolerance );

std::optional<RouteDifferenceBounds>
find_route_difference_bounds(
  const adore::map::Route& original_route,
  const adore::map::Route& modified_route,
  const double xy_tolerance );

RouteFrame
make_route_frame( const map::Route& route, double s );

double
signed_lateral_offset( const RouteFrame& frame, const math::Point2d& point );

math::Point2d
shifted_point( const RouteFrame& frame, double lateral_offset );

// Project a world point onto the route reference line. Returns arc-length s,
// signed lateral offset l (+left of the tangent) and the orthogonal distance of
// the nearest polyline segment. Returns nullopt if the route has fewer than two
// points or no finite projection exists.
std::optional<RouteProjectionSample>
project_point_to_route_segments( const map::Route& route, double x, double y );

std::optional<std::pair<double, map::MapPoint>>
find_reference_point_near_s( const map::Route& route, double query_s );

// ---------------------------------------------------------------------------
// Obstacle / participant projection onto the route (obstacle_avoidance_projection.cpp)
// ---------------------------------------------------------------------------

struct ObstacleEnvelope
{
  int id = -1;
  std::vector<int> participant_ids;

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

const char*
route_corridor_object_class_name( RouteCorridorObjectClass object_class );

bool
contains_participant_id( const std::vector<int>& ids, int id );

void
fill_world_footprint_from_participant( RouteCorridorConflict& conflict,
                                       const dynamics::TrafficParticipant& participant,
                                       const ObstacleAvoidanceParams& params );

bool
project_obstacle_to_route_analytic( const map::Route& route,
                                    const dynamics::TrafficParticipant& participant,
                                    const ObstacleAvoidanceParams& params,
                                    double ego_s,
                                    double ego_half_width,
                                    ObstacleEnvelope& envelope );

void
refresh_obstacle_envelope_derived_values( ObstacleEnvelope& envelope,
                                          const ObstacleAvoidanceParams& params );

std::optional<ParticipantFootprintOnRoute>
project_participant_footprint_to_route(
  const map::Route& route,
  const dynamics::TrafficParticipant& participant,
  const ObstacleAvoidanceParams& params );

// ---------------------------------------------------------------------------
// Static-obstacle detection / clustering (obstacle_avoidance_grouping.cpp)
// ---------------------------------------------------------------------------

struct AvoidanceGroup
{
  std::vector<ObstacleEnvelope> obstacles;
  ObstacleEnvelope envelope;
  bool uses_hull_curve = false;
  bool hard_merged = false;
};

// Participant classification helpers (shared with the oncoming module).
bool
participant_has_future_motion_prediction(
  const dynamics::TrafficParticipant& participant,
  double min_motion_speed,
  double min_motion_distance );

bool
participant_is_slow_opposite_direction_traffic(
  const map::Route& route,
  const dynamics::TrafficParticipant& participant,
  const ObstacleAvoidanceParams& params );

bool
participant_heading_is_opposite_to_route(
  const map::Route& route,
  const dynamics::TrafficParticipant& participant,
  double participant_s,
  const ObstacleAvoidanceParams& params );

AvoidanceGroup
make_avoidance_group_from_obstacle( const ObstacleEnvelope& obstacle );

void
append_obstacle_to_avoidance_group( AvoidanceGroup& group,
                                    const ObstacleEnvelope& obstacle,
                                    const ObstacleAvoidanceParams& params,
                                    bool hard_merge );

std::vector<AvoidanceGroup>
make_avoidance_groups_from_clusters(
  const std::vector<ObstacleEnvelope>& obstacle_hulls,
  const ObstacleAvoidanceParams& params );

bool
avoidance_group_contains_participant_id( const AvoidanceGroup& group, int id );

std::optional<AvoidanceGroup>
find_static_obstacle_group_on_route(
  const map::Route& route,
  const dynamics::VehicleStateDynamic& ego,
  const dynamics::TrafficParticipantSet& traffic_participants,
  const dynamics::PhysicalVehicleParameters& ego_params,
  const ObstacleAvoidanceParams& params,
  const std::vector<int>* ignored_participant_ids = nullptr );

// ---------------------------------------------------------------------------
// Lateral-shift profile math + modified-route construction (obstacle_avoidance_shift.cpp)
// ---------------------------------------------------------------------------

struct AvoidanceAlphaSample
{
  double alpha = 0.0;
  const char* source = "max";
};

void
apply_avoidance_speed_profile( map::Route& route,
                               double ego_s,
                               double shift_start_s,
                               double maneuver_end_s,
                               const dynamics::PhysicalVehicleParameters& vehicle_params,
                               const ObstacleAvoidanceParams& params );

double
avoidance_shift_alpha_at_s( double s,
                            const ObstacleEnvelope& obstacle,
                            const ObstacleAvoidanceParams& params );

AvoidanceAlphaSample
avoidance_shift_alpha_at_s( double s,
                            const AvoidanceGroup& group,
                            const ObstacleAvoidanceParams& params );

double
required_signed_shift_for_obstacle(
  const ObstacleEnvelope& obstacle,
  double nominal_lateral_shift,
  const dynamics::PhysicalVehicleParameters& ego_params,
  const ObstacleAvoidanceParams& params );

double
choose_larger_magnitude_shift( double a, double b );

double
hard_bridge_alpha_at_s( double s,
                        const ObstacleEnvelope& previous,
                        const ObstacleEnvelope& next,
                        const ObstacleAvoidanceParams& params );

double
avoidance_shift_offset_at_s(
  double s,
  const AvoidanceGroup& group,
  double nominal_lateral_shift,
  const dynamics::PhysicalVehicleParameters& ego_params,
  const ObstacleAvoidanceParams& params );

map::Route
build_modified_avoidance_route( const map::Route& route,
                                const AvoidanceGroup& group,
                                double lateral_shift,
                                double ego_s,
                                const dynamics::PhysicalVehicleParameters& ego_params,
                                const ObstacleAvoidanceParams& params );

// ---------------------------------------------------------------------------
// Drivable-area / lane-interval queries (obstacle_avoidance_drivable_area.cpp)
// ---------------------------------------------------------------------------

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

bool
lane_has_usable_borders( const map::Lane& lane );

bool
border_contains_s( const map::Border& border, double s, double slack );

bool
lane_contains_s( const map::Lane& lane, double s, double slack );

bool
add_lane_lateral_interval( const RouteFrame& frame,
                           const map::Lane& lane,
                           double lane_s,
                           double lane_s_overlap_slack,
                           LateralInterval& interval );

std::optional<LateralInterval>
get_allowed_lateral_interval_at_route_point(
  const map::Route& route,
  const map::MapPoint& route_point,
  double route_s,
  double lateral_shift,
  bool allow_adjacent_driving_lanes,
  const ObstacleAvoidanceParams& params );

OppositeDirectionLaneQuery
query_opposite_direction_lateral_interval_at_route_point(
  const map::Route& route,
  const map::MapPoint& route_point,
  double route_s,
  bool shift_left,
  const ObstacleAvoidanceParams& params );

// ---------------------------------------------------------------------------
// Oncoming-traffic reasoning (obstacle_avoidance_oncoming.cpp)
// ---------------------------------------------------------------------------

struct OppositeLaneConflictInterval
{
  bool valid = false;
  bool occupies_opposite_lane = false;

  double start_s = 0.0;
  double end_s = 0.0;

  std::string reason;
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

bool
participant_footprint_overlaps_ego_lane(
  const map::Route& route,
  const ParticipantFootprintOnRoute& footprint,
  const dynamics::PhysicalVehicleParameters& ego_params,
  const ObstacleAvoidanceParams& params );

std::optional<EgoLaneOncomingThreat>
find_ego_lane_oncoming_threat(
  const map::Route& route,
  const dynamics::VehicleStateDynamic& ego,
  const dynamics::TrafficParticipantSet& traffic_participants,
  const dynamics::PhysicalVehicleParameters& ego_params,
  const ObstacleAvoidanceParams& params );

OppositeLaneConflictInterval
compute_opposite_lane_conflict_interval(
  const map::Route& route,
  const AvoidanceGroup& group,
  double lateral_shift,
  const dynamics::PhysicalVehicleParameters& ego_params,
  const ObstacleAvoidanceParams& params );

std::optional<double>
compute_arrival_time_from_trajectory(
  const map::Route& route,
  const dynamics::Trajectory& trajectory,
  double now_time,
  double conflict_start_s,
  double conflict_end_s );

std::optional<double>
compute_ego_clear_time_from_trajectory(
  const map::Route& route,
  const dynamics::Trajectory& trajectory,
  double now_time,
  double conflict_end_s,
  double initial_s_hint );

planner::OncomingConflictResult
check_oncoming_gap( const map::Route& route,
                    const dynamics::VehicleStateDynamic& ego,
                    const dynamics::TrafficParticipantSet& traffic_participants,
                    const AvoidanceGroup& group,
                    double lateral_shift,
                    const dynamics::PhysicalVehicleParameters& ego_params,
                    const dynamics::Trajectory* candidate_ego_trajectory,
                    const ObstacleAvoidanceParams& params );

// ---------------------------------------------------------------------------
// Lateral-shift candidate generation / validation / scoring (obstacle_avoidance_candidates.cpp)
// ---------------------------------------------------------------------------

enum class AvoidanceCandidateType
{
  InLane,
  AdjacentSameDirection,
  OppositeDirection
};

enum class ShiftDirection
{
  Left,
  Right
};

struct ShiftCandidate
{
  double shift = 0.0; // +left, -right
  bool valid = false;

  // Intended validation mode. This is deliberately explicit so an opposite-lane
  // maneuver is not accidentally rejected by the current-lane check before the
  // oncoming-gap logic can run.
  AvoidanceCandidateType type = AvoidanceCandidateType::InLane;

  // True if the complete ego footprint stays inside the current route lane.
  // False means the shift uses adjacent driving-lane space and is therefore an
  // overtaking / lane-crossing maneuver.
  bool in_lane = false;
};

struct TrajectoryValidationResult
{
  bool valid = true;
  double min_lane_margin = std::numeric_limits<double>::infinity();
  double min_obstacle_lateral_margin = std::numeric_limits<double>::infinity();
  std::string reason;
};

struct RouteShiftPlanCandidate
{
  ShiftCandidate shift_candidate;
  ObstacleAvoidanceParams params;

  ObstacleAvoidanceMode mode = ObstacleAvoidanceMode::None;

  map::Route modified_route;
  dynamics::Trajectory trajectory;

  bool uses_opposite_lane = false;
  OppositeLaneConflictInterval opposite_conflict_interval;
  TrajectoryValidationResult validation;
  OncomingConflictResult oncoming;

  double ego_s_modified = std::numeric_limits<double>::quiet_NaN();
  double score = std::numeric_limits<double>::infinity();
  std::string rejection_reason;
};

bool
candidate_uses_opposite_direction_lane(
  const map::Route& route,
  const AvoidanceGroup& group,
  double lateral_shift,
  bool in_lane,
  AvoidanceCandidateType candidate_type,
  const dynamics::PhysicalVehicleParameters& ego_params,
  const ObstacleAvoidanceParams& params );

bool
candidate_respects_drivable_area( const map::Route& route,
                                  const AvoidanceGroup& group,
                                  double lateral_shift,
                                  const dynamics::PhysicalVehicleParameters& ego_params,
                                  const ObstacleAvoidanceParams& params,
                                  bool allow_adjacent_driving_lanes );

ShiftCandidate
make_candidate_from_obstacle_hulls(
  ShiftDirection direction,
  const AvoidanceGroup& group,
  const dynamics::PhysicalVehicleParameters& ego_params,
  const ObstacleAvoidanceParams& params );

bool
candidate_respects_opposite_direction_area(
  const map::Route& route,
  const AvoidanceGroup& group,
  double lateral_shift,
  const dynamics::PhysicalVehicleParameters& ego_params,
  const ObstacleAvoidanceParams& params );

void
evaluate_shift_candidate( ShiftCandidate& candidate,
                          const map::Route& route,
                          const AvoidanceGroup& group,
                          const dynamics::PhysicalVehicleParameters& ego_params,
                          const ObstacleAvoidanceParams& params );

std::vector<ShiftCandidate>
generate_shift_candidate_variants(
  const AvoidanceGroup& group,
  const dynamics::PhysicalVehicleParameters& ego_params,
  const ObstacleAvoidanceParams& params );

std::vector<ShiftCandidate>
generate_opposite_lane_candidate_variants(
  const map::Route& route,
  const AvoidanceGroup& group,
  const dynamics::PhysicalVehicleParameters& ego_params,
  const ObstacleAvoidanceParams& params );

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
  double initial_s_hint );

double
score_route_shift_candidate( const RouteShiftPlanCandidate& candidate,
                             const ObstacleAvoidanceParams& params );

// ---------------------------------------------------------------------------
// Stop / hold route + trajectory construction (obstacle_avoidance_stop_route.cpp)
// ---------------------------------------------------------------------------

double
normalized_stop_before_obstacle( const ObstacleAvoidanceParams& params );

// set_route_points_from_s_to_zero is now public (declared in
// planning/obstacle_avoidance.hpp) so the decision maker can reuse it; oa_detail
// callers resolve it via the enclosing planner namespace.

void
insert_zero_speed_stop_point( map::Route& route, double stop_s );

map::Route
build_stop_route_before_obstacle(
  const map::Route& route,
  const ObstacleEnvelope& obstacle,
  const dynamics::VehicleStateDynamic& ego,
  const dynamics::PhysicalVehicleParameters& vehicle_params,
  double stop_before_obstacle,
  double safety_margin,
  const ObstacleAvoidanceParams& params );

} // namespace oa_detail
} // namespace planner
} // namespace adore
