/********************************************************************************
 * Copyright (c) 2026 Contributors to the Eclipse Foundation
 *
 * SPDX-License-Identifier: EPL-2.0
 ********************************************************************************/

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "planning/active_avoidance.hpp"
#include "planning/obstacle_avoidance.hpp"
#include "planning/speed_profiles.hpp"
#include "../src/obstacle_avoidance_internal.hpp"

namespace
{

adore::map::Route
make_straight_route( double length, double step )
{
  adore::map::Route route;
  for( double s = 0.0; s <= length + 1e-9; s += step )
  {
    adore::map::MapPoint point;
    point.x = s;
    point.y = 0.0;
    point.s = s;
    route.reference_line[s] = point;
  }
  return route;
}

double
max_speed_at_or_inf( const adore::map::Route& route, double s )
{
  const auto it = route.reference_line.find( s );
  if( it == route.reference_line.end() )
  {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return it->second.max_speed.value_or( std::numeric_limits<double>::infinity() );
}

adore::planner::ObstacleAvoidanceParams
test_params()
{
  adore::planner::ObstacleAvoidanceParams params;
  params.max_speed_during_avoidance = 2.0;
  params.min_motion_speed = 0.05;
  params.planned_braking_deceleration = 1.0;
  return params;
}

adore::dynamics::PhysicalVehicleParameters
test_vehicle_params()
{
  adore::dynamics::PhysicalVehicleParameters vehicle_params;
  vehicle_params.acceleration_min = -2.0;
  return vehicle_params;
}

} // namespace

TEST( ObstacleAvoidance, AvoidanceSpeedProfileCapsOnlyAtShiftStart )
{
  const auto route = make_straight_route( 100.0, 10.0 );
  const auto params = test_params();
  const auto vehicle_params = test_vehicle_params();

  const auto profiled_route =
    adore::planner::RouteSpeedPolicy::apply_avoidance_speed_profile(
      route,
      0.0,
      50.0,
      70.0,
      vehicle_params,
      params );

  EXPECT_GT( max_speed_at_or_inf( profiled_route, 0.0 ), params.max_speed_during_avoidance );
  EXPECT_GT( max_speed_at_or_inf( profiled_route, 40.0 ), params.max_speed_during_avoidance );
  EXPECT_NEAR( max_speed_at_or_inf( profiled_route, 50.0 ), params.max_speed_during_avoidance, 1e-9 );
  EXPECT_NEAR( max_speed_at_or_inf( profiled_route, 70.0 ), params.max_speed_during_avoidance, 1e-9 );
  EXPECT_TRUE( std::isinf( max_speed_at_or_inf( profiled_route, 80.0 ) ) );
}

TEST( ObstacleAvoidance, SegmentedAvoidanceSpeedRestoresMissionSpeedInLongGap )
{
  auto route = make_straight_route( 110.0, 10.0 );
  for( auto& [s, point] : route.reference_line )
  {
    static_cast<void>( s );
    point.max_speed = 8.0;
  }

  const auto params = test_params();
  const auto vehicle_params = test_vehicle_params();
  const std::vector<adore::planner::AvoidanceSpeedSegment> segments{
    { 20.0, 30.0 },
    { 80.0, 90.0 } };

  const auto profiled_route =
    adore::planner::RouteSpeedPolicy::
      apply_segmented_avoidance_speed_profile(
        route,
        0.0,
        segments,
        vehicle_params,
        params );

  EXPECT_NEAR( max_speed_at_or_inf( profiled_route, 20.0 ), 2.0, 1e-9 );
  EXPECT_NEAR( max_speed_at_or_inf( profiled_route, 30.0 ), 2.0, 1e-9 );
  EXPECT_NEAR( max_speed_at_or_inf( profiled_route, 40.0 ), 8.0, 1e-9 );
  EXPECT_NEAR( max_speed_at_or_inf( profiled_route, 50.0 ), 8.0, 1e-9 );
  EXPECT_NEAR(
    max_speed_at_or_inf( profiled_route, 60.0 ),
    std::sqrt( 44.0 ),
    1e-9 );
  EXPECT_NEAR(
    max_speed_at_or_inf( profiled_route, 70.0 ),
    std::sqrt( 24.0 ),
    1e-9 );
  EXPECT_NEAR( max_speed_at_or_inf( profiled_route, 80.0 ), 2.0, 1e-9 );
  EXPECT_NEAR( max_speed_at_or_inf( profiled_route, 90.0 ), 2.0, 1e-9 );
  EXPECT_NEAR( max_speed_at_or_inf( profiled_route, 100.0 ), 8.0, 1e-9 );
}

TEST( ObstacleAvoidance, SegmentedAvoidanceSpeedKeepsShortGapBrakeSafe )
{
  auto route = make_straight_route( 70.0, 5.0 );
  for( auto& [s, point] : route.reference_line )
  {
    static_cast<void>( s );
    point.max_speed = 8.0;
  }

  const auto params = test_params();
  const auto vehicle_params = test_vehicle_params();
  const std::vector<adore::planner::AvoidanceSpeedSegment> segments{
    { 20.0, 30.0 },
    { 40.0, 50.0 } };

  const auto profiled_route =
    adore::planner::RouteSpeedPolicy::
      apply_segmented_avoidance_speed_profile(
        route,
        0.0,
        segments,
        vehicle_params,
        params );

  EXPECT_NEAR( max_speed_at_or_inf( profiled_route, 30.0 ), 2.0, 1e-9 );
  EXPECT_NEAR(
    max_speed_at_or_inf( profiled_route, 35.0 ),
    std::sqrt( 14.0 ),
    1e-9 );
  EXPECT_NEAR( max_speed_at_or_inf( profiled_route, 40.0 ), 2.0, 1e-9 );
}

TEST( ObstacleAvoidance, SegmentedAvoidanceSpeedMergesOverlappingRamps )
{
  auto route = make_straight_route( 80.0, 10.0 );
  for( auto& [s, point] : route.reference_line )
  {
    static_cast<void>( s );
    point.max_speed = 8.0;
  }

  const auto params = test_params();
  const auto vehicle_params = test_vehicle_params();
  const std::vector<adore::planner::AvoidanceSpeedSegment> segments{
    { 20.0, 40.0 },
    { 35.0, 60.0 } };

  const auto profiled_route =
    adore::planner::RouteSpeedPolicy::
      apply_segmented_avoidance_speed_profile(
        route,
        0.0,
        segments,
        vehicle_params,
        params );

  EXPECT_NEAR( max_speed_at_or_inf( profiled_route, 20.0 ), 2.0, 1e-9 );
  EXPECT_NEAR( max_speed_at_or_inf( profiled_route, 30.0 ), 2.0, 1e-9 );
  EXPECT_NEAR( max_speed_at_or_inf( profiled_route, 40.0 ), 2.0, 1e-9 );
  EXPECT_NEAR( max_speed_at_or_inf( profiled_route, 50.0 ), 2.0, 1e-9 );
  EXPECT_NEAR( max_speed_at_or_inf( profiled_route, 60.0 ), 2.0, 1e-9 );
  EXPECT_NEAR( max_speed_at_or_inf( profiled_route, 70.0 ), 8.0, 1e-9 );
}

TEST( ObstacleAvoidance, AvoidanceSpeedFloorNeverExceedsConfiguredCap )
{
  auto params = test_params();
  params.max_speed_during_avoidance = 1.0;
  params.min_avoidance_speed = 2.0;

  EXPECT_NEAR(
    adore::planner::avoidance_speed_for_shift(
      1.0, 4.0, params ),
    1.0,
    1e-9 );
}

TEST( ObstacleAvoidance, StopBeforeObstacleDoesNotImmediatelySetZeroEverywhere )
{
  const auto route = make_straight_route( 100.0, 5.0 );
  const auto params = test_params();
  const auto vehicle_params = test_vehicle_params();

  const auto stop_plan =
    adore::planner::RouteStopPolicy::plan_stop_on_route(
      route,
      0.0,
      4.0,
      30.0,
      vehicle_params,
      params );

  ASSERT_TRUE( stop_plan.valid );
  EXPECT_GT( max_speed_at_or_inf( stop_plan.route, 0.0 ), 0.0 );
  EXPECT_GT( max_speed_at_or_inf( stop_plan.route, 20.0 ), 0.0 );
  EXPECT_NEAR( max_speed_at_or_inf( stop_plan.route, 25.0 ), std::sqrt( 10.0 ), 1e-9 );
  EXPECT_NEAR( max_speed_at_or_inf( stop_plan.route, 30.0 ), 0.0, 1e-9 );
  EXPECT_NEAR( max_speed_at_or_inf( stop_plan.route, 40.0 ), 0.0, 1e-9 );
}

TEST( ObstacleAvoidance, StopProfileFallsBackToMaximumBrakingWhenUnreachable )
{
  const auto route = make_straight_route( 100.0, 10.0 );
  const auto params = test_params();
  const auto vehicle_params = test_vehicle_params();

  const auto stop_plan =
    adore::planner::RouteStopPolicy::plan_stop_on_route(
      route,
      20.0,
      10.0,
      25.0,
      vehicle_params,
      params );

  ASSERT_TRUE( stop_plan.valid );
  EXPECT_TRUE( std::isinf( max_speed_at_or_inf( stop_plan.route, 10.0 ) ) );
  EXPECT_NEAR( max_speed_at_or_inf( stop_plan.route, 20.0 ), 10.0, 1e-9 );
  EXPECT_NEAR( max_speed_at_or_inf( stop_plan.route, 30.0 ), std::sqrt( 60.0 ), 1e-9 );
  EXPECT_NEAR( max_speed_at_or_inf( stop_plan.route, 40.0 ), std::sqrt( 20.0 ), 1e-9 );
  EXPECT_NEAR( max_speed_at_or_inf( stop_plan.route, 45.0 ), 0.0, 1e-9 );
  EXPECT_NEAR( max_speed_at_or_inf( stop_plan.route, 50.0 ), 0.0, 1e-9 );
}

TEST( ObstacleAvoidance, ReachableStopTargetStaysFixedWithIntermediateBraking )
{
  const auto route = make_straight_route( 100.0, 10.0 );
  const auto params = test_params();
  const auto vehicle_params = test_vehicle_params();

  // From 10 m/s, comfort braking at 1 m/s² needs 50 m and maximum braking
  // at 2 m/s² needs 25 m. The requested target is 40 m ahead, so it must
  // remain fixed and use the required intermediate 1.25 m/s².
  const auto stop_plan =
    adore::planner::RouteStopPolicy::plan_stop_on_route(
      route,
      20.0,
      10.0,
      60.0,
      vehicle_params,
      params );

  ASSERT_TRUE( stop_plan.valid );
  EXPECT_NEAR( max_speed_at_or_inf( stop_plan.route, 20.0 ), 10.0, 1e-9 );
  EXPECT_NEAR( max_speed_at_or_inf( stop_plan.route, 30.0 ), std::sqrt( 75.0 ), 1e-9 );
  EXPECT_NEAR( max_speed_at_or_inf( stop_plan.route, 50.0 ), 5.0, 1e-9 );
  EXPECT_NEAR( max_speed_at_or_inf( stop_plan.route, 60.0 ), 0.0, 1e-9 );
  EXPECT_NEAR( max_speed_at_or_inf( stop_plan.route, 70.0 ), 0.0, 1e-9 );
}

TEST( ObstacleAvoidance, RouteTrajectoryStopsAtFirstZeroSpeedPoint )
{
  const auto route = make_straight_route( 30.0, 10.0 );
  adore::planner::SpeedProfile speed_profile;
  speed_profile.s_to_speed = {
    { 0.0, 4.0 },
    { 10.0, 0.0 },
    { 20.0, 0.0 },
    { 30.0, 2.0 }
  };

  adore::dynamics::VehicleStateDynamic ego;
  ego.time = 42.0;

  const auto trajectory =
    adore::planner::generate_trajectory_from_speed_profile(
      speed_profile, route, ego, 0.1 );

  ASSERT_FALSE( trajectory.states.empty() );
  // Relative clock: the sampler emits t=0..t_final; the caller re-bases to ego.time.
  EXPECT_NEAR( trajectory.states.front().time, 0.0, 1e-9 );
  EXPECT_NEAR( trajectory.states.back().x, 10.0, 1e-9 );
  EXPECT_NEAR( trajectory.states.back().vx, 0.0, 1e-9 );
  for( const auto& state : trajectory.states )
  {
    EXPECT_LE( state.x, 10.0 + 1e-9 );
  }
}

// Regression: the stop-at-zero guard must key off the segment END speed, not the
// start. A route whose first point is zero speed (ego starting from standstill)
// must still build a moving trajectory and drive off, not truncate to a hold.
TEST( ObstacleAvoidance, RouteFromStandstillDrivesOff )
{
  const auto route = make_straight_route( 50.0, 10.0 );
  adore::planner::SpeedProfile speed_profile;
  speed_profile.s_to_speed = {
    { 0.0, 0.0 },   // ego currently stopped
    { 10.0, 3.0 },
    { 20.0, 5.0 },
    { 30.0, 5.0 }
  };

  adore::dynamics::VehicleStateDynamic ego;
  ego.time = 0.0;

  const auto trajectory =
    adore::planner::generate_trajectory_from_speed_profile(
      speed_profile, route, ego, 0.1 );

  ASSERT_GT( trajectory.states.size(), 2U );
  EXPECT_NEAR( trajectory.states.front().x, 0.0, 1e-9 );
  EXPECT_GT( trajectory.states.back().x, 1.0 );

  bool moves = false;
  for( const auto& state : trajectory.states )
  {
    if( state.vx > 1.0 )
    {
      moves = true;
    }
  }
  EXPECT_TRUE( moves );
}

TEST( ObstacleAvoidance, ZeroSpeedRouteProducesRouteAlignedHold )
{
  const auto route = make_straight_route( 20.0, 10.0 );
  adore::planner::SpeedProfile speed_profile;
  speed_profile.s_to_speed = {
    { 0.0, 0.0 },
    { 10.0, 0.0 },
    { 20.0, 0.0 }
  };

  adore::dynamics::VehicleStateDynamic ego;
  ego.time = 7.0;

  const auto trajectory =
    adore::planner::generate_trajectory_from_speed_profile(
      speed_profile, route, ego, 0.1 );

  ASSERT_EQ( trajectory.states.size(), 2U );
  // Relative clock (see above): hold is [t=0, t=time_step].
  EXPECT_NEAR( trajectory.states.front().time, 0.0, 1e-9 );
  EXPECT_NEAR( trajectory.states.back().time, 0.1, 1e-9 );
  EXPECT_NEAR( trajectory.states.front().x, 0.0, 1e-9 );
  EXPECT_NEAR( trajectory.states.back().x, 0.0, 1e-9 );
  EXPECT_NEAR( trajectory.states.back().vx, 0.0, 1e-9 );
}

// The ego reference is the rear axle. Full shift therefore starts before the
// ego front reaches the obstacle and ends only after the ego rear clears it.
TEST( ObstacleAvoidance, ShiftSpansFullEgoFootprintAlongsideObstacle )
{
  adore::planner::oa_detail::ObstacleEnvelope obstacle;
  obstacle.object_s_min = 20.0;
  obstacle.object_s_max = 25.0;
  obstacle.object_l_min = -0.5;
  obstacle.object_l_max = 0.5;
  obstacle.overlaps_ego_corridor = true;

  adore::planner::oa_detail::AvoidanceGroup group;
  group.obstacles.push_back( obstacle );

  auto vehicle_params = test_vehicle_params();
  vehicle_params.body_width = 2.0;

  auto params = test_params();
  params.front_clearance = 7.0;
  params.rear_clearance = 7.0;
  params.side_clearance = 1.0;
  constexpr double nominal_left_shift = 3.0;

  const auto offset_at = [&]( double s ) {
    return adore::planner::oa_detail::avoidance_shift_offset_at_s(
      s, group, nominal_left_shift, vehicle_params, params );
  };

  // Full shift (0.5 obstacle + 1.0 side_clearance + 1.0 ego half-width = 2.5)
  // spans the obstacle itself.
  EXPECT_NEAR( offset_at( 20.0 ), 2.5, 1e-9 );
  EXPECT_NEAR( offset_at( 22.5 ), 2.5, 1e-9 );
  EXPECT_NEAR( offset_at( 25.0 ), 2.5, 1e-9 );

  const double ego_front_offset =
    vehicle_params.wheelbase + vehicle_params.front_axle_to_front_border;
  const double ego_rear_offset =
    vehicle_params.rear_border_to_rear_axle;
  const double hold_start = 20.0 - ego_front_offset;
  const double hold_end = 25.0 + ego_rear_offset;
  const double shift_start = hold_start - params.front_clearance;
  const double shift_end = hold_end + params.rear_clearance;

  EXPECT_NEAR( offset_at( shift_start - 0.1 ), 0.0, 1e-9 );
  EXPECT_NEAR( offset_at( shift_end + 0.1 ), 0.0, 1e-9 );
  EXPECT_NEAR( offset_at( hold_start ), 2.5, 1e-9 );
  EXPECT_NEAR( offset_at( hold_end ), 2.5, 1e-9 );
}

TEST( ObstacleAvoidance, CommittedHullSurvivesMissingDetection )
{
  const auto route = make_straight_route( 100.0, 1.0 );
  adore::dynamics::VehicleStateDynamic ego;
  ego.x = 0.0;
  ego.y = 0.0;

  adore::dynamics::TrafficParticipantSet participants;
  const auto vehicle_params = test_vehicle_params();
  auto params = test_params();

  adore::planner::AvoidanceShiftContribution contribution;
  contribution.participant_ids = { 17 };
  contribution.object_s_min = 25.0;
  contribution.object_s_max = 30.0;
  contribution.object_l_min = -0.5;
  contribution.object_l_max = 0.5;
  const std::vector<adore::planner::AvoidanceShiftContribution> committed = {
    contribution
  };

  const auto group =
    adore::planner::oa_detail::find_static_obstacle_group_on_route(
      route,
      ego,
      participants,
      vehicle_params,
      params,
      &committed,
      nullptr );

  ASSERT_TRUE( group.has_value() );
  ASSERT_EQ( group->obstacles.size(), 1U );
  EXPECT_EQ( group->obstacles.front().participant_ids.front(), 17 );
  EXPECT_NEAR( group->obstacles.front().object_s_min, 25.0, 1e-9 );
  EXPECT_NEAR( group->obstacles.front().object_l_max, 0.5, 1e-9 );
}

TEST( ObstacleAvoidance, DrivenCorridorObjectIsForcedIntoMissionFrame )
{
  const auto route = make_straight_route( 100.0, 1.0 );
  adore::dynamics::VehicleStateDynamic ego;
  ego.x = 0.0;
  ego.y = 0.0;

  adore::dynamics::VehicleStateDynamic object_state;
  object_state.x = 30.0;
  object_state.y = 3.0;
  object_state.yaw_angle = 0.0;
  object_state.vx = 0.0;

  adore::dynamics::PhysicalVehicleParameters object_params;
  object_params.body_length = 4.0;
  object_params.body_width = 2.0;

  adore::dynamics::TrafficParticipantSet participants;
  participants.participants.emplace(
    23,
    adore::dynamics::TrafficParticipant(
      object_state,
      23,
      adore::dynamics::CAR,
      object_params ) );

  auto vehicle_params = test_vehicle_params();
  vehicle_params.body_width = 2.0;
  auto params = test_params();
  params.ego_corridor_safety_margin = 0.5;

  const auto without_force =
    adore::planner::oa_detail::find_static_obstacle_group_on_route(
      route, ego, participants, vehicle_params, params );
  EXPECT_FALSE( without_force.has_value() );

  const std::vector<int> forced_ids = { 23 };
  const auto forced_group =
    adore::planner::oa_detail::find_static_obstacle_group_on_route(
      route,
      ego,
      participants,
      vehicle_params,
      params,
      nullptr,
      &forced_ids );

  ASSERT_TRUE( forced_group.has_value() );
  ASSERT_EQ( forced_group->obstacles.size(), 1U );
  EXPECT_TRUE( forced_group->obstacles.front().overlaps_ego_corridor );
  EXPECT_GT( forced_group->obstacles.front().object_l_min, 1.5 );
}

TEST( ObstacleAvoidance, LaterObservationOnlyExpandsCommittedHull )
{
  const auto route = make_straight_route( 100.0, 1.0 );
  adore::dynamics::VehicleStateDynamic ego;

  adore::planner::AvoidanceShiftContribution contribution;
  contribution.participant_ids = { 31 };
  contribution.object_s_min = 28.0;
  contribution.object_s_max = 32.0;
  contribution.object_l_min = -0.5;
  contribution.object_l_max = 0.5;
  const std::vector<adore::planner::AvoidanceShiftContribution> committed = {
    contribution
  };

  adore::dynamics::VehicleStateDynamic observed_state;
  observed_state.x = 30.0;
  observed_state.y = 0.0;
  observed_state.yaw_angle = 0.0;

  adore::dynamics::PhysicalVehicleParameters observed_params;
  observed_params.body_length = 6.0;
  observed_params.body_width = 2.0;

  adore::dynamics::TrafficParticipantSet participants;
  participants.participants.emplace(
    31,
    adore::dynamics::TrafficParticipant(
      observed_state,
      31,
      adore::dynamics::CAR,
      observed_params ) );

  auto ego_params = test_vehicle_params();
  ego_params.body_width = 2.0;
  auto params = test_params();
  params.ego_corridor_safety_margin = 0.5;

  const auto group =
    adore::planner::oa_detail::find_static_obstacle_group_on_route(
      route,
      ego,
      participants,
      ego_params,
      params,
      &committed,
      nullptr );

  ASSERT_TRUE( group.has_value() );
  ASSERT_EQ( group->obstacles.size(), 1U );
  EXPECT_LE( group->obstacles.front().object_s_min, 27.0 );
  EXPECT_GE( group->obstacles.front().object_s_max, 33.0 );
  EXPECT_LE( group->obstacles.front().object_l_min, -1.0 );
  EXPECT_GE( group->obstacles.front().object_l_max, 1.0 );
}

TEST( ObstacleAvoidance, ActiveMergeAddsOnlyDrivenCorridorIntrusions )
{
  const auto route = make_straight_route( 100.0, 1.0 );
  adore::dynamics::VehicleStateDynamic ego;

  adore::planner::AvoidanceShiftContribution contribution;
  contribution.participant_ids = { 41 };
  contribution.object_s_min = 20.0;
  contribution.object_s_max = 24.0;
  contribution.object_l_min = -0.5;
  contribution.object_l_max = 0.5;
  const std::vector<adore::planner::AvoidanceShiftContribution> committed = {
    contribution
  };

  adore::dynamics::VehicleStateDynamic unrelated_state;
  unrelated_state.x = 45.0;
  unrelated_state.y = 0.0;
  unrelated_state.yaw_angle = 0.0;

  adore::dynamics::PhysicalVehicleParameters object_params;
  object_params.body_length = 4.0;
  object_params.body_width = 2.0;

  adore::dynamics::TrafficParticipantSet participants;
  participants.participants.emplace(
    42,
    adore::dynamics::TrafficParticipant(
      unrelated_state,
      42,
      adore::dynamics::CAR,
      object_params ) );

  auto ego_params = test_vehicle_params();
  ego_params.body_width = 2.0;
  auto params = test_params();
  params.ego_corridor_safety_margin = 0.5;

  const auto group =
    adore::planner::oa_detail::find_static_obstacle_group_on_route(
      route,
      ego,
      participants,
      ego_params,
      params,
      &committed,
      nullptr );

  ASSERT_TRUE( group.has_value() );
  ASSERT_EQ( group->obstacles.size(), 1U );
  EXPECT_EQ( group->obstacles.front().participant_ids.front(), 41 );
}

TEST( ObstacleAvoidance, CompatibleRouteGeometryToleratesResampling )
{
  const auto baseline = make_straight_route( 100.0, 1.0 );
  const auto resampled = make_straight_route( 100.0, 2.5 );

  EXPECT_TRUE(
    adore::planner::routes_have_compatible_geometry(
      baseline, resampled ) );

  auto different = resampled;
  for( auto& [s, point] : different.reference_line )
  {
    static_cast<void>( s );
    point.y += 2.0;
  }

  EXPECT_FALSE(
    adore::planner::routes_have_compatible_geometry(
      baseline, different ) );
}

TEST( ObstacleAvoidance, PersistentProfileIgnoresLaterGlobalRampChanges )
{
  const auto route = make_straight_route( 100.0, 1.0 );
  adore::dynamics::VehicleStateDynamic ego;
  adore::dynamics::TrafficParticipantSet participants;
  auto ego_params = test_vehicle_params();
  ego_params.body_width = 2.0;

  adore::planner::AvoidanceShiftContribution contribution;
  contribution.participant_ids = { 51 };
  contribution.object_s_min = 20.0;
  contribution.object_s_max = 30.0;
  contribution.object_l_min = -0.5;
  contribution.object_l_max = 0.5;
  contribution.has_persistent_profile = true;
  contribution.signed_shift = 2.5;
  contribution.ramp_start_s = 10.0;
  contribution.full_shift_start_s = 20.0;
  contribution.full_shift_end_s = 30.0;
  contribution.ramp_end_s = 40.0;

  const std::vector<adore::planner::AvoidanceShiftContribution> committed = {
    contribution
  };
  auto params = test_params();
  params.side_clearance = 1.0;
  params.front_clearance = 2.0;
  params.rear_clearance = 2.0;

  const auto group =
    adore::planner::oa_detail::find_static_obstacle_group_on_route(
      route,
      ego,
      participants,
      ego_params,
      params,
      &committed,
      nullptr );

  ASSERT_TRUE( group.has_value() );
  const double offset_with_short_global_ramp =
    adore::planner::oa_detail::avoidance_shift_offset_at_s(
      15.0, group.value(), 3.0, ego_params, params );

  params.front_clearance = 20.0;
  params.rear_clearance = 20.0;
  const double offset_with_long_global_ramp =
    adore::planner::oa_detail::avoidance_shift_offset_at_s(
      15.0, group.value(), 3.0, ego_params, params );

  EXPECT_NEAR(
    offset_with_short_global_ramp,
    offset_with_long_global_ramp,
    1e-9 );
  EXPECT_GT( offset_with_short_global_ramp, 0.0 );
  EXPECT_LT( offset_with_short_global_ramp, contribution.signed_shift );
}

TEST( ObstacleAvoidance, DisabledDrivableAreaStillAllowsEnabledInLaneCandidate )
{
  adore::planner::oa_detail::ShiftCandidate candidate;
  candidate.shift = 2.0;
  candidate.valid = true;
  candidate.type =
    adore::planner::oa_detail::AvoidanceCandidateType::InLane;

  adore::planner::oa_detail::AvoidanceGroup group;
  auto params = test_params();
  params.enforce_drivable_area = false;
  params.in_lane_shift_enabled = true;
  params.adjacent_lane_enabled = false;
  params.opposite_lane_enabled = false;

  adore::planner::oa_detail::evaluate_shift_candidate(
    candidate,
    make_straight_route( 20.0, 1.0 ),
    group,
    test_vehicle_params(),
    params );

  EXPECT_TRUE( candidate.valid );
  EXPECT_TRUE( candidate.in_lane );
}
