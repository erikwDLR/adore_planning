/********************************************************************************
 * Copyright (c) 2026 Contributors to the Eclipse Foundation
 *
 * SPDX-License-Identifier: EPL-2.0
 ********************************************************************************/

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

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

// The lateral shift is keyed directly to the obstacle: full shift spans
// [object_s_min, object_s_max], ramps up over front_clearance before it and down
// over rear_clearance after it. With equal clearances it is symmetric about the
// obstacle centre (no implicit ego-length offset).
TEST( ObstacleAvoidance, ShiftSpansObstacleAndIsSymmetricWithEqualClearances )
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

  // Shift begins exactly front_clearance before, ends rear_clearance after.
  EXPECT_NEAR( offset_at( 20.0 - 7.0 - 0.1 ), 0.0, 1e-9 );
  EXPECT_NEAR( offset_at( 25.0 + 7.0 + 0.1 ), 0.0, 1e-9 );

  // Equal clearances -> symmetric about the obstacle centre.
  const double centre = 0.5 * ( 20.0 + 25.0 );
  EXPECT_NEAR( offset_at( centre - 4.0 ), offset_at( centre + 4.0 ), 1e-9 );
  EXPECT_NEAR( offset_at( centre - 6.5 ), offset_at( centre + 6.5 ), 1e-9 );
}
