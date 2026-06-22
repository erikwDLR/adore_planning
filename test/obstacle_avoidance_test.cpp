/********************************************************************************
 * Copyright (c) 2026 Contributors to the Eclipse Foundation
 *
 * SPDX-License-Identifier: EPL-2.0
 ********************************************************************************/

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "planning/obstacle_avoidance.hpp"

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
  params.trajectory_step_size = 0.1;
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
  const auto route = make_straight_route( 100.0, 10.0 );
  const auto params = test_params();
  const auto vehicle_params = test_vehicle_params();

  const auto stop_plan =
    adore::planner::RouteStopPolicy::plan_stop_on_route(
      route,
      0.0,
      4.0,
      30.0,
      adore::planner::StopReason::StaticObstacleAhead,
      vehicle_params,
      params );

  ASSERT_TRUE( stop_plan.valid );
  EXPECT_GT( max_speed_at_or_inf( stop_plan.route, 0.0 ), 0.0 );
  EXPECT_GT( max_speed_at_or_inf( stop_plan.route, 20.0 ), 0.0 );
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
      adore::planner::StopReason::ModifiedRouteConflict,
      vehicle_params,
      params );

  ASSERT_TRUE( stop_plan.valid );
  EXPECT_TRUE( std::isinf( max_speed_at_or_inf( stop_plan.route, 10.0 ) ) );
  EXPECT_NEAR( max_speed_at_or_inf( stop_plan.route, 20.0 ), 0.0, 1e-9 );
  EXPECT_NEAR( max_speed_at_or_inf( stop_plan.route, 30.0 ), 0.0, 1e-9 );
}
