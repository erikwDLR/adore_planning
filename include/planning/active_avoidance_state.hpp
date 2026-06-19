/********************************************************************************
 * Copyright (c) 2026 Contributors to the Eclipse Foundation
 *
 * SPDX-License-Identifier: EPL-2.0
 ********************************************************************************/

#pragma once

#include <cmath>
#include <limits>
#include <optional>
#include <vector>

#include <adore_map/route.hpp>

#include "planning/obstacle_avoidance.hpp"

namespace adore
{
namespace planner
{

// Persistent state of an active obstacle-avoidance maneuver. Lives in the
// planner library (not the ROS node) so the maneuver lifecycle and its ghost
// memory can be operated on and unit-tested without any ROS dependency. The
// decision-maker node owns the instance; the pure operations live alongside this
// type in the planner library.
struct ActiveAvoidanceState
{
  bool active = false;

  // The avoidance route selected for the active maneuver. Temporary
  // braking/waiting stop profiles are built on copies so the maneuver can
  // continue once a transient conflict clears.
  map::Route base_modified_route;
  map::Route modified_route;

  int obstacle_id = -1;
  std::vector<int> obstacle_ids;

  double shift_start_s = 0.0;
  double shift_end_s = 0.0;
  double release_s = 0.0;

  // Route-s of the nearest avoided obstacle's leading edge. The lateral shift
  // reaches its maximum here (ramp-up ends, plateau begins), so it is used as
  // the turn-indicator cutoff. Infinity until a maneuver populates it.
  double obstacle_s_min = std::numeric_limits<double>::infinity();

  double lateral_shift = 0.0;
  bool in_lane = false;

  ObstacleAvoidanceManeuver maneuver;

  // Geometry/time memory for obstacles that may disappear from perception while
  // ego is still passing them or while they conflict with the active route.
  std::vector<ObstacleGhostEnvelope> ghost_memory;

  // Memory of the obstacle behind a stop/wait decision that did not start a
  // shift maneuver (StopBeforeObstacle / WaitForOncoming). Bridges short
  // perception dropouts so ego does not oscillate between stopping and
  // resuming while approaching the obstacle.
  std::optional<ObstacleGhostEnvelope> stop_hold;

  // Last valid projection on the active modified route. This keeps progress
  // monotonic while the route is laterally offset from the mission route.
  // last_modified_time records when that projection was taken so implausible
  // forward jumps can be bounded by odometry.
  double last_modified_s = std::numeric_limits<double>::quiet_NaN();
  double last_modified_time = std::numeric_limits<double>::quiet_NaN();

  void reset()
  {
    active = false;
    base_modified_route = map::Route{};
    modified_route = map::Route{};

    obstacle_id = -1;
    obstacle_ids.clear();

    shift_start_s = 0.0;
    shift_end_s = 0.0;
    release_s = 0.0;
    obstacle_s_min = std::numeric_limits<double>::infinity();

    lateral_shift = 0.0;
    in_lane = false;

    maneuver = ObstacleAvoidanceManeuver{};
    ghost_memory.clear();
    stop_hold.reset();

    last_modified_s = std::numeric_limits<double>::quiet_NaN();
    last_modified_time = std::numeric_limits<double>::quiet_NaN();
  }
};

} // namespace planner
} // namespace adore
