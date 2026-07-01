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

  // Oncoming-wait latch. Once the opposite-lane monitor decides to stop for an
  // oncoming participant, hold that stop until the participant has cleared the
  // conflict interval (or vanished for a hold time), instead of re-deciding
  // go/stop every cycle. Re-deciding each cycle near the decision boundary makes
  // ego oscillate between braking and creeping while it waits. oncoming_wait_release_s
  // is the near edge (conflict_start_s) of the opposite-lane conflict interval; the
  // oncoming travels against the route direction, so it has cleared once its route-s
  // drops below this value.
  bool   oncoming_wait_active = false;
  int    oncoming_wait_participant_id = -1;
  double oncoming_wait_release_s = std::numeric_limits<double>::quiet_NaN();
  double oncoming_wait_last_seen_time = std::numeric_limits<double>::quiet_NaN();

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

    oncoming_wait_active = false;
    oncoming_wait_participant_id = -1;
    oncoming_wait_release_s = std::numeric_limits<double>::quiet_NaN();
    oncoming_wait_last_seen_time = std::numeric_limits<double>::quiet_NaN();

    last_modified_s = std::numeric_limits<double>::quiet_NaN();
    last_modified_time = std::numeric_limits<double>::quiet_NaN();
  }
};

} // namespace planner
} // namespace adore
