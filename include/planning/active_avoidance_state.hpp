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
// planner library (not the ROS node) so the maneuver lifecycle can be operated
// on and unit-tested without any ROS dependency. The decision-maker node owns
// the instance; the pure operations live alongside this type in the planner
// library.
struct ActiveAvoidanceState
{
  bool active = false;

  // The avoidance route selected for the active maneuver. Temporary
  // braking/waiting stop profiles are built on copies so the maneuver can
  // continue once a transient conflict clears.
  map::Route base_modified_route;

  // Fixed geometric reference frame in which every persistent obstacle hull
  // and shift contribution is expressed. Live traffic-light and weather speed
  // overlays may change, but active replans must not silently switch this frame.
  map::Route mission_route_baseline;

  double shift_start_s = 0.0;
  double shift_end_s = 0.0;
  double release_s = 0.0;

  // Route-s of the nearest avoided obstacle's leading edge. The lateral shift
  // reaches its maximum here (ramp-up ends, plateau begins), so it is used as
  // the turn-indicator cutoff. Infinity until a maneuver populates it.
  double obstacle_s_min = std::numeric_limits<double>::infinity();
  double lateral_shift = 0.0;
  double avoidance_speed = 0.0;
  bool in_lane = false;

  // Persistent per-object shift contributions of the active maneuver, ordered by
  // object_s_min. Committed contributions stay frozen; a newly appearing object that
  // intrudes the corridor on the modified route is appended so the modified
  // route is rebuilt as each object's own curve -- no artificial hold, earlier
  // objects' shapes untouched.
  std::vector<AvoidanceShiftContribution> committed_contributions;

  ObstacleAvoidanceManeuver maneuver;

  // Oncoming-wait latch. Once the opposite-lane monitor decides to stop for an
  // oncoming participant, hold that stop until the participant has cleared the
  // conflict interval (or vanished), instead of re-deciding
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

  // Clear the oncoming-wait latch. Used on maneuver reset, on a freshly
  // (re)committed maneuver, and when the wait releases mid-maneuver.
  void clear_oncoming_wait()
  {
    oncoming_wait_active = false;
    oncoming_wait_participant_id = -1;
    oncoming_wait_release_s = std::numeric_limits<double>::quiet_NaN();
    oncoming_wait_last_seen_time = std::numeric_limits<double>::quiet_NaN();
  }

  void reset()
  {
    active = false;
    base_modified_route = map::Route{};
    mission_route_baseline = map::Route{};

    shift_start_s = 0.0;
    shift_end_s = 0.0;
    release_s = 0.0;
    obstacle_s_min = std::numeric_limits<double>::infinity();

    lateral_shift = 0.0;
    avoidance_speed = 0.0;
    in_lane = false;

    committed_contributions.clear();

    maneuver = ObstacleAvoidanceManeuver{};

    clear_oncoming_wait();

    last_modified_s = std::numeric_limits<double>::quiet_NaN();
    last_modified_time = std::numeric_limits<double>::quiet_NaN();
  }
};

} // namespace planner
} // namespace adore
