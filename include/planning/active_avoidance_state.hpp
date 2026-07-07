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

// Per-participant memory of the largest object dimensions observed for a
// maneuver obstacle. Perception reports a footprint whose size is not fixed and
// grows as ego approaches (a partially observed obstacle looks smaller at first,
// then larger). Since only under-estimation is dangerous, we latch the maximum
// object-local extent ever seen and never shrink it. Only the object-LOCAL
// length/width are stored, not a route-frame s/l span: unioning route-frame
// projections of a moving object would smear into a large "swept hull". The
// consumer re-projects the current pose with these max dimensions each cycle.
struct TrackedObstacleEnvelope
{
  int participant_id = -1;
  double max_length = 0.0;
  double max_width = 0.0;
};

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
  // Trailing edge of the committed obstacle span. With obstacle_s_min and
  // lateral_shift it reconstructs the committed shift as a synthetic "hold" region
  // on a replan, so the maneuver keeps its shift geometrically (no obstacle-id
  // memory) even if perception drops the object mid-shift.
  double obstacle_s_max = -std::numeric_limits<double>::infinity();

  double lateral_shift = 0.0;
  bool in_lane = false;

  ObstacleAvoidanceManeuver maneuver;

  // Commit latch. Set once ego has physically begun the lateral shift
  // (ego_s_modified >= shift_start_s). From then on the maneuver is driven to its
  // release point and a disappearing obstacle no longer snaps ego back to the
  // original line; the vehicle detects present objects directly, so a lost
  // detection mid-shift is bridged by finishing the committed maneuver rather
  // than by any obstacle memory. Sticky: only reset() clears it, so a dynamic
  // replan that moves shift_start_s cannot un-commit an in-progress maneuver.
  bool committed = false;

  // Largest object dimensions seen for each maneuver obstacle, keyed by
  // participant id. Fed directly from perception every cycle (not from the
  // conflict path, which ignores the maneuver's own obstacles), latched to the
  // maximum, held for the whole maneuver with no decay, and cleared on reset().
  std::vector<TrackedObstacleEnvelope> tracked_obstacles;

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
    modified_route = map::Route{};

    obstacle_id = -1;
    obstacle_ids.clear();

    shift_start_s = 0.0;
    shift_end_s = 0.0;
    release_s = 0.0;
    obstacle_s_min = std::numeric_limits<double>::infinity();
    obstacle_s_max = -std::numeric_limits<double>::infinity();

    lateral_shift = 0.0;
    in_lane = false;

    maneuver = ObstacleAvoidanceManeuver{};
    committed = false;
    tracked_obstacles.clear();

    clear_oncoming_wait();

    last_modified_s = std::numeric_limits<double>::quiet_NaN();
    last_modified_time = std::numeric_limits<double>::quiet_NaN();
  }
};

} // namespace planner
} // namespace adore
