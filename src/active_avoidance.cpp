/********************************************************************************
 * Copyright (c) 2026 Contributors to the Eclipse Foundation
 *
 * SPDX-License-Identifier: EPL-2.0
 ********************************************************************************/

#include "planning/active_avoidance.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace adore
{
namespace planner
{
namespace
{

// Tolerance added on top of the odometry-based advance when judging whether a
// new projection onto the active modified route is plausible. Covers projection
// refinement noise between cycles.
constexpr double MAX_PLAUSIBLE_MODIFIED_S_JUMP = 2.0;

// extra_l_margin widens the lateral match band. Ghost envelopes can stem from
// projections onto different reference lines (original vs. laterally shifted
// modified route); their l values then differ by up to the lateral shift, so
// matching must tolerate that offset.
bool
envelopes_overlap_with_margin(
    const planner::ObstacleGhostEnvelope& a,
    const planner::RouteCorridorConflict& b,
    const planner::ObstacleAvoidanceParams& params,
    double extra_l_margin = 0.0 )
{
    const double l_margin =
        params.ghost_obstacle_match_l_margin + std::max( 0.0, extra_l_margin );
    return a.object_s_max + params.ghost_obstacle_match_s_margin >= b.object_s_min &&
           a.object_s_min - params.ghost_obstacle_match_s_margin <= b.object_s_max &&
           a.object_l_max + l_margin >= b.object_l_min &&
           a.object_l_min - l_margin <= b.object_l_max;
}

bool
ghost_envelopes_overlap_with_margin(
    const planner::ObstacleGhostEnvelope& a,
    const planner::ObstacleGhostEnvelope& b,
    const planner::ObstacleAvoidanceParams& params,
    double extra_l_margin = 0.0 )
{
    const double l_margin =
        params.ghost_obstacle_match_l_margin + std::max( 0.0, extra_l_margin );
    return a.object_s_max + params.ghost_obstacle_match_s_margin >= b.object_s_min &&
           a.object_s_min - params.ghost_obstacle_match_s_margin <= b.object_s_max &&
           a.object_l_max + l_margin >= b.object_l_min &&
           a.object_l_min - l_margin <= b.object_l_max;
}

planner::ObstacleGhostEnvelope
make_ghost_envelope(
    const planner::RouteCorridorConflict& conflict,
    double now,
    const planner::ObstacleAvoidanceParams& params )
{
    planner::ObstacleGhostEnvelope envelope;
    envelope.object_s_min = conflict.object_s_min;
    envelope.object_s_max = conflict.object_s_max;
    envelope.object_l_min = conflict.object_l_min;
    envelope.object_l_max = conflict.object_l_max;
    envelope.inflated_s_min = conflict.inflated_s_min;
    envelope.inflated_s_max = conflict.inflated_s_max;
    envelope.inflated_l_min = conflict.inflated_l_min;
    envelope.inflated_l_max = conflict.inflated_l_max;
    envelope.object_class = conflict.object_class;
    envelope.first_seen_time = now;
    envelope.last_seen_time = now;
    envelope.seen_count = 1;
    envelope.hold_until_s =
        conflict.object_s_max +
        std::max( 0.0, params.rear_clearance ) +
        std::max( 0.0, params.ghost_obstacle_release_extra_s );
    envelope.hold_until_passed =
        conflict.object_class == planner::RouteCorridorObjectClass::StaticOrSlow;
    envelope.is_ghost = false;
    envelope.last_participant_id = conflict.participant_id;
    envelope.object_center_x = conflict.object_center_x;
    envelope.object_center_y = conflict.object_center_y;
    envelope.object_yaw = conflict.object_yaw;
    envelope.object_length = conflict.object_length;
    envelope.object_width = conflict.object_width;
    envelope.footprint_x = conflict.footprint_x;
    envelope.footprint_y = conflict.footprint_y;
    envelope.has_world_footprint = conflict.has_world_footprint;
    return envelope;
}

} // namespace

std::optional<planner::ObstacleGhostEnvelope>
make_original_obstacle_envelope_from_participant(
    const map::Route& route,
    const dynamics::TrafficParticipant& participant,
    double now,
    const planner::ObstacleAvoidanceParams& params )
{
    planner::RouteCorridorConflict conflict;
    conflict.participant_id = participant.id;
    conflict.object_class = planner::RouteCorridorObjectClass::StaticOrSlow;
    conflict.object_center_x = participant.state.x;
    conflict.object_center_y = participant.state.y;
    conflict.object_yaw = participant.state.yaw_angle;
    conflict.object_length =
        std::max( params.min_vehicle_dimension, participant.physical_parameters.body_length );
    conflict.object_width =
        std::max( params.min_vehicle_dimension, participant.physical_parameters.body_width );
    conflict.has_world_footprint = true;

    const double half_length = 0.5 * conflict.object_length;
    const double half_width = 0.5 * conflict.object_width;
    const double cos_yaw = std::cos( conflict.object_yaw );
    const double sin_yaw = std::sin( conflict.object_yaw );
    const std::array<std::pair<double, double>, 4> local_corners = {{
        { -half_length, -half_width },
        { -half_length,  half_width },
        {  half_length,  half_width },
        {  half_length, -half_width }
    }};

    conflict.object_s_min = std::numeric_limits<double>::infinity();
    conflict.object_s_max = -std::numeric_limits<double>::infinity();
    conflict.object_l_min = std::numeric_limits<double>::infinity();
    conflict.object_l_max = -std::numeric_limits<double>::infinity();

    for( std::size_t i = 0; i < local_corners.size(); ++i )
    {
        const auto& [local_x, local_y] = local_corners[i];
        const double x =
            conflict.object_center_x + local_x * cos_yaw - local_y * sin_yaw;
        const double y =
            conflict.object_center_y + local_x * sin_yaw + local_y * cos_yaw;
        conflict.footprint_x[i] = x;
        conflict.footprint_y[i] = y;

        dynamics::VehicleStateDynamic corner_state;
        corner_state.x = x;
        corner_state.y = y;
        const double corner_s = route.get_s( corner_state );
        if( !std::isfinite( corner_s ) )
        {
            continue;
        }

        const auto route_pose = route.get_pose_at_s( corner_s );
        const double dx = x - route_pose.x;
        const double dy = y - route_pose.y;
        const double corner_l =
            -dx * std::sin( route_pose.yaw ) + dy * std::cos( route_pose.yaw );

        conflict.object_s_min = std::min( conflict.object_s_min, corner_s );
        conflict.object_s_max = std::max( conflict.object_s_max, corner_s );
        conflict.object_l_min = std::min( conflict.object_l_min, corner_l );
        conflict.object_l_max = std::max( conflict.object_l_max, corner_l );
    }

    if( !std::isfinite( conflict.object_s_min ) ||
        !std::isfinite( conflict.object_s_max ) )
    {
        return std::nullopt;
    }

    conflict.inflated_s_min = conflict.object_s_min - std::max( 0.0, params.rear_clearance );
    conflict.inflated_s_max = conflict.object_s_max + std::max( 0.0, params.front_clearance );
    conflict.inflated_l_min = conflict.object_l_min - std::max( 0.0, params.side_clearance );
    conflict.inflated_l_max = conflict.object_l_max + std::max( 0.0, params.side_clearance );

    auto envelope = make_ghost_envelope( conflict, now, params );
    envelope.created_from_original_avoidance_obstacle = true;
    envelope.hold_until_passed = true;
    return envelope;
}

void
update_obstacle_ghost_memory(
    ActiveAvoidanceState& state,
    const planner::RouteCorridorCheckResult& safety,
    double ego_s,
    double now,
    const planner::ObstacleAvoidanceParams& params )
{
    // Master switch: with ghosting disabled, never retain memory of obstacles that
    // left perception. A disappearing detection (e.g. a pedestrian mis-detected as
    // an obstacle) then stops asserting a conflict immediately instead of being
    // held until ego passes its remembered position.
    if( !params.ghost_memory_enabled )
    {
        state.ghost_memory.clear();
        return;
    }

    for( auto& envelope : state.ghost_memory )
    {
        envelope.is_ghost = true;
        ++envelope.consecutive_missing_cycles;
    }

    // Process every conflict seen this cycle, not only the most relevant one,
    // so simultaneously visible obstacles do not age out as "missing".
    const auto detected_conflicts =
        !safety.conflicts.empty()
            ? safety.conflicts
            : ( safety.has_conflict
                    ? std::vector<planner::RouteCorridorConflict>{ safety.conflict }
                    : std::vector<planner::RouteCorridorConflict>{} );

    // Envelopes may have been created on a differently shifted reference line;
    // widen the lateral match band by the active shift.
    const double match_extra_l_margin = std::fabs( state.lateral_shift );

    for( const auto& detected_conflict : detected_conflicts )
    {
        // Ghost memory only bridges perception dropouts for STATIC obstacles
        // (the ones ego is steadily passing). Moving objects must never be
        // ghosted: once an oncoming / same-direction / crossing participant
        // leaves perception it is genuinely gone, and a retained ghost would
        // keep braking ego after, e.g., an oncoming vehicle has already passed.
        if( detected_conflict.object_class !=
            planner::RouteCorridorObjectClass::StaticOrSlow )
        {
            continue;
        }

        auto match_it =
            std::find_if(
                state.ghost_memory.begin(),
                state.ghost_memory.end(),
                [&]( const planner::ObstacleGhostEnvelope& envelope )
                {
                    return envelopes_overlap_with_margin(
                        envelope,
                        detected_conflict,
                        params,
                        match_extra_l_margin );
                } );

        const auto updated =
            make_ghost_envelope( detected_conflict, now, params );

        if( match_it != state.ghost_memory.end() )
        {
            const double first_seen_time = match_it->first_seen_time;
            const int seen_count = match_it->seen_count + 1;
            const bool original_obstacle =
                match_it->created_from_original_avoidance_obstacle;
            *match_it = updated;
            match_it->first_seen_time = first_seen_time;
            match_it->seen_count = seen_count;
            match_it->consecutive_missing_cycles = 0;
            match_it->created_from_original_avoidance_obstacle = original_obstacle;
            match_it->hold_until_passed =
                original_obstacle ||
                ( match_it->object_class == planner::RouteCorridorObjectClass::StaticOrSlow &&
                  match_it->seen_count >= 2 );
        }
        else
        {
            state.ghost_memory.push_back( updated );
        }
    }

    state.ghost_memory.erase(
        std::remove_if(
            state.ghost_memory.begin(),
            state.ghost_memory.end(),
            [&]( const planner::ObstacleGhostEnvelope& envelope )
            {
                const bool passed_release =
                    envelope.hold_until_passed &&
                    std::isfinite( ego_s ) &&
                    ego_s > envelope.hold_until_s;
                const bool timeout_release =
                    envelope.is_ghost &&
                    !envelope.created_from_original_avoidance_obstacle &&
                    now >= envelope.last_seen_time + std::max( 0.0, params.ghost_obstacle_hold_time ) &&
                    ( envelope.object_class != planner::RouteCorridorObjectClass::StaticOrSlow ||
                      envelope.seen_count < 2 ||
                      envelope.consecutive_missing_cycles >=
                          std::max( 1, params.ghost_dynamic_max_missing_cycles ) );
                const bool lifetime_release =
                    envelope.is_ghost &&
                    std::isfinite( envelope.first_seen_time ) &&
                    now >= envelope.first_seen_time +
                               std::max( 0.0, params.ghost_obstacle_max_lifetime );

                return passed_release || timeout_release || lifetime_release;
            } ),
        state.ghost_memory.end() );
}

void
start_active_avoidance_state(
    ActiveAvoidanceState& state,
    const planner::ObstacleAvoidanceResult& oa_result,
    const map::Route& original_route,
    const dynamics::TrafficParticipantSet& traffic_participants,
    const dynamics::VehicleStateDynamic& ego,
    const planner::ObstacleAvoidanceParams& params,
    bool preserve_existing_ghost_memory )
{
    const auto previous_ghost_memory =
        params.ghost_memory_enabled && preserve_existing_ghost_memory
            ? state.ghost_memory
            : std::vector<planner::ObstacleGhostEnvelope>{};

    state.active = true;
    state.base_modified_route = oa_result.modified_route;
    state.modified_route = oa_result.modified_route;

    state.obstacle_id = oa_result.obstacle_id;
    state.obstacle_ids = oa_result.obstacle_ids;

    state.shift_start_s = oa_result.shift_start_s;
    state.shift_end_s = oa_result.shift_end_s;
    state.release_s = oa_result.shift_end_s;
    state.obstacle_s_min = oa_result.obstacle_s_min;

    state.lateral_shift = oa_result.lateral_shift;
    state.in_lane = oa_result.in_lane;
    state.maneuver = oa_result.maneuver;

    state.last_modified_s = std::numeric_limits<double>::quiet_NaN();
    state.last_modified_time = std::numeric_limits<double>::quiet_NaN();
    state.ghost_memory.clear();

    // A freshly (re)committed maneuver supersedes any prior oncoming-wait hold.
    state.clear_oncoming_wait();

    // The maneuver itself remains active, but a disabled master switch must not
    // seed or preserve any obstacle memory—not even for the first cycle after a
    // fresh plan or dynamic replan.
    if( !params.ghost_memory_enabled )
    {
        return;
    }

    const auto memory_seed_obstacle_ids =
        oa_result.obstacle_ids.empty()
            ? std::vector<int>{ oa_result.obstacle_id }
            : oa_result.obstacle_ids;

    for( const int obstacle_id : memory_seed_obstacle_ids )
    {
        const auto original_participant_it =
            traffic_participants.participants.find( obstacle_id );
        if( original_participant_it == traffic_participants.participants.end() )
        {
            continue;
        }

        const auto original_envelope =
            make_original_obstacle_envelope_from_participant(
                original_route,
                original_participant_it->second,
                ego.time,
                params );
        if( original_envelope.has_value() )
        {
            state.ghost_memory.push_back( original_envelope.value() );
        }
    }

    for( const auto& previous_envelope : previous_ghost_memory )
    {
        const auto duplicate_it =
            std::find_if(
                state.ghost_memory.begin(),
                state.ghost_memory.end(),
                [&]( const planner::ObstacleGhostEnvelope& current_envelope )
                {
                    return ghost_envelopes_overlap_with_margin(
                        current_envelope,
                        previous_envelope,
                        params,
                        std::fabs( oa_result.lateral_shift ) );
                } );

        if( duplicate_it == state.ghost_memory.end() )
        {
            state.ghost_memory.push_back( previous_envelope );
        }
    }
}

std::optional<planner::RouteCorridorConflict>
most_relevant_ghost_conflict(
    const ActiveAvoidanceState& state,
    double ego_s )
{
    std::optional<planner::RouteCorridorConflict> best;

    for( const auto& envelope : state.ghost_memory )
    {
        if( !envelope.is_ghost )
        {
            continue;
        }
        if( envelope.created_from_original_avoidance_obstacle )
        {
            continue;
        }
        if( std::isfinite( ego_s ) && ego_s > envelope.hold_until_s )
        {
            continue;
        }

        planner::RouteCorridorConflict conflict;
        conflict.participant_id = envelope.last_participant_id;
        conflict.object_class = envelope.object_class;
        conflict.object_s_min = envelope.object_s_min;
        conflict.object_s_max = envelope.object_s_max;
        conflict.object_l_min = envelope.object_l_min;
        conflict.object_l_max = envelope.object_l_max;
        conflict.inflated_s_min = envelope.inflated_s_min;
        conflict.inflated_s_max = envelope.inflated_s_max;
        conflict.inflated_l_min = envelope.inflated_l_min;
        conflict.inflated_l_max = envelope.inflated_l_max;
        conflict.distance_s = std::isfinite( ego_s )
            ? std::max( 0.0, envelope.object_s_min - ego_s )
            : std::numeric_limits<double>::infinity();
        conflict.time_to_conflict = 0.0;
        conflict.currently_overlaps_route_corridor = true;
        conflict.requires_stop = true;
        conflict.object_center_x = envelope.object_center_x;
        conflict.object_center_y = envelope.object_center_y;
        conflict.object_yaw = envelope.object_yaw;
        conflict.object_length = envelope.object_length;
        conflict.object_width = envelope.object_width;
        conflict.footprint_x = envelope.footprint_x;
        conflict.footprint_y = envelope.footprint_y;
        conflict.has_world_footprint = envelope.has_world_footprint;
        conflict.reason = "ghost envelope retained after temporary detection loss";

        if( !best.has_value() || conflict.distance_s < best->distance_s )
        {
            best = conflict;
        }
    }

    return best;
}

std::optional<planner::ObstacleGhostEnvelope>
find_unpassed_original_ghost(
    const ActiveAvoidanceState& state,
    double ego_s )
{
    for( const auto& envelope : state.ghost_memory )
    {
        if( envelope.created_from_original_avoidance_obstacle &&
            envelope.hold_until_passed &&
            std::isfinite( ego_s ) &&
            ego_s <= envelope.hold_until_s )
        {
            return envelope;
        }
    }

    return std::nullopt;
}


// Build a RouteCorridorConflict from an active opposite-lane monitor result so
// the route-based stop policy can consume it. reference_s is the ego s on the
// route used to derive the longitudinal distance to the conflict.
planner::RouteCorridorConflict
make_oncoming_monitor_conflict(
    const planner::ObstacleAvoidanceMonitorResult& monitor_result,
    double reference_s )
{
    planner::RouteCorridorConflict conflict;
    conflict.participant_id = monitor_result.oncoming.participant_id;
    conflict.object_class = planner::RouteCorridorObjectClass::Oncoming;
    conflict.object_s_min = monitor_result.oncoming.conflict_start_s;
    conflict.object_s_max = monitor_result.oncoming.conflict_end_s;
    conflict.distance_s =
        std::max( 0.0, monitor_result.oncoming.conflict_start_s - reference_s );
    conflict.time_to_conflict = monitor_result.oncoming.oncoming_arrival_time;
    conflict.predicted_spatiotemporal_conflict = true;
    conflict.requires_stop = true;
    conflict.reason = monitor_result.reason;
    return conflict;
}

bool
static_or_slow_conflict_has_side_clearance(
    const planner::RouteCorridorConflict& conflict,
    const dynamics::PhysicalVehicleParameters& vehicle_params,
    const planner::ObstacleAvoidanceParams& params )
{
    if( conflict.object_class != planner::RouteCorridorObjectClass::StaticOrSlow ||
        conflict.currently_overlaps_ego_footprint ||
        !std::isfinite( conflict.object_l_min ) ||
        !std::isfinite( conflict.object_l_max ) )
    {
        return false;
    }

    const double ego_half_width =
        0.5 * std::max( params.min_vehicle_dimension, vehicle_params.body_width );
    const double actual_clearance =
        actual_lateral_clearance_to_centered_ego(
            conflict.object_l_min,
            conflict.object_l_max,
            ego_half_width );
    const double required_clearance = std::max( 0.0, params.side_clearance );

    return actual_clearance + 0.02 >= required_clearance;
}

RouteStopPlan
compute_route_stop_plan(
    const map::Route& active_route,
    const dynamics::VehicleStateDynamic& ego,
    const dynamics::PhysicalVehicleParameters& vehicle_params,
    const planner::RouteCorridorConflict& conflict,
    const planner::ObstacleAvoidanceParams& params )
{
    RouteStopPlan plan;
    const double conflict_hint =
        std::isfinite( conflict.object_s_min )
            ? conflict.object_s_min
            : std::numeric_limits<double>::quiet_NaN();
    plan.ego_s =
        project_s_on_reference_line(
            active_route,
            ego,
            conflict_hint );

    if( !std::isfinite( plan.ego_s ) )
    {
        return plan;
    }

    const double conflict_s =
        std::isfinite( conflict.object_s_min )
            ? conflict.object_s_min
            : plan.ego_s + params.stop_before_obstacle;
    const double ego_front_offset =
        vehicle_params.wheelbase + vehicle_params.front_axle_to_front_border;

    plan.stop_s =
        conflict_s -
        std::max( 0.0, params.stop_before_obstacle ) -
        ego_front_offset;

    plan.braking_deceleration =
        planned_braking_deceleration( vehicle_params, params );
    plan.required_braking_distance =
        std::max( 0.0, ego.vx ) * std::max( 0.0, ego.vx ) /
        ( 2.0 * plan.braking_deceleration );
    plan.brake_start_s =
        plan.stop_s -
        plan.required_braking_distance -
        std::max( 0.0, params.modified_route_braking_safety_margin );
    plan.valid =
        std::isfinite( plan.stop_s ) &&
        std::isfinite( plan.brake_start_s );

    return plan;
}

bool
should_stop_for_active_conflict(
    const map::Route& active_route,
    const dynamics::VehicleStateDynamic& ego,
    const dynamics::PhysicalVehicleParameters& vehicle_params,
    const planner::RouteCorridorConflict& conflict,
    const planner::ObstacleAvoidanceParams& params )
{
    if( conflict.currently_overlaps_ego_footprint )
    {
        return true;
    }

    const auto stop_plan =
        compute_route_stop_plan(
            active_route,
            ego,
            vehicle_params,
            conflict,
            params );

    if( !stop_plan.valid )
    {
        return true;
    }

    if( conflict.predicted_spatiotemporal_conflict &&
        conflict.time_to_conflict <= params.modified_route_stop_ttc_threshold )
    {
        if( conflict.object_class != planner::RouteCorridorObjectClass::StaticOrSlow )
        {
            return true;
        }

        return stop_plan.ego_s >= stop_plan.brake_start_s;
    }

    if( conflict.currently_overlaps_route_corridor &&
        stop_plan.ego_s >= stop_plan.brake_start_s )
    {
        return true;
    }

    return false;
}

std::optional<double>
compute_monotonic_ego_s_modified(
    double ego_s_modified_raw,
    const dynamics::VehicleStateDynamic& vehicle_state_dynamic,
    ActiveAvoidanceState& state )
{
    double ego_s_modified = ego_s_modified_raw;

    if( std::isfinite( ego_s_modified_raw ) &&
        std::isfinite( state.last_modified_s ) )
    {
        // Enforce monotonic progression: never go backward. Forward
        // jumps beyond what the vehicle can have travelled since the
        // last cycle are projection artifacts (e.g. matches at the
        // search-window edge); advance by odometry instead of
        // latching the jump permanently.
        const double dt =
            std::isfinite( state.last_modified_time )
                ? std::max(
                      0.0,
                      vehicle_state_dynamic.time - state.last_modified_time )
                : 0.0;
        const double odometry_advance =
            std::max( 0.0, vehicle_state_dynamic.vx ) * dt;
        const double max_plausible_advance =
            odometry_advance + MAX_PLAUSIBLE_MODIFIED_S_JUMP;

        if( ego_s_modified_raw < state.last_modified_s )
        {
            ego_s_modified = state.last_modified_s;
        }
        else if( ego_s_modified_raw - state.last_modified_s >
                 max_plausible_advance )
        {
            ego_s_modified = state.last_modified_s + odometry_advance;
        }
    }
    else if( !std::isfinite( ego_s_modified_raw ) &&
             std::isfinite( state.last_modified_s ) )
    {
        // Fallback: projection lost, use last valid value
        ego_s_modified = state.last_modified_s;
    }

    if( !std::isfinite( ego_s_modified ) )
    {
        return std::nullopt;
    }

    state.last_modified_s = ego_s_modified;
    state.last_modified_time = vehicle_state_dynamic.time;
    return ego_s_modified;
}

} // namespace planner
} // namespace adore
