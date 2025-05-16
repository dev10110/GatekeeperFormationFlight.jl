"""
File: gatekeeper_dubins_obs_3d_collisions.jl

This file defines the Gatekeeper problem for 3D Dubins paths with obstacles and inter-agent collisions
"""

using Dubins3D
using StaticArrays, LinearAlgebra

import .dubins_3d_tracking_controller

@kwdef struct GKDubinsObs3DInterAgent{TOBS,TR,TOF,TF,TAC} <: MultiGatekeeperProblem
    static_obstacles::TOBS # list of obstacles
    reference_path::TR # reference trajectory of the leader
    offset::TOF = [SVector(0.0, 0.0, 0.0)] # vector of desired offsets from the leader (xyz)
    turning_radius::TF = 10.0 # turning radius
    pitch_limits = (-deg2rad(10), deg2rad(15)) # pitch limits
    v_min::TF = 0.8 # minimum velocity
    v_max::TF = 1.0 # maximum velocity
    agent_radius::TF = 0.5 # radius of the agents for inter-agent collision checks
end

function get_reference_path(gk::GKDubinsObs3DInterAgent)
    return gk.reference_path
end

function get_offset(gk::GKDubinsObs3DInterAgent)
    return gk.offset
end

function get_obstacles(gk::GKDubinsObs3DInterAgent)::AbstractVector{AbstractObstacle}
    return gk.static_obstacles
end

function get_single_agent_subproblem(
    gk::GKDUbinsObs3DInterAgent,
    agent_idx::Int,
)::GKDubinsObs3D
    return GKDubinsObs3D(
        obstacles = gk.static_obstacles,
        reference_path = gk.reference_path,
        offset = gk.offset[agent_idx],
        turning_radius = gk.turning_radius,
        pitch_limits = gk.pitch_limits,
        v_min = gk.v_min,
        v_max = gk.v_max,
    )
end

function get_single_agent_subproblem(
    gk::GKDubinsObs3DInterAgent,
    agent_idx::Int,
    committed_traj::CompositeTrajectory,
)::GKDubinsObs3D

    interagent_obstacles = [
        make_spherical_obstacle(
            commited_traj.nominal[i],
            committed_traj.backup[i],
            committed_traj.switch_time[i],
            gk.agent_radius,
        ) for i = 1:length(committed_traj.nominal) if i != agent_idx
    ]

    return GKDubinsObs3D(
        obstacles = vcat(gk.static_obstacles, interagent_obstacles),
        reference_path = gk.reference_path,
        offset = gk.offset[agent_idx],
        turning_radius = gk.turning_radius,
        pitch_limits = gk.pitch_limits,
        v_min = gk.v_min,
        v_max = gk.v_max,
    )
end

function make_spherical_obstacle(
    nominal_trajectory, # This is an odesolution / basically a function
    backup_trajectory,  # This is a vector of Dubins3D maneuvers
    switch_time::Float64,
    agent_radius::Float64,
)::TimeVaryingSphere

    function position_function(t::Float64)::SVector{3,Float64}
        if t < switch_time
            return SVector{3,f}(nominal_trajectory(t)[1:3])
        else
            return SVector{3,f}(
                dubins_3d_tracking_controller.get_reference_state_and_input(
                    backup_trajectory,
                    t - switch_time,
                )[1][1:3],
            )
        end
    end


    return TimeVaryingSphere(position_function, agent_radius)
end