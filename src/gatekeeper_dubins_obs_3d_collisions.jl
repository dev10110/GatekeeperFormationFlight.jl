"""
File: gatekeeper_dubins_obs_3d_collisions.jl

This file defines the Gatekeeper problem for 3D Dubins paths with obstacles and inter-agent collisions
"""

using Dubins3D
using StaticArrays, LinearAlgebra

import .dubins_3d_tracking_controller

"""
Enum for different priority types for inter-agent collision checks.

Arbitrary priority type uses the orders of the offsets
Early TSwitch plans in order of the current switch time
"""
@enum PriorityType begin
    ARBITRARY
    EARLY_TSWITCH
end

@kwdef struct GKDubinsObs3DInterAgent{TOBS,TR,TOF,TF,TAC} <: GatekeeperProblem
    obstacles::TOBS # list of obstacles
    reference_path::TR # reference trajectory of the leader
    offset::TOF = [SVector(0.0, 0.0, 0.0)] # vector of desired offsets from the leader (xyz)
    turning_radius::TF = 10.0 # turning radius
    pitch_limits = (-deg2rad(10), deg2rad(15)) # pitch limits
    v_min::TF = 0.8 # minimum velocity
    v_max::TF = 1.0 # maximum velocity
    agent_radius::TF = 0.5 # radius of the agents for inter-agent collision checks
    priority_type::PriorityType = PriorityType.ARBITRARY # priority type for inter-agent collision checks
    agent_committed::TAC = []
end

function get_reference_path(gk::GKDubinsObs3DInterAgent)
    return gk.reference_path
end

function get_offset(gk::GKDubinsObs3DInterAgent)
    return gk.offset
end

function get_obstacles(gk::GKDubinsObs3DInterAgent)::AbstractVector{AbstractStaticObstacle}

    # Convert the vector of agent_committed trajectories to a vector of obstacles

    return gk.obstacles
end

##################################################################
# GATEKEEPER PROBLEM INTERFACE IMPLEMENTATION
##################################################################
"""
    get_reference_state_and_input(problem::GatekeeperProblem, path, time)

Gets the reference state and input at some time along the path.



Directly calls the method 'get_reference_state_and_input defined in the 
Dubins3DTrackingController module
"""
function get_reference_state_and_input(
    gk::GKDubinsObs3D,
    path::AbstractVector{Dubins3D.DubinsManeuver3D}, # BACKPUP PATH
    time::F, # BACKUP TIME
)::Vector{Tuple{SVector{5,F},SVector{3,F}}} where {F<:Real}
    return Dubins3DTrackingController.get_reference_state_and_input(path, time)
end