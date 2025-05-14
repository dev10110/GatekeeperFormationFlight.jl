"""
File: GatekeeperDubinsObs3D

This file defines the Gatekeeper problem for 3D Dubins paths with obstacles.
"""

using Dubins3D
using StaticArrays, LinearAlgebra

import .dubins_3d_tracking_controller

@kwdef struct GKDubinsObs3D{TOBS,TR,TOF,TF} <: GatekeeperProblem
    obstacles::TOBS # list of obstacles
    reference_path::TR # reference trajectory of the leader
    offset::TOF = SVector(0.0, 0.0, 0.0) # vector of desired offsets from the leader (xyz)
    turning_radius::TF = 10.0 # turning radius
    pitch_limits = (-deg2rad(10), deg2rad(15)) # pitch limits
    v_min::TF = 0.8 # minimum velocity
    v_max::TF = 1.0 # maximum velocity
end

function get_reference_path(gk::GKDubinsObs3D)
    return gk.reference_path
end
function get_offset(gk::GKDubinsObs3D)
    return gk.offset
end
function get_obstacles(gk::GKDubinsObs3D)::AbstractVector{AbstractStaticObstacle}
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
    path::AbstractVector{Dubins3D.DubinsManeuver3D}, # path
    time::F, # time
)::Tuple{SVector{5,F},SVector{3,F}} where {F<:Real}
    return Dubins3DTrackingController.get_reference_state_and_input(path, time)
end

"""
    get_reference_state_and_input(problem::GatekeeperProblem, path, offset, time)

Gets the reference state and input at some time along the path given the offset
Directly calls the method 'get_reference_state_and_input defined in the
Dubins3DTrackingController module
"""
function get_reference_state_and_input(
    gk::GKDubinsObs3D,
    path::AbstractVector{Dubins3D.DubinsManeuver3D},
    offset::AbstractVector{F},
    time::F,
)::Tuple{SVector{5,F},SVector{3,F}} where {F<:Real}
    return Dubins3DTrackingController.get_reference_state_and_input(path, time, offset)
end

"""
    tracking_controller(problem::GatekeeperProblem, state, state_desired, input_desired)

Directly calls the method 'tracking_controller defined in the 
Dubins3DTrackingController module
"""
function tracking_controller(
    gk::GKDubinsObs3D,
    curr_state::AbstractVector{F}, # Current State
    desired_state::SVector{5,F}, # Desired State
    desired_input::SVector{3,F}, # Desired Input
)::SVector{3,F} where {F<:Real}
    return Dubins3DTrackingController.tracking_controller(
        curr_state,
        desired_state,
        desired_input,
    )
end

function apply_input_bounds(
    gk::GKDubinsObs3D,
    inputs::SVector{3,F}, # inputs
)::SVector{3,F} where {F<:Real}
    # maximum velocity over minimum turning radius
    ω_max = gk.v_max / gk.turning_radius

    # Apply input bounds
    v = clamp(inputs[1], gk.v_min, gk.v_max)
    ω = clamp(inputs[2], -ω_max, ω_max)
    ψ = clamp(inputs[3], gk.pitch_limits[1], gk.pitch_limits[2])

    return SVector(v, ω, ψ)
end

"""
    state_dynamics!(problem::GatekeeperProblem, D, state, inputs)

Defines the dynamics of the system. Modifies D in place with the system dynamics
"""
function state_dynamics!(
    gk::GKDubinsObs3D,
    D, # D
    state::AbstractVector{F}, # Current State
    input::AbstractVector{F}, # Inputs
) where {F<:Real}
    # Extract inputs
    v = input[1]
    ω = input[2]
    ψ = input[3]

    # Modifies D in place
    D[1] = v * cos(state[4]) * cos(ψ)
    D[2] = v * sin(state[4]) * cos(ψ)
    D[3] = v * sin(ψ)
    D[4] = ω

    return
end

"""
    path_length(problem::GatekeeperProblem, path)

Return the length of the path using the metric defined by the problem
"""
function path_length(gk::GKDubinsObs3D, p::Dubins3D.DubinsManeuver3D)
    return p.length
end

function path_length(gk::GKDubinsObs3D, p::AbstractVector{Dubins3D.DubinsManeuver3D})
    f = x -> path_length(gk, x)
    return sum(f, p)
end

"""
    shortest_path(problem::GatekeeperProblem, from_state::ST, to_state::ST)

Find the shortest path between two states using the architecture/method defined by the problem
"""
function shortest_path(
    gk::GKDubinsObs3D,
    from_state::SVector{5,F},
    to_state::SVector{5,F},
) where {F<:Real} # State Type
    return DubinsManeuver3D(from_state, to_state, gk.turning_radius, gk.pitch_limits)
end

function shortest_path(
    gk::GKDubinsObs3D,
    from_state::Vector{F},
    to_state::SVector{5,F},
)::Union{Dubins3D.DubinsManeuver3D,Nothing} where {F<:Real}
    try
        return DubinsManeuver3D(
            SVector{5,Float64}(from_state),
            to_state,
            gk.turning_radius,
            gk.pitch_limits,
        )
    catch
        return nothing
    end
end

"""
    construct_reconnection_sites(problem::GatekeeperProblem, step_size::Float64)

along a reference path, find the reconnection sites, aka the set of points where a trajectory could possible reconnect

Returns a Vector of points along the reference path containing the index of the refernece path where the reconnection
could happen, and the state
"""
function construct_reconnection_sites(
    gk::GKDubinsObs3D,
    reconnection_step_size::F,
)::Vector{Tuple{Int64,SVector{5,F}}} where {F<:Real} # ST is state type
    # Calculate the total number of reconnection sites to be computed
    total_sites = sum(
        ceil(Int, path_length(gk, path) / reconnection_step_size) for
        path in get_reference_path(gk)
    )

    # Preallocate the reconnection sites vector
    reconnection_sites = Vector{Tuple{Int64,SVector{5,F}}}(undef, total_sites)

    site_index = 1
    for (path_idx, path) in enumerate(get_reference_path(gk))
        # num_samples = ceil(Int, path.length / reconnection_step_size)
        # pts = compute_sampling(path; numberOfSamples = num_samples)

        # for p in pts
        #     reconnection_sites[site_index] = (path_idx, p)
        #     site_index += 1
        # end

        for τ in range(0, path.length, step = reconnection_step_size)
            pose_t::SVector{5,F} = Dubins3D.compute_at_len(path, τ)
            reconnection_sites[site_index] = (path_idx, pose_t)
            site_index += 1
        end
    end

    return reconnection_sites
end # reconnection step size


"""
    get_remaining_nominal(problem::GatekeeperProblem, path_idx::Int64)

Returns the remaining nominal path from the reference path, starting at path_idx to the end
"""
function get_remaining_nominal(
    prob::GKDubinsObs3D,
    path_idx::Int64,
    connection_pt::SVector{5,F},
) where {F<:Real}

    # remaining path
    remaining_path = get_reference_path(prob)[path_idx+1:end]

    # Construct path upto end of the segment
    # if connection_pt is the same as qf it might error
    try
        segment_path = DubinsManeuver3D(
            connection_pt,
            get_reference_path(prob)[path_idx].qf,
            prob.turning_radius,
            prob.pitch_limits,
        )

        return vcat(segment_path, remaining_path)
    catch
        return remaining_path
    end


end

##################################################################
# Compute Nominal Trajectories
##################################################################


"""
    compute_nominal_trajectoreis(prob::GKDubinsObs3D)

Compute the nominal trajectories for all agents in a 3D Gatekeeper formation-flight problem.

# Arguments
- `prob::GKDubinsObs3D`
  The problem instance, containing initial conditions, waypoints,
  dynamics models, timing parameters, and any inter-agent constraints.

# Returns
- `trajectories`
  A collection (e.g., `Vector` of custom `Trajectory` objects or named tuples)
  where each element holds the time-parameterized nominal states
  (positions, velocities, orientations, etc.) for one agent.
"""
function compute_nominal_trajectories(prob::GKDubinsObs3D)
    # compute the nominal trajectories based on the reference path and offsets
    # returns a list of nominal trajectories
    nominal_trajectories = []

    for offset in prob.offsets
        nominal_trajectory = compute_nominal_trajectory(prob.reference_path, offset)
        push!(nominal_trajectories, nominal_trajectory)
    end

    return nominal_trajectories
end

function compute_nominal_trajectory(reference_path, offset::SVector{3,Float64})
    # compute the nominal trajectory based on the reference path and offset
    # projects the offset into the plane of the point on the reference path
    nominal_path = Vector{SVector{5,Float64}}()
    # @show reference_path[1, :]
    # @show offset

    # reference path is of shape (N, 5)
    # where N is the number of points in the path

    # iterate through each point
    for p in eachrow(reference_path)
        # p is a 5D vector (x, y, z, yaw, pitch, roll) because not using roll...so pretend its 0
        # offset is a 3D vector (x, y, z)

        # basically need to get the coordinate frame of the reference point
        # apply the transformation defined by the coordinate frame to the offset
        # then add the offset to the reference point
        transformation_matrix = get_transformation_matrix(p[4], p[5], 0.0)
        projected_offset = transformation_matrix * offset

        @show p[4], p[5]
        @show projected_offset

        # compute the new point on the nominal trajectory
        new_point = SVector{5,Float64}(
            p[1] + projected_offset[1],
            p[2] + projected_offset[2],
            p[3] + projected_offset[3],
            p[4], # yaw
            p[5], # pitch
        )

        # add the new point to the nominal trajectory
        push!(nominal_path, new_point)
    end

    return reduce(hcat, nominal_path)'  # Convert to matrix with points × 6 dimensions
end