using StaticArrays, LinearAlgebra
using Dubins

##################################################################
# TYPE DEFINITION
##################################################################

@kwdef struct GKDubinsWezes2D{TW,TR,TO,TF} <: GatekeeperProblem
    wezes::TW
    reference_path::TR
    offset::TO = SVector(0.0, 0.0, 0.0)
    turning_radius::TF = 0.1
    v_min::TF = 0.8
    v_max::TF = 1.0
end

function get_reference_path(gk::GKDubinsWezes2D)
    return gk.reference_path
end

function get_offset(gk::GKDubinsWezes2D)
    return gk.offset
end

function get_obstacles(gk::GKDubinsWezes2D)::Vector{AbstractWez}
    return gk.wezes
end

##################################################################
# GATEKEEPER PROBLEM INTERFACE IMPLEMENTATION
##################################################################
"""
    get_reference_state_and_input(problem::GatekeeperProblem, path, time)

Gets the reference state and input at some time along the path.

Directly calls the method 'get_reference_state_and_input defined in the 
2d_tracking_controller.jl file
"""
function get_reference_state_and_input(
    gk::GKDubinsWezes2D,
    path::Vector{DubinsPath}, # path
    time::F, # time
)::Tuple{SVector{3,F},SVector{2,F}} where {F<:Real}
    return get_reference_state_and_input(path, time)
end

"""
    get_reference_state_and_input(problem::GatekeeperProblem, path, offset, time)

Gets the reference state and input at some time along the path given the offset
Directly calls the method 'get_reference_state_and_input defined in the
2d_tracking_controller.jl file
"""
function get_reference_state_and_input(
    gk::GKDubinsWezes2D,
    path::AbstractVector{DubinsPath},
    offset::AbstractVector{F},
    time::F,
)::Tuple{SVector{3,F},SVector{2,F}} where {F<:Real}
    return get_reference_state_and_input(path, time, offset)
end

"""
    tracking_controller(problem::GatekeeperProblem, state, state_desired, input_desired)

Directly calls the method 'tracking_controller defined in the 
2d_tracking_controller.jl file
"""
function tracking_controller(
    gk::GKDubinsWezes2D,
    curr_state::AbstractVector{F}, # Current State
    desired_state::SVector{3,F}, # Desired State
    desired_input::SVector{2,F}, # Desired Input
)::SVector{2,F} where {F<:Real}
    return tracking_controller(curr_state, desired_state, desired_input)
end


function apply_input_bounds(
    gk::GKDubinsWezes2D,
    inputs::SVector{2,F}, # inputs
)::SVector{2,F} where {F<:Real}
    # maximum velocity over minimum turning radius
    ω_max = gk.v_max / gk.turning_radius

    # Apply input bounds
    v = clamp(inputs[1], gk.v_min, gk.v_max)
    ω = clamp(inputs[2], -ω_max, ω_max)

    return SVector(v, ω)
end

"""
    state_dynamics!(problem::GatekeeperProblem, D, state, inputs)

Defines the dynamics of the system. Modifies D in place with the system dynamics
"""
function state_dynamics!(
    gk::GKDubinsWezes2D,
    D, # D
    state::AbstractVector{F}, # Current State
    input::AbstractVector{F}, # Inputs
) where {F<:Real}
    # Extract inputs
    v = input[1]
    ω = input[2]

    # Modifies D in place
    D[1] = v * cos(state[3])
    D[2] = v * sin(state[3])
    D[3] = ω

    return
end


"""
    path_length(problem::GatekeeperProblem, path)

Return the length of the path using the metric defined by the problem
"""
function path_length(gk::GKDubinsWezes2D, p::Dubins.DubinsPath)
    return dubins_path_length(p)
end

function path_length(gk::GKDubinsWezes2D, p::Vector{Dubins.DubinsPath})
    len = 0.0
    for path in p
        len += dubins_path_length(path)
    end

    return len
end

"""
    shortest_path(problem::GatekeeperProblem, from_state::ST, to_state::ST)

Find the shortest path between two states using the architecture/method defined by the problem
"""
function shortest_path(
    gk::GKDubinsWezes2D,
    from_state::AbstractVector{F},
    to_state::AbstractVector{F},
) where {F<:Real} # State Type
    errcode, path =
        dubins_shortest_path(Vector(from_state), Vector(to_state), gk.turning_radius)
    @assert errcode == Dubins.EDUBOK

    return path
end

"""
    construct_reconnection_sites(problem::GatekeeperProblem, step_size::Float64)

along a reference path, find the reconnection sites, aka the set of points where a trajectory could possible reconnect

Returns a Vector of points along the reference path containing the index of the refernece path where the reconnection
coudl happen, and teh state
"""
function construct_reconnection_sites(
    gk::GKDubinsWezes2D,
    reconnection_step_size::F,
)::Vector{Tuple{Int64,SVector{3,F}}} where {F<:Real} # ST is state type
    # Get a list of reconnections on each segment
    reconnection_sites = Tuple{Int64,SVector{3,F}}[]

    for (path_idx, path) in enumerate(get_reference_path(gk))
        errcode, pts = dubins_path_sample_many_with_endpoint(path, reconnection_step_size)
        @assert errcode == Dubins.EDUBOK

        for p in pts
            push!(reconnection_sites, (path_idx, p))
        end
    end

    return reconnection_sites
end # reconnection step size

"""
    get_remaining_nominal(problem::GatekeeperProblem, path_idx::Int64)

Returns the remaining nominal path from the reference path, starting at path_idx to the end
"""
function get_remaining_nominal(
    prob::GKDubinsWezes2D,
    path_idx::Int64,
    connection_pt::AbstractVector{F},
) where {F<:Real}

    # Construct path upto end of the segment
    errcode, segment_end_pt = dubins_path_endpoint(get_reference_path(prob)[path_idx])
    @assert errcode == Dubins.EDUBOK

    errcode, segment_path = dubins_shortest_path(
        Vector(connection_pt),
        Vector(segment_end_pt),
        prob.turning_radius,
    )
    @assert errcode == Dubins.EDUBOK

    # add the remaining segments
    remaining_path = get_reference_path(prob)[path_idx+1:end]

    return vcat(segment_path, remaining_path)
end


###################################################################
# DUBINS HELPERS
###################################################################
function dubins_path_sample_many_with_endpoint(path::DubinsPath, step_size::Float64)

    configurations = SVector{3,Float64}[]
    x = 0.0
    L = dubins_path_length(path)

    (step_size <= 0) && return (EDUBPARAM, nothing)

    # sample along the steps
    if (step_size < L)
        while x < L
            errcode, q = Dubins.dubins_path_sample(path, x)
            push!(configurations, q)
            @assert errcode == Dubins.EDUBOK
            (errcode != 0) && (return errcode, nothing)
            x += step_size
        end
    end

    # sample the endpoint
    errcode, q = Dubins.dubins_path_sample(path, L)
    @assert errcode == Dubins.EDUBOK
    (errcode != 0) && (return errcode, nothing)
    push!(configurations, q)

    return EDUBOK, configurations
end