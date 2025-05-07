using StaticArrays, LinearAlgebra
using OrdinaryDiffEq, DiffEqCallbacks


"""
    GatekeeperProblem

Abstract type for the underlying gatekeeper problem. I.e. 2D, 3D, etc. 
"""
abstract type GatekeeperProblem end

"""
    GETTERS AND SETTERS AS NECESSARY
"""

"""
    get_reference_path(problem::GatekeeperProblem)

returns problem.reference_path (or whatever it is called)
"""
function get_reference_path(::GatekeeperProblem)
    throw("Not Implemented")
end

function get_obstacles(::GatekeeperProblem)::Vector{AbstractObstacle}
    throw("Not Implemented")
end

function get_offset(::GatekeeperProblem)
    throw("Not Implemented")
end


"""
    get_reference_state_and_input(problem::GatekeeperProblem, path, time)

Gets the reference state and input at some time along the path
"""
function get_reference_state_and_input(
    ::GatekeeperProblem,
    ::Any, # path
    ::F, # time
)::Tuple{ST,IT} where {F<:Real}
    throw("Not implemented")
end # State Type, Input Type

"""
    tracking_controller(problem::GatekeeperProblem, state, state_desired, input_desired)
"""
function tracking_controller(
    ::GatekeeperProblem,
    ::ST, # Current State
    ::ST, # Desired State
    ::IT, # Desired Input
)::ST where {ST,IT} # State Type, Input Type
    throw("Not implemented")
end


function apply_input_bounds(
    ::GatekeeperProblem,
    ::Any, # inputs
)
    throw("Not implemented")
end

"""
    state_dynamics!(problem::GatekeeperProblem, D, state, inputs)

Defines the dynamics of the system. Modifies D in place with the system dynamics
"""
function state_dynamics!(
    ::GatekeeperProblem,
    ::Any, # D
    ::ST, # Current State
    ::IT, # Inputs
) where {ST,IT} # State Type, Input Type
    throw("Not implemented")
end

"""
    path_length(problem::GatekeeperProblem, path)

Return the length of the path using the metric defined by the problem
"""
function path_length(::GatekeeperProblem, ::Any)
    throw("Not implemented")
end

"""
    shortest_path(problem::GatekeeperProblem, from_state::ST, to_state::ST)

Find the shortest path between two states using the architecture/method defined by the problem
"""
function shortest_path(::GatekeeperProblem, from_state::ST, to_state::ST) where {ST} # State Type
    throw("Not implemented")
end

"""
    construct_reconnection_sites(problem::GatekeeperProblem, step_size::Float64)

along a reference path, find the reconnection sites, aka the set of points where a trajectory could possible reconnect

Returns a Vector of points along the reference path containing the index of the refernece path where the reconnection
coudl happen, and teh state
"""
function construct_reconnection_sites(
    ::GatekeeperProblem,
    ::F,
)::Vector{Tuple{Int64,ST}} where {F<:Real} # ST is state type
    throw("Not implemented")
end # reconnection step size

"""
    get_remaining_nominal(problem::GatekeeperProblem, path_idx::Int64)

Returns the remaining nominal path from the reference path, starting at path_idx to the end
"""
function get_remaining_nominal(::GatekeeperProblem, path_idx::Int64, connection_pt)
    throw("Not implemented")
end




"""
    GatekeeperCoefficients(kwargs...)

Class to contain the coefficients used in the general case for gatekeeper
- max_Ts_horizon = 1.0,               # maximum switching time
- reconnection_step_size = 0.1,       # resolution used for checking reconnection point
- collision_check_step_size = 0.01,   # resolution used for checking collision along a path
- integration_max_step_size = 0.05,   # max integration step size in nominal tracking
- switch_step_size = 0.05             # resolution used to decrease switch time
"""
@kwdef struct GatekeeperCoefficients{TF<:Real}
    max_Ts_horizon::TF = 1.0             # maximum switching time
    reconnection_step_size::TF = 0.1     # resolution used for checking reconnection point
    collision_check_step_size::TF = 0.01   # resolution used for checking collision along a path
    integration_max_step_size::TF = 0.05   # max integration step size in nominal tracking
    switch_step_size::TF = 0.05            # resolution used to decrease switch time
end

"""
    GatekeeperInstance
A struct to contain the problem and coefficients for the gatekeeper instance.
Functions that operate on this struct implement the high level gatekeeper algorithm
"""
struct GatekeeperInstance
    problem::GatekeeperProblem
    coefficients::GatekeeperCoefficients{TF} where {TF<:Real}
end

struct CompositeTrajectory{TN,TB,TS<:Real}
    nominal::TN
    backup::TB
    switch_time::TS
end


"""
    simulate_closed_loop_gatekeeper(gk::GatekeeperInstance, initial_state, timespan)

The high level driver function to simulate the closed look gatekeeper algorithm
"""
function simulate_closed_loop_gatekeeper(gk::GatekeeperInstance, initial_state, timespan)
    if is_colliding(
        get_obstacles(gk.problem),
        get_reference_path(gk.problem),
        gk.coefficients.collision_check_step_size,
    )
        @warn "Reference path is not safe !!"
    end

    first_candidate_trajectory =
        construct_candidate_trajectory(gk, initial_state, timespan[1]) # Initial composite trajectory

    params = [gk, first_candidate_trajectory]

    odeproblem =
        ODEProblem(closed_loop_tracking_composite!, Vector(initial_state), timespan, params)

    update_committed_callback =
        IterativeCallback(time_choice_gatekeeper, update_committed_affect!)

    odesol_gatekeeper = solve(
        odeproblem,
        Tsit5(),
        dtmax = gk.coefficients.integration_max_step_size,
        dt = 0.001,
        callback = update_committed_callback,
    )

    return odesol_gatekeeper
end


####################################
# ITERATIVE CALLBACK IMPLEMENTATION
####################################

"""
   time_choice_gatekeeper(integrator)

Callback to update the committed trajectory
"""
function time_choice_gatekeeper(integrator)
    time = integrator.t
    state = integrator.u
    params = integrator.p

    gk::GatekeeperInstance = params[1]
    committed_traj = params[2]

    return max(time, committed_traj.switch_time) + gk.coefficients.switch_step_size
end

function update_committed_affect!(integrator)
    # Get time, state, paramters
    time = integrator.t
    state = integrator.u
    params = integrator.p

    # Extract the parameters
    gk = params[1]
    committed_traj = params[2]

    # Attempt to construct a new candidate trajectory
    candidate_trajectory = construct_candidate_trajectory(gk, state, time)

    # If candidate trajectory is constructed successfully, update
    if !isnothing(candidate_trajectory)
        integrator.p[2] = candidate_trajectory
    end

    return
end


####################################
# TRACKING FUNCTIONS 
####################################

"""
    closed_loop_tracking_composite!(D, state, params, time)

defines the closed loop dynamics for trackign a composite trajectory
"""
function closed_loop_tracking_composite!(D, state, params, time::F) where {F<:Real}
    gk, committed_traj = params
    Ts = committed_traj.switch_time

    if time <= Ts
        return closed_loop_tracking_nominal!(D, state, gk, time)
    else
        return closed_loop_tracking_backup!(
            D,
            gk.problem,
            state,
            committed_traj.backup,
            time - Ts,
        )
    end
end

"""
    closed_loop_tracking_nominal!(D, state, gk::GatekeeperInstance, time)

Defines the closed loop dynamics for tracking a nominal trajectory. Called by closed_loop_tracking_composite!
and also by the ODE solver
"""
function closed_loop_tracking_nominal!(D, state, gk::GatekeeperInstance, time)
    # Determine the desired state and input
    state_desired, input_desired = get_reference_state_and_input(
        gk.problem,
        get_reference_path(gk.problem),
        get_offset(gk.problem),
        time,
    )

    inputs = tracking_controller(gk.problem, state, state_desired, input_desired)
    inputs = apply_input_bounds(gk.problem, inputs)

    state_dynamics!(gk.problem, D, state, inputs)
    return
end

"""
    closed_loop_tracking_backup!(D, prob::GatekeeperProblem, state, backup_path, backup_time)

Defines the closed loop dynamics for tracking a backup trajectory. Called by closed_loop_tracking_composite! 
"""
function closed_loop_tracking_backup!(
    D,
    prob::GatekeeperProblem,
    state,
    backup_path,
    backup_time,
)
    state_desired, input_desired =
        get_reference_state_and_input(prob, backup_path, backup_time)

    inputs = tracking_controller(prob, state, state_desired, input_desired)
    inputs = apply_input_bounds(prob, inputs)

    state_dynamics!(prob, D, state, inputs)
    return
end


####################################
# TRAJECTORY CONSTRUCTION
####################################

"""
    construct_candidate_trajectory(gk::GatekeeperInstance, state::ST, time::Float64)

Attempts to construct a candidate trajectory for the gatekeeper algorithm.
Returns a `CompositeTrajectory` if successful, or `nothing` if not.

This is the lowest level of abstraction that is not necessarily implementation specific
"""
function construct_candidate_trajectory(
    gk::GatekeeperInstance,
    state::ST,
    time::F, # time
)::Union{CompositeTrajectory,Nothing} where {ST,F<:Real} # State Type

    # Construct the nominal trajectory
    nominal_solution = construct_candidate_nominal_trajectory(gk, state, time)

    if isnothing(nominal_solution)
        return nothing
    end

    # Starting at the ned, work backwards to find the best switching time
    nominal_end_time = nominal_solution.t[end]

    for switch_time in range(
        start = nominal_end_time,
        stop = time,
        step = -gk.coefficients.switch_step_size,
    )

        # Get the state at the switch time
        nominal_end_state = nominal_solution(switch_time)

        # construct a backup from this switch time
        backup_path = construct_candidate_backup_trajectory(gk, Vector(nominal_end_state))

        # If found a backup path successfully, return it
        if !isnothing(backup_path)
            return CompositeTrajectory(nominal_solution, backup_path, switch_time)
        end

    end

    # If no backup path was found, return nothing
    return nothing
end

function termination_condition(state, time, integrator)
    return collision_distance(get_obstacles(integrator.p.problem), state)
    # return (dist < 1e-2) ? 0.0 : dist # address some numerical precision issues?
end

function construct_candidate_nominal_trajectory(
    gk::GatekeeperInstance,
    state::ST,
    time::F,
) where {ST,F<:Real} # State Type
    # Check if initial condition is safe
    if is_colliding(get_obstacles(gk.problem), state)
        @warn "robot is in collision. Collision distance: $(collision_distance(get_obstacles(gk.problem), state))"
        return nothing
    end

    # set up the ODE problem
    tspan = (time, time + gk.coefficients.max_Ts_horizon)
    params = gk

    # construct the ode problem
    odeproblem = ODEProblem(closed_loop_tracking_nominal!, Vector(state), tspan, params)

    termination_callback = ContinuousCallback(termination_condition, terminate!)

    odesol = solve(
        odeproblem,
        Tsit5(),
        dtmax = gk.coefficients.integration_max_step_size,
        dt = 0.001,
        callback = termination_callback,
    )

    return odesol
end

"""
    construct_candidate_backup_trajectory(gk::GatekeeperInstance, state::ST)

From some initial state, attempts to construct the best backup trajectory
"""
function construct_candidate_backup_trajectory(
    gk::GatekeeperInstance,
    state::ST,
)::Union{Any,Nothing} where {ST}

    # Get the set of reconnection sites
    reconnection_sites =
        construct_reconnection_sites(gk.problem, gk.coefficients.reconnection_step_size)

    # find connection paths
    connection_paths = construct_reconnection_paths(gk, reconnection_sites, state)

    # Find the index of the shortest connection
    shortest_idx = argmin(path_length(gk.problem, p) for p in connection_paths)

    # Check if a reconnection path is safe, and return the best
    for connection_idx = shortest_idx:length(connection_paths)
        # Get the path
        connection_path = connection_paths[connection_idx]

        # Would need an implementation of is_colliding for paths
        # For now assuming a check that works with the available methods
        if !is_colliding(
            get_obstacles(gk.problem),
            connection_path,
            gk.coefficients.collision_check_step_size,
        )
            # Construct the path 
            path_idx, connection_pt = reconnection_sites[connection_idx]
            remaining_nominal = get_remaining_nominal(gk.problem, path_idx, connection_pt)

            full_path = vcat(connection_path, remaining_nominal)

            return full_path
        end
    end

    return nothing
end

"""
    construct_reconnection_paths(problem::GatekeeperProblem, reconnection_sites::Vector{Tuple(Int64, ST)}, from_state::ST)

Given a set of reconnection sites, and an initial state, return a list of reconnection paths
"""
function construct_reconnection_paths(
    gk::GatekeeperInstance,
    reconnection_sites::Vector{Tuple{Int64,ST}},
    from_state,
)::Vector{Any} where {ST}
    # Get the shortest path from the initial state to each reconnection site as a vector
    connection_paths =
        map(site -> shortest_path(gk.problem, from_state, site[2]), reconnection_sites)

    return connection_paths
end