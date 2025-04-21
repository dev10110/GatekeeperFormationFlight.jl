# Copyright (c) 2025 Devansh R Agrawal - All rights reserved.

using Dubins
using StaticArrays, LinearAlgebra
using OrdinaryDiffEq, DiffEqCallbacks

"""
    GatekeeperProblem(kwargs...)

construct a `GatekeeperProblem`. The arguments are:
- wezes,                              # list of wezes
- reference_path,                     # path of the leader
- offset = SVector(0.0, 0.0, 0.0),    # desired offset from the leaders path
- turning_radius = 0.1,               # max turning radius of a robot
- max_Ts_horizon = 1.0,               # maximum switching time
- reconnection_step_size = 0.1,       # resolution used for checking reconnection point
- collision_check_step_size = 0.01,   # resolution used for checking collision along a path
- integration_max_step_size = 0.05,   # max integration step size in nominal tracking
- switch_step_size = 0.05             # resolution used to decrease switch time
"""
@kwdef struct GatekeeperProblem{TW,TR,TO,TF}
    wezes::TW                            # list of wezes
    reference_path::TR                   # path of the leader
    offset::TO = SVector(0.0, 0.0, 0.0)  # desired offset from the leaders path
    turning_radius::TF = 0.1             # max turning radius of a robot
    max_Ts_horizon::TF = 1.0             # maximum switching time
    reconnection_step_size::TF = 0.1     # resolution used for checking reconnection point
    collision_check_step_size::TF = 0.01   # resolution used for checking collision along a path
    integration_max_step_size::TF = 0.05   # max integration step size in nominal tracking
    switch_step_size::TF = 0.05            # resolution used to decrease switch time
end

Base.show(io::IO, prob::GatekeeperProblem) = print(
    io,
    "Gatekeeper Problem(|wezes|: ",
    length(prob.wezes),
    ", offset: ",
    prob.offset,
    ", turning radius: ",
    prob.turning_radius,
    ")",
)
Base.show(io::IO, ::MIME"text/plain", prob::GatekeeperProblem) = print(
    io,
    "Gatekeeper Problem\n",
    " - Number of wezes: ",
    length(prob.wezes),
    "\n - offset: ",
    prob.offset,
    "\n - turning radius: ",
    prob.turning_radius,
)


"""
    CompositeTrajectory(nominal_trajectory, backup_trajectory, switch_time)

defines a composite trajectory that includes a nominal trjectory, a backup trajectory, and a switch time
"""
struct CompositeTrajectory{TN,TB,TS}
    nominal_trajectory::TN
    backup_trajectory::TB
    switch_time::TS
end


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

"""
    construct_reconnection_sites(reference_path::Vector{DubinsPath}, reconnection_step_size=0.01)

along a reference path, define the set of points where a trajectory can possibly reconnect. Returns a `Vector{Tuple(Int64, SVector{3, Float64})}`. 
Each entry of this vector contains first the index of the reference path where the reconnection can happen, and second the state of reconnection. 
"""
function construct_reconnection_sites(
    reference_path::Vector{DubinsPath},
    reconnection_step_size = 0.01,
)

    # get a list of reconnections on each segment
    reconnection_sites = Tuple{Int64,SVector{3,Float64}}[] # list of [(segment_idx, sample point)]

    for (path_idx, path) in enumerate(reference_path)

        errcode, pts = dubins_path_sample_many_with_endpoint(path, reconnection_step_size)
        @assert errcode == EDUBOK

        for p in pts
            push!(reconnection_sites, (path_idx, p))
        end
    end
    return reconnection_sites
end


"""
    construct_reconnection_paths(reconnection_sites, initial_state, turning_radius)

given a list of reconnection sites, and the state from which we want to start, this function returns a list of dubins paths that connect to the reconnection sites. 
"""
function construct_reconnection_paths(reconnection_sites, initial_state, turning_radius)

    N = length(reconnection_sites)
    connection_paths = Vector{DubinsPath}(undef, N)

    for i = 1:N
        connection_pt = reconnection_sites[i][2]
        errcode, path = dubins_shortest_path(initial_state, connection_pt, turning_radius)
        @assert errcode == EDUBOK
        connection_paths[i] = path
    end
    return connection_paths

end

"""
    construct_candidate_backup_trajectory(initial_state, prob::GatekeeperProblem)

from some initial state, this function construsts the best backup trajectory
"""
function construct_candidate_backup_trajectory(initial_state, prob::GatekeeperProblem)
    # returns nothing if it fails to find a path

    # the reference path has a set of set of dubins segments
    reconnection_sites =
        construct_reconnection_sites(prob.reference_path, prob.reconnection_step_size)

    # find connections paths
    connection_paths =
        construct_reconnection_paths(reconnection_sites, initial_state, prob.turning_radius)

    # find the index of the shortest connection
    shortest_idx = argmin(dubins_path_length(p) for p in connection_paths)

    # check if a reconnection is safe wrt to the wezes, return the best
    for connection_idx = shortest_idx:length(connection_paths)

        # check for safety along the connection
        connection_path = connection_paths[connection_idx]

        if !is_colliding(prob.wezes, connection_path, prob.collision_check_step_size)

            # construct upto the end of this segment
            path_idx, connection_pt = reconnection_sites[connection_idx]
            errcode, segment_end_pt = dubins_path_endpoint(prob.reference_path[path_idx])
            @assert errcode == Dubins.EDUBOK

            errcode, segment_path =
                dubins_shortest_path(connection_pt, segment_end_pt, prob.turning_radius)
            @assert errcode == Dubins.EDUBOK

            # add the remaining segments
            remaining_path = prob.reference_path[(path_idx+1):end]

            # construct the full backup path
            full_path = vcat(connection_path, segment_path, remaining_path)

            # # check the full backup path (not needed if the reference path is safe)
            # if !path_is_collision_free(full_path, prob.wezes, prob.collision_check_step_size)
            #     continue
            # end

            return full_path
        end
    end

    return nothing

end

"""
    closed_loop_tracking_nominal!(D, state, prob::GatekeeperProblem, time)

defines the closed_loop dynamics for tracking the nominal trajectory
"""
function closed_loop_tracking_nominal!(D, state, prob::GatekeeperProblem, time)

    # determine a desired state for this robot
    state_desired, input_desired =
        get_reference_state_and_input(prob.reference_path, time, prob.offset)

    # run a tracking controller for this robot
    v, ω = tracking_controller(state, state_desired, input_desired)

    # apply input bounds
    v, ω = apply_input_bounds(v, ω)
    # works better if v_max = 1.05 or greater

    # follower dynamics
    D[1] = v * cos(state[3])
    D[2] = v * sin(state[3])
    D[3] = ω

    return
end

"""
    closed_loop_tracking_backup!(D, state, backup_path::Vector{DubinsPath}, backup_time)

defines the closed_loop dynamics for tracking the backup trajectory
"""
function closed_loop_tracking_backup!(
    D,
    state,
    backup_path::Vector{DubinsPath},
    backup_time,
)

    # determine a desired state for this robot
    state_desired, input_desired = get_reference_state_and_input(backup_path, backup_time)

    # run a tracking controller for this robot
    v, ω = tracking_controller(state, state_desired, input_desired)

    # apply input bounds
    v, ω = apply_input_bounds(v, ω)

    # follower dynamics
    D[1] = v * cos(state[3])
    D[2] = v * sin(state[3])
    D[3] = ω

    return
end

# simulate tracking the composite trajectory
"""
    closed_loop_tracking_composite!(D, state, params, time)

defines the closed_loop dynamics for tracking a composite trajectory
"""
function closed_loop_tracking_composite!(D, state, params, time)

    prob = params[1]
    committed_traj = params[2]

    # find the switch time of the committed trajectory 
    Ts = committed_traj.switch_time

    # if before switch time, run the normal tracking controller
    if time <= Ts
        return closed_loop_tracking_nominal!(D, state, prob, time)
    else
        # if after, run a tracking controller on the backup path
        return closed_loop_tracking_backup!(
            D,
            state,
            committed_traj.backup_trajectory,
            time - Ts,
        )
    end
    return
end


"""
    terminate_condition(state,time,integrator)

callback to terminate integration when there is a collision
"""
function terminate_condition(state, time, integrator)

    # create the robot
    robot = Robot(state...)

    # get all the wezes
    wezes = integrator.p.wezes

    # get min distance to a wez
    return minimum(collision_distance(wez, robot) for wez in wezes)

end

"""
    construct_candidate_nominal_trajectory(time, state, prob::GatekeeperProblem)

does a closed-loop simulation of the nominal controller until either the time runs out or until it hits a wez
"""
function construct_candidate_nominal_trajectory(time, state, prob::GatekeeperProblem)

    # check if the initial condition is already in a wez
    if is_colliding(prob.wezes, Robot(state))
        @warn "Robot is already in a wez. Collision distance: $(collision_distance(prob.wezes, Robot(state...)))"
        return nothing
    end

    # setup the ode problem
    tspan = (time, time + prob.max_Ts_horizon)
    params = prob

    odeproblem = ODEProblem(closed_loop_tracking_nominal!, Vector(state), tspan, params)

    # make it stop if it enters the wez
    termination_callback = ContinuousCallback(terminate_condition, terminate!)

    # solve the odeproblem
    odesol = solve(
        odeproblem,
        Tsit5();
        dt = 0.001,
        dtmax = prob.integration_max_step_size,
        callback = termination_callback,
    )

    return odesol

end


"""
    construct_candidate_trajectory(time, state, prob::GatekeeperProblem)

constructs a nominal trajectory, and working backwards tries to construct backup trajectories. 
If a safe trajectory is found, this is returned as the candidate trajectory. 
if not, `nothing` is returned.
"""
function construct_candidate_trajectory(time, state, prob::GatekeeperProblem)

    # construct the nominal trajectory
    nominal_solution = construct_candidate_nominal_trajectory(time, state, prob)

    if isnothing(nominal_solution)
        return nothing
    end

    # starting at the end, work backwards and see which is the best switching time
    nominal_end_time = nominal_solution.t[end]

    for switch_time in
        range(start = nominal_end_time, stop = time, step = -prob.switch_step_size)

        # get the state at the switch time
        nominal_end_state = nominal_solution(switch_time)

        # construct a backup from this switch time
        backup_path =
            construct_candidate_backup_trajectory(SVector{3}(nominal_end_state...), prob)

        # if backup was found, return it
        if !isnothing(backup_path)

            # construct the composite trajectory
            candidate_traj = CompositeTrajectory(nominal_solution, backup_path, switch_time)
            return candidate_traj
        end
    end

    return nothing
end

# construct a callback to update the committed trajectory 
"""
    update_committed_affect!(integrator)

callback to recompute the committed trajectory. Will replace the integrator's parameter if a new committed trajectory is found
"""
function update_committed_affect!(integrator)
    # get the current time, state, params
    time = integrator.t
    state = integrator.u
    params = integrator.p

    # grab the reference traj, offsets
    prob = params[1]
    committed_trajectory = params[2]

    #     println("** Update committed called at t=$(time) **")

    # construct a new candidate trajectory from the current state
    candidate_trajectory = construct_candidate_trajectory(time, state, prob)

    if !isnothing(candidate_trajectory)
        integrator.p[2] = candidate_trajectory
        #         println("  -> Successfully created new committed trajectory. Next switch time: $(candidate_trajectory.switch_time)")
    else
        #         println("  -> no new committed trajectory")
        NaN
    end
    return
end

# construct a callback to update the committed trajectory 
function time_choice_gatekeeper(integrator)

    # get the current time, state, params
    time = integrator.t
    state = integrator.u
    params = integrator.p

    # grab the reference traj, offsets
    prob = params[1]
    committed_trajectory = params[2]



    return max(time, committed_trajectory.switch_time) + prob.switch_step_size
end

"""
    simulate_closed_loop_gatekeeper(initial_state, tspan, prob::GatekeeperProblem)
    
returns the closed-loop trajectory from running gatekeeper
"""
function simulate_closed_loop_gatekeeper(initial_state, tspan, prob::GatekeeperProblem)

    # first check that the reference path is indeed safe
    if is_colliding(prob.wezes, prob.reference_path, 0.1 * prob.collision_check_step_size)
        @warn "Reference path is not safe!!"
    end

    # construct the first committed trajectory
    first_candidate_trajectory =
        construct_candidate_trajectory(tspan[1], initial_state, prob)
    if isnothing(first_candidate_trajectory)
        @warn "Could not create first candidate trajectory"
        return nothing
    end

    # setup the odeproblem
    params = [prob, first_candidate_trajectory]

    odeproblem =
        ODEProblem(closed_loop_tracking_composite!, Vector(initial_state), tspan, params)

    # update_committed_callback = PeriodicCallback(update_committed_affect!, prob.switch_step_size)
    update_committed_callback =
        IterativeCallback(time_choice_gatekeeper, update_committed_affect!)

    odesol_gatekeeper = solve(
        odeproblem,
        Tsit5();
        dtmax = 0.001, # prob.integration_max_step_size, 
        dt = 0.001,
        callback = update_committed_callback,
    )

    return odesol_gatekeeper

end

# function plot_composite_traj!(traj::CompositeTrajectory; label="candidate", nominal_color=:blue, backup_color=:green, kwargs...)

#     # extract the switch time
#     Ts = traj.switch_time

#     # plot the nominal part of the candidate
#     nominal = traj.nominal_trajectory
#     plot!(
#         t->nominal(t)[1], t->nominal(t)[2], 
#         nominal.t[1], Ts, label="$(label) (nominal)"; 
#         color=nominal_color, kwargs...
#     )

#     # plot the backup part of the candidate
#     backup = traj.backup_trajectory
#     for (i, p) in enumerate(backup)
#         Plots.plot!(p; color=backup_color, label=(i==1 ? "$(label) (backup)" : false), kwargs...)
#     end

#     # create some markers
#     start_state = nominal.u[1]
#     switch_state = nominal(Ts)
#     _, end_state = dubins_path_endpoint(backup[end])
#     key_states = (start_state, switch_state, end_state)
#     scatter!([s[1] for s in key_states], [s[2] for s in key_states], color=:black, label=false)

# end
