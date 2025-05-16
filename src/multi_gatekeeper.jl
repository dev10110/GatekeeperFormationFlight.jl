"""
File: src/multi_gatekeeper.jl

Extension of gatekeeper.jl to simulate multiple agents at once
"""
using Dubins3D
using StaticArrays, LinearAlgebra

module MultiGatekeeper

export MultiGatekeeperProblem, get_single_agent_subproblem, get_collision_radius


# ========== TYPE DEFINITIONS ==========
abstract type MultiGatekeeperProblem <: GatekeeperProblem end
function get_single_agent_subproblem(
    gk::MultiGatekeeperProblem,
    agent_idx::Int,
)::GatkeeperProblem end

"""
    get_single_agent_subproblem(gk::MultiGatekeeperProblem, agent_idx::Int, committed_traj::CompositeTrajectory)

Returns the single agent subproblem for the given agent index. Use the committed trajectories
    of all agents != agent_idx as dynamic obstacles
"""
function get_single_agent_subproblem(
    gk::MultiGatekeeperProblem,
    agent_idx::Int,
    committed_traj::CompositeTrajectory,
)::GatekeeperProblem end


function get_agent_collision_radius(gk::MultiGatekeeperProblem)::Float64
    return gk.interagent_collision_radius
end


# ========== PUBLIC API ==========

"""
    simulate_closed_loop_gatekeeper(gk::MultiGatekeeperProblem, initial_state, timespan)
"""
function simulate_closed_loop_gatekeeper(
    gk::GatekeeperInstance{GP},
    initial_state::AbstractMatrix,
    timespan,
) where {GP<:MultiGatekeeperProblem}
    n_agents::Int = size(initial_state, 1)
    @info "Simulating closed-loop gatekeeper algorithm for $(n_agents) agents"

    first_candidate_trajectory =
        construct_candidate_trajectory(gk, initial_state, timespan[1])

    if isnothing(first_candidate_trajectory)
        @error "No candidate trajectory found. Cannot proceed with simulation."
        return nothing
    end

    params = (gk, first_candidate_trajectory)

    odeproblem =
        ODEProblem(multi_closed_loop_tracking_composite!, initial_state, timespan, params)

    update_committed_callback =
        IterativeCallback(min_switch_time_gatekeeper, update_agent_committed_callback!)

    odesol_gatekeeper = solve(
        odeproblem,
        Tsit5(),
        dtmax = gk.coefficients.integration_max_step_size,
        dt = gk.coefficients.integration_step_size,
        callback = update_committed_callback,
    )

    return odesol_gatekeeper
end

# ========== PRIVATE FUNCTIONS ==========


####################################
# MAIN ODE SOLVER CALLBACKS
####################################

"""
    min_switch_time_gatekeeper(integrator)

Returns the next time to attempt to update the committed trajectory. As long as

If time >= min(switch_time), then all agents are in the backup trajectory phase, so
we should attempt to update the committed trajectory at the next multiple of the switch_step_size
aka as soon as possible.

If time < min(switch_time) then all agents are in the nominal trajectory phase. Thus, there is no
need to update the committed trajectory yet, so the next time to update is the min of the switch times + the step size
"""
function min_switch_time_gatekeeper(integrator)
    time = integator.t
    params = integrator.p
    gk, committed_traj = integrator.p

    return max(time, minimum(committed_traj.switch_time)) + gk.coefficients.switch_step_size
end

"""
    update_agent_committed_callback!(integrator)

If this function is called that means at least one agent has reached the switch time.
"""
function update_agent_committed_callback!(integrator)
    time = integrator.t
    state = integrator.u
    gk, committed_traj = integrator.p

    # Loop through all agents, and check if they have reached their switch time
    n_agents::Int = size(state, 1)
    for agent_idx = 1:n_agents
        if time >= committed_traj.switch_time[agent_idx]

            # If they have, then we want to update that agent's committed trajectory
            # But updating the agent's candidate effectrively updates the entire candidate
            try_update_agent_committed!(committed_traj, gk, state, agent_idx, time)

            # So if we were able to successfully update the agent's trajectory, then here we commit it,
            # and then use this updated committed trajectory for the rest of the agents
            if !isnothing(updated_candidate)
                committed_traj = updated_candidate
            end
        end
    end

    # No matter what, the committed_trajectory is either the same or updated in the function above
    integrator.p = (gk, committed_traj)
    return
end

"""
    multi_closed_loop_tracking_composite!(D, state, params, time)

Mutating function that defines the closed loop dynamics for many identical agents tracking
independent composite trajectories.

Effectively converts the multi-agent problem into many single agent problems to re-use code
from single-agent problems
"""
function multi_closed_loop_tracking_composite!(D, state, params, time::F)
    gk, committed_traj = params
    Ts = committed_traj.switch_time

    n_agents::Int = size(state, 1)

    for agent = 1:n_agents
        if time <= Ts[agent]
            closed_loop_tracking_nominal!(
                view(D, agent),
                state[agent],
                get_single_agent_subproblem(gk.problem, agent),
                time,
            )
        else
            closed_loop_tracking_backup!(
                view(D, agent),
                get_single_agent_subproblem(gk.problem, agent),
                state[agent],
                committed_traj.backup[agent],
                time - Ts[agent],
            )
        end
    end
end

####################################
# More fine grained implementation details
####################################

"""
    try_update_agent_committed!(committed_traj, gk, state, agent_idx, time)

Attempt to update the committed trajectory for a single agent. Modifies committed_traj
    in place if successful. Effectively, considers all other agent's committed
    trajectories as dynamic obstacles, and attempts to find a new candidate trajectory
    for the agent that is collision free with respect to all obstacles and other agents

More or less solve the single agent problem with the other agents as dynamic obstacles
"""
function try_update_agent_committed!(
    committed_traj::CompositeTrajectory,
    gk::GatekeeperInstance{GP},
    state,
    agent_idx::Int,
    time,
) where {GP<:MultiGatekeeperProblem}

    agent_gk = GatekeeperInstance(
        get_single_agent_subproblem(gk.problem, agent_idx, committed_traj),
        gk.coefficients,
    )

    # Get the agent's nominal trajectory
    nominal_solution =
        construct_candidate_nominal_trajectory(agent_gk, state[agent_idx], time)

    nominal_end_time = nominal_solution.t[end]

    for switch_time in range(
        start = nominal_end_time,
        stop = time,
        step = -gk.coefficients.switch_step_size,
    )
        nominal_end_state = nominal_solution(switch_time)
        backup_path = construct_candidate_backup_trajectory(agent_gk, nominal_end_state)

        if !isnothing(backup_path)
            # Update the committed trajectory in place
            committed_traj.nominal[agent_idx] = nominal_solution
            committed_traj.backup[agent_idx] = backup_path
            committed_traj.switch_time[agent_idx] = switch_time
            return
        end
    end
end


function construct_candidate_backup_trajectory(
    gk::GatekeeperInstance{GP},
    committed_traj::CompositeTrajectory,
    state,
    time,
    agent_idx::Int,
) where {GP<:MultiGatekeeperProblem}

    # Get the set of reconnection sites
    reconnection_sites =
        construct_reconnection_sites(gk.problem, gk.coefficients.reconnection_step_size)

    # find paths
    connection_paths =
        construct_reconnection_paths(gk, reconnection_sites, state[agent_idx])

    shortest_path = argmin(path_length(gk.problem, p) for p in connection_paths)

    # Check if the path is safe, and return the first that is
    for connection_idx = shortest_idx:length(connection_paths)
        # Get the path
        path_candidate = connection_paths[connection_idx]

        # Check if the path collides with any static obstacles
        if is_colliding(
            get_obstacles(gk.problem),
            path_candidate,
            gk.coefficients.collision_check_step_size,
        )
            continue
        end

        # If the path is safe from static obstacles, we must now forward propagate the 
        # state, using the backup trajectory, until the agent reaches the reconnection point
        agent_backup = propagate_agent_as_backup(
            gk,
            committed_traj,
            state,
            time,
            agent_idx,
            path_candidate,
        )
    end
end

"""
    propagate_agent_as_nominal(gk::GatekeeperInstance{GP}, committed_traj::CompositeTrajectory, state, time, agent_idx::Int)

Propagates the agent's trajectory using the nominal controller, and the rest using the full
composite trajectory tracker. 

Does this until there is a collision (caused by the agent's trajectory)

Returns:
1. An ODESolution object that contains the trajectory of the system.
"""
function propagate_agent_as_nominal(
    gk::GatekeeperInstance{GP},
    committed_traj::CompositeTrajectory,
    state,
    time,
    agent_idx::Int,
) where {GP<:MultiGatekeeperProblem}

    # Set up the ODE Problem
    tspan = (time, time + gk.coefficients.max_Ts_horizon)
    params = (gk, committed_traj)

    """
        ode_func!(D, state, params, time)

    This is, unfortunately, just the same as the multi_closed_loop_tracking_composite!
    function, but we need to define it here as a closure so that we can require it use
    the nominal controller for the agent we are propagating
    """
    n_agents::Int = size(state, 1)
    Ts = committed_traj.switch_time
    function ode_func!(D, s, p, t::F) where {F<:Real}
        for agent = 1:n_agents
            # alyways use the nominal controller for the agent
            # we are propagating
            if t <= Ts[agent] || agent == agent_idx
                closed_loop_tracking_nominal!(
                    view(D, agent),
                    s[agent],
                    get_single_agent_subproblem(gk.problem, agent),
                    t,
                )
            else
                closed_loop_tracking_backup!(
                    view(D, agent),
                    get_single_agent_subproblem(gk.problem, agent),
                    s[agent],
                    committed_traj.backup[agent],
                    t - Ts[agent],
                )
            end
        end
    end

    """
        termination_condition(s, t, i)

    Another unforunately very similar block of code to the termination_condition function
    But also required to be defined here as a closure so we can only consider one agent in the
    collision checking, as well as consider itner-agent collisions

    Returns the minimum distance from the agent to any static obstacles or other agent at that current timestep
    """
    function ode_termination_condition(s, t, i)
        # only need to check for collisions for the agent we are propagating
        # as the rest are following the by construction safe committed trajectory

        min_dist_to_obstacle = collision_distance(get_obstacles(gk.problem), s[agent_idx])
        min_dist_to_other_agents = minimum(
            norm(s[agent_idx] .- s[agent]) - 2 * get_collision_radius(gk.problem) for
            agent = 1:n_agents if agent != agent_idx
        )

        return min(dist_to_obstacles, dist_to_other_agents)
    end

    odeproblem = ODEProblem(ode_func!, state, tspan, params)
    termination_callback = ContinuousCallback(ode_termination_condition, terminate!)

    odesol = solve(
        odeproblem,
        Tsit5(),
        dtmax = gk.coefficients.integration_max_step_size,
        dt = gk.coefficients.integration_step_size,
        callback = termination_callback,
    )

    return odesol
end

"""
    propagate_agent_as_backup(gk::GatekeeperInstance{GP}, committed_traj::CompositeTrajectory, state, time, agent_idx::Int)

Propagates the agent's trajectory using the backup controller, and the rest using the full
composite trajectory tracker. Terminates the propagation when the agent reaches the reconnection point
or collides
"""
function propagate_agent_as_backup(
    gk::GatekeeperInstance{GP},
    committed_traj::CompositeTrajectory,
    state,
    time,
    agent_idx::Int,
    path_candidate,
) where {GP<:MultiGatekeeperProblem}
    # TODO
    return nothing
end

end # module MultiGatekeeper

