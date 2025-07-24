"""
File: src/multi_gatekeeper.jl

Extension of gatekeeper.jl to simulate multiple agents at once
"""
module MultiGatekeeper

export MultiGatekeeperProblem, get_single_agent_subproblem

using Dubins3D
using StaticArrays, LinearAlgebra
using OrdinaryDiffEq, DiffEqCallbacks

using ..Gatekeeper

# ========== TYPE DEFINITIONS ==========
abstract type MultiGatekeeperProblem <: GatekeeperProblem end
function get_single_agent_subproblem(
    gk::MultiGatekeeperProblem,
    agent_idx::Int,
)::GatekeeperProblem end

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
function Gatekeeper.simulate_closed_loop_gatekeeper(
    gk::GatekeeperInstance{GP},
    initial_state,
    timespan,
) where {GP<:MultiGatekeeperProblem}
    n_agents::Int = size(initial_state, 1)
    state_dim::Int = size(initial_state, 2)

    print_simulation_banner(n_agents, timespan)

    # Build candidate trajectory incrementally, agent by agent
    # Each agent greedily finds a candidate trajectory that is collision free
    candidate_trajectory = CompositeTrajectory(
        Vector{Any}(),  # or use the specific type if known
        Vector{Any}(),
        Vector{Float64}(),
    )

    valid_agents = 0

    for agent_idx = 1:n_agents
        @info "Constructing candidate trajectory for agent $(agent_idx) at x0 = $(initial_state[agent_idx, :])"

        # Add room to the candidate trajectory for the agent
        push!(candidate_trajectory.nominal, nothing)
        push!(candidate_trajectory.backup, nothing)
        push!(candidate_trajectory.switch_time, 0.0)

        success = try_update_agent_committed!(
            candidate_trajectory,
            gk,
            initial_state,
            agent_idx,
            timespan[1],
        )
        if !success
            @error "No candidate trajectory found for agent $(agent_idx). Cannot proceed with simulation."
            pop!(candidate_trajectory.nominal)
            pop!(candidate_trajectory.backup)
            pop!(candidate_trajectory.switch_time)
            break
            # return nothing, nothing
        end
        valid_agents += 1
    end

    if valid_agents == 0
        @error "No valid agents found. Cannot proceed with simulation."
        return nothing, nothing
    end

    @info "Candidate trajectory found for $(valid_agents) agents"

    params = (gk, candidate_trajectory)

    @show initial_state
    initial_state = initial_state[1:valid_agents, :]
    @show initial_state

    odeproblem =
        ODEProblem(multi_closed_loop_tracking_composite!, initial_state, timespan, params)

    update_committed_callback =
        IterativeCallback(min_switch_time_gatekeeper, update_agent_committed_callback!)

    saved_composites = SavedValues(Float64, CompositeTrajectory)
    saving_callback =
        SavingCallback((u, t, integrator) -> deepcopy(integrator.p[2]), saved_composites)

    callbacks = CallbackSet(update_committed_callback, saving_callback)

    odesol_gatekeeper = solve(
        odeproblem,
        Tsit5(),
        dtmax = gk.coefficients.integration_max_step_size,
        dt = gk.coefficients.integration_step_size,
        callback = callbacks,
    )

    @info "Simulation complete. Returning solution."

    return odesol_gatekeeper, saved_composites
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
    time = integrator.t
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
        end
    end

    # No matter what, the committed_trajectory is either the same or updated in the function above
    # integrator.p = (gk, committed_traj)
    return
end

"""
    multi_closed_loop_tracking_composite!(D, state, params, time)

Mutating function that defines the closed loop dynamics for many identical agents tracking
independent composite trajectories.

Effectively converts the multi-agent problem into many single agent problems to re-use code
from single-agent problems
"""
function multi_closed_loop_tracking_composite!(D, state, params, time::Float64)
    gk, committed_traj = params
    Ts = committed_traj.switch_time

    n_agents::Int = size(state, 1)

    for agent = 1:n_agents
        if time <= Ts[agent]
            Gatekeeper.closed_loop_tracking_nominal!(
                view(D, agent, :),
                state[agent, :],
                get_single_agent_subproblem(gk.problem, agent),
                time,
            )
        else
            Gatekeeper.closed_loop_tracking_backup!(
                view(D, agent, :),
                get_single_agent_subproblem(gk.problem, agent),
                state[agent, :],
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

Returns true if successful, false otherwise
"""
function try_update_agent_committed!(
    committed_traj::CompositeTrajectory,
    gk::GatekeeperInstance{GP},
    state,
    agent_idx::Int,
    time,
)::Bool where {GP<:MultiGatekeeperProblem}
    # Convert to a single agent problem with dynamic obstacles!
    agent_gk = GatekeeperInstance(
        get_single_agent_subproblem(gk.problem, agent_idx, committed_traj),
        gk.coefficients,
    )

    # Get the agent's nominal trajectory (until collision)
    nominal_solution = Gatekeeper.construct_candidate_nominal_trajectory(
        agent_gk,
        state[agent_idx, :],
        time,
    )

    mas_check_until_time = max([st for st in committed_traj.switch_time]..., 0.0)

    # @show mas_check_until_time =
    #     # max([x.switch_time for x in committed_traj if x !== nothing])

    if isnothing(nominal_solution)
        @warn "No nominal solution found for agent $(agent_idx) at time $(time). Cannot update committed trajectory."
        return false
    end

    nominal_end_time = nominal_solution.t[end]

    for switch_time in range(
        start = nominal_end_time,
        stop = time,
        step = -gk.coefficients.switch_step_size,
    )
        nominal_end_state = nominal_solution(switch_time)

        # TODO this might need to change
        # Construct the backup trajectory for the agent
        # Need to check that the backup doesn't collide with any other agents
        # Up to the time that all agents have returned to the leader trajectory
        backup_path = Gatekeeper.construct_candidate_backup_trajectory(
            agent_gk,
            nominal_end_state,
            switch_time,
            mas_check_until_time,
        )

        if !isnothing(backup_path)
            # Update the committed trajectory in place
            committed_traj.nominal[agent_idx] = nominal_solution
            committed_traj.backup[agent_idx] = backup_path
            committed_traj.switch_time[agent_idx] = switch_time
            return true
        end
    end

    @warn "Failed to find valid switch time. Returning nothing."
    return false
end

function filtered_candidate_backup_trajectory(
    gk::GP,
    state::ST,
    agent_idx::Int,
    committed_traj::CompositeTrajectory,
) where {GP<:GatekeeperProblem,ST}

end

function print_simulation_banner(n_agents::Int, timespan)
    println("===============================================")
    println("      Multi-Agent Gatekeeper Simulation        ")
    println("===============================================")
    println("Number of agents: $n_agents")
    println("Timespan: [$(timespan[1]), $(timespan[2])]")
    println("===============================================")
end

end # module MultiGatekeeper

