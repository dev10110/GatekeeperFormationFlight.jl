module JointTrajOpt3D

using GatekeeperFormationFlight
using LinearAlgebra, StaticArrays
using OrdinaryDiffEq, DiffEqCallbacks
using InfiniteOpt, Ipopt, JuMP
using Interpolations

GFF = GatekeeperFormationFlight
TC = GFF.Dubins3DTrackingController

sumsquares(x) = sum(xi^2 for xi in x)

@kwdef struct OptimizationParams{F<:Real}
    ## TrajOpt Params
    planning_horizon::F = 10.0
    resolve_step_size::F = 0.1
    discretization_step_size::F = 0.1

    ## ODE Params
    integration_step_size::F = 0.01
    integration_max_step_size::F = 0.1
end

struct OptimalFormationControl3D{F<:Real,TOB,TR,VTOF}
    params::OptimizationParams{F}
    obstacles::TOB
    reference_path::TR
    offsets::VTOF
    turning_radius::F
    pitch_limits::SVector{2,F}
    v_min::F
    v_max::F
    ω_max::F
    δ::F # inter-agent collision radius
    apply_input_bounds::Function

    function OptimalFormationControl3D(
        params::OptimizationParams{F},
        obstacles::TOB,
        reference_path::TR;
        offsets::VTOF = Vector(SVector{3,F}(0.0, 0.0, 0.0)),
        turning_radius::F = 10.0,
        pitch_limits::SVector{2,F} = SVector{2,F}(-π / 4, π / 4),
        v_min::F = 0.8,
        v_max::F = 1.0,
        δ::F = 1.0, # inter-agent collision radius,
    ) where {F<:Real,TOB,TR,VTOF}
        ω_max = v_max / turning_radius

        function apply_input_bounds(v, ω, γ)
            v = clamp.(v, v_min, v_max)
            ω = clamp.(ω, -ω_max, ω_max)
            γ = clamp.(γ, pitch_limits[1], pitch_limits[2])
            return (v, ω, γ)
        end

        new{F,TOB,TR,VTOF}(
            params,
            obstacles,
            reference_path,
            offsets,
            turning_radius,
            pitch_limits,
            v_min,
            v_max,
            ω_max,
            δ,
            apply_input_bounds,
        )
    end
end

struct Trajectory{TF,TX,TU,TP,TO}
    t0::TF
    tf::TF
    x_interp::TX
    u_interp::TU
    τ::TF
    reference_path::TP
    offset::TO
end

function dynamics(state, input)
    x, y, z, yaw = state
    v, ω, γ = input
    return [v * cos(yaw) * cos(γ), v * sin(yaw) * cos(γ), v * sin(γ), ω]
end



"""
    tracking_controller(state, x_d, u_d)
"""
function tracking_controller(state, x_d, u_d)
    ctrls =
        [TC.tracking_controller(state[i, :], x_d[i, :], u_d[i, :]) for i = 1:size(x_d, 1)]

    v = [c[1] for c in ctrls]
    ω = [c[2] for c in ctrls]
    γ = [c[3] for c in ctrls]

    return (v, ω, γ)
end

function get_reference_state_and_input(reference_path, t::F) where {F<:Real}
    return TC.get_reference_state_and_input(reference_path, t)
end
function get_reference_state_and_input(reference_path, t::F, offset) where {F<:Real}
    return TC.get_reference_state_and_input(
        reference_path,
        t,
        @SVector[offset[1], offset[2], offset[3]]
    )
end

function total_path_length(reference_path)
    return sum(x -> x.length, reference_path)
end



"""
    create_infinite_model(t0, x0, prob; previous_solution = nothing, optimizer = Ipopt.Optimizer)
Constructs an infinite-dimensional optimal control model for 3D trajectory optimization using the InfiniteOpt.jl framework.

# Problem Description
This function formulates the following optimal control problem:

**Objective:**
Minimize the formation tracking error over the planning horizon:
Constructs an infinite-dimensional optimal control model for 3D trajectory optimization using the InfiniteOpt.jl framework.
"""
function create_infinite_model(
    t0,
    x0,
    prob;
    previous_solution = nothing,
    optimizer = optimizer_with_attributes(Ipopt.Optimizer, "print_level" => 5),
)
    N_agents = size(prob.offsets, 1)

    # Define input and state limits
    ω_lims = (-prob.ω_max, prob.ω_max)
    v_lims = (prob.v_min, prob.v_max)
    γ_lims = (prob.pitch_limits[1], prob.pitch_limits[2])

    τ_fixed = t0 + prob.params.planning_horizon

    # Get reference state and input at time τ for agent i
    xd_fn =
        (τ, i) ->
            get_reference_state_and_input(prob.reference_path, τ, prob.offsets[i, :])[1]
    ud_fn =
        (τ, i) ->
            get_reference_state_and_input(prob.reference_path, τ, prob.offsets[i, :])[2]

    # Check initial time is within path length
    @assert 0.0 <= t0 <= total_path_length(prob.reference_path)

    # Define time span and discretization
    tspan = [t0, t0 + prob.params.planning_horizon]
    dt = prob.params.discretization_step_size
    num_supports = ceil(Int, (tspan[2] - tspan[1]) / dt)

    # Create InfiniteOpt model with specified optimizer
    model = InfiniteModel(optimizer)

    # Define infinite parameter (time) and variables (states, controls, terminal time)
    @infinite_parameter(model, t in tspan, num_supports = num_supports)
    @variables(
        model,
        begin
            # State Variables: x = [x, y, z, yaw]
            x[1:N_agents, 1:4], Infinite(t)
            # Control Variables: u = [v, ω, γ]
            u[1:N_agents, 1:3], Infinite(t)
            # Terminal time variable
            # τ
            # Obstacle Slack
            obstacle_slack[1:N_agents, 1:length(prob.obstacles)] >= 0, Infinite(t)

            # Interagent Slack
            interagent_slack[1:N_agents, 1:N_agents] >= 0, Infinite(t)
        end
    )

    # Reference state as a parameter function for tracking
    @parameter_function(model, xd[i = 1:N_agents, j = 1:4] == (t -> xd_fn(t, i)[j]))

    # Objective: minimize squared tracking error in position over the horizon
    # @objective(
    #     model,
    #     Min,
    #     integral(sumsquares(x[1:3] - xd[1:3]), t) # tracking error
    # )

    # function obstacle_penalty(obs_x, obs_y, obs_r, x, y)
    #     dist_sq = (obs_x - x)^2 + (obs_y - y)^2
    #     return max(0, (obs_r^2 - dist_sq) / obs_r^2)^2
    # end
    # function obstacle_barrier(obs_x, obs_y, obs_r, x, y)
    #     dist_sq = (obs_x - x)^2 + (obs_y - y)^2
    #     safety_radius = obs_r * 1.1 # 10% larger than obstacle radius
    #     if dist_sq <= safety_radius^2
    #         return 1000.0 / (dist_sq - obs_r^2 + 0.1)
    #     else
    #         return 0.0
    #     end
    # end

    # @register(model, obstacle_penalty(a, b, c, d, e))
    # @register(model, obstacle_barrier(a, b, c, d, e))
    @objective(
        model,
        Min,
        1.0 * integral(
            sumsquares(x[1:N_agents, 1:3] - xd[1:N_agents, 1:3]) +
            0.1 * sumsquares(x[1:N_agents, 4] - xd[1:N_agents, 4]),
            t,
        ) + # tracking error
        # 10.0 * integral((x[4] - xd[4])^2, t) + # yaw tracking error
        0.001 * integral(sumsquares(u), t) + # input regularization
        100000.0 * integral(sum(obstacle_slack), t) + # heavy penalty on slack variables
        100000.0 * integral(sum(interagent_slack), t) # heavy penalty on inter-agent slack
    )

    # System dynamics constraints (differential equations)
    @constraint(
        model,
        dynamics_con[i = 1:N_agents, j = 1:4],
        ∂(x[i, j], t) == dynamics(x[i, :], u[i, :])[j]
    )

    # Input constraints (velocity, yaw rate, pitch)
    @constraint(model, input_lims_v, v_lims[1] .<= u[1:N_agents, 1] .<= v_lims[2])
    @constraint(model, input_lims_ω, ω_lims[1] .<= u[1:N_agents, 2] .<= ω_lims[2])
    @constraint(model, input_lims_γ, γ_lims[1] .<= u[1:N_agents, 3] .<= γ_lims[2])

    # Initial condition constraints
    @constraint(
        model,
        initial_conditions[i = 1:N_agents, j = 1:4],
        x[i, j](tspan[1]) == x0[i, j]
    )

    # Safety constraints for static obstacles (cylindrical)
    # function obsH(cyl_x, cyl_y, cyl_r, rob_x, rob_y)
    #     # Returns positive if outside the obstacle
    #     return (cyl_x - rob_x)^2 + (cyl_y - rob_y)^2 - cyl_r^2
    # end
    # @register(model, obsH(a, b, c, d, e))

    for (i, obs) in enumerate(prob.obstacles)
        obs_x, obs_y, obs_z = obs.center
        obs_r = obs.radius * 1.1
        @constraint(
            model,
            (x[:, 1] .- obs_x) .^ 2 .+ (x[:, 2] .- obs_y) .^ 2 .+ obstacle_slack[:, i] .>=
            (obs_r)^2
        ) # 2D cylindrical obstacle
        # Enforce safety constraint for each obstacle
        # @constraint(model, obsH(obs_x, obs_y, obs_r, x[1], x[2]) >= 0)
    end

    for i = 1:N_agents
        for j = 1:N_agents
            if i != j
                # Inter-agent distance constraint
                @constraint(
                    model,
                    (x[i, 1] - x[j, 1])^2 +
                    (x[i, 2] - x[j, 2])^2 +
                    (x[i, 3] - x[j, 3])^2 +
                    interagent_slack[i, j] >= prob.δ^2
                )
            end
        end
    end

    # Terminal constraint to enforce arrival at reference state at τ
    # Define a generic version for InfiniteOpt registration
    # function terminal_constraint(x1, x2, x3, x4, τ)
    # function terminal_constraint(a, b, c, d, τ)
    #     xr = get_reference_state_and_input(prob.reference_path, τ, prob.offset)[1]
    #     return ((a - xr[1])^2 + (b - xr[2])^2 + (c - xr[3])^2 + (d - xr[4])^2)
    # end
    # @register(model, terminal_constraint(a, b, c, d, t))
    # @constraint(
    #     model,
    #     terminal_con,
    #     terminal_constraint(
    #         x[1](tspan[2]),
    #         x[2](tspan[2]),
    #         x[3](tspan[2]),
    #         x[4](tspan[2]),
    #         τ_fixed,
    #         # τ,
    #     ) == 3.0, # tuning parameter
    # )

    # τ must be within the planning horizon
    # @constraint(model, tspan[1] <= τ <= tspan[2])

    # Specify initial guess for solver (warm start)
    if isnothing(previous_solution)
        # Reference state and input functions (for tracking)
        xd_ref_fn = τ -> get_reference_state_and_input(prob.reference_path, τ)[1]
        ud_ref_fn = τ -> get_reference_state_and_input(prob.reference_path, τ)[2]
        # Use reference trajectory as initial guess
        for agent = 1:N_agents
            for i = 1:4
                # set_start_value_function(x[i], t -> xd_fn(t)[i])
                set_start_value_function(x[agent, i], t -> xd_ref_fn(t)[i])
            end
            for i = 1:3
                # set_start_value_function(u[i], t -> ud_fn(t)[i])
                set_start_value_function(u[agent, i], t -> ud_ref_fn(t)[i])
            end
        end
        # JuMP.set_start_value(τ, tspan[2])
    else
        # Use previous solution as initial guess
        for agent = 1:N_agents
            for i = 1:4
                set_start_value_function(
                    x[agent, i],
                    (t -> get_desired_state_and_input(t, previous_solution)[1][agent, i]),
                )
            end
            for i = 1:3
                set_start_value_function(
                    u[agent, i],
                    (t -> get_desired_state_and_input(t, previous_solution)[2][agent, i]),
                )
            end
        end
        # JuMP.set_start_value(τ, previous_solution.τ)
    end

    return model
end

function solve_model!(model; print_level = 0, time_limit = 60.0, max_iter = 30000)
    set_optimizer_attribute(model, "print_level", print_level)
    unset_silent(model)

    set_time_limit_sec(model, time_limit)
    set_optimizer_attribute(model, "max_iter", max_iter)

    set_optimizer_attribute(model, "tol", 1e-5)
    set_optimizer_attribute(model, "constr_viol_tol", 1e-5)
    set_optimizer_attribute(model, "acceptable_tol", 1e-3)
    # set_optimizer_attribute(model, "acceptable_constr_viol_tol", 1e-3)
    set_optimizer_attribute(model, "mu_strategy", "adaptive")
    set_optimizer_attribute(model, "nlp_scaling_method", "gradient-based")

    # set_optimizer_attribute(model, "tol", 1e-3)
    # set_optimizer_attribute(model, "dual_inf_tol", 1e-2)

    optimize!(model)

    if termination_status(model) ∉ [MOI.OPTIMAL, MOI.LOCALLY_SOLVED]
        @error "Model did not solve to optimality: $(termination_status(model))"
    end

    return model
end

function extract_traj(prob, model)
    t_vals = parameter_by_name(model, "t") |> value

    # Get time and state dimensions
    N_agents = size(prob.offsets, 1)
    t_len = length(t_vals)
    state_dim = 4
    control_dim = 3

    x_vars = model[:x]
    u_vars = model[:u]

    x_values_raw = value.(x_vars)
    u_values_raw = value.(u_vars)

    # x_values_raw is a 3×4 vector of vectors (each of length t_len)
    # We want to stack these into a [t_len, 3, 4] array
    # Create x_vals and u_vals as vectors of (N_agents x state_dim) and (N_agents x control_dim) arrays for each time step
    x_vals = [
        reshape(
            reduce(
                hcat,
                [x_values_raw[agent, j][t] for agent = 1:N_agents, j = 1:state_dim],
            ),
            N_agents,
            state_dim,
        ) for t = 1:t_len
    ]
    u_vals = [
        reshape(
            reduce(
                hcat,
                [u_values_raw[agent, j][t] for agent = 1:N_agents, j = 1:control_dim],
            ),
            N_agents,
            control_dim,
        ) for t = 1:t_len
    ]

    # Create the 'interpolant'
    x_interpolant = linear_interpolation(t_vals, x_vals, extrapolation_bc = Line())
    u_interpolant = linear_interpolation(t_vals, u_vals, extrapolation_bc = Line())

    # Create the trajectory object
    t0 = t_vals[1]
    tf = t_vals[end]

    return Trajectory(
        t0,
        tf,
        x_interpolant,
        u_interpolant,
        t_vals[end],
        prob.reference_path,
        prob.offsets,
    )
end

function construct_new_trajectory(t, x, prob; previous_solution = nothing)
    model = create_infinite_model(t, x, prob; previous_solution = previous_solution)
    solve_model!(model)
    return extract_traj(prob, model)
end

"""
    track_trajectory(t, x, trajectory)

Returns the control input to track the trajectory at time `t` given the current state `x`.
"""
function track_trajectory(t, x, trajectory)
    x_wrapped = copy(x)
    x_wrapped[:, 4] = mod.(x_wrapped[:, 4] .+ π, 2π) .- π # wrap yaw to [-π, π]

    x_d, u_d = get_desired_state_and_input(t, trajectory)
    u = tracking_controller(x_wrapped, x_d, u_d)
    return u
end

"""
    get_desired_state_and_input(t, trajectory)
Called by track_trajectory. Returns the desired state and input at time `t` from the trajectory.

"""
function get_desired_state_and_input(t, trajectory::Trajectory)
    if t <= trajectory.tf
        # track solved answer
        x_d = trajectory.x_interp(t)
        u_d = trajectory.u_interp(t)
    else
        # track the reference path
        ref = [
            get_reference_state_and_input(
                trajectory.reference_path,
                trajectory.τ + (t - trajectory.tf),
                trajectory.offset[i, :],
            ) for i = 1:size(trajectory.offset, 1)
        ]

        # ref is vector of tuples (state, input) of length n_agents
        # want x_d to be a matrix of shape (n_agents, 4)
        # want u_d to be a matrix of shape (n_agents, 3)
        x_d = hcat([r[1] for r in ref]...)' # states
        u_d = hcat([r[2] for r in ref]...)' # inputs
    end

    return x_d, u_d
end

function closed_loop_tracking!(D, state, params, time)
    prob, trajectory = params

    v, ω, γ = track_trajectory(time, state, trajectory)
    v, ω, γ = prob.apply_input_bounds(v, ω, γ)

    D[:, 1] = v .* cos.(state[:, 4]) .* cos.(γ)
    D[:, 2] = v .* sin.(state[:, 4]) .* cos.(γ)
    D[:, 3] = v .* sin.(γ)
    D[:, 4] = ω

    return
end


function update_trajectory_affect!(integrator)
    time = integrator.t
    state = integrator.u
    params = integrator.p

    prob, previous_trajectory = params

    new_trajectory =
        construct_new_trajectory(time, state, prob; previous_solution = previous_trajectory)

    # failed to construct a new trajectory
    if isnothing(new_trajectory)
        return
    end

    println("\treplacing the trajectory at time t=$(time)")
    integrator.p = (prob, new_trajectory)

    return
end

function wrap_angle_affect!(integrator)
    # Wrap angles to [-π, π]
    integrator.u[:, 4] = mod.(integrator.u[:, 4] .+ π, 2π) .- π
    return
end



"""
    simulate_closed_loop_trajopt(x0, tspan, prob)

Simulates a closed-loop trajectory optimization for a given initial state `x0`, time span `tspan`, and problem definition `prob`.

# Arguments
- `x0`: Initial state vector of the system.
- `tspan`: Tuple or vector specifying the time span for the simulation (e.g., `(t0, tf)`).
- `prob`: Problem definition containing parameters and settings for trajectory optimization and integration.

# Returns
- `odesol`: The solution object returned by the ODE solver, containing the simulated trajectory.

# Description
This function constructs an initial trajectory based on the provided initial state and time, sets up a closed-loop tracking ODE problem, and simulates the system using a specified ODE solver. The trajectory is periodically updated during the simulation using a callback mechanism to ensure the system tracks the optimized trajectory.

# Notes
- The function asserts that the initial trajectory is successfully constructed.
- The ODE is solved using the `Tsit5()` solver with user-defined integration step sizes.
- The trajectory update is performed at intervals defined by `prob.resolve_step_size`.
"""
function simulate_closed_loop_trajopt(prob, x0, tspan)
    println("Simulating closed-loop trajectory optimization...")
    println("Number of Agents: $(size(x0, 1))")
    println("Shape of x0: $(size(x0))")
    println("Shape of Offsets: $(size(prob.offsets))")

    println("Constructing Initial Trajectories...")
    initial_trajectories = construct_new_trajectory(tspan[1], x0[:, 1:4], prob)
    @assert !isnothing(initial_trajectories) "Failed to construct initial trajectories."
    println("Initial Trajectories constructed successfully!")

    # Setup the problem
    params = (prob, initial_trajectories)

    odeproblem = ODEProblem(closed_loop_tracking!, x0[:, 1:4], tspan, params)

    update_trajectory_callback =
        PeriodicCallback(update_trajectory_affect!, prob.params.resolve_step_size)

    wrap_angles_callback = DiscreteCallback((u, t, integrator) -> true, wrap_angle_affect!)

    callbacks = CallbackSet(update_trajectory_callback, wrap_angles_callback)

    odesol = solve(
        odeproblem,
        Tsit5(),
        dtmax = prob.params.integration_max_step_size,
        dt = prob.params.integration_step_size,
        callback = callbacks,
    )

    return odesol
end

end # module TrajOpt3D