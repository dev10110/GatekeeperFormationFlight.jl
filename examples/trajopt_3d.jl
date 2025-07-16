module TrajOpt3D

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

struct OptimalFormationControl3D{F<:Real,TOB,TR,TOF}
    params::OptimizationParams{F}
    obstacles::TOB
    reference_path::TR
    offset::TOF
    turning_radius::F
    pitch_limits::SVector{2,F}
    v_min::F
    v_max::F
    ω_max::F
    apply_input_bounds::Function

    function OptimalFormationControl3D(
        params::OptimizationParams{F},
        obstacles::TOB,
        reference_path::TR;
        offset::TOF = SVector{3,F}(0.0, 0.0, 0.0),
        turning_radius::F = 10.0,
        pitch_limits::SVector{2,F} = SVector{2,F}(-π / 4, π / 4),
        v_min::F = 0.8,
        v_max::F = 1.0,
    ) where {F<:Real,TOB,TR,TOF}
        ω_max = v_max / turning_radius

        function apply_input_bounds(v, ω, γ)
            v = clamp(v, v_min, v_max)
            ω = clamp(ω, -ω_max, ω_max)
            γ = clamp(γ, pitch_limits[1], pitch_limits[2])
            return (v, ω, γ)
        end

        new{F,TOB,TR,TOF}(
            params,
            obstacles,
            reference_path,
            offset,
            turning_radius,
            pitch_limits,
            v_min,
            v_max,
            ω_max,
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
    return TC.tracking_controller(state, x_d, u_d)
end

function get_reference_state_and_input(reference_path, t::F) where {F<:Real}
    return TC.get_reference_state_and_input(reference_path, t)
end
function get_reference_state_and_input(reference_path, t::F, offset) where {F<:Real}
    return TC.get_reference_state_and_input(reference_path, t, offset)
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
    # Define input and state limits
    ω_lims = (-prob.ω_max, prob.ω_max)
    v_lims = (prob.v_min, prob.v_max)
    γ_lims = (prob.pitch_limits[1], prob.pitch_limits[2])

    τ_fixed = t0 + prob.params.planning_horizon

    # Reference state and input functions (for tracking)
    xd_fn = τ -> get_reference_state_and_input(prob.reference_path, τ, prob.offset)[1]
    ud_fn = τ -> get_reference_state_and_input(prob.reference_path, τ, prob.offset)[2]

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
    @variables(model, begin
        # State Variables: x = [x, y, z, yaw]
        x[1:4], Infinite(t)
        # Control Variables: u = [v, ω, γ]
        u[1:3], Infinite(t)
        # Terminal time variable
        # τ
        # Obstacle Slack
        slack[1:length(prob.obstacles)] >= 0, Infinite(t)
    end)

    # Reference state as a parameter function for tracking
    @parameter_function(model, xd[i = 1:4] == (t -> xd_fn(t)[i]))

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
        1000.0 * integral(sumsquares(x[1:3] - xd[1:3]) + 0.1 * (x[4] - xd[4])^2, t) + # tracking error
        10.0 * integral((x[4] - xd[4])^2, t) + # yaw tracking error
        0.001 * integral(sumsquares(u), t) + # input regularization
        100000.0 * integral(sum(slack), t) # heavy penalty on slack variables
    )

    # System dynamics constraints (differential equations)
    @constraint(model, dynamics_con[i = 1:4], ∂(x[i], t) == dynamics(x, u)[i])

    # Input constraints (velocity, yaw rate, pitch)
    @constraint(model, input_lims_v, v_lims[1] <= u[1] <= v_lims[2])
    @constraint(model, input_lims_ω, ω_lims[1] <= u[2] <= ω_lims[2])
    @constraint(model, input_lims_γ, γ_lims[1] <= u[3] <= γ_lims[2])

    # Initial condition constraints
    @constraint(model, initial_conditions[i = 1:4], x[i](tspan[1]) == x0[i])

    # Safety constraints for static obstacles (cylindrical)
    # function obsH(cyl_x, cyl_y, cyl_r, rob_x, rob_y)
    #     # Returns positive if outside the obstacle
    #     return (cyl_x - rob_x)^2 + (cyl_y - rob_y)^2 - cyl_r^2
    # end
    # @register(model, obsH(a, b, c, d, e))

    for (i, obs) in enumerate(prob.obstacles)
        obs_x, obs_y, obs_z = obs.center
        obs_r = obs.radius * 1.1
        @constraint(model, (x[1] - obs_x)^2 + (x[2] - obs_y)^2 + slack[i] >= (obs_r)^2) # 2D cylindrical obstacle
        # Enforce safety constraint for each obstacle
        # @constraint(model, obsH(obs_x, obs_y, obs_r, x[1], x[2]) >= 0)
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
        for i = 1:4
            # set_start_value_function(x[i], t -> xd_fn(t)[i])
            set_start_value_function(x[i], t -> xd_ref_fn(t)[i])
        end
        for i = 1:3
            # set_start_value_function(u[i], t -> ud_fn(t)[i])
            set_start_value_function(u[i], t -> ud_ref_fn(t)[i])
        end
        # JuMP.set_start_value(τ, tspan[2])
    else
        # Use previous solution as initial guess
        for i = 1:4
            set_start_value_function(
                x[i],
                (t -> get_desired_state_and_input(t, previous_solution)[1][i]),
            )
        end
        for i = 1:3
            set_start_value_function(
                u[i],
                (t -> get_desired_state_and_input(t, previous_solution)[2][i]),
            )
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
    # check that terminal constraint is not overly violated
    # term_const = constraint_by_name(model, "terminal_con") |> value
    # if norm(term_const) > 5.0 # Tuning parmeter
    #     @error "Terminal constraint violation is too high: $(norm(term_const))"
    #     return nothing
    # end

    t_vals = parameter_by_name(model, "t") |> value
    x1_vals = variable_by_name(model, "x[1]") |> value
    x2_vals = variable_by_name(model, "x[2]") |> value
    x3_vals = variable_by_name(model, "x[3]") |> value
    x4_vals = variable_by_name(model, "x[4]") |> value
    x4_vals = [atan(sin(val), cos(val)) for val in x4_vals] # wrap yaw to [-π, π]
    # x5_vals = variable_by_name(model, "x[5]") |> value

    x_vals = zip(x1_vals, x2_vals, x3_vals, x4_vals) |> collect
    x_vals = [[x...] for x in x_vals]

    u1_vals = variable_by_name(model, "u[1]") |> value
    u2_vals = variable_by_name(model, "u[2]") |> value
    u3_vals = variable_by_name(model, "u[3]") |> value

    u_vals = zip(u1_vals, u2_vals, u3_vals) |> collect
    u_vals = [[u...] for u in u_vals]

    # Create the 'interpolant'
    x_interpolant = linear_interpolation(t_vals, x_vals, extrapolation_bc = Line())
    u_interpolant = linear_interpolation(t_vals, u_vals, extrapolation_bc = Line())

    # Create the trajectory object
    t0 = t_vals[1]
    tf = t_vals[end]

    # τ = variable_by_name(model, "τ") |> value

    # return Trajectory(t0, tf, x_interpolant, u_interpolant, τ, prob.reference_path)
    return Trajectory(
        t0,
        tf,
        x_interpolant,
        u_interpolant,
        t_vals[end],
        prob.reference_path,
        prob.offset,
    )
end


function construct_new_trajectory(t, x, prob; previous_solution = nothing)
    # if is_colliding(prob.obstacles, x)
    #     @error "Cannot construct trajectory: state is colliding with an obstacle. (x=$(x), t=$(t))"
    #     return nothing
    # end

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
    x_wrapped[4] = mod(x_wrapped[4] + π, 2π) - π # wrap yaw to [-π, π]

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
        x_d, u_d = get_reference_state_and_input(
            trajectory.reference_path,
            trajectory.τ + (t - trajectory.tf),
            trajectory.offset,
        )
    end

    return x_d, u_d
end

function closed_loop_tracking!(D, state, params, time)
    prob, trajectory = params

    v, ω, γ = track_trajectory(time, state, trajectory)
    v, ω, γ = prob.apply_input_bounds(v, ω, γ)

    D[1] = v * cos(state[4]) * cos(γ)
    D[2] = v * sin(state[4]) * cos(γ)
    D[3] = v * sin(γ)
    D[4] = ω

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
    integrator.u[4] = mod(integrator.u[4] + π, 2π) - π
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

    first_trajectory = construct_new_trajectory(tspan[1], x0, prob)
    @assert !isnothing(first_trajectory)

    # Setup the problem
    params = (prob, first_trajectory)

    odeproblem = ODEProblem(closed_loop_tracking!, Vector(x0[1:4]), tspan, params)

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