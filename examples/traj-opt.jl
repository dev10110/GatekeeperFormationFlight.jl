

module TrajOpt

using GatekeeperFormationFlight
using LinearAlgebra, StaticArrays
using OrdinaryDiffEq, DiffEqCallbacks
using InfiniteOpt, Ipopt
using Interpolations

GFF = GatekeeperFormationFlight

# define sumsquares
sumsquares(x) = sum(xi^2 for xi in x)

@kwdef struct OptimalWezAvoidanceProblem{TW, TR, TO, TF}
    wezes::TW
    reference_path::TR
    offset::TO
    turning_radius::TF=0.1
    discretization_step_size::TF=0.01
    planning_horizon::TF = 0.5
    resolve_step_size::TF = 0.2
    integration_max_step_size::TF=0.001
end    

# define the dubins dynamics 
function dynamics(state, input)
    x,y,ψ = state
    v, ω = input
    return [v * cos(ψ), v * sin(ψ), ω]
end

# positive => safe
function _collision_distance(cx, cy, cψ, rx, ry, rψ)

    wez = Cbez(cx, cy, cψ)
    robot = Robot(rx, ry, rψ)

    return collision_distance(wez, robot)
    
end

# function terminal_constraint_(x1, x2, x3, τ, prob)
#         xr = get_reference_state(prob.reference_path, τ)
#         d = (x1 - xr[1])^2 + (x2 - xr[2])^2 + (x3 - xr[3])^2

#     return 100*d
#     # return (x2 - xr[2])^2 # + (x3 - xr[3])^2
#     # @show xr
#     # return (x3 - xr[3])^2
# end


function create_infinite_model(t0, initial_state, prob::OptimalWezAvoidanceProblem;
        optimizer=optimizer_with_attributes(Ipopt.Optimizer, "print_level" => 5), 
        previous_solution = nothing
    )

    # create ω_lims
    ω_lims = (-10.0, 10.0)
    
    # create v_lims
    v_lims = (0.8, 1.0)
    
    # tmp_funcs
    xd_fn = τ -> get_reference_state(prob.reference_path, τ, prob.offset)
    ud_fn = τ -> get_reference_state_and_input(prob.reference_path, τ, prob.offset)[2]

    # create tspan
    @assert 0.0 <= t0 <= total_path_length(prob.reference_path)
    
    tspan = [t0, t0 + prob.planning_horizon]
    dt = prob.discretization_step_size
    num_supports =  ceil(Int, (tspan[2] - tspan[1])/ prob.discretization_step_size)
    
    # start creating the model
    model = InfiniteModel(optimizer);
    
    @infinite_parameter(model, t in tspan, num_supports = num_supports)
    @variables(model, begin
        # state variables
        x[1:3], Infinite(t)
    
        # control variables
        u[1:2], Infinite(t)

        # terminal constraint
        τ # to pick a point on the reference path to join
    end
    )

    @parameter_function(model, xd[i=1:3] == (t -> xd_fn(t)[i]) )

    @objective(model, Min, integral(sumsquares(x[1:2] - xd[1:2]), t) )

    # dynamics constraints
    @constraint(model, dynamics_con[i = 1:3], ∂(x[i], t) == dynamics(x, u)[i])

    # input constraints
    @constraint(model, input_lims_v, v_lims[1] <= u[1] <= v_lims[2])
    @constraint(model, input_lims_ω, ω_lims[1] <= u[2] <= ω_lims[2])

    # initial conditions
    @constraint(model, initial_con[i=1:3], x[i](tspan[1]) == initial_state[i])

    # safety constraints
    function obsH(cx, cy, cψ, x1, x2, x3)
        return _collision_distance(cx, cy, cψ, x1, x2, x3)
    end
    @register(model, obsH(a, b, c, d, e, f) )


    for i=1:length(prob.wezes)
        cx = prob.wezes[i].x
        cy = prob.wezes[i].y
        cψ = prob.wezes[i].ψ
    
        @constraint(model, obsH(cx, cy, cψ, x[1], x[2], x[3]) >= 0)
    end

    # terminal constraints    
    function terminal_constraint(x1, x2, x3, τ)
        xr = get_reference_state(prob.reference_path, τ)
        return 100 * ((x1 - xr[1])^2 + (x2 - xr[2])^2 + (x3 - xr[3])^2)
    end
    @register(model, terminal_constraint(a, b, c, d) )
    @constraint(model, terminal_con, terminal_constraint(x[1](tspan[2]), x[2](tspan[2]), x[3](tspan[2]), τ) == 0)
    @constraint(model, 0 <= τ <= tspan[2])

    # specify initial guess
    if isnothing(previous_solution)
        for i=1:3
            set_start_value_function(x[i], (t -> xd_fn(t)[i]) )
        end
        for i=1:2
            set_start_value_function(u[i], (t->ud_fn(t)[i]) )
        end
        JuMP.set_start_value(τ, tspan[2] - prob.offset[1])
        
    else
        for i=1:3
            set_start_value_function(x[i], (t -> get_desired_state_and_input(t, previous_solution)[1][i]) )
        end
        for i=1:2
            set_start_value_function(u[i], (t -> get_desired_state_and_input(t, previous_solution)[2][i]) )
        end
        JuMP.set_start_value(τ, previous_solution.τ)
    end

    return model
end


function solve_model!(opt_model; print_level = 0, time_limit = 10.0, max_iter=10000)
    set_optimizer_attribute(opt_model, "print_level", print_level);
    unset_silent(opt_model)
    set_time_limit_sec(opt_model, time_limit)
    set_optimizer_attribute(opt_model, "max_iter", max_iter);

    optimize!(opt_model)

    return opt_model
end


struct Trajectory{TF, TX, TU, TP}
    t0::TF
    tf::TF
    x_interp::TX
    u_interp::TU
    τ::TF
    reference_path::TP
end
    

function extract_traj(trajopt_prob, trajopt_model)

    # check that the terminal constraint is not violated by too much
    term_cons = constraint_by_name(trajopt_model, "terminal_con") |> value
    if norm(term_cons) >= 1e-2 # tuning param!
        return nothing
    end
    
    t_vals = value(parameter_by_name(trajopt_model, "t"));

    x1_vals = value(variable_by_name(trajopt_model, "x[1]"))
    x2_vals = value(variable_by_name(trajopt_model, "x[2]"))
    x3_vals = value(variable_by_name(trajopt_model, "x[3]"))
            
    x_vals = zip(x1_vals, x2_vals, x3_vals) |> collect
    x_vals = [[x...] for x in x_vals]
            
    u1_vals = value(variable_by_name(trajopt_model, "u[1]"))
    u2_vals = value(variable_by_name(trajopt_model, "u[2]"))
    
    u_vals = zip(u1_vals, u2_vals) |> collect
    u_vals = [[u...] for u in u_vals]

    # create the interpolant
    x_interpolant = linear_interpolation(t_vals, x_vals, extrapolation_bc=Line())
    u_interpolant = linear_interpolation(t_vals, u_vals, extrapolation_bc=Line())


    # create the trajectory
    t0 = t_vals[1]
    tf = t_vals[end]
    τ = value(variable_by_name(trajopt_model, "τ"));
    
    return Trajectory(t0, tf, x_interpolant, u_interpolant, τ, trajopt_prob.reference_path)

end

function get_desired_state_and_input(t, traj::Trajectory)
    if t <= traj.tf
        # track the solved answer
        x_d = traj.x_interp(t)
        u_d = traj.u_interp(t)
    else
        # now track the reference path
        x_d, u_d = get_reference_state_and_input(traj.reference_path, traj.τ + (t - traj.tf))
    end
    return x_d, u_d
end
function track_trajectory(t, x, traj::Trajectory)
    x_d, u_d = get_desired_state_and_input(t, traj)
    u = tracking_controller(x, x_d, u_d)
    return u

end

function construct_new_trajectory(t, x, trajopt_prob::OptimalWezAvoidanceProblem; previous_solution=nothing)

    println("resolving at t=$(t)")

    # if we are in a wez, dont resolve for a trajectory 
    if collision_distance(trajopt_prob.wezes, Robot(x...)) <= -1e-5
        println(" - inside a wez, not resolving")
        return nothing
    end

    model = create_infinite_model(t, x, trajopt_prob; previous_solution=previous_solution)
    solve_model!(model)

    # println(solution_summary(optimizer_model(model)))
    
    return extract_traj(trajopt_prob, model)
end


function closed_loop_tracking!(D, state, params, time)

    prob, traj = params

    # run a tracking controller for this robot
    v, ω = track_trajectory(time, state, traj)

    # apply input bounds
    v, ω = apply_input_bounds(v, ω)

    # follower dynamics
    D[1] = v * cos(state[3])
    D[2] = v * sin(state[3])
    D[3] = ω

    return
end

function update_trajectory_affect!(integrator)
    # get the current time, state, params
    time = integrator.t
    state = integrator.u
    params = integrator.p
    prob, previous_traj = params

    # construct new trajectory
    trajectory = construct_new_trajectory(time, state, prob; previous_solution=previous_traj)

    if !isnothing(trajectory)
        # replace trajectory
        println(" - replacing trajectory at t=$(time)")
        integrator.p[2] = trajectory
    end
    
    return
end



# make the closed-loop simulation
function simulate_closed_loop_trajopt(initial_state, tspan, prob::OptimalWezAvoidanceProblem)

    println("*** starting to solve for initial_state = $(initial_state)")

    # construct the first trajectory
    first_trajectory = construct_new_trajectory(tspan[1], initial_state, prob)
    @assert !isnothing(first_trajectory)

    # setup the odeproblem
    params = [prob, first_trajectory]

    odeproblem =
        ODEProblem(closed_loop_tracking!, Vector(initial_state), tspan, params)

    update_trajectory_callback = PeriodicCallback(update_trajectory_affect!, prob.resolve_step_size)

    odesol = solve(
        odeproblem,
        Tsit5();
        dtmax = prob.integration_max_step_size, 
        dt = prob.integration_max_step_size,
        callback = update_trajectory_callback,
    )

    return odesol

end


end