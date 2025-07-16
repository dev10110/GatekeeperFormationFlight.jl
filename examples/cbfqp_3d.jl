"""
CBFQP3D
=================

This modules implements a CBF-QP for the 3D dubins vehicle

We need to augment the state such that 
x = [x, y, z, ψ, γ] (x, y, z, yaw, pitch)
u = [v, ω, γ_dot] (linear velocity, angular velocity, pitch rate)

We impose constraints on the value of ω and γ, but not γ_dot
"""
module CBFQP3D

using ForwardDiff, LinearAlgebra, StaticArrays
using GatekeeperFormationFlight
using OSQP, SparseArrays
using OrdinaryDiffEq

GFF = GatekeeperFormationFlight
GK = GatekeeperFormationFlight.Gatekeeper


function h(obs::O, x) where {O<:GFF.AbstractStaticObstacle}
    return collision_distance(obs, x)
end

function dhdx(obs::O, x) where {O<:GFF.AbstractStaticObstacle}
    return ForwardDiff.gradient(xi -> h(obs, xi), x)
end

"""
State evolution function
"""
function f(x)
    return SVector(0.0, 0.0, 0.0, 0.0, 0.0)
end

function g(x)
    ψ = x[4]
    γ = x[5]
    rval = SMatrix{5,3}(
        [
            cos(ψ)*cos(γ) 0.0 0.0
            sin(ψ)*cos(γ) 0.0 0.0
            sin(γ) 0.0 0.0
            0.0 1.0 0.0
            0.0 0.0 1.0
        ],
    )

    return rval
end

function Lfh(obs::O, x) where {O<:GFF.AbstractStaticObstacle}
    return dhdx(obs, x)' * f(x)
end

function Lgh(obs::O, x) where {O<:GFF.AbstractStaticObstacle}
    return dhdx(obs, x)' * g(x)
end

########################################
# CBF-QP  Rigid Constraints
########################################

function create_cbf_constraints(
    x,
    u_d,
    obs::VO,
    α,
) where {O<:GFF.AbstractStaticObstacle,VO<:AbstractVector{O}}
    A = vcat([Lgh(o, x) for o in obs]...)
    l = [-Lfh(o, x) - α * h(o, x) for o in obs]
    u = [Inf for o in obs]

    return A, l, u
end

function create_input_constraints(vlims, ωlims, γ_lims)

    A = I(3)
    l = [vlims[1], ωlims[1], γ_lims[1]]
    u = [vlims[2], ωlims[2], γ_lims[2]]

    return A, l, u
end

function cbf_qp(
    x,
    u_d,
    obs::VO,
    α,
    vlims,
    ωlims,
    γlims,
) where {O<:GFF.AbstractStaticObstacle,VO<:AbstractVector{O}}

    P = diagm([1.0, 200.0, 1.0])
    q = -u_d

    # create the cbf constraints
    A_cbf, l_cbf, u_cbf = create_cbf_constraints(x, u_d, obs, α)

    # create the input constraints
    A_input, l_input, u_input = create_input_constraints(vlims, ωlims, γlims)

    # combine the constraints
    A = vcat(A_cbf, A_input)
    l = vcat(l_cbf, l_input)
    u = vcat(u_cbf, u_input)

    m = OSQP.Model()
    OSQP.setup!(m; P = sparse(P), q = q, A = sparse(A), l = l, u = u, verbose = false)
    results = OSQP.solve!(m)

    return results
end

########################################
# CBF-QP  Relaxed Constraints
########################################

function create_relaxed_input_constraints(v_lims, ω_lims, γ_lims, N_Z)
    A_in, l, u = create_input_constraints(v_lims, ω_lims, γ_lims)

    A = hcat(A_in, zeros(3, N_Z))

    return A, l, u
end

function create_relaxed_CBF_constraints(
    x,
    u_d,
    obs::VO,
    α,
) where {O<:GFF.AbstractStaticObstacle,VO<:AbstractVector{O}}

    A, l, u = create_cbf_constraints(x, u_d, obs, α)
    N_Z = length(obs)

    A_relaxed = [[A;; I(N_Z)]; [zeros(N_Z, 3);; I(N_Z)]]
    l_relaxed = vcat(l, zeros(N_Z))
    u_relaxed = vcat(u, Inf * ones(N_Z))

    return A_relaxed, l_relaxed, u_relaxed
end

function cbf_qp_relaxed(
    x,
    u_d,
    obs::VO,
    α,
    vlims,
    ωlims,
    γlims,
    ϵ,
) where {O<:GFF.AbstractStaticObstacle,VO<:AbstractVector{O}}

    N_Z = length(obs)

    P = diagm([ϵ, 200.0 * ϵ, 20.0 * ϵ, ones(N_Z)...])
    q = vcat(-u_d, zeros(N_Z))

    # create the cbf constraints
    A_cbf, l_cbf, u_cbf = create_relaxed_CBF_constraints(x, u_d, obs, α)

    # create the input constraints
    A_input, l_input, u_input = create_relaxed_input_constraints(vlims, ωlims, γlims, N_Z)

    # combine the constraints
    A = vcat(A_cbf, A_input)
    l = vcat(l_cbf, l_input)
    u = vcat(u_cbf, u_input)

    m = OSQP.Model()
    OSQP.setup!(m; P = sparse(P), q = q, A = sparse(A), l = l, u = u, verbose = false)
    results = OSQP.solve!(m)

    return results
end

function cbf_qp_controller(
    x,
    u_d,
    obs::VO,
    α,
    vlims,
    ωlims,
    γlims;
    ϵ = 1e-2,
) where {O<:GFF.AbstractStaticObstacle,VO<:AbstractVector{O}}
    res = cbf_qp(x, u_d, obs, α, vlims, ωlims, γlims)

    if res.info.status == :Solved
        return SVector(res.x[1], res.x[2], res.x[3])
    end

    res_relaxed = cbf_qp_relaxed(x, u_d, obs, α, vlims, ωlims, γlims, ϵ)

    return SVector(res_relaxed.x[1], res_relaxed.x[2], res_relaxed.x[3])
end

function closed_loop_CBF!(D, state, gk, time, α, ϵ)
    problem = gk.problem

    x_d, u_d = GK.get_reference_state_and_input(
        problem,
        GK.get_reference_path(problem),
        GK.get_offset(problem),
        time,
    )

    v, ω, γ = GK.tracking_controller(problem, state, x_d, u_d)

    # inputs = SVector{3}(v, ω, γ)

    u_cbf = cbf_qp_controller(
        SVector{5}(state),
        [v, ω, γ],
        problem.obstacles,
        α,
        (problem.v_min, problem.v_max),
        (-problem.v_max / problem.turning_radius, problem.v_max / problem.turning_radius),
        problem.pitch_limits,
        ϵ = ϵ,
    )

    inputs = GK.apply_input_bounds(problem, SVector{3}(u_cbf))
    # inputs = GK.apply_input_bounds(problem, inputs)

    GK.state_dynamics!(problem, D, state, inputs)
end

function simulate_closed_loop_cbf(
    initial_state,
    tspan,
    gk::GatekeeperInstance;
    alpha = 50.0,
    epsilon = 1e-2,
)
    println("Simulating Closed Loop CBF from $(initial_state) over $(tspan)")

    function do_closed_loop_CBF!(D, s, g, t)
        closed_loop_CBF!(D, s, g, t, alpha, epsilon)
    end

    odeproblem = ODEProblem(do_closed_loop_CBF!, Vector(initial_state), tspan, gk)

    odesol = solve(
        odeproblem,
        Tsit5(),
        dtmax = gk.coefficients.integration_max_step_size,
        dt = gk.coefficients.integration_step_size,
    )

    return odesol
end

end # module CBFQWP3D
