module CBFQP

using ForwardDiff, LinearAlgebra, StaticArrays
using GatekeeperFormationFlight
using OSQP, SparseArrays
using OrdinaryDiffEq

GFF = GatekeeperFormationFlight

function h(wez::W, x) where {W <: GFF.AbstractWez}
    return collision_distance(wez, x)
end

function dhdx(wez::W, x) where {W <: GFF.AbstractWez}
    return ForwardDiff.gradient(xi -> h(wez, xi), x)
end

function f(x)
    return SVector(0,0,0.)
end

function g(x)
    θ = x[3]
    return @SMatrix [
        [cos(θ);; 0];
        [sin(θ);; 0];
        [0 ;; 1]
        ]
end

function Lfh(wez::W, x) where {W <: GFF.AbstractWez}
    return dhdx(wez, x)' * f(x)
end

function Lgh(wez::W, x) where {W <: GFF.AbstractWez}
    return dhdx(wez, x)' * g(x)
end

function create_CBF_constraints(x, u_d, wezes::VW, α=100.0) where {W <: GFF.AbstractWez, VW <: AbstractVector{W}}
    # create the main constraints
    A = vcat([Lgh(w, x) for w in wezes]...)
    l = [ -Lfh(w, x) - α * h(w, x) for w in wezes]
    u = [Inf for w in wezes]

    return A, l, u
end

function create_input_constraints(vlims, ωlims)
    A = I(2)
    l = [vlims[1], ωlims[1]]
    u = [vlims[2], ωlims[2]]

    return A, l, u
end

function create_relaxed_input_constraints(vlims, ωlims, N_Z)
    A_input, l, u = create_input_constraints(vlims, ωlims)

    A = hcat(A_input, zeros(2, N_Z))

    return A, l, u
end

function create_relaxed_CBF_constraints(x, u_d, wezes::VW, α=100.0) where {W <: GFF.AbstractWez, VW <: AbstractVector{W}}

    A, l, u = create_CBF_constraints(x, u_d, wezes, α)

    N_Z = length(wezes)
    
    A_relaxed = [[A;; I(N_Z)]; [zeros(N_Z, 2) ;; I(N_Z)]]
    l_relaxed = vcat(l, zeros(N_Z))
    u_relaxed = vcat(u, Inf * ones(N_Z))

    return A_relaxed, l_relaxed, u_relaxed
    
end


function cbf_qp(x, u_d, wezes::VW; α=100.0, vlims=(0.8, 1.0), ωlims=(-10.0, 10.0)) where {W <: GFF.AbstractWez, VW <: AbstractVector{W}}

    P = diagm([2500.0, 1.0]) # to prioritize the ω control
    q = - u_d

    # create cbf constraints
    A_cbf, l_cbf, u_cbf = create_CBF_constraints(x, u_d, wezes, α)

    # create input constraints
    A_input, l_input, u_input = create_input_constraints(vlims, ωlims)

    # create all constraints
    A = vcat(A_cbf, A_input)
    l = vcat(l_cbf, l_input)
    u = vcat(u_cbf, u_input)

    m = OSQP.Model()
    OSQP.setup!(m; P=sparse(P), q=q, A=sparse(A), l=l, u=u, verbose=false)
    results = OSQP.solve!(m)

    return results

end

function cbf_qp_relaxed(x, u_d, wezes::VW; α=100.0, vlims=(0.8, 1.0), ωlims=(-10.0, 10.0), ϵ=1e-7) where {W <: GFF.AbstractWez, VW <: AbstractVector{W}}

    N_Z = length(wezes)
    
    P = diagm([2500*ϵ, ϵ, ones(N_Z)...])
    q = vcat(-u_d,  zeros(N_Z))

    # create the main constraints
    A_cbf, l_cbf, u_cbf = create_relaxed_CBF_constraints(x, u_d, wezes, α)

    # create input constraints
    A_input, l_input, u_input = create_relaxed_input_constraints(vlims, ωlims, N_Z)

    # create all constraints
    A = vcat(A_cbf, A_input)
    l = vcat(l_cbf, l_input)
    u = vcat(u_cbf, u_input)

    # return A, l, u
    
    m = OSQP.Model()
    OSQP.setup!(m; P=sparse(P), q=q, A=sparse(A), l=l, u=u, verbose=false)
    results = OSQP.solve!(m)

    return results

end

function cbf_qp_controller(x, u_d, wezes::VW; α=100.0, vlims=(0.8, 1.0), ωlims=(-10.0, 10.0), ϵ=1e-5) where {W <: GFF.AbstractWez, VW <: AbstractVector{W}}

    res = cbf_qp(x, u_d, wezes; α=α, vlims=vlims, ωlims=ωlims)

    if res.info.status == :Solved
        return SVector(res.x[1], res.x[2])
    end

    # ok so something went wrong, solve the relaxed version
    res_relaxed = cbf_qp_relaxed(x, u_d, wezes; α=α, vlims=vlims, ωlims=ωlims, ϵ=ϵ)

    return SVector(res_relaxed.x[1], res_relaxed.x[2])
end




"""
    CBFProblem(kwargs...)

construct a `CBFProblem`. The arguments are:
- wezes,                              # list of wezes
- reference_path,                     # path of the leader
- offset = SVector(0.0, 0.0, 0.0),    # desired offset from the leaders path
- turning_radius = 0.1,               # max turning radius of a robot
- max_Ts_horizon = 1.0,               # maximum switching time
- integration_max_step_size = 0.05,   # max integration step size in nominal tracking
- switch_step_size = 0.05             # resolution used to decrease switch time
- α = 100.0                           # α used in the CBF-QP
"""
@kwdef struct CBFProblem{TW,TR,TO,TF}
    wezes::TW                            # list of wezes
    reference_path::TR                   # path of the leader
    offset::TO = SVector(0.0, 0.0, 0.0)  # desired offset from the leaders path
    turning_radius::TF = 0.1             # max turning radius of a robot
    integration_max_step_size::TF = 0.0005   # max integration step size in nominal tracking
    α::TF = 100.0                        # α used in the CBF-QP
end


Base.show(io::IO, prob::CBFProblem) = print(
    io,
    "CBF Problem(|wezes|: ",
    length(prob.wezes),
    ", offset: ",
    prob.offset,
    ", turning radius: ",
    prob.turning_radius,
    ")",
)
Base.show(io::IO, ::MIME"text/plain", prob::CBFProblem) = print(
    io,
    "CBF Problem\n",
    " - Number of wezes: ",
    length(prob.wezes),
    "\n - offset: ",
    prob.offset,
    "\n - turning radius: ",
    prob.turning_radius,
)


"""
    closed_loop_CBF!(D, state, prob::CBFProblem, time)

defines the closed_loop dynamics for a CBF-QP
"""
function closed_loop_CBF!(D, state, prob, time)

    # determine a desired state for this robot
    state_desired, input_desired = get_reference_state_and_input(prob.reference_path, time, prob.offset)
    
    # run a tracking controller for this robot
    v, ω = tracking_controller(state, state_desired, input_desired)

    # run the cbf-qp
    v, ω = cbf_qp_controller(SVector{3}(state), [v, ω], prob.wezes; α=prob.α, vlims=(0.8, 1.0), ωlims=(-10.0, 10.0), ϵ=1e-5)

    # apply input bounds
    v, ω = apply_input_bounds(v, ω)

    # follower dynamics
    D[1] = v * cos(state[3])
    D[2] = v * sin(state[3])
    D[3] = ω
end


"""
    simulate_closed_loop_cbf(initial_state, tspan, prob::CBFProblem)
    
returns the closed-loop trajectory from running a CBF-QP controller.
"""
function simulate_closed_loop_cbf(initial_state, tspan, prob::CBFProblem)

    # setup the odeproblem
    odeproblem = ODEProblem(closed_loop_CBF!, Vector(initial_state), tspan, prob)

    # # update_committed_callback = PeriodicCallback(update_committed_affect!, prob.switch_step_size)
    # update_committed_callback =
    #     IterativeCallback(time_choice_gatekeeper, update_committed_affect!)

    odesol = solve(
        odeproblem,
        Tsit5();
        dtmax = prob.integration_max_step_size, 
        dt = prob.integration_max_step_size, 
        # callback = update_committed_callback,
    )

    return odesol

end




end