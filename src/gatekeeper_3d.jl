

using Dubins3D
using StaticARrays, LinearAlgebra


@kwdef struct GatekeeperProblem3D{TVO,TR,TOF}
    obstacles::TVO # list of obstacles 
    reference_path:TR # reference trajectory of the leader
    offset::TOF = Vector(SVector(0.0, 0.0, 0.0)) # vector of desired offsets from the leader
    turning_radius::F = 10.0 # turning radius
    pitch_limits::F = (-deg2rad(10), deg2rad(15)) # pitch limits
    max_Ts_horizon::F = 1.0 # maximum switching time
    reconnection_step_size::F = 0.1 # resolution for checking reconnection points
    collision_check_step_size::F = 0.01 # resolution for collision checking
    integration_max_step_size::F = 0.05 # max integration step size in nominal tracking
    switch_step_size::F = 0.05 # resolution used to decrease switching time
end

"""
    CompositeTrajectory(nominal_trajectory, backup_trajectory, switch_time)

defines a composite trajectory that includes a nominal trjectory, a backup trajectory, and a switch time
"""
struct CompositeTrajectory{TN,TB,TS}
    nominal_trajectory::TN
    backup_trajectory::TB
    switch_time::TS
end

"""
    simulate_closed_loop_gatekeeper(initial_state, tspan, prob::GatekeeperProblem)

param 

returns the closed-loop trajectory from running gatekeeper
"""
function simulate_closed_loop_gatekeeper(prob::GatekeeperProblem3D, tspan)
    # Check if reference path is safe
    if is_colliding(prob.obstacles, prob.reference_path)
        @warn "Reference path is not safe"
        return nothing
    end

    # Construct first committed trajectories
    

end