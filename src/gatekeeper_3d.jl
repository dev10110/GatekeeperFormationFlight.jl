using Dubins3D
using StaticArrays, LinearAlgebra


@kwdef struct GatekeeperProblem3D{TVO,TR,TOF,F}
    obstacles::TVO # list of obstacles 
    reference_path::TR # reference trajectory of the leader
    offsets::TOF = Vector(SVector(0.0, 0.0, 0.0)) # vector of desired offsets from the leader
    turning_radius::F = 10.0 # turning radius
    pitch_limits = (-deg2rad(10), deg2rad(15)) # pitch limits
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
# struct compositetrajectory{tn,tb,ts}
#     nominal_trajectory::tn
#     backup_trajectory::tb
#     switch_time::ts
# end

"""
    simulate_closed_loop_gatekeeper(initial_state, tspan, prob::GatekeeperProblem)

param 

returns the closed-loop trajectory from running gatekeeper
"""
function simulate_closed_loop_gatekeeper(prob::GatekeeperProblem3D, tspan)
    # Check if reference path is safe
    # if is_colliding(prob.obstacles, prob.reference_path)
    #     @warn "Reference path is not safe"
    #     return nothing
    # end

    @show prob.reference_path
    nominal_trajectories = compute_nominal_trajectories(prob)
end


"""
    compute_nominal_trajectoreis(prob::GatekeeperProblem3D)

Compute the nominal trajectories for all agents in a 3D Gatekeeper formation-flight problem.

# Arguments
- `prob::GatekeeperProblem3D`
  The problem instance, containing initial conditions, waypoints,
  dynamics models, timing parameters, and any inter-agent constraints.

# Returns
- `trajectories`
  A collection (e.g., `Vector` of custom `Trajectory` objects or named tuples)
  where each element holds the time-parameterized nominal states
  (positions, velocities, orientations, etc.) for one agent.
"""
function compute_nominal_trajectories(prob::GatekeeperProblem3D)
    # compute the nominal trajectories based on the reference path and offsets
    # returns a list of nominal trajectories
    nominal_trajectories = []

    for offset in prob.offsets
        nominal_trajectory = compute_nominal_trajectory(prob.reference_path, offset)
        push!(nominal_trajectories, nominal_trajectory)
    end

    return nominal_trajectories
end

function compute_nominal_trajectory(reference_path, offset::SVector{3,Float64})
    # compute the nominal trajectory based on the reference path and offset
    # projects the offset into the plane of the point on the reference path
    nominal_path = Vector{SVector{6,Float64}}()
    @show reference_path[1, :]
    @show offset

    # reference path is of shape (N, 5)
    # where N is the number of points in the path

    # iterate through each point
    for p in eachrow(reference_path)
        # p is a 5D vector (x, y, z, yaw, pitch, roll) because not using roll...so pretend its 0
        # offset is a 3D vector (x, y, z)

        # basically need to get the coordinate frame of the reference point
        # apply the transformation defined by the coordinate frame to the offset
        # then add the offset to the reference point
        transformation_matrix = get_transformation_matrix(p[4], p[5], 0.0)
        projected_offset = transformation_matrix * offset

        @show p[4], p[5]
        @show projected_offset

        # compute the new point on the nominal trajectory
        new_point = SVector{6,Float64}(
            p[1] + projected_offset[1],
            p[2] + projected_offset[2],
            p[3] + projected_offset[3],
            p[4], # yaw
            p[5], # pitch
            0.0,  # roll
        )

        # add the new point to the nominal trajectory
        push!(nominal_path, new_point)
    end

    return reduce(hcat, nominal_path)'  # Convert to matrix with points × 6 dimensions
end

function get_transformation_matrix(yaw, pitch, roll)
    # COORDINATE SYSTEM CONVENTION:
    # - Yaw: rotation around Z-axis (positive = right/clockwise from above)
    # - Pitch: rotation around Y-axis (positive = nose up)
    # - Roll: rotation around X-axis (positive = right wing down)

    yaw_transform = [
        cos(yaw) -sin(yaw) 0
        sin(yaw) cos(yaw) 0
        0 0 1
    ]
    pitch_transform = [
        cos(pitch) 0 sin(pitch)
        0 1 0
        -sin(pitch) 0 cos(pitch)
    ]
    roll_transform = [
        1 0 0
        0 cos(roll) -sin(roll)
        0 sin(roll) cos(roll)
    ]
    # Note: The order of transformations matters.
    # Combine the transformations
    # transformation_matrix = roll_transform * pitch_transform * yaw_transform
    # transformation_matrix = yaw_transform * pitch_transform * roll_transform
    transformation_matrix = roll_transform * pitch_transform * yaw_transform
    return transformation_matrix
end