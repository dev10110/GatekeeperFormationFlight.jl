"""
Module: Dubins3DTrackingController
========================
This file contains the 3D tracking controller for Dubins paths.

EVERYTHING IN THIS FILE ASSUMES V_MAX is 1.0 -- time, etc. are normalized as such

It is intended to be included by writing:
```
import .dubins_3d_tracking_controller

Dubins3DTrackingController.tracking_controller(...)
Dubins3DTrackingController.get_reference_state_and_input(...)
```
"""

module Dubins3DTrackingController
export tracking_controller, get_reference_state_and_input

using StaticArrays, LinearAlgebra
using Dubins3D

# Define the transformation matrix function locally
# one shot https://en.wikipedia.org/wiki/Rotation_matrix
# function get_transformation_matrix(
#     yaw::F,
#     pitch::F,
#     roll::F,
# )::SMatrix{3,3,F} where {F<:Real}

#     # Precompute trigonometric values
#     ca, sa = cos(yaw), sin(yaw)
#     cb, sb = cos(pitch), sin(pitch)
#     cy, sy = cos(roll), sin(roll)

#     # Directly compute the combined matrix elements
#     return @SMatrix F[
#         ca*cb ca*sb*sy-sa*cy ca*sb*cy+sa*sy
#         sa*cb sa*sb*sy+ca*cy sa*sb*cy-ca*sy
#         -sb cb*sy cb*cy
#     ]
# end

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

function wrapToPi(angle)
    return atan(sin(angle), cos(angle))
end

"""
    x, x_d, u_d; k1=450.0, k2=200.0, k3=50.0, kp_pitch=1.0

returns the '[v, ω_yaw, pitch]' to track a state 'x_d' from current state 'x', given a feedforward term 'u_d'. 
"""
function tracking_controller(
    state,
    state_d,
    input_d;
    k1::F = 100000.0, # velocity error gain
    k2::F = 1000.0, # yaw error gain
    k3::F = 100.0,  # yaw rate error gain
    k4::F = 1.0, # pitch error gain
)::SVector{3,F} where {F<:Real}
    # Extract States
    x, y, z, yaw, pitch = state  # pitch from state, but will be set as control
    xr, yr, zr, yaw_r = state_d
    vr, ωyaw_r, pitch_r = input_d  # pitch_r is the desired pitch input

    # We assume roll is always zero
    pitch_for_path = F(0.0)
    roll = F(0.0)

    # Simplified rotation matrix with roll=0
    R_body_to_world = get_transformation_matrix(yaw, pitch_for_path, roll)

    pos_error_world = @SVector F[xr-x, yr-y, zr-z]

    # Transform error to body frame
    pos_error_body = transpose(R_body_to_world) * pos_error_world
    # pos_error_body = R_body_to_world * pos_error_world

    # Extract Components
    xe, ye, ze = pos_error_body

    # Compute Yaw Orientation Error only
    yaw_e = yaw_r - yaw

    # half angle formulation for yaw
    c_yaw, s_yaw = cos(yaw_e / 2), sin(yaw_e / 2)

    # normalization
    z_norm = sqrt(1 + xe^2 + ye^2 + ze^2)

    # compute the control inputs
    v = vr + k1 * (xe / z_norm)

    # Angular controller for yaw only
    ω_yaw = ωyaw_r + k2 * (ye / z_norm) + k3 * s_yaw

    # Pitch with feedback control based on z-error
    pitch_adjustment = k4 * (ze / z_norm)
    pitch = pitch_r + pitch_adjustment

    return @SVector F[v, ω_yaw, pitch]
end

"""
    pre_extrapolate(path, time)

get the `state, input` for a reference path for negative time (t < 0), v=1.0, ω_all=0.0
"""
function pre_extrapolate(path::AbstractVector{DubinsManeuver3D}, time)
    @assert time < 0

    x0, y0, z0, yaw0, pitch0 = path[1].qi
    roll0 = 0.0

    yaw0 = wrapToPi(yaw0)
    pitch0 = wrapToPi(pitch0)
    roll0 = wrapToPi(roll0)

    # assume v = 1.0
    d = abs(time)

    x = x0 - d * cos(yaw0) * cos(pitch0)
    y = y0 - d * sin(yaw0) * cos(pitch0)
    z = z0 - d * sin(pitch0)

    return SVector(x, y, z, yaw0, pitch0), SVector(1.0, 0.0, pitch0)
end

"""
    post_extrapolate(path, time)

get the `state, input` for a reference path beyond the end of the path (t > end_time), v=1.0, ω_all=0.0
"""
function post_extrapolate(path::AbstractVector{DubinsManeuver3D}, time)
    end_time = sum(x -> x.length, path)

    @assert time > end_time # assumes v = 1.0

    xf, yf, zf, yawf, pitchf = path[end].qf
    rollf = 0.0

    yawf = wrapToPi(yawf)
    pitchf = wrapToPi(pitchf)
    rollf = wrapToPi(rollf)

    d = time - end_time
    x = xf + d * cos(yawf) * cos(pitchf)
    y = yf + d * sin(yawf) * cos(pitchf)
    z = zf + d * sin(pitchf)

    return SVector(x, y, z, yawf, pitchf), SVector(1.0, 0.0, pitchf)
end

"""
    get_reference_state_and_input(path::Vector{DubinsManeuver3D}, time)

get the `(x_d, u_d)` to follow a `path` at some `time`
"""
function get_reference_state_and_input(
    path::Vector{DubinsManeuver3D},
    time::F,
) where {F<:Real}
    segment_lengths = [p.length for p in path]
    cumulative_lengths = cumsum(segment_lengths)

    if time < 0
        return pre_extrapolate(path, time)
    elseif time > cumulative_lengths[end]
        return post_extrapolate(path, time)
    end

    idx = findfirst(time .<= cumulative_lengths)
    @assert !isnothing(idx) "Time is out of bounds"

    # find relative time in the segment
    τ = idx == 1 ? time : time - cumulative_lengths[idx-1]

    # normalize time to the segment length
    τ_fractional = τ / segment_lengths[idx]
    state_idx = clamp(ceil(Int, τ_fractional * 1000), 1, 1000)

    states = compute_sampling(path[idx]; numberOfSamples = 1000)
    state = states[state_idx]
    # state = states[:, state_idx]

    # reference states
    x, y, z, yaw, pitch = state
    yaw = wrapToPi(yaw)

    # estimate ω by taking the difference between the current and next state yaw
    # then estimate the time step by the segment length divided by the number of samples
    Δt = segment_lengths[idx] / 1000

    ω = 0.0
    v = 1.0
    if state_idx < size(states, 2) - 1
        next_state = states[:, state_idx+1]
        Δω = next_state[4] - yaw
        ω = Δω / Δt # needs to change by Δω over Δt
    end

    ω = wrapToPi(ω)

    return @SVector([x, y, z, yaw, pitch]), @SVector([1.0, ω, pitch])
end

"""
    get_reference_state_and_input(path::Vector{DubinsManeuver3D}, time)

get the `(x_d, u_d)` to follow a `path` at some `time`
"""
function get_reference_state_and_input(
    path::Vector{DubinsManeuver3D},
    time::F,
    offset::SVector{3,F},
) where {F<:Real}
    # TODO Time
    x_d, u_d = get_reference_state_and_input(path, time)
    x, y, z, yaw, pitch = x_d
    # pitch = u_d[3]


    R::SMatrix{3,3,F} = get_transformation_matrix(yaw, 0.0, 0.0)
    # @show yaw
    # @show R_body_to_world
    body_offset = R * offset

    # Apply the offset to the position
    x_out =
        @SVector([x + body_offset[1], y + body_offset[2], z + body_offset[3], yaw, pitch])

    return x_out, u_d
end

end # module Dubins3DTrackingController