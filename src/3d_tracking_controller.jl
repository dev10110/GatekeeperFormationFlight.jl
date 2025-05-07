using StaticArrays, LinearAlgebra

"""
    x, x_d, u_d; k1=450.0, k2=200.0, k3=50.0

returns the '[v, ω_yaw, ω_pitch, ω_roll]' to track a state 'x_d' from current state 'x', given a feedforward term 'u_d'. 
"""
function tracking_controller(
    state,
    state_d,
    input_d;
    k1 = 450.0,
    k2 = 200.0,
    k3 = 50.0,
    k4 = 200.0,
    k5 = 50.0,
    k6 = 200.0,
    k7 = 50.0,
)
    # Extract States
    x, y, z, yaw, pitch, roll = state
    xr, yr, zr, yaw_r, pitch_r, roll_r = state_d
    vr, ωyaw_r, ωpitch_r, ωroll_r = input_d

    R_body_to_world = rotation_matrix(yaw, pitch, roll)

    pos_error_world = SVector(xr - x, yr - y, zr - z)

    # Transform error to body frame
    pos_error_body = R_body_to_world' * pos_error_world

    # Extract Components
    xe, ye, ze = pos_error_body

    # Compute Orientation Errors
    yaw_e = yaw_r - yaw
    pitch_e = pitch_r - pitch
    roll_e = roll_r - roll

    # half angle formulation
    c_yaw, s_yaw = cos(yaw_e / 2), sin(yaw_e / 2)
    c_pitch, s_pitch = cos(pitch_e / 2), sin(pitch_e / 2)
    c_roll, s_roll = cos(roll_e / 2), sin(roll_e / 2)

    z_norm = sqrt(1 + xe^2 + ye^2 + ze^2)

    # compute the control inputs
    v = vr + k1 * (xe / z_norm)

    # Angular controllers with proper coupling
    ω_yaw = ωyaw_r + k2 * (vr + (ye * c_yaw - xe * s_yaw) / z_norm) + k3 * s_yaw

    ω_pitch = ωpitch_r + k4 * (vr + (ze * c_pitch - xe * s_pitch) / z_norm) + k5 * s_pitch
    ω_roll = ωroll_r + k6 * (vr * (ze * c_roll - ye * s_roll) / z_norm) + k7 * s_roll

    return @SVector [v, ω_yaw, ω_pitch, ω_roll]
end

"""
    pre_extrapolate(path, time)

get the `state, input` for a reference path for negative time (t < 0), v=1.0, ω_all=0.0
"""
function pre_extrapolate(path::Vector{DubinsPath}, time)
    @assert time < 0

    x0, y0, z0, yaw0, pitch0, roll0 = path[1].qi

    yaw0 = wrapToPi(yaw0)
    pitch0 = wrapToPi(pitch0)
    roll0 = wrapToPi(roll0)

    # assume v = 1.0
    d = abs(time)

    x = x0 - d * cos(yaw0) * cos(pitch0)
    y = y0 - d * sin(yaw0) * cos(pitch0)
    z = z0 - d * sin(pitch0)

    return SVector(x, y, z, yaw0, pitch0, roll0), SVector(1.0, 0.0, 0.0, 0.0)
end

"""
    post_extrapolate(path, time)

get the `state, input` for a reference path beyond the end of the path (t > end_time), v=1.0, ω_all=0.0
"""
function post_extrapolate(path::Vector{DubinsPath}, time)
    end_time = total_path_length(path)

    @assert time > end_time # assumes v = 1.0

    xf, yf, zf, yawf, pitchf, rollf = path[end].qi

    yawf = wrapToPi(yawf)
    pitchf = wrapToPi(pitchf)
    rollf = wrapToPi(rollf)

    d = time - end_time
    x = xf + d * cos(yawf) * cos(pitchf)
    y = yf + d * sin(yawf) * cos(pitchf)
    z = zf + d * sin(pitchf)

    return SVector(x, y, z, yawf, pitchf, rollf), SVector(1.0, 0.0, 0.0, 0.0)
end

"""
    get_reference_state_and_input(path::Vector{DubinsPath}, time)

get the `(x_d, u_d)` to follow a `path` at some `time`
"""
function get_reference_state_and_input(path::Vector{DubinsPath}, time)

end
