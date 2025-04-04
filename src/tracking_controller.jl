using StaticArrays, LinearAlgebra

"""
    tracking_controller(x, x_d, u_d; k1=450.0, k2=200.0, k3=50.0)

returns the `[v, ω]` to track a state `x_d` from current state `x`, given a feedforward term `u_d`. 
"""
function tracking_controller(state, state_d, input_d; k1 = 450.0, k2 = 200.0, k3 = 50.0)

    x, y, ψ = state
    xr, yr, ψr = state_d
    vr, ωr = input_d

    # compute the error in the body fixed frame
    xe = cos(ψ) * (xr - x) + sin(ψ) * (yr - y)
    ye = -sin(ψ) * (xr - x) + cos(ψ) * (yr - y)
    ψe = ψr - ψ

    # useful quantities
    c = cos(ψe / 2)
    s = sin(ψe / 2)
    z = sqrt(1 + xe^2 + ye^2)

    # compute the control inputs
    v = vr + k1 * (xe / z)
    ω = ωr + k2 * (vr * (ye * c - xe * s) / z) + k3 * s

    return @SVector [v, ω]
end


"""
    pre_extrapolate(path, time)

return the `state, input` for a reference path for negative time. Assumes the robot moves with v=1.0, ω=0.0.
"""
function pre_extrapolate(path::Vector{DubinsPath}, time)
    @assert time < 0

    x0, y0, ψ0 = path[1].qi
    ψ0 = wrapToPi(ψ0)

    # assume velocity = 1.0
    d = abs(time)

    x = x0 - d * cos(ψ0)
    y = y0 - d * sin(ψ0)

    return SVector(x, y, ψ0), SVector(1.0, 0.0)
end

"""
    post_extrapolate(path, time)

return the `state, input` for a reference path beyond the end of the path, assumes the robot moves with v=1.0, ω=0.0.
"""
function post_extrapolate(path, time)
    end_time = total_path_length(path)

    @assert time > end_time

    errcode, (xf, yf, ψf) = dubins_path_endpoint(path[end])
    @assert errcode == Dubins.EDUBOK

    d = time - end_time

    x = xf + d * cos(ψf)
    y = yf + d * sin(ψf)

    ψf = wrapToPi(ψf)

    return SVector(x, y, ψf), SVector(1.0, 0.0)
end

"""
    get_reference_state_and_input(path::Vector{DubinsPath}, time)

get the `(x_d, u_d)` to follow a `path` at some `time`
"""
function get_reference_state_and_input(path::Vector{DubinsPath}, time)

    segment_lengths = [dubins_path_length(p) for p in path]
    cummulative_lengths = cumsum(segment_lengths)

    #     # check that we are querying within the horizons
    #     @assert 0 <= time <= cummulative_lengths[end]

    if time < 0
        return pre_extrapolate(path, time)
    end

    if time > cummulative_lengths[end]
        return post_extrapolate(path, time)
    end

    # determine which segment we are in
    idx = findfirst(time .<= cummulative_lengths)
    @assert !isnothing(idx)

    # find the relative time in that segment
    τ = idx == 1 ? time : time - cummulative_lengths[idx-1]

    # get the state in that segment
    errcode, state = dubins_path_sample(path[idx], τ)
    @assert errcode == Dubins.EDUBOK

    # check that the angles are appropriately wrapped
    x, y, ψ = state
    ψ = wrapToPi(ψ)

    # now we need to check which subsegment it is in
    subsegment_lengths = [dubins_segment_length(path[idx], i) for i = 1:3]
    sub_idx = findfirst(τ .<= cumsum(subsegment_lengths))
    if isnothing(sub_idx)
        sub_idx = 3
    end

    # now look up the path type
    path_type = dubins_path_type(path[idx])

    # since the travel speed is 1.0 units
    r = path[idx].ρ
    v = 1.0
    ω = get_ω_sign(path[idx], sub_idx) * v / r

    return SVector(x, y, ψ), SVector(v, ω)
end


"""
    get_reference_state_and_input(path::Vector{DubinsPath}, time, offset)

get the `(x_d, u_d)` to follow a `path` at some `time`, with a given `offset`.
"""
function get_reference_state_and_input(ref_path, time, offset)

    d_tangential, d_normal = offset

    # assuming v = 1, 
    t_offset = d_tangential

    # grab the ref state
    state, input = get_reference_state_and_input(ref_path, time + t_offset)
    x, y, ψ = state

    # get the direction vector
    s = SVector(cos(ψ), sin(ψ))

    # rotate it
    R = @SMatrix [[0;; -1]; [1;; 0.0]]
    n = R * s

    # compute the offset state
    ox, oy = SVector(x, y) + n * d_normal

    # return the offset control inputs
    # TODO(Dev): check if the input needs to be changed
    return SVector(ox, oy, ψ), input

end
