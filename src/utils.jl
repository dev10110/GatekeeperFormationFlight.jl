# Copyright (c) 2025 Devansh R Agrawal - All rights reserved.

using Dubins, LinearAlgebra

"""
    wrapToPi(θ)

returns θ wrapped to lie within (-π, π]
"""
function wrapToPi(x)
    return atan(sin(x), cos(x))
end

"""
    total_path_length(path::Vector{DubinsPath})

returns the total path length of a vector of DubinPath
"""
function total_path_length(path::Vector{DubinsPath})
    return sum(dubins_path_length(p) for p in path)
end

"""
    total_path_length(path::DubinsPath)

returns the total path length of a dubins path
"""
function total_path_length(path::DubinsPath)
    return dubins_path_length(path)
end


"""
    apply_input_bounds(v, ω; v_min = 0.8, v_max = 1.0, R_min = 0.1)

apply input bounds to a v, ω. 
Extracted as a function to make sure input bounds get applied everywhere uniformly.
"""
function apply_input_bounds(v, ω; v_min = 0.8, v_max = 1.0, R_min = 0.1)

    # turning radius is R = 0.1 at v = 1.0, therefore, ω_max = v/r = 10.0
    ω_max = v_max / R_min

    v = clamp(v, v_min, v_max)
    ω = clamp(ω, -ω_max, ω_max)

    return v, ω
end



"""
    get_ω_sign(path::DubinsPath, index)

get the sign of the ω for the `index`-th subpart of a `DubinsPath`
"""
function get_ω_sign(path::DubinsPath, index)
    path_type = path.path_type

    segment_type = Dubins.DIRDATA[path_type][index]

    if segment_type == Dubins.:L_SEG
        return 1
    elseif segment_type == Dubins.:S_SEG
        return 0
    else
        # segment_type == Dubins.:R_SEG
        return -1
    end
end
