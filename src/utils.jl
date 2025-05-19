# Copyright (c) 2025 Devansh R Agrawal - All rights reserved.

using Dubins, LinearAlgebra, StaticArrays

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

    # @todo "this errors when using custom Dubins, not when using the original Dubins impl"
    segment_type = nothing
    try
        segment_type = Dubins.DIRDATA[Int(path_type)][index]
    catch
        segment_type = Dubins.DIRDATA[path_type][index]
    end
    # segment_type = Dubins.DIRDATA[Int(path_type)][index]

    if segment_type == Dubins.:L_SEG
        return 1
    elseif segment_type == Dubins.:S_SEG
        return 0
    else
        # segment_type == Dubins.:R_SEG
        return -1
    end
end

"""
    get_transformation_matrix(yaw, pitch, roll)

returns the transformation matrix for a given yaw, pitch, roll
from the body frame to the inertial frame
"""
# function get_transformation_matrix(yaw, pitch, roll)
#     # COORDINATE SYSTEM CONVENTION:
#     # - Yaw: rotation around Z-axis (positive = right/clockwise from above)
#     # - Pitch: rotation around Y-axis (positive = nose up)
#     # - Roll: rotation around X-axis (positive = right wing down)

#     yaw_transform = [
#         cos(yaw) -sin(yaw) 0
#         sin(yaw) cos(yaw) 0
#         0 0 1
#     ]
#     pitch_transform = [
#         cos(pitch) 0 sin(pitch)
#         0 1 0
#         -sin(pitch) 0 cos(pitch)
#     ]
#     roll_transform = [
#         1 0 0
#         0 cos(roll) -sin(roll)
#         0 sin(roll) cos(roll)
#     ]
#     # Note: The order of transformations matters.
#     # Combine the transformations
#     # transformation_matrix = roll_transform * pitch_transform * yaw_transform
#     # transformation_matrix = yaw_transform * pitch_transform * roll_transform
#     transformation_matrix = roll_transform * pitch_transform * yaw_transform
#     return transformation_matrix
# end

function get_transformation_matrix(
    yaw::F,
    pitch::F,
    roll::F,
)::SMatrix{3,3,F} where {F<:Real}
    # Precompute trigonometric values
    ca, sa = cos(yaw), sin(yaw)
    cb, sb = cos(pitch), sin(pitch)
    cy, sy = cos(roll), sin(roll)

    # Directly compute the combined matrix elements
    return @SMatrix F[
        ca*cb ca*sb*sy-sa*cy ca*sb*cy+sa*sy
        sa*cb sa*sb*sy+ca*cy sa*sb*cy-ca*sy
        -sb cb*sy cb*cy
    ]
end