using StaticArrays, LinearAlgebra
using RecipesBase

"""
    Robot(x, y, z, yaw, pitch, roll)
    Robot([x, y, z], [yaw, pitch, roll])

Create a robot with position (x, y, z) and rotation (yaw, pitch, roll)
"""
struct Robot3{F}
    pos::SVector{3,F} # x, y, z
    rot::SVector{3,F} # yaw, pitch, roll
end

function Robot3(pos::VF, rot::VF) where {F,VF<:AbstractVector{F}}
    @assert length(pos) == 3
    @assert length(rot) == 3
    return Robot3(SVector{3,F}(pos), SVector{3,F}(rot))
end

function Robot3(pose::SVector{5,F}) where {F}
    return Robot3(
        SVector{3,F}(pose[1], pose[2], pose[3]),
        SVector{3,F}(pose[4], pose[5], 0.0),
    )
end

function Robot3(pose::VF) where {F,VF<:AbstractVector{F}}
    @assert length(pose) == 5
    return Robot3(
        SVector{3,F}(pose[1], pose[2], pose[3]),
        SVector{3,F}(pose[4], pose[5], 0.0),
    )
end

function StaticArrays.SVector(r::Robot3{F}) where {F}
    return SVector{6,F}(r.pos..., r.rot...)
end


@recipe function plot_robot3!(r::Robot3)
    # plot the origin marker
    @series begin
        seriestype := :scatter
        label -> false
        color -> :black
        marker -> :dot
        markersize -> 5
        aspect_ratio -> :equal
        [r.pos[1]], [r.pos[2]], [r.pos[3]]
    end

    # normalize the rotation vector
    rot_unit_vector = normalize(r.rot)

    # plot arrow in direction of the rotation vector
    @series begin
        seriestype := :path
        linewidth -> 2
        arrowhead_length -> 0.1
        arrowhead_width -> 0.1
        label -> false
        color -> :black
        marker -> false
        arrow -> true
        aspect_ratio -> :equal
        [r.pos[1], r.pos[1] + 0.5 * rot_unit_vector[1]],
        [r.pos[2], r.pos[2] + 0.5 * rot_unit_vector[2]],
        [r.pos[3], r.pos[3] + 0.5 * rot_unit_vector[3]]
    end
end

