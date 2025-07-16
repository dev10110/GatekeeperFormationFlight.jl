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

    # normalize the rotation vector
    # Convert yaw, pitch, roll to a direction unit vector (assuming ZYX order)
    yaw, pitch, roll = r.rot
    # Direction vector in robot's local frame (forward along x)
    local_dir = SVector(1.0, 0.0, 0.0)

    # Create rotation matrices using LinearAlgebra
    # Rotation around z-axis (yaw)
    sz, cz = sin(yaw), cos(yaw)
    Rz = SMatrix{3,3}([cz -sz 0; sz cz 0; 0 0 1])

    # Rotation around y-axis (pitch)
    sy, cy = sin(pitch), cos(pitch)
    Ry = SMatrix{3,3}([cy 0 sy; 0 1 0; -sy 0 cy])

    # Rotation around x-axis (roll)
    sx, cx = sin(roll), cos(roll)
    Rx = SMatrix{3,3}([1 0 0; 0 cx -sx; 0 sx cx])

    # Apply rotations: R = Rz * Ry * Rx (ZYX order)
    global_dir = Rz * Ry * Rx * local_dir
    rot_unit_vector = normalize(global_dir)
    arrow_length = 15.0
    # Calculate endpoint of the direction arrow
    end_point = r.pos .+ arrow_length * rot_unit_vector

    @series begin
        seriestype := :path3d
        label := false
        color := :black
        aspect_ratio := :equal
        linewidth := 1.5
        opacity := 0.8
        # For 3D lines, we need x, y, z vectors where each element corresponds to a point
        [r.pos[1], end_point[1]], [r.pos[2], end_point[2]], [r.pos[3], end_point[3]]
    end


    # plot the origin marker
    @series begin
        seriestype := :scatter
        label := false
        # color := :black
        marker := :dot
        markersize := 5
        aspect_ratio := :equal
        [r.pos[1]], [r.pos[2]], [r.pos[3]]
    end
end

