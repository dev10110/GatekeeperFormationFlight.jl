
using StaticArrays, LinearAlgebra
using RecipesBase

abstract type AbstractObstacle end

"""
    is_colliding(obstacle::AbstractObstacle, robot::Robot, tolerance)
    is_colliding(obstacles::Vector{AbstractObstacle}, robot::Robot, tolerance)

returns 'collision_distance(obstacle, robot) <= tolerance'
"""
function is_colliding end

"""
    collision_distance(obstacle::AbstractObstacle, robot::Robot)
    collision_distance(obstacles::Vector{AbstractObstacle}, robot::Robot)

returns the distance to collision for this robot to the obstacle. Positive if safe.
"""
function collision_distance end


"""
Check if a 3D point is within a distance tol of an obstacle
"""
function is_colliding(obstacle::O, v::SVector{3,F}, tol = 0) where {O<:AbstractObstacle,F}
    return collision_distance(obstacle, v) <= tol
end

"""
Check if a Robot3 is within a distance tol of an obstacle
"""
function is_colliding(obstacle::O, r::Robot3, tol = 0) where {O<:AbstractObstacle}
    return is_colliding(obstacle, r.pos, tol)
end

"""
Check if a 3D point is within a distance tol of any obstacle in the vector obstacles
"""
function is_colliding(
    obstacles::Vector{O},
    v::SVector{3,F},
    tol = 0,
) where {O<:AbstractObstacle,F}
    return any(obs -> is_colliding(obs, v, tol), obstacles)
end

"""
Returns true if the robot is within distasnce tol of any obstacle o ∈ obstacles
"""
function is_colliding(obstacles::Vector{O}, r::Robot3, tol = 0) where {O<:AbstractObstacle}
    return is_colliding(obstacles, r.pos, tol)
end

function is_colliding(
    obstacles::Vector{O},
    v::Vector{F},
    tol = 0,
) where {O<:AbstractObstacle,F}
    @assert length(v) >= 3

    return is_colliding(obstacles, SVector{3,F}(v[1:3]...), tol)
end

"""
    collision_distance(obstacle::AbstractObstacle, robot::Robot)
    collision_distance(obstacles::Vector{AbstractObstacle}, robot::Robot)
end
"""




"""
    collision_distance(obstacle::AbstractObstacle, robot::Robot)
"""

###############################################################
### Spherical #################################################
###############################################################
"""
    Sphere(x, y, z, R)

Constructs a static spherical obstacle
"""
struct Sphere{F} <: AbstractObstacle
    pos::SVector{3,F}
    R::F
end

function Sphere(x::F, y::F, z::F, r::F) where {F}
    return Sphere(SVector{3,F}([x, y, z]), r)
end

"""
    collision_distance(s::Sphere, x::SVector{3,F})
"""
function collision_distance(s::Sphere, x::SVector{3,F}) where {F}
    # get the distance to the center of the sphere
    d = sqrt((s.pos - x)' * (s.pos - x))

    # return the distance to collision
    return d - s.R
end

###############################################################
### Cylindrical ###############################################
###############################################################
"""
    Cylinder(O::SVector{3, F}, E::SVector{3, F}, R::F)

Constructs a static cylindrical obstacle, with origin O, extemum E (vector representing the axis of the cylindar) and radius R
"""
struct Cylinder{F} <: AbstractObstacle
    O::SVector{3,F}
    E::SVector{3,F}
    R::F
end
Cylinder(O::SVector{3,F}, E::SVector{3,F}) where {F} = Cylinder(O, E, convert(F, 0.15))

## TODO



###############################################################
### Plotting ##################################################
###############################################################
@recipe function plot_sphere(s::O) where {O<:Sphere}
    seriestype := :surface
    alpha --> 0.8

    u = range(0, 2π, length = 30)
    v = range(0, π, length = 30)

    x = s.pos[1] .+ s.R * cos.(u) * sin.(v)'
    y = s.pos[2] .+ s.R * sin.(u) * sin.(v)'
    z = s.pos[3] .+ s.R * ones(length(u)) * cos.(v)'

    x, y, z
end
