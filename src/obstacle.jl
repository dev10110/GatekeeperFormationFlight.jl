
using StaticArrays, LinearAlgebra
using RecipesBase

abstract type AbstractObstacle end

abstract type AbstractStaticObstacle <: AbstractObstacle end

"""
    is_colliding(obstacle::AbstractStaticObstacle, robot::Robot, tolerance)
    is_colliding(obstacles::Vector{AbstractStaticObstacle}, robot::Robot, tolerance)

returns 'collision_distance(obstacle, robot) <= tolerance'
"""
function is_colliding end

"""
    collision_distance(obstacle::AbstractStaticObstacle, robot::Robot)
    collision_distance(obstacles::Vector{AbstractStaticObstacle}, robot::Robot)

returns the distance to collision for this robot to the obstacle. Positive if safe.
"""
function collision_distance end


"""
Check if a 3D point is within a distance tol of an obstacle
"""
function is_colliding(
    obstacle::O,
    v::SVector{3,F},
    tol = 0.0,
) where {O<:AbstractStaticObstacle,F}
    return collision_distance(obstacle, v) <= tol
end

"""
Check if a Robot3 is within a distance tol of an obstacle
"""
function is_colliding(obstacle::O, r::Robot3, tol = 0.0) where {O<:AbstractStaticObstacle}
    return is_colliding(obstacle, r.pos, tol)
end

"""
Check if a 3D point is within a distance tol of any obstacle in the vector obstacles
"""
function is_colliding(
    obstacles::Vector{O},
    v::SVector{3,F},
    tol = 0.0,
) where {O<:AbstractStaticObstacle,F}
    return any(obs -> is_colliding(obs, v, tol), obstacles)
end

"""
Returns true if the robot is within distasnce tol of any obstacle o ∈ obstacles
"""
function is_colliding(
    obstacles::Vector{O},
    r::Robot3,
    tol = 0.0,
) where {O<:AbstractStaticObstacle}
    return is_colliding(obstacles, r.pos, tol)
end

function is_colliding(
    obstacles::Vector{O},
    v::Vector{F},
    tol = 0.0,
) where {O<:AbstractStaticObstacle,F}
    @assert length(v) >= 3

    return is_colliding(obstacles, SVector{3,F}(v[1:3]...), tol)
end

"""
    collision_distance(obstacle::AbstractStaticObstacle, robot::Robot)
    collision_distance(obstacles::Vector{AbstractStaticObstacle}, robot::Robot)
end
"""




"""
    collision_distance(obstacle::AbstractStaticObstacle, robot::Robot)
"""

###############################################################
### Spherical #################################################
###############################################################
"""
    Sphere(x, y, z, R)

Constructs a static spherical obstacle
"""
struct Sphere{F} <: AbstractStaticObstacle
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
### Vertical Cylinder ###############################################
###############################################################
"""
    VerticalCylinder(O::SVector{3, F}, R::F)

Constructs a static vertical cylindrical obstacle, with origin O and radius R
"""
struct Cylinder{F} <: AbstractStaticObstacle where {F<:Real}
    center::SVector{3,F}
    radius::F
end
Cylinder(center::SVector{3,F}) where {F} = Cylinder(center, E, 10.0)
Cylinder(x::F, y::F, r::F) where {F} = Cylinder(SVector{3,F}([x, y, 0.0]), r)

"""
    collision_distance(center::Cylinder, x::SVector{3,F})
"""
function collision_distance(c::Cylinder, x::SVector{3,F}) where {F}
    # get the distance to the vertical line that defines the cylinder
    # dot product with the normal vector

    # Infinite vertical cylinder...
    diff = c.center[1:2] - x[1:2]
    return norm(diff) - c.radius

    # n = @SVector [0.0, 0.0, 1.0]
    # ||(center - x) - ((center - x) dot n) * n||

    # vec = diff - (diff' * n) * n
    # return norm(vec) - c.radius
end


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

# @recipe function plot_cylinder(c::Cylinder)
#     seriestype := :surface
#     alpha --> 0.8

#     u = range(0, 2π, length = 30) # Angular parameter (full circle)
#     v = range(-c.radius, c.radius, length = 30) # Radial parameter (problem!)

#     x = c.center[1] .+ c.radius * cos.(u) * ones(length(v))'
#     y = c.center[2] .+ c.radius * sin.(u) * ones(length(v))'
#     z = c.center[3] .+ ones(length(u)) * v'

#     x, y, z
# end

@recipe function plot_cylinder(c::Cylinder, height = 100.0)  # Add height parameter
    # 3D Cylinder
    seriestype := :surface
    colorbar --> false
    label --> false
    alpha := 0.5

    u = range(0, 2π, length = 30)  # Angular parameter (around cylinder)
    h = range(0, height, length = 30)  # Height parameter (up the cylinder)

    # Create a proper cylinder with constant radius
    x = c.center[1] .+ c.radius * cos.(u) * ones(length(h))'
    y = c.center[2] .+ c.radius * sin.(u) * ones(length(h))'
    z = c.center[3] .+ ones(length(u)) * h'  # This makes it vertical

    return x, y, z
end

struct PlotCircle{F}
    center::SVector{2,F}
    radius::F
end
PlotCircle(cylinder::Cylinder) = PlotCircle(
    SVector{2,Float64}([cylinder.center[1], cylinder.center[2]]),
    cylinder.radius,
)


@recipe function plot_circle(c::PlotCircle)
    seriestype := :path
    alpha --> 0.8
    color --> :blue
    linewidth --> 2

    u = range(0, 2π, length = 30)

    x = c.center[1] .+ c.radius * cos.(u)
    y = c.center[2] .+ c.radius * sin.(u)

    return x, y
end