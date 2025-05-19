"""
File: src/obstacle.jl

This file defines the types and functions for obstacles 
"""

using StaticArrays, LinearAlgebra
using RecipesBase

using ..RobotTypes

abstract type AbstractObstacle end

abstract type AbstractStaticObstacle <: AbstractObstacle end
abstract type AbstractDynamicObstacle <: AbstractObstacle end

"""
returns 'collision_distance(obstacle, robot) <= tolerance'
"""
function is_colliding end

"""
returns the distance to collision for this robot to the obstacle. Positive if safe.
"""
function collision_distance end

"""
    is_colliding(obstacle::AbstractStaticObstacle, v::SVector{3,F}, tol = 0.0)

Base level is_colliding function for a single obstacle.
Checks if a point is within a distance tol of some obstacle
"""
function is_colliding(
    obstacle::O,
    v::VF,
    time::F,
    tol::F = 1e-3,
) where {O<:AbstractObstacle,F<:Real,VF<:AbstractVector{F}}
    return collision_distance(obstacle, v, time) <= tol
end

"""
    is_colliding(obstacle::Vector{AbstractObstacle}, v::AbstractVector{F}, time, tol = 0.0)
"""
function is_colliding(
    obs::VAO,
    v::VF,
    time::F,
    tol::F,
) where {F<:Real,AO<:AbstractObstacle,VAO<:AbstractVector{AO},VF<:AbstractVector{F}}
    return any(o -> is_colliding(o, v, time, tol), obs)
end

"""
Check if a Robot3 is within a distance tol of an obstacle
"""
function is_colliding(obstacle_or_obstacles::OBS, r::Robot3, tol = 0.0) where {OBS}
    return is_colliding(obstacle_or_obstacles, r.pos, tol)
end

"""
    collision_distance(obstacle::AbstractStaticObstacle, x::AbstractVector{F}, tol, time)

Adatper for when static obstacles are used in a dynamic context
"""
function collision_distance(
    obstacle::O,
    x::VF,
    time::F,
) where {O<:AbstractStaticObstacle,F<:Real,VF<:AbstractVector{F}}
    return collision_distance(obstacle, x)
end

"""
    collision_distance(obstacles::Vector{AbstractStaticObstacle}, x::AbstractVector{F}, time::F)

Returns the minimum distance to collision from one point to a vector of obstacles at some time t
"""
function collision_distance(
    obstacles::VO,
    x::VF,
    time::Float64,
) where {O<:AbstractObstacle,VO<:AbstractVector{O},F<:Real,VF<:AbstractVector{F}}
    return minimum(o -> collision_distance(o, x, time), obstacles; init = Inf)
end

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
Cylinder(center::SVector{3,F}) where {F<:Real} = Cylinder(center, E, 10.0)
Cylinder(x::F, y::F, r::F) where {F<:Real} = Cylinder(SVector{3,F}([x, y, 0.0]), r)

"""
    collision_distance(center::Cylinder, x::SVector{3,F})

Returns the distance to collision from a point x to an infinite height cylinder
"""
function collision_distance(c::Cylinder, x::VF) where {F<:Real,VF<:AbstractVector{F}}
    diff = @SVector F[c.center[1]-x[1], c.center[2]-x[2]]
    return norm(diff) - c.radius
end

###############################################################
### Time Varying Sphere########################################
###############################################################
struct TimeVaryingSphere{F} <: AbstractDynamicObstacle where {F<:Real}
    pos_at_t::Function  # Function t::F -> SVector{3, F}
    radius::F
end

function TimeVaryingSphere(gk_solution, radius::F) where {F<:Real}
    pos_at_t = t -> SVector{3,F}(gk_solution(t)[1], gk_solution(t)[2], gk_solution(t)[3])
    return TimeVaryingSphere(pos_at_t, radius)
end

function collision_distance(
    s::TimeVaryingSphere,
    x::AbstractVector{F},
    t::Float64,
) where {F<:Real}
    return norm(s.pos_at_t(t) - x) - s.radius
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