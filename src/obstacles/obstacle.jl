"""
File: src/obstacle.jl

This file defines the types and functions for obstacles 
"""

using StaticArrays, LinearAlgebra
using RecipesBase, Plots

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

# function collision_distance(
#     obstacle::O,
#     x::VF,
# ) where {O<:AbstractStaticObstacle,F<:Real,VF<:AbstractVector{F}}
#     return collision_distance(obstacle, @SVector [x[1], x[2], x[3]])
# end

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

function collision_distance(s::Sphere, x::AbstractVector{F}) where {F<:Real}
    diff = @SVector [x[1] - s.pos[1], x[2] - s.pos[2], x[3] - s.pos[3]]
    return norm(diff) - s.R
end

struct GroundedHalfDome{F} <: AbstractStaticObstacle where {F<:Real}
    pos::SVector{2,F}
    R::F
end

function GroundedHalfDome(x::F, y::F, r::F) where {F<:Real}
    return GroundedHalfDome(SVector{2,F}([x, y]), r)
end

function collision_distance(s::GroundedHalfDome, x::AbstractVector{F}) where {F<:Real}
    # Calculate the distance to the center of the dome
    diff = @SVector [x[1] - s.pos[1], x[2] - s.pos[2], x[3]]
    return norm(diff) - s.R
end

function collision_distance(s::GroundedHalfDome, x::SVector{3,F}) where {F<:Real}
    diff = @SVector [x[1] - s.pos[1], x[2] - s.pos[2], x[3]]
    return norm(diff) - s.R
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

function TimeVaryingSphere(pos_at_t::Function, radius::F) where {F<:Real}
    return TimeVaryingSphere{F}(pos_at_t, radius)
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
    pos = s.pos_at_t(t)
    n = length(pos)

    # Q = @SMatrix Float64[
    #     1.0 0.0 0.0
    #     0.0 1.0 0.0
    #     0.0 0.0 0.0
    # ]

    # Avoid allocation: use view if slicing is needed
    # x_n = (length(x) == n) ? x : @view x[1:n]
    # diff = pos .- x_n
    diff = pos[SOneTo(2)] - x[SOneTo(2)]
    return norm(diff) - s.radius
    # return norm(Q * diff) - s.radius

    # return norm(pos .- x_n) - s.radius
end

###############################################################
### FPV Gate ########################################
###############################################################
"""
FPV Gate
----
Always aligned on the x-axis. Blocks entire y and z axis other than cylinder
through.
"""
struct FPVGate{F} <: AbstractStaticObstacle where {F<:Real}
    pos::SVector{3,F}
end

function FPVGate(x::F, y::F, z::F) where {F<:Real}
    return FPVGate(SVector{3,F}([x, y, z]))
end

const FPVGateWidth::Float64 = 0.9  # Actually 0.7, padded for safety
const FPVGateAllowedRadius::Float64 = 0.05  # Radius of the cylinder through the gate

function collision_distance(g::FPVGate, x::AbstractVector{F}, time::F) where {F<:Real}
    # first check distance to be in the plane defined by the gate
    plane_dist = abs(x[1] - g.pos[1]) - (FPVGateWidth / 2.0)

    if plane_dist > FPVGateAllowedRadius
        return plane_dist
    end

    # Create cylinder of free space through the gate
    axis_dist = FPVGateAllowedRadius - norm(@SVector [x[2] - g.pos[2], x[3] - g.pos[3]])
    return max(plane_dist, axis_dist)
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

@recipe function plot_dome(s::O) where {O<:GroundedHalfDome}
    seriestype := :surface
    colorbar --> false
    label --> false
    alpha --> 0.3

    u = range(0, 2π, length = 30)
    v = range(0, π / 2, length = 30)  # Only upper hemisphere

    x = s.pos[1] .+ s.R * cos.(u) * sin.(v)'
    y = s.pos[2] .+ s.R * sin.(u) * sin.(v)'
    z = s.R * ones(length(u)) * cos.(v)'  # z position is zero at base

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
    alpha := 0.3

    u = range(0, 2π, length = 30)  # Angular parameter (around cylinder)
    h = range(0, height, length = 30)  # Height parameter (up the cylinder)

    # Create a proper cylinder with constant radius
    x = c.center[1] .+ c.radius * cos.(u) * ones(length(h))'
    y = c.center[2] .+ c.radius * sin.(u) * ones(length(h))'
    z = c.center[3] .+ ones(length(u)) * h'  # This makes it vertical

    return x, y, z
end

@recipe function plot_gate(g::FPVGate, height = FPVGateWidth)
    # Plot a horizontal cylinder (aligned with x-axis) representing the gate passthrough
    seriestype := :surface
    colorbar --> false
    label --> false
    alpha := 0.3

    n_points = 60
    # Cylinder parameters
    x = range(g.pos[1] - height / 2, g.pos[1] + height / 2, length = n_points)
    theta = range(0, 2π, length = n_points)

    # Meshgrid for cylinder
    X = repeat(x', n_points, 1)
    Y = g.pos[2] .+ FPVGateAllowedRadius * cos.(theta) * ones(length(x))'
    Z = g.pos[3] .+ FPVGateAllowedRadius * sin.(theta) * ones(length(x))'

    return X, Y, Z
end

struct FPVGate2D{F<:Real}
    pos::SVector{3,F}
    color::Any
end

@recipe function plot_fpvgate2d(g::FPVGate2D)
    # seriestype := :shape
    # alpha --> 0.3
    # color --> g.color
    # label --> false

    # Gate parameters
    x0 = g.pos[1]
    y0 = g.pos[2]
    width = FPVGateWidth
    radius = FPVGateAllowedRadius

    # Rectangle bounds (left and right of the passthrough hole)
    x_min = x0 - width / 2.0
    x_max = x0 + width / 2.0
    y_min = y0 - radius
    y_max = y0 + radius

    # Left rectangle (from x_min to left edge of hole)
    left_x = [x_min, x_min, x_max, x_max, x_max]
    left_y = [y_min, y_min - 50.0, y_min - 50.0, y_min, y_min]

    # Right rectangle (from right edge of hole to x_max)
    right_x = [x_min, x_min, x_max, x_max, x_max]
    right_y = [y_max, y_max + 50.0, y_max + 50.0, y_max, y_max]

    @series begin
        seriestype := :shape
        alpha --> 0.5
        color --> g.color

        return left_x, left_y
    end

    @series begin
        seriestype := :shape
        alpha --> 0.5
        color --> g.color

        return right_x, right_y
    end
end

struct PlotCircle{F}
    center::SVector{2,F}
    radius::F
    color::Any
end

PlotCircle(cylinder::Cylinder) = PlotCircle(
    SVector{2,Float64}([cylinder.center[1], cylinder.center[2]]),
    cylinder.radius,
    :black,
)
PlotCircle(o::GroundedHalfDome) =
    PlotCircle(SVector{2,Float64}([o.pos[1], o.pos[2]]), o.R, :purple)

PlotCircle(s::Sphere) = PlotCircle(SVector{2,Float64}([s.pos[1], s.pos[2]]), s.R, :orange)

@recipe function plot_circle(c::PlotCircle)
    seriestype := :path
    alpha --> 0.8
    color --> c.color
    linewidth --> 2

    u = range(0, 2π, length = 30)

    x = c.center[1] .+ c.radius * cos.(u)
    y = c.center[2] .+ c.radius * sin.(u)

    return x, y
end

function get_2d_repr(obs::AbstractStaticObstacle)
    if obs isa Sphere
        return PlotCircle(obs)
    elseif obs isa GroundedHalfDome
        return PlotCircle(obs)
    elseif obs isa Cylinder
        return PlotCircle(obs)
    elseif obs isa FPVGate
        return FPVGate2D(obs.pos, :black)
    else
        error("Unsupported obstacle type for 2D representation: $(typeof(obs))")
    end
end