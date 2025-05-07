# Copyright (c) 2025 Devansh R Agrawal - All rights reserved.

using StaticArrays, LinearAlgebra
using RecipesBase

abstract type AbstractWez <: AbstractObstacle end

"""
    is_colliding(wez::AbstractWez, robot::Robot, tol)
    is_colliding(wezes::Vector{AbstractWez}, robot::Robot, tol)

returns `collision_distance(wez, robot) <= tol`
"""
function is_colliding end

"""
    collision_distance(wez::AbstractWez, robot::Robot)
    collision_distance(wezez::Vector{AbstractWez}, robot::Robot)

returns the distance to collision for this robot to the wez. Positive if safe. If the robot is within the wez, it will return a negative number. 
"""
function collision_distance end

"""
    wez_coordinates(θ, wez::AbstractWez, robot::Robot)

returns the (x, y) coordinates of the wez boundary relative to a robot, parameterized by the polar angle θ ∈ [0, 2π]. 
"""
function wez_coordinates end


"""
   minimum_distance(c::AbstractWez p::SVector{2})

utility function to get the smallest distance to a wez from the current location, indepednent of robot orientations.
Currently only implemented for Cardioid and CircularWez.
"""
function minimum_distance end

#############################################################
### Cardioids ###############################################
#############################################################

"""
    Cardioid(x, y, Rmin=0.0, Rmax=0.15)

Construct a Cardioid WEZ at position x, y with radii Rmin, and Rmax.
"""
struct Cardioid{F} <: AbstractWez
    x::F
    y::F
    Rmin::F
    Rmax::F
end
Cardioid(x::F, y::F) where {F} = Cardioid(x, y, zero(F), convert(F, 0.15))


function _cardioid(θ, λ, ξ, Rmin, Rmax)
    ρ = ((cos(ξ) + 1) / 2 * (Rmax - Rmin) + Rmin) * (1 / 2) * (1 + sin(π / 2 - λ + θ))
    return ρ
end

function wez_polar(θ, c::Cardioid, r::Robot)
    λ = atan(r.y - c.y, r.x - c.x)
    ξ = r.ψ - λ - π
    return _cardioid(θ, λ, ξ, c.Rmin, c.Rmax)
end

function wez_coordinates(θ, c::Cardioid, r::Robot)
    rho = wez_polar(θ, c, r)
    x = c.x + rho * cos(θ)
    y = c.y + rho * sin(θ)
    return @SVector [x, y]
end

# positive => safe
function collision_distance(c::Cardioid, r::Robot)

    # get the current distance
    d = sqrt((r.x - c.x)^2 + (r.y - c.y)^2)

    # determine the appropriate θ (only possible to intersect if θ = λ)
    θ = atan(r.y - c.y, r.x - c.x)

    # get the radius of the cardioid at this angle
    ρ = wez_polar(θ, c, r)

    return d - ρ
end

function is_colliding(c::Cardioid, r::Robot, tol = 0)
    d = sqrt((r.x - c.x)^2 + (r.y - c.y)^2)

    if d > c.Rmax + tol
        return false
    else
        return collision_distance(c, r) <= tol
    end
end


function minimum_distance(c::Cardioid, p::SVector{2})
    return norm((@SVector [c.x - p[1], c.y - p[2]])) - c.Rmax
end


#############################################################
### Circular ################################################
#############################################################

"""
    CircularWez(x, y, R=0.15)

Define a Circluar WEZ at some x, y with radius R.
"""
struct CircularWez{F} <: AbstractWez
    x::F
    y::F
    R::F
end
CircularWez(x::F, y::F) where {F} = CircularWez(x, y, convert(F, 0.15))

function wez_polar(θ, c::CircularWez, r::Robot)
    return c.R
end

function wez_coordinates(θ, c::CircularWez, r::Robot)
    x = c.x + c.R * cos(θ)
    y = c.y + c.R * sin(θ)

    return @SVector [x, y]
end

# positive => safe
function collision_distance(c::CircularWez, r::Robot)

    # get the current distance
    d = sqrt((r.x - c.x)^2 + (r.y - c.y)^2)

    return d - c.R
end

function minimum_distance(c::CircularWez, p::SVector{2})
    return collision_distance(c, r)
end


#############################################################
### CBEZ ####################################################
#############################################################

"""
    Cbez(x, y, ψ)
    Cbez(x, y, ψ, μ=0.9, ā=0.25, v=1.0, R=π/2, t=π/2)

Define a Curve-Only Basic WEZ at some x, y with a heading ψ. 
"""
struct Cbez{F} <: AbstractWez
    x::F
    y::F
    ψ::F # initial heading of the cbez
    μ::F # μ = vT/vP
    ā::F
    v::F
    R::F
    t::F
end

Cbez(x, y, ψ) = Cbez(
    x,
    y,
    ψ,
    0.9,  # μ
    0.25, # ā
    1.0,  # v
    π / 2,  # R
    π / 2,   # t
)

function wez_coordinates(λ, cbez::Cbez, robot::Robot)

    # extract vars
    ψT = robot.ψ

    ψP = cbez.ψ
    μ = cbez.μ
    R = cbez.R
    v = cbez.v
    t = cbez.t

    # find the p'
    p = @SVector [cbez.x, cbez.y]
    p_prime = p + (@SVector [-μ * R * cos(ψT), -μ * R * sin(ψT)])

    # find the d
    # note, sinc(x) = sin(πx)/(πx) in julia
    d_prime = (v * t) * sinc(wrapToPi(λ - ψP) / π)

    # return both the x and y coordinates
    T = @SVector [d_prime * cos(λ), d_prime * sin(λ)]

    # this is the point on the surface
    s = p_prime + T

    return s
end


function collision_distance(cbez::Cbez, robot::Robot)

    # extract vars
    ψT = robot.ψ

    ψP = cbez.ψ
    μ = cbez.μ
    R = cbez.R
    v = cbez.v
    t = cbez.t

    # find the p'
    p = @SVector [cbez.x, cbez.y]
    p_prime = p + (@SVector [-μ * R * cos(ψT), -μ * R * sin(ψT)])

    # find the λ' (the heading to the target from p_prime)
    λ_prime = atan(robot.y - p_prime[2], robot.x - p_prime[1])

    # find the d
    # note, sinc(x) = sin(πx)/(πx) in julia
    d_prime = (v * t) * sinc(wrapToPi(λ_prime - ψP) / π)

    # compute the actual distance to p_prime
    d = norm((@SVector [robot.x, robot.y]) - p_prime)

    return d - d_prime
end


#############################################################
### Collision Checking ######################################
#############################################################

function is_colliding(c::W, x::AbstractVector{F}, tol = 0) where {W<:AbstractWez,F}
    r = Robot(x)
    return is_colliding(c, r, tol)
end

function is_colliding(
    vc::VW,
    x::AbstractVector{F},
    tol = 0,
) where {W<:AbstractWez,VW<:AbstractVector{W},F}
    r = Robot(x)
    return is_colliding(vc, r, tol)
end

function is_colliding(c::W, r::Robot, tol = 0) where {W<:AbstractWez}
    return collision_distance(c, r) <= tol
end

function is_colliding(c::W, x::SVector{3,F}, tol = 0) where {W<:AbstractWez,F}
    r = Robot(x)
    return is_colliding(c, r, tol)
end

function is_colliding(
    vc::VW,
    r::Robot,
    tol = 0,
) where {W<:AbstractWez,VW<:AbstractVector{W}}
    for c in vc
        if is_colliding(c, r, tol)
            return true
        end
    end
    return false
end

function is_colliding(
    ws::VW,
    x::SVector{3,F},
    tol = 0,
) where {W<:AbstractWez,VW<:AbstractVector{W},F}
    r = Robot(x)
    return is_colliding(ws, r, tol)
end

function collision_distance(ws::VW, r::Robot) where {W<:AbstractWez,VW<:AbstractVector{W}}
    if length(ws) == 0
        return Inf
    end

    return minimum(collision_distance(w, r) for w in ws)
end

function collision_distance(c::W, x::SVector{3,F}) where {W<:AbstractWez,F}
    r = Robot(x)
    return collision_distance(c, r)
end

function collision_distance(
    ws::VW,
    x::AbstractVector{F},
) where {W<:AbstractWez,VW<:AbstractVector{W},F}
    r = Robot(x)
    return collision_distance(ws, r)
end


#############################################################
### Plotting ################################################
#############################################################

@recipe function plot_wez(c::W) where {W<:AbstractWez}

    x = [c.x]
    y = [c.y]

    # plot the origin marker
    @series begin
        seriestype := :scatter
        label --> false
        color --> :blue
        marker --> :x
        aspect_ratio --> :equal
        x, y
    end
end

@recipe function plot_wez(c::Cbez)
    R = c.R / 3
    ψ = c.ψ

    x = [c.x, c.x + R * cos(ψ)]
    y = [c.y, c.y + R * sin(ψ)]

    # plot the origin marker
    @series begin
        seriestype := :scatter
        label --> false
        color --> :blue
        marker --> :x
        aspect_ratio --> :equal
        [c.x], [c.y]
    end

    # plot the arrow
    @series begin
        seriestype := :path
        label --> false
        color --> :blue
        marker --> false
        arrow --> true
        aspect_ratio --> :equal
        x, y
    end
end


@recipe function plot_wez(c::W, r::Robot) where {W<:AbstractWez}

    # compute the wez boundary
    pts = [wez_coordinates(t, c, r) for t in range(0, 2π, length = 100)]
    x = [x[1] for x in pts]
    y = [x[2] for x in pts]

    # plot the wez boundary
    @series begin
        seriestype := :path
        label --> false
        color --> :gray
        aspect_ratio --> :equal
        x, y
    end

    # plot the wez
    @series begin
        c
    end

end
