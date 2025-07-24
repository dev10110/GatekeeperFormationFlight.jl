"""
File: src/obstacles/Obstacles.jl

Defines a modle Obstacles that contains relevant types and methods for
    handling static and dynamic obstacles and wezes
"""
module Obstacles

export AbstractObstacle, AbstractStaticObstacle, AbstractDynamicObstacle, AbstractWez
export is_colliding, collision_distance

# Generic Obstacles
export PlotCircle, Sphere, Cylinder, GroundedHalfDome, TimeVaryingSphere

# Wezes
export Cardioid, CircularWez, Cbez

# Demo Obstacles
export FPVGate

export get_2d_repr

include("obstacle.jl")
include("wez.jl")

end