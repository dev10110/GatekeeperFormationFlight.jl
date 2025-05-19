# Copyright (c) 2025 Devansh R Agrawal - All rights reserved.

module GatekeeperFormationFlight

# 2D Exports
export Robot, Cardioid, CircularWez, Cbez

# Export Abstract Gatekeeper Types
export GatekeeperInstance, GatekeeperCoefficients

# Export Types of Dubins Problems
export GKDubinsWezes2D, GKDubinsObs3D, GKDubinsObs3DInterAgent

# 3D Exports
export Robot3, Sphere, Cylinder, PlotCircle
export Dubins3DRRTProblem

export wez_coordinates
export is_colliding, collision_distance, minimum_distance
# export plot_scenario!, plot_scenario
export Node, rrt_star, get_best_path
export DubinsRRTProblem
export total_path_length, apply_input_bounds
export get_reference_state_and_input, tracking_controller

export GatekeeperProblem, simulate_closed_loop_gatekeeper, construct_candidate_trajectory

include("robot/RobotTypes.jl")
using .RobotTypes

include("obstacles/Obstacles.jl")
using .Obstacles


include("utils.jl")
include("collision_check.jl")
include("plotting_utils.jl")

# RRT* Implementation
include("rrt_star.jl")
include("dubins_rrt_star.jl")
include("3d_dubins_rrt_star.jl")

# High level gatekeeper algorithm
include("gatekeeper.jl")
using .Gatekeeper
using .Gatekeeper: path_length

include("multi_gatekeeper.jl")
using .MultiGatekeeper

# 2D GATEKEEPER IMPLEMENTATION
include("2d_tracking_controller.jl")
include("gatekeeper_dubins_wezes_2d.jl")

# 3D GATEKEEPER IMPLEMENTATION
include("gatekeeper_dubins_obs_3d.jl")

# Include the Dubins3D tracking controller but don't export it
include("dubins_3d_tracking_controller.jl")

include("multi_gatekeeper_dubins_obs_3d.jl")
end
