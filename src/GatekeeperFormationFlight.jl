# Copyright (c) 2025 Devansh R Agrawal - All rights reserved.

module GatekeeperFormationFlight

# 2D Exports
export Robot, Cardioid, CircularWez, Cbez

# 3D Exports
export Robot3, Sphere
export Dubins3DRRTProblem

export wez_coordinates
export is_colliding, collision_distance, minimum_distance
# export plot_scenario!, plot_scenario
export Node, rrt_star, get_best_path
export DubinsRRTProblem
export total_path_length, apply_input_bounds
export get_reference_state_and_input, tracking_controller

export GatekeeperProblem, simulate_closed_loop_gatekeeper, construct_candidate_trajectory

include("utils.jl")
include("robot.jl")
include("wez.jl")
include("collision_check.jl")
include("plotting_utils.jl")
include("rrt_star.jl")
include("dubins_rrt_star.jl")
include("tracking_controller.jl")

include("robot_3d.jl")
include("obstacle.jl")
include("3d_dubins_rrt_star.jl")

include("gatekeeper.jl")

end
