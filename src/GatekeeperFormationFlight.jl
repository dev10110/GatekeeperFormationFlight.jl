module GatekeeperFormationFlight

export Robot, Cardioid, CircularWez, Cbez
export wez_coordinates
export is_colliding, collision_distance, minimum_distance
# export plot_scenario!, plot_scenario
export Node, rrt_star, get_best_path
export DubinsRRTProblem
export total_path_length, apply_input_bounds
export get_reference_state_and_input, tracking_controller

export GatekeeperProblem, simulate_closed_loop_gatekeeper

include("utils.jl")
include("robot.jl")
include("wez.jl")
include("collision_check.jl")
include("plotting_utils.jl")
include("rrt_star.jl")
include("dubins_rrt_star.jl")
include("tracking_controller.jl")
include("gatekeeper.jl")

end
