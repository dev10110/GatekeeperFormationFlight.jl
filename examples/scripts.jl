using Revise
using Pkg
Pkg.activate(".")
using GatekeeperFormationFlight

using StaticArrays, Random
using Plots
pyplot()

include("3d_example_utils.jl")


# PROBLEM CONSTANTS
# Define problem constants
PITCH_LIMITS = SVector{2,Float64}(deg2rad(-15), deg2rad(20))
TURNING_RADIUS::Float64 = 10.0 # Minimum turning radius for the Dubins vehicle
N_OBSTACLES::Int = 25 # Number of obstacles
V_MIN = 0.8
V_MAX = 1.0
AGENT_RADIUS = 1.5

RRT_ITERATIONS = 400

domain = (
    SVector(0.0, 0.0, 0.0, -1.0 * π, PITCH_LIMITS[1]),
    SVector(100.0, 100.0, 100.0, 1.0 * π, PITCH_LIMITS[2]),
)

x0 = @SVector [0.0, 0.0, 0.0, 0.0, 0.0]
xg = @SVector [100.0, 100.0, 70.0, 0.0, 0.0];


coeffs = GatekeeperCoefficients(
    switch_step_size = 1.0,
    reconnection_step_size = 2.0,
    max_Ts_horizon = 50.0,
    integration_max_step_size = 0.1,
    integration_step_size = 0.01,
    collision_check_step_size = 0.1,
)

offsets = [
    SVector(0.0, 0.0, 0.0),
    SVector(-3.0, -5.0, 0.0),
    SVector(-3.0, 5.0, 0.0),
]

# amount of time back the path is started
X_PADDING = 10.

initial_positions = [
    SVector(x0[1] + off[1] - X_PADDING, x0[2] + off[2], x0[3] + off[3], x0[4], x0[5])
    for off in offsets
]



function do_full_scenario(seed)
    Random.seed!(seed)
    obstacles = ExampleUtils3D.create_random_scenario(5)


    rrt_problem = Dubins3DRRTProblem(
        domain, TURNING_RADIUS, obstacles
    )
    success, waypoints = ExampleUtils3D.solve_3d_rrt(x0, xg, rrt_problem; rrt_iterations = RRT_ITERATIONS)

    if !success
        println("Failed to find RRT* path")
        return
    end

    println("Making Reference Trajectory")
    reference_trajectory = ExampleUtils3D.make_reference_trajectory(waypoints, TURNING_RADIUS, PITCH_LIMITS, x0, xg; X_PADDING = X_PADDING);


    path_length = sum(x->x.length, reference_trajectory)
    println("Path length: $path_length")

    println("Initializing Gatekeeper Problem")
    multi_gk_problem = GKDubinsObs3DInterAgent(;
        static_obstacles = obstacles,
        reference_path = reference_trajectory,
        offset = offsets,
        turning_radius = TURNING_RADIUS,
        pitch_limits = PITCH_LIMITS,
        v_min = V_MIN,
        v_max = V_MAX,
        agent_radius = AGENT_RADIUS,
    )

    multi_gk_instance = GatekeeperInstance(multi_gk_problem, coeffs)

    initial_state = reduce(hcat, initial_positions)'
    tspan = [0., path_length]

    println("Starting Simulation!")
    solution = simulate_closed_loop_gatekeeper(multi_gk_instance, initial_state, tspan);
    println("Finished Simulation!")

    p = ExampleUtils3D.plot_interagent_solution(
        solution,
        multi_gk_instance,
    )

    println("Writing paper to file.")
    savefig(p, "../paper_figures/3d_gk_solution.png")
end

