using GatekeeperFormationFlight
using Plots, LinearAlgebra, StaticArrays, Random

using Dubins3D

pyplot()

PITCH_LIMITS = SVector{2,Float64}(deg2rad(-15), deg2rad(20))
TURNING_RADIUS::Float64 = 10.0 # Minimum turning radius for the Dubins vehicle

function create_random_scenario(N_obstacles = 10, env_size = 100, radius_avg = 5)
    obstacles = [
        Cylinder(rand() * env_size, rand() * env_size, (rand() * radius_avg) + radius_avg) for i = 1:N_obstacles
    ]
    return obstacles
end

Random.seed!(1234)
obstacles = create_random_scenario(1)

domain = (
    SVector(0.0, 0.0, 0.0, -1.0 * π, PITCH_LIMITS[1]),
    SVector(100.0, 100.0, 100.0, 1.0 * π, PITCH_LIMITS[2]),
)

# Initialize the 3D Dubins RRT* path planning problem
prob = Dubins3DRRTProblem(domain, TURNING_RADIUS, obstacles)

# Create the initial RRT node at the start position
start_pose = @SVector [1.0, 1.0, 1.0, 0.0, 0.0]
nodes = [Node(start_pose)]

# Run the RRT* algorithm with 400 iterations
nodes = rrt_star(prob, nodes, 400)

# Define goal state and find path to it
goal = @SVector [100.0, 100.0, 70.0, 0.0, 0.0]
success, waypoints = get_best_path(prob, nodes, goal)

# Ensure a valid path was found
@assert success

waypoints = reduce(hcat, waypoints)'  # Convert to matrix with points × 6 dimensions
reference_trajectory = Vector{Dubins3D.DubinsManeuver3D}()

pad_straight_start_pose = @SVector [-10.0, 1.0, 1.0, 0.0, 0.0]
pad_straight_end_pose = @SVector [110.0, 100.0, 70.0, 0.0, 0.0]

# Add a straight line segment to the path
start_straight_maneuver = DubinsManeuver3D(
    pad_straight_start_pose,
    start_pose,
    prob.turning_radius,
    SVector{2,Float64}(domain[1][5], domain[2][5]),  # Pitch constraints
)
end_straight_maneuver = DubinsManeuver3D(
    goal,
    pad_straight_end_pose,
    prob.turning_radius,
    SVector{2,Float64}(domain[1][5], domain[2][5]),  # Pitch constraints
)

# Construct the reference trajectory as a seriers of DubinsManeuvers
push!(reference_trajectory, start_straight_maneuver)
for i = 2:size(waypoints)[1]
    local_maneuver = DubinsManeuver3D(
        waypoints[i-1, :],
        waypoints[i, :],
        prob.turning_radius,
        [domain[1][5], domain[2][5]],  # Pitch constraints
    )

    push!(reference_trajectory, local_maneuver)
end
push!(reference_trajectory, end_straight_maneuver)

offsets = [SVector(-3.0, -5.0, 0.0), SVector(-3.0, 5.0, 0.0)]

follower_robots = [
    SVector{5,Float64}(-13.0, -4.0, 0.0, 0.0, 0.0)
    SVector{5,Float64}(-13.0, 6.0, 0.0, 0.0, 0.0)
]

coeffs = GatekeeperCoefficients(
    switch_step_size = 1.0,
    reconnection_step_size = 2.0,
    max_Ts_horizon = 50.0,
    integration_max_step_size = 0.1,
    integration_step_size = 0.01,
    collision_check_step_size = 0.01,
)

multi_gk_problem = GKDubinsObs3DInterAgent(;
    static_obstacles = obstacles,
    reference_path = reference_trajectory,
    offset = offsets,
    turning_radius = TURNING_RADIUS,
    pitch_limits = PITCH_LIMITS,
    # v_min = 0.8,
    # v_max = 1.0,
    # agent_radius = 0.5,
)

multi_gk_instance = GatekeeperInstance(multi_gk_problem, coeffs)

tspan = [
    0.0,
    GatekeeperFormationFlight.path_length(multi_gk_instance.problem, reference_trajectory),
]

solution = simulate_closed_loop_gatekeeper(multi_gk_instance, follower_robots, tspan)
