using GatekeeperFormationFlight
using Plots, LinearAlgebra, StaticArrays, Random
using Dubins3D

# Use PyPlot backend for better 3D visualization
pyplot()

PITCH_LIMITS = SVector{2,Float64}(deg2rad(-15), deg2rad(20))
TURNING_RADIUS::Float64 = 10.0 # Minimum turning radius for the Dubins vehicle

# Function to generate random cylindrical obstacles within the environment
function create_random_scenario(N_obstacles = 10, env_size = 100, radius_avg = 5)
    obstacles = [
        Cylinder(rand() * env_size, rand() * env_size, (rand() * radius_avg) + radius_avg) for i = 1:N_obstacles
    ]
    return obstacles
end

# Set random seed for reproducible results
# Good Examples:
# Random.seed!(22341234)
# Random.seed!(52341234)
# Random.seed!(72341234)
Random.seed!(13341234) # goated 10 obstacle seed
# Random.seed!(63341234) # good seed with 4
# Random.seed!(55551234)
obstacles = create_random_scenario(10) #10 is good seed

# Define initial robot configuration in 3D space
robot = Robot3(SVector(1.0, 1.0, 1.0), SVector(0.0, 0.0, 0.0))

# Define the state space domain boundaries
# Format: (lower_bounds, upper_bounds)
# State dimensions: [x, y, z, heading, pitch, roll]
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

# Prepare path for visualization
path = []
waypoints = reduce(hcat, waypoints)'  # Convert to matrix with points × 6 dimensions

# Initialize the main 3D plot
p1 = plot(robot)

# Plot waypoints in 3D space
plot!(waypoints[:, 1], waypoints[:, 2], waypoints[:, 3], label = false)

# for the gatekeeper problem
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

push!(reference_trajectory, start_straight_maneuver)

# Generate smooth Dubins path between consecutive waypoints
for i = 2:size(waypoints)[1]
    # Create a Dubins maneuver between waypoint pairs
    loc_maneuver = DubinsManeuver3D(
        Vector(waypoints[i-1, :]),
        Vector(waypoints[i, :]),
        prob.turning_radius,
        [domain[1][5], domain[2][5]],  # Pitch constraints
    )

    # Sample points along the Dubins curve
    loc_path = compute_sampling(loc_maneuver, numberOfSamples = 100)
    push!(path, loc_path)
    push!(reference_trajectory, loc_maneuver)
end

push!(reference_trajectory, end_straight_maneuver)

reference_path = []

# Plot the smooth path segments
for i in eachindex(path)
    segment_matrix = reduce(hcat, path[i])'
    plot!(
        segment_matrix[:, 1],
        segment_matrix[:, 2],
        segment_matrix[:, 3],
        seriestype = :path,
        label = false,
        color = :red,
        linewidth = 2,
    )
    push!(reference_path, segment_matrix)
end

# (N, 5) -> should add roll dimension ?
ref_matrix = reduce(vcat, reference_path)

offsets = [SVector(-3.0, -5.0, 0.0), SVector(-3.0, 5.0, 0.0)]
# follower_robots = [
#     Robot3(SVector(-3.0, -5.0, 0.0), SVector(0.0, 0.0, 0.0)),
#     Robot3(SVector(-3.0, 5.0, 0.0), SVector(0.0, 0.0, 0.0)),
# ]

follower_robots = [
    SVector{5,Float64}(-13.0, -4.0, 0.0, 0.0, 0.0),
    SVector{5,Float64}(-13.0, 6.0, 0.0, 0.0, 0.0),
]
# coeffs = GatekeeperCoefficients(
#     switch_step_size = 2e-3,
#     reconnection_step_size = 0.01,
#     max_Ts_horizon = 0.5,
#     integration_max_step_size = 1e-3,
#     collision_check_step_size = 1e-3,
# )
coeffs = GatekeeperCoefficients(
    switch_step_size = 1.0,
    reconnection_step_size = 2.0,
    max_Ts_horizon = 50.0,
    integration_max_step_size = 0.1,
    integration_step_size = 0.01,
    collision_check_step_size = 0.01,
)

gk_instances = []
for offset in offsets
    gk_instance = GatekeeperInstance(
        GKDubinsObs3D(;
            obstacles = obstacles,
            reference_path = reference_trajectory,
            offset = offset,
            turning_radius = TURNING_RADIUS,
            pitch_limits = PITCH_LIMITS,
            v_min = 0.8,
            v_max = 1.0,
        ),
        coeffs,
    )
    push!(gk_instances, gk_instance)
end

# solve the problems and plot solutions
tspan = [
    0.0,
    GatekeeperFormationFlight.path_length(gk_instances[1].problem, reference_trajectory),
]
# tspan = [0.0, 100.0]

gk_solutions = [
    simulate_closed_loop_gatekeeper(gk_instances[i], follower_robots[i], tspan) for
    i = 1:length(follower_robots)
]


TMax = GatekeeperFormationFlight.path_length(gk_instances[1].problem, reference_trajectory)

# Compute the nominal trajectories for the gatekeeper problem
# nominal_trajectories =
#     GatekeeperFormationFlight.compute_nominal_trajectories(gatekeeper_problem)

####### PLOT NOMINAL TRAJECTORIES #######
for off in offsets
    # plot!(nom[:, 1], nom[:, 2], nom[:, 3], label = false, color = :blue)
    plot!(
        τ -> get_reference_state_and_input(
            gk_instances[1].problem,
            reference_trajectory,
            off,
            τ,
        )[1][1],
        τ -> get_reference_state_and_input(
            gk_instances[1].problem,
            reference_trajectory,
            off,
            τ,
        )[1][2],
        τ -> get_reference_state_and_input(
            gk_instances[1].problem,
            reference_trajectory,
            off,
            τ,
        )[1][3],
        0.0,
        TMax,
        linestyle = :dash,
        label = false,
        linecolor = :blue,
    )
end

# Plot the 3D obstacles
for i in eachindex(obstacles)
    plot!(obstacles[i])
end

###### PLOT THE GATEKEEPER SOLUTIONS ######
for i = 1:length(follower_robots)
    plot!(gk_solutions[i], idxs = (1, 2, 3), label = "gk_sol_$(i)", linewidth = 2)
end

# Configure the main 3D plot (camera angle 30, 30)
plot!(aspect_ratio = :equal, size = (600, 600))
xlims!(0, 100)
ylims!(0, 100)
zlims!(0, 100)
plot!(camera = (30, 30))
title!("3D Dubins RRT* Path (Camera: 30, 30)")
xlabel!("X")
ylabel!("Y")
zlabel!("Z")

# Create second subplot - 3D view from top (camera angle 0, 0)
p2 = plot(robot, aspect_ratio = :equal, size = (600, 800))
plot!(waypoints[:, 1], waypoints[:, 2], waypoints[:, 3], label = false)

for i in eachindex(path)
    segment_matrix = reduce(hcat, path[i])'
    plot!(
        segment_matrix[:, 1],
        segment_matrix[:, 2],
        segment_matrix[:, 3],
        seriestype = :path,
        label = false,
        color = :red,
        linewidth = 2,
    )
end

# for nom in nominal_trajectories
#     plot!(nom[:, 1], nom[:, 2], nom[:, 3], label = false, color = :blue)
# end


for i in eachindex(obstacles)
    plot!(obstacles[i])
end

# Configure the second 3D plot (camera angle 0, 0)
plot!(aspect_ratio = :equal, size = (600, 800))
xlims!(0, 100)
ylims!(0, 100)
zlims!(0, 100)
plot!(camera = (0, 0), legend = false)
title!("3D Dubins RRT* Path (Camera: 0, 0)")
xlabel!("X")
# Remove only y-axis tick labels while preserving x and z axis labels
yaxis!(ticklabels = false, grid = true)
zlabel!("Z")

# Create third subplot - 2D projection for simplified view
p3 = plot(aspect_ratio = :equal, size = (600, 600), legend = false)
xlims!(0, 100)
ylims!(0, 100)
xlabel!("X")
ylabel!("Y")
title!("2D Projection of Dubins RRT* Path")

# Plot the path projected onto the XY plane
for i in eachindex(path)
    segment_matrix = reduce(hcat, path[i])'
    plot!(
        segment_matrix[:, 1],
        segment_matrix[:, 2],
        seriestype = :path,
        label = false,
        color = :red,
        linewidth = 2,
    )
end

for off in offsets
    # plot!(nom[:, 1], nom[:, 2], nom[:, 3], label = false, color = :blue)
    plot!(
        τ -> get_reference_state_and_input(
            gk_instances[1].problem,
            reference_trajectory,
            off,
            τ,
        )[1][1],
        τ -> get_reference_state_and_input(
            gk_instances[1].problem,
            reference_trajectory,
            off,
            τ,
        )[1][2],
        0.0,
        TMax,
        linestyle = :dash,
        label = false,
        linecolor = :blue,
    )
end

for i = 1:length(follower_robots)
    plot!(gk_solutions[i], idxs = (1, 2), label = "gk_sol_$(i)", linewidth = 2)
end

# Plot the obstacles as circles in 2D projection
for i in eachindex(obstacles)
    plot!(PlotCircle(obstacles[i]), label = false, color = :black, linewidth = 2)
end

# Combine all subplots into a single figure with custom layout
# Main plot takes 70% width, side plots stacked vertically with equal heights
lay = @layout [a{0.7w} [b{0.5h}; c{0.5h}]]

p = plot(p1, p2, p3, layout = lay, size = (1200, 800))
p[:plot_title] = "3D Dubins RRT* Path Planning w/Follower Trajectories"

savefig(p, "3d_gatekeeper_paths.png")

# Display the final visualization
# display(p)

@show "Starting the Animation Process"

sim_length = 100

modified_range = range(0, TMax, length = sim_length)
modified_range = vcat(modified_range, [TMax for i = 1:floor(Int, sim_length * 0.15)])

anim = @animate for (idx, t) in enumerate(modified_range)
    println("Animation Time: $t")

    # grab the states
    robots_ = [Robot3(gk_solutions[i](t)) for i = 1:length(gk_solutions)]

    # start the plot
    plot()

    # plot the reference paths
    for off in offsets
        plot!(
            τ -> get_reference_state_and_input(
                gk_instances[1].problem,
                reference_trajectory,
                off,
                τ,
            )[1][1],
            τ -> get_reference_state_and_input(
                gk_instances[1].problem,
                reference_trajectory,
                off,
                τ,
            )[1][2],
            τ -> get_reference_state_and_input(
                gk_instances[1].problem,
                reference_trajectory,
                off,
                τ,
            )[1][3],
            0.0,
            TMax;
            label = false,
            color = :gray,
            linestyle = :dash,
        )
    end

    # plot the trace of the robots upto this point in time
    for i = 1:length(robots_)
        plot!(
            τ -> gk_solutions[i](τ)[1],
            τ -> gk_solutions[i](τ)[2],
            τ -> gk_solutions[i](τ)[3],
            0.0,
            t,
            label = "gk_sol_$(i)",
            color = (i == 1 ? :black : :green),
            linewidth = 2,
        )
    end

    # Plot the obstacles
    for obs in obstacles
        plot!(obs)
    end

    # plot the robots
    for rob in robots_
        plot!(rob)
    end

    plot!(legend = false, camera = (Int(floor((idx / length(modified_range)) * 70)), 30))
end



gif(anim, "3d_gatekeeper.gif")