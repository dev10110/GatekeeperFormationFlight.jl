using GatekeeperFormationFlight
using Plots, LinearAlgebra, StaticArrays, Random
using Dubins3D

# Use PyPlot backend for better 3D visualization
pyplot()

# Function to generate random cylindrical obstacles within the environment
function create_random_scenario(N_obstacles = 10)
    obstacles =
        [Cylinder(rand() * 100, rand() * 100, (rand() * 5) + 5) for i = 1:N_obstacles]
    return obstacles
end

# Set random seed for reproducible results
# Good Examples:
# Random.seed!(22341234)
# Random.seed!(52341234)
# Random.seed!(72341234)
Random.seed!(13341234)
obstacles = create_random_scenario(10)

# Define initial robot configuration in 3D space
robot = Robot3(SVector(1.0, 1.0, 1.0), SVector(0.0, 0.0, 0.0))

# Define the state space domain boundaries
# Format: (lower_bounds, upper_bounds)
# State dimensions: [x, y, z, heading, pitch, roll]
domain = (
    SVector(0.0, 0.0, 0.0, -1.0 * π, deg2rad(-10), deg2rad(-5)),
    SVector(100.0, 100.0, 100.0, 1.0 * π, deg2rad(15), deg2rad(5)),
)
turning_radius = 10.0  # Minimum turning radius for the Dubins vehicle

# Initialize the 3D Dubins RRT* path planning problem
prob = Dubins3DRRTProblem(domain, turning_radius, obstacles)

# Create the initial RRT node at the start position
nodes = [Node(SVector(1.0, 1.0, 1.0, 0.0, 0.0, 0.0))]

# Run the RRT* algorithm with 400 iterations
nodes = rrt_star(prob, nodes, 200)

# Define goal state and find path to it
goal = SVector(70.0, 70.0, 70.0, 0.0, 0.0, 0.0)
success, waypoints = get_best_path(prob, nodes, goal)

# Ensure a valid path was found
@assert success

# Prepare path for visualization
path = []
waypoints = reduce(hcat, waypoints)'  # Convert to matrix with points × 6 dimensions

# @show size(waypoints)
# @show waypoints

# Initialize the main 3D plot
p1 = plot(robot)

# Plot waypoints in 3D space
plot!(waypoints[:, 1], waypoints[:, 2], waypoints[:, 3], label = false)

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
end

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
end

# Plot the 3D obstacles
for i in eachindex(obstacles)
    plot!(obstacles[i])
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
p3 = plot(aspect_ratio = :equal, size = (600, 600))
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

# Plot the obstacles as circles in 2D projection
for i in eachindex(obstacles)
    plot!(PlotCircle(obstacles[i]), label = false, color = :red)
end

# Combine all subplots into a single figure with custom layout
# Main plot takes 70% width, side plots stacked vertically with equal heights
lay = @layout [a{0.7w} [b{0.5h}; c{0.5h}]]

p = plot(p1, p2, p3, layout = lay, size = (1200, 800))

# Display the final visualization
display(p)