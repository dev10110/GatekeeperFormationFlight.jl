using GatekeeperFormationFlight
using Plots, LinearAlgebra, StaticArrays, Random
using Dubins3D

pyplot()

function create_random_scenario(N_obstacles = 10)
    obstacles = [Cylinder(rand() * 100, rand() * 100, rand() * 10) for i = 1:N_obstacles]
    return obstacles
end

Random.seed!(22341234)
obstacles = create_random_scenario(10)

# Create a sample robot in 3D space
robot = Robot3(SVector(1.0, 1.0, 1.0), SVector(0.0, 0.0, 0.0))

# Create a 3D Dubins Problem - inits problem with default settings
prob = Dubins3DRRTProblem(obstacles)

nodes = [Node(SVector(1.0, 1.0, 1.0, 0.0, 0.0, 0.0))]

nodes = rrt_star(prob, nodes, 50)

goal = SVector(70.0, 70.0, 70.0, 0.0, 0.0, 0.0)
success, waypoints = get_best_path(prob, nodes, goal)

@assert success

path = []
waypoints = reduce(hcat, waypoints)' # points x 6

@show size(waypoints)

p1 = plot(robot)

for i = 2:size(waypoints)[1]
    loc_maneuver = DubinsManeuver3D(
        Vector(waypoints[i-1, :]),
        Vector(waypoints[i, :]),
        prob.turning_radius,
        [deg2rad(-15.0), deg2rad(20.0)],
    )

    loc_path = compute_sampling(loc_maneuver, numberOfSamples = 50)
    push!(path, loc_path)
end

# plot the path
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

# # plot the obstacles
for i in eachindex(obstacles)
    plot!(obstacles[i])
end

xlims!(0, 100)
ylims!(0, 100)
zlims!(0, 100)
title!("3D Dubins RRT* Path")
xlabel!("X")
ylabel!("Y")
zlabel!("Z")

p2 = plot()
xlims!(0, 100)
ylims!(0, 100)
xlabel!("X")
ylabel!("Y")
title!("2D Projection of Dubins RRT* Path")

# plot the path in 2D
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

# plot the obstacles in 2D
for i in eachindex(obstacles)
    plot!(PlotCircle(obstacles[i]), label = false, color = :red)
end

p = plot(p1, p2)
# Plot in 2D 
display(p)