using GatekeeperFormationFlight
using Plots, LinearAlgebra, StaticArrays
using Dubins3D

pyplot()

# Create a sample robot in 3D space
robot = Robot3(SVector(1.0, 1.0, 1.0), SVector(0.0, 0.0, 0.0))

# Create a vector of obstacles in the 3D Environment
obstacles = [
    Sphere(5.0, 5.0, 5.0, 3.0),
    Sphere(20.0, 20.0, 20.0, 10.0),
    Sphere(60.0, 40.0, 40.0, 5.0),
]


# Create a 3D Dubins Problem - inits problem with default settings
prob = Dubins3DRRTProblem(obstacles)

nodes = [Node(SVector(1.0, 1.0, 1.0, 0.0, 0.0, 0.0))]

nodes = rrt_star(prob, nodes, 300)

goal = SVector(70.0, 70.0, 70.0, 0.0, 0.0, 0.0)
success, waypoints = get_best_path(prob, nodes, goal)

@show "Success: $success"
@assert success

waypoints = reduce(hcat, waypoints)' # points x 6
@show "Size of Waypoints: $(size(waypoints))"

# for i = 1:length(waypoints)
#     @show "($(waypoints[i][1]), $(waypoints[i][2]), $(waypoints[i][3]))"
# end

# plot the waypoints
p = plot(
    waypoints[:, 1],
    waypoints[:, 2],
    waypoints[:, 3],
    seriestype = :path,
    label = "Path",
    color = :blue,
    linewidth = 2,
)

path = []

# @show length(waypoints)

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
@show "size of path: $(length(path))"
for i in eachindex(path)
    @show "size of path i: $(size(path[i]))"
    segment_matrix = reduce(hcat, path[i])'
    @show "size of segment_matrix fr $(size(segment_matrix))"
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
# plot!(obstacles[1])

xlims!(0, 100)
ylims!(0, 100)
zlims!(0, 100)
title!("3D Dubins RRT* Path")
xlabel!("X")
ylabel!("Y")
zlabel!("Z")



display(p)


"""
# Plot the robot
p = plot(robot)

# Show plot
plot!(p, legend = false, title = "3D Robot", xlabel = "X", ylabel = "Y", zlabel = "Z")
xlims!(p, 0, 2)
ylims!(p, 0, 2)
zlims!(p, 0, 2)

display(p)
"""

