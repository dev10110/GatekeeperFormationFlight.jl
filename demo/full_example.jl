# Copyright (c) 2025 Devansh R Agrawal - All rights reserved.

using GatekeeperFormationFlight
using Plots, LinearAlgebra, StaticArrays, Random
using Dubins

GFF = GatekeeperFormationFlight

function create_scenario()
    # create a set of wezes
    wezes = CircularWez[]
    for y in -2.0:0.1:3.0
        if !(0.6 <= y <= 0.8)
            push!(wezes, CircularWez(0.3, y, 0.1))
        end
        if !(0.2 <= y <= 0.4)
            push!(wezes, CircularWez(0.7, y, 0.1))
        end
    end
    return wezes
end


println("Creating Scenario...")
# change the seed to get different environments
# seed = 27182818 with 24 wezes is a good looking env.
Random.seed!(0)
#wezes = create_random_scenario(5)
wezes = create_scenario()


# create a set of robots
leader_robot = Robot(-0.25, 0.0, 0.0)
follower_robots = [
                    Robot(-0.3, -0.05, 0.0), 
                    Robot(-0.3, 0.05, 0.0)
            ]
robots = vcat(leader_robot, follower_robots...)

N_robots = length(robots)


println("...scenario created.")

# plot()
# for wez in wezes, robot in robots
#     plot!(wez, robot)
# end
# # gui() 


println("Planning path for leader...")
# plan a path for the leader using RRT*
domain = ((@SVector [0, 0, -1.0 * π]), (@SVector [1, 1, 1.0 * π]))
turning_radius = 0.1
rrt_problem = DubinsRRTProblem(domain, turning_radius, wezes)

# start the tree with the root node
nodes = [Node(SVector(0, 0, 0.0))]

# add 1000 nodes to the tree
nodes = rrt_star(rrt_problem, nodes, 1000)

# see if there is a path 
success_code, waypoints = get_best_path(rrt_problem, nodes, @SVector [1.0, 1.0, 0])

# uncomment the following code if you want to add more nodes to the RRT* problem:
## while we havent found a path, add some nodes and check if we have a feasible path
## iter_counter = 0
## while !success_code  && iter_counter < 10
##     nodes = rrt_star(rrt_problem, nodes, 500)
##     success_code, waypoints = get_best_path(rrt_problem, nodes, [1.0, 1.0, 0])
##     iter_counter += 1
## end

@assert success_code

# prepend and append the start and the goal
waypoints = [SVector(leader_robot), waypoints..., SVector(1.25, 1.0, 0.0)]

path = DubinsPath[]
# add all the waypoints to the path
for i = 2:length(waypoints)
    e, p = dubins_shortest_path(waypoints[i-1], waypoints[i], rrt_problem.turning_radius)
    @assert e == Dubins.EDUBOK
    push!(path, p)
end

println("...leader path planned.")

# small function to help plot things
function plot_scenario!(
    wezes::VW,
    robots::VR;
    draw_bbox = true,
    title = nothing,
    kwargs...,
) where {W<:GFF.AbstractWez,VW<:AbstractVector{W},R<:Robot,VR<:AbstractVector{R}}

    if draw_bbox
        # plot the bounding box
        plot!([0, 1, 1, 0, 0], [0, 0, 1, 1, 0], label = false, color = :black)
        plot!(xlims = (-0.4, 1.4), ylims = (-0.4, 1.4))
    end

    # plot the wezes
    for wez in wezes, robot in robots
        plot!(wez, robot)
    end

    # plot the robot
    for robot in robots
        color = is_colliding(wezes, robot) ? :red : :green
        plot!(robot; color = color)
    end

    if isnothing(title)
        # get the minimum collision distance
        mind = Inf
        for r in robots, w in wezes
            d = collision_distance(w, r)
            mind = min(mind, d)
        end
        plot!(title = "Min Wez Distance: $(round(mind; digits=2))")

        if mind <= 0
            plot!(titlefont = font(:red))
        else
            plot!(titlefont = font(:black))
        end
    else
        plot!(title = title)
    end

    plot!(aspect_ratio = :equal) 

end

println("Plotting leader path...")
plot()
plot_scenario!(wezes, robots)
plot!(path, color = :black, label = false, linewidth = 2)
title!("best path")
plot!()
savefig("rrt_best_path.svg")
println("...leader path plotted.")

## Now we can construct a `GatekeeperProblem`:
println("Creating Gatekeeper Problems...")
offsets = [SVector(robot) - SVector(leader_robot) for robot in robots]

# For all three robots, we can define the set of problems as 
gk_problems = [
    GatekeeperProblem(;
        wezes = wezes,
        reference_path = path,
        offset = offsets[i],
        switch_step_size = 2e-3,
        reconnection_step_size = 0.01,
        max_Ts_horizon = 0.5,
        integration_max_step_size = 1e-3,
        collision_check_step_size = 1e-3,
    ) for i = 1:N_robots
]
println("...gatekeeper problems created.")


# We can now `solve` the problems and plot the solutions:
println("Solving Gatekeeper Problems...")
tspan = [0.0, total_path_length(path)]
gk_solutions = [
    simulate_closed_loop_gatekeeper(SVector(robots[i]), tspan, gk_problems[i]) for
    i = 1:N_robots
] 
println("...gatekeeper problems solved.")

Tmax = total_path_length(path)


# plot reference path
println("Plotting Gatekeeper Solutions...")
plot()
plot_scenario!(wezes, robots)
# plot offset path
for i = 1:N_robots
    plot!(
        τ -> get_reference_state_and_input(path, τ, offsets[i])[1][1],
        τ -> get_reference_state_and_input(path, τ, offsets[i])[1][2],
        0.0,
        Tmax,
        linestyle = :dash,
        label = false,
        linecolor = :black,
    )
end
# plot the gk solution
for i = 1:N_robots
    plot!(
        t -> gk_solutions[i](t)[1],
        t -> gk_solutions[i](t)[2],
        0.0,
        Tmax,
        label = "gk_sol_$(i)",
        color = (i == 1 ? :black : :green),
        linewidth = 2,
    )
end

plot!(aspect_ratio = :equal)
savefig("gatekeeper_paths.svg")
println("...gatekeeper solutions plotted.")



# Finally, lets animate the solutions:
make_animation = true
if make_animation
    println("Creating animation of Gatekeeper Solutions...")
    anim = @animate for t in range(0, Tmax, length=60)

        # grab the states
        robots_ = [Robot(gk_solutions[i](t)) for i=1:N_robots]

        # start the plot
        plot()

        # plot the reference paths
        for i=1:3
            plot!(τ -> get_reference_state_and_input(path, τ, offsets[i])[1][1], τ -> get_reference_state_and_input(path, τ, offsets[i])[1][2], 0, Tmax;
                label=false, color=:gray, linestyle=:dash)
        end

        # plot the trace of the robots upto this point of time
        for i=1:3
            plot!(τ -> gk_solutions[i](τ)[1], τ -> gk_solutions[i](τ)[2], 0.0, t, label="gk_sol_$(i)", color=(i==1 ? :black : :green), linewidth=2)
        end

        # plot the current state of affairs
        plot_scenario!(wezes, robots_)

        plot!()
    end

    gif(anim, "gatekeeper.gif")
    println("...animation created.")
end


# save the trajectories to a csv file
if true
    println("Saving Gatekeeper Trajectories to CSV...")
    using CSV, DataFrames
    df = DataFrame(
        time = Float64[],
        robot_id = Int[],
        x = Float64[],
        y = Float64[], 
        yaw = Float64[],
    )
    for i = 1:N_robots
        for t in range(0, Tmax, length=1000)
            push!(df, (t, i, gk_solutions[i](t)[1], gk_solutions[i](t)[2], gk_solutions[i](t)[3]))
        end
    end

    # rescale DataFrame
    T = 20.0 # desired t_max
    tmax = maximum(df.time)
    df.time = map(ti -> ti * (T / tmax), df.time)
    df.x = map(xi -> 4 * xi - 2, df.x) # scale to [-2, 2]
    df.y = map(yi -> 4 * yi - 2, df.y) # scale to [-2, 2]
    df.yaw = map(yaw -> 0 * yaw, df.yaw) # force all yaws to 0

    CSV.write("gatekeeper_trajectories.csv", df)
    println("...saved csv")

    println("Creating polynomial representation of trajectories...")
    include("utils_polynomial.jl")
    # for each robot, create a polynomial representation of the trajectory
    for i = 1:N_robots
        # print the robot id
        println("Creating polynomial representation for robot $(i)...")
        # extract the trajectory
        
        ts = df[df.robot_id .== i, :].time
        xs = df[df.robot_id .== i, :].x
        ys = df[df.robot_id .== i, :].y
        zs = 1.0 .+ 0 * xs # assume z is constant at 1.0
        yaws = df[df.robot_id .== i, :].yaw

        # create the polynomial representation
        δt = 1.0 # time step for the polynomial representation
        polys_x = create_polynominal_representation(ts, xs, δt)
        polys_y = create_polynominal_representation(ts, ys, δt)
        polys_z = create_polynominal_representation(ts, zs, δt)
        polys_yaw = create_polynominal_representation(ts, yaws, δt)

        # save the polynomials to a file
        write_poly_file("robot_$(i)_polynomials.csv", polys_x, polys_y, polys_z, polys_yaw)
    end
end

println("** fin **")
plot!()
# see gatekeeper.gif for the animated solution.


