# Copyright (c) 2025 Devansh R Agrawal - All rights reserved.

using GatekeeperFormationFlight
using Plots, LinearAlgebra, StaticArrays, Random
using Dubins

GFF = GatekeeperFormationFlight
include("utils_plotting.jl")

foam_obs_L = 0.6 # meters on a side
cf_padding = 0.10 # meters
circle_obs_radius =  foam_obs_L / sqrt(2) + cf_padding # meters
true_interagent_distance = 0.20 # meters

scale_factor = 3.0 # scale factor for the real world
# scale the circle_obs_radius
obs_r = circle_obs_radius / scale_factor
println("obs_r: ", obs_r)

function create_scenario()
    # create a set of wezes
    wezes = CircularWez[]
    push!(wezes, CircularWez(0.3, 0.4, obs_r)) 
    push!(wezes, CircularWez(0.75, 0.6, obs_r))
    push!(wezes, CircularWez(0.5, 0.9, obs_r))
    # push!(wezes, CircularWez(0.7, 0.6, obs_r)) # right wez
    # for y in -2.0:obs_r:2.0
    #     if !(0.6 <= y <= 0.8)
    #         push!(wezes, CircularWez(0.3, y, obs_r))
    #     end
    #     if !(0.2 <= y <= 0.75)
    #         push!(wezes, CircularWez(0.7, y, obs_r))
    #     end
    # end
    return wezes
end


println("Creating Scenario...")
# change the seed to get different environments
# seed = 27182818 with 24 wezes is a good looking env.
Random.seed!(0)
#wezes = create_random_scenario(5)
wezes = create_scenario()


# create a set of robots



leader_robot = Robot(-0.1, 0.0, 0.0)
L = 2 * true_interagent_distance / scale_factor # distance between the leader and the followers
half_angle = π / 6 # angle between the leader and the followers # 30 degrees
follower_robots = [
                    Robot(leader_robot.x - L * cos(half_angle), leader_robot.y - L * sin(half_angle), 0.0), 
                    Robot(leader_robot.x - L * cos(half_angle), leader_robot.y + L * sin(half_angle), 0.0)
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
success_code, waypoints = get_best_path(rrt_problem, nodes, @SVector [1.0, 0.0, 0]) # new go

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
waypoints = [SVector(leader_robot), waypoints..., SVector(1.33, 0.0, 0.0)]

path = DubinsPath[]
# add all the waypoints to the path
for i = 2:length(waypoints)
    e, p = dubins_shortest_path(waypoints[i-1], waypoints[i], rrt_problem.turning_radius)
    @assert e == Dubins.EDUBOK
    push!(path, p)
end

println("...leader path planned.")


println("Plotting leader path...")
plot()
plot_scenario!(wezes, robots)
plot!(path, color = :black, label = false, linewidth = 2)
title!("best path")
plot!()
savefig("rrt_best_path.svg")
println("...leader path plotted.")

## Now we can construct a `GatekeeperProblem`:
println("Starting Gatekeeper Simulations...")
offsets = [SVector(robot) - SVector(leader_robot) for robot in robots]


# For all three robots, we can define the set of problems as 
gk_problems = []
gk_solutions = []

for i=1:N_robots

    other_agents = []
    for j=1:(i-1)
        other_agent_path = gk_solutions[j]
        push!(other_agents, other_agent_path)
    end

    prob = GatekeeperProblem(;
        wezes = wezes,
        reference_path = path,
        offset = offsets[i],
        switch_step_size = 1e-2,
        reconnection_step_size = 5e-2,
        max_Ts_horizon = 0.5,
        integration_max_step_size = 1e-3,
        collision_check_step_size = 5e-3,
        other_agents = other_agents,
        interagent_collision_distance=(true_interagent_distance / scale_factor), # will get scaled by a factor when applied to real world!
    )

    push!(gk_problems, prob)

    # solve the problem for this agent
    tspan = [0.0, total_path_length(path)]
    gk_sol = simulate_closed_loop_gatekeeper(SVector(robots[i]), tspan, gk_problems[i])
    push!(gk_solutions, gk_sol)
    println("...solved Gatekeeper Problem for robot $(i).")
end
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


# compute inter-agent distance and plot it 
println("Plotting Inter-agent Distances...")
plot()
for i = 1:N_robots
    for j = (i+1):N_robots
        sol_a = gk_solutions[i]
        sol_b = gk_solutions[j]

        plot!(t-> norm(sol_a(t)[SOneTo(2)] - sol_b(t)[SOneTo(2)]), 0, Tmax, label="dist_$(i)_$(j)")
    end
end
xlabel!("Time (TU)")
ylabel!("Inter-agent distance (LU)")
hline!([0, true_interagent_distance / scale_factor], linestyle=:dash, color=:red, label="min inter-agent distance")
title!("Inter-agent distances")
savefig("inter_agent_distances.svg")
println("...inter-agent distances plotted.")

# Finally, lets animate the solutions:
make_animation = false
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
    # scale everything
    df.time = map(ti -> ti * (T / tmax), df.time)
    df.x = map(xi -> scale_factor * (xi - 0.5), df.x)
    df.y = map(yi -> scale_factor * (yi - 0.5), df.y)
    # df.yaw = map(yaw -> 0 * yaw, df.yaw) # force all yaws to 0

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
        zs = 0.5 .+ 0 * xs # assume z is constant at 1.0
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


