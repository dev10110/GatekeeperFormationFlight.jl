# Gatekeeper

## Tutorial

## Example


Consider the same environment as in the tracking controllers page.
```@setup gk
using GatekeeperFormationFlight
using Plots, LinearAlgebra, StaticArrays, Random
using Dubins

GFF = GatekeeperFormationFlight

# create an environment
function create_random_scenario(N_wezes=24)
    wezes_1 = [Cardioid(rand(), rand()) for i=1:N_wezes/2]
    wezes_2 = [Cardioid(0.5 + 0.2 * randn(), 0.5 + 0.2 * randn()) for i=1:N_wezes/2]
    wezes = vcat(wezes_1, wezes_2)
    return wezes
end
Random.seed!(27182818)
wezes = create_random_scenario(24)

# create a set of robots
leader_robot = Robot(-0.25,0.,0.)
follower_robots = [
    Robot(-0.3, -0.05, 0.0), 
    Robot(-0.3, 0.05, 0.0), 
    ]
robots = vcat(leader_robot, follower_robots...)

# plan a path for the leader using RRT*
domain = (
    (@SVector [0, 0, -1.0*π]),
    (@SVector [1, 1,  1.0*π])
)
turning_radius = 0.1
rrt_problem = DubinsRRTProblem(domain, turning_radius, wezes)

# start the tree with the root node
nodes = [Node(SVector(0,0,0.)), ]

# add 1000 nodes to the tree
nodes = rrt_star(rrt_problem, nodes, 1000)

# see if there is a path 
success_code, waypoints = get_best_path(rrt_problem, nodes, @SVector [1.0, 1.0, 0])

# while we havent found a path, add some nodes and check if we have a feasible path
iter_counter = 0
while !success_code  && iter_counter < 10
    nodes = rrt_star(rrt_problem, nodes, 500)
    success_code, waypoints = get_best_path(rrt_problem, nodes, [1.0, 1.0, 0])
    iter_counter += 1
end
        
@assert success_code

# prepend and append the start and the goal
waypoints = [
    SVector(leader_robot), 
    waypoints..., 
    SVector(1.25,1,0.)
]

path = DubinsPath[]
# add all the waypoints to the path
for i=2:length(waypoints)
    e, p = dubins_shortest_path(waypoints[i-1], waypoints[i], rrt_problem.turning_radius)
    @assert e == Dubins.EDUBOK
    push!(path, p)
end

# small function to help plot things
function plot_scenario!(wezes::VW, robots::VR; draw_bbox=true, title=nothing, kwargs...) where {W <: GFF.AbstractWez, VW <: AbstractVector{W}, R <: Robot, VR <: AbstractVector{R}}

    if draw_bbox
        # plot the bounding box
        plot!([0, 1, 1, 0, 0], [0, 0, 1, 1, 0], label = false, color = :black)
        plot!(xlims = (-0.4, 1.4), ylims = (-0.1, 1.1))
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

    plot!()

end

plot()
plot_scenario!(wezes, robots)
plot!(path, color=:black, label=false, linewidth=2)
title!("best path")
plot!()
```


```@example gk
plot!()
```

Now we can construct a `GatekeeperProblem`:

```@example gk

offsets = [SVector(robot) - SVector(leader_robot) for robot in robots]

prob = GatekeeperProblem(;
        wezes=wezes, 
        reference_path=path,
        offset=offsets[1], 
        switch_step_size=2e-3, 
        reconnection_step_size=0.01, 
        max_Ts_horizon=0.5, 
        integration_max_step_size=1e-3, 
        collision_check_step_size=1e-3, 
        ) 
```

For all three robots, we can define the set of problems as 

```@example gk
gk_problems = [
    GatekeeperProblem(;
        wezes=wezes, 
        reference_path=path, offset=offsets[i], 
        switch_step_size=2e-3, 
        reconnection_step_size=0.01, 
        max_Ts_horizon=0.5, 
        integration_max_step_size=1e-3,
        collision_check_step_size=1e-3,
        ) 
    for i=1:length(robots)
]
```

We can now `solve` the problems and plot the solutions:
```@example gk
tspan = [0.0, total_path_length(path)]
gk_solutions = [
    simulate_closed_loop_gatekeeper(
        SVector(robots[i]), 
        tspan, 
        gk_problems[i])
    for i=1:length(robots)
]

Tmax = total_path_length(path)

# plot reference path
plot()
plot_scenario!(wezes, robots)
# for p in path
#     plot!(p, color=:black, linestyle=:dash, label=false)
# end

# plot offset path
for i=1:length(robots)
    plot!(τ-> get_reference_state_and_input(path, τ, offsets[i])[1][1], 
    τ-> get_reference_state_and_input(path, τ, offsets[i])[1][2],
    0.0, Tmax, linestyle=:dash, label=false, linecolor=:black)
end

# plot the gk solution
for i=1:length(robots)
    plot!(t->gk_solutions[i](t)[1], t->gk_solutions[i](t)[2], 0.0, Tmax, label="gk_sol_$(i)", color=(i==1 ? :black : :green), linewidth=2)
end

plot!(aspect_ratio=:equal)
```

Finally, lets animate the solutions:
```@example gk

@gif for t in range(0, Tmax, length=120)

    # grab the states
    robots_ = [Robot(gk_solutions[i](t)) for i=1:length(gk_solutions)]

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
```

## Docs

### Public:
```@autodocs; canonical=false
Modules = [GatekeeperFormationFlight]
Pages = ["gatekeeper.jl"]
Private = false
```

### Private:
```@autodocs; canonical=false
Modules = [GatekeeperFormationFlight]
Pages = ["gatekeeper.jl"]
Public = false
```