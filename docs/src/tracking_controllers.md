# Tracking Controller

## Tutorial

Given a dubins path, we can now run a simulation of tracking it. 

Given a `path::Vector{DubinsPath}`, we have the following methods:
- `x_d, u_d = get_reference_state_and_input(path, time)` which returns state and input to track the trajectory at some `time`
- `x_d, u_d = get_reference_state_and_input(path, time, offset)` which returns the same, but for an offset path. the offset is specified as `offset = (d, n)` where `d` is the tangential offset, and `n` is the offset in the normal direction. 
- `u = tracking_controller(x, x_d, u_d)` which returns the control input to track a trajectory with a given desired state `x_d` and a feedforward control input `u_d`. 

## Example


Consider the same environment as in the RRT* page.
```@setup tracking
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


```@example tracking
plot!()
```

We can construct the offset paths as follows:
```@example tracking
function offset_path_1(t)
    offset = SVector(follower_robots[1]) - SVector(leader_robot)
    x_d, u_d = get_reference_state_and_input(path, t, offset)
    return x_d[SOneTo(2)]
end

function offset_path_2(t)
    offset = SVector(follower_robots[2]) - SVector(leader_robot)
    x_d, u_d = get_reference_state_and_input(path, t, offset)
    return x_d[SOneTo(2)]
end

# get the total time span
tspan = (0.0, total_path_length(path))

# plot the paths
plot!(t -> offset_path_1(t)[1], t -> offset_path_1(t)[2], tspan...; color=:gray, linestyle=:dash, label=false)
plot!(t -> offset_path_2(t)[1], t -> offset_path_2(t)[2], tspan...; color=:gray, linestyle=:dash, label=false)
```

However these offset paths do not respect input bounds. 
We can now simulate a robot tracking this trajectory:

```@example tracking
using OrdinaryDiffEq

# define the closed-loop controller
function closed_loop_tracking(x, params, t)
    
    # extract params
    ref_path, offset = params

    # get the nominal tracking control input 
    x_d, u_d = get_reference_state_and_input(ref_path, t, offset)
    u = tracking_controller(x, x_d, u_d)

    # apply input bounds
    v, ω = apply_input_bounds(u...)

    # compute xdot
    θ = x[3]
    xdot = SVector(v * cos(θ), v * sin(θ), ω)

    return xdot
end

# define the ODE problem
tspan = (0, total_path_length(path))
params_1 = (path, SVector(follower_robots[1]) - SVector(leader_robot))
params_2 = (path, SVector(follower_robots[2]) - SVector(leader_robot))

prob_1 = ODEProblem(closed_loop_tracking, SVector(follower_robots[1]), tspan, params_1)
prob_2 = ODEProblem(closed_loop_tracking, SVector(follower_robots[2]), tspan, params_2)

# solve the problems
sol_1 = solve(prob_1)
sol_2 = solve(prob_2)

# plot the solution
plot!(t -> sol_1(t)[1], t -> sol_1(t)[2], tspan..., color=:green, linewidth=2, label=false)
plot!(t -> sol_2(t)[1], t -> sol_2(t)[2], tspan..., color=:green, linewidth=2, label=false)
```

We can animate the result too:
```@example tracking

@gif for t in range(tspan..., length=120)

    leader_state, _ = get_reference_state_and_input(path, t)
    follower_1_state = sol_1(t)
    follower_2_state = sol_2(t)

    robots_ = Robot.([leader_state, follower_1_state, follower_2_state])

    # plot the reference paths
    plot()
    plot!(t -> offset_path_1(t)[1], t -> offset_path_1(t)[2], tspan...; color=:gray, linestyle=:dash, label=false)
    plot!(t -> offset_path_2(t)[1], t -> offset_path_2(t)[2], tspan...; color=:gray, linestyle=:dash, label=false)

    # plot the robots at the current time
    plot_scenario!(wezes, robots_)

end
```

This closed-loop tracking controller will be exported as part of the gatekeeper methods later.

## Docs

### Public:
```@autodocs; canonical=false
Modules = [GatekeeperFormationFlight]
Pages = ["tracking_controller.jl"]
Private = false
```

### Private:
```@autodocs; canonical=false
Modules = [GatekeeperFormationFlight]
Pages = ["tracking_controller.jl"]
Public = false
```