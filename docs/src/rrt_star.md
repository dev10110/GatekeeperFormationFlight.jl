# RRT*

We use RRT* as an offline method to solve for the path through an environment with wezes.

Note, we are using Dev's fork of `Dubins.jl`, available at [http://github.com/dev10110/Dubins.jl](http://github.com/dev10110/Dubins.jl)

## Example: 

First define the environment:
```@example rrt
using GatekeeperFormationFlight
using Plots, LinearAlgebra, StaticArrays, Random
using Dubins

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

# plot everything
plot(wezes, robots)
```

Define and solve the RRT* problem:
```@example rrt

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
```

Now we have a list of waypoints, construct a path through them
```@example rrt
path = DubinsPath[]
# add all the waypoints to the path
for i=2:length(waypoints)
    e, p = dubins_shortest_path(waypoints[i-1], waypoints[i], rrt_problem.turning_radius)
    @assert e == Dubins.EDUBOK
    push!(path, p)
end
path
```
`path` is now of type `Vector{DubinsPath}`. Plot it:

```@example rrt
plot(wezes, robots)
plot!(path, color=:black, label=false, linewidth=2)
title!("best path")
plot!()
```

Notice we have provided a function to call `plot!` on `DubinsPath` or on `Vector{DubinsPath}`. 

We can also animate the trajectory 
```
@example rrt

@gif for t in range(0, total_path_length(path), length=120)

    leader_state, leader_input = get_reference_state_and_input(path, t)

    leader = Robot(leader_state)

    plot(wezes, leader)
end
```
`get_reference_state_and_input` is explained on the next page.

## Docs

```@autodocs; canonical=false
Modules=[GatekeeperFormationFlight]
Pages = ["dubins_rrt_star.jl"]
```
