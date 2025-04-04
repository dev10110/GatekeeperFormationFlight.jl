# Robots and Wezes

## Tutorial

To construct a robot use
```
robot = Robot(x, y, psi)
```

To construct a wez, use one of
```
wez = Cardioid(x, y)
wez = CircularWez(x, y)
wez = Cbez(x, y, ψ)
```
each of these wezes is a subtype of `AbstractWez`. See the docs below to customize each wez.

To check collision between a robot and a wez, use
```
is_colliding(wez, robot)
```

you can also get the `collision_distance(wez, robot)` or the `wez_coordinates(theta, wez, robot)`. 

To plot the robot or the wezes, simply
```
using Plots

plot()
plot!(wez)
plot!(robot)
```


## Example

```@example wez
using GatekeeperFormationFlight
using Plots, LinearAlgebra, StaticArrays, Random

# function to create an environment
function create_random_scenario(N_wezes=24)
    wezes_1 = [Cardioid(rand(), rand()) for i=1:N_wezes/2]
    wezes_2 = [Cardioid(0.5 + 0.2 * randn(), 0.5 + 0.2 * randn()) for i=1:N_wezes/2]
    wezes = vcat(wezes_1, wezes_2)
    return wezes
end

# create the environment
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
plot_scenario(wezes, robots)
```



## Docs

### Public
```@autodocs; canonical=false
Modules = [GatekeeperFormationFlight]
Pages = ["robot.jl", "wez.jl", "plotting_utils.jl"]
Private = false
```

### Private
```@autodocs; canonical=false
Modules = [GatekeeperFormationFlight]
Pages = ["robot.jl", "wez.jl", "plotting_utils.jl"]
Public = false
```