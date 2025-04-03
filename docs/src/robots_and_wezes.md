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


## Docs

```@docs; canonical=false
Robot
```

```@docs; canonical=false
Cardioid
```

```@docs; canonical=false
CircularWez
```

```@docs; canonical=false
Cbez
```

```@docs; canonical=false
is_colliding
```

```@docs; canonical=false
collision_distance
```

```@docs; canonical=false
wez_coordinates
```







