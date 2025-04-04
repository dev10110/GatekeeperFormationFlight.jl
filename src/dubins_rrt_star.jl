import Dubins

using .RRTStar
using StaticArrays
import Dubins

# # provide an override to the Dubins library
# function Dubins.dubins_shortest_path(q0, q1, ρ)
#     return Dubins.dubins_shortest_path(Vector(q0), Vector(q1), ρ)
# end

"""
    dubins_distance(q1, q2, turning_radius=0.1)
    
get the distance between two states, if connected by a dubins path
"""
function dubins_distance(q1, q2, turning_radius = 0.1)

    e, p = Dubins.dubins_shortest_path(q1, q2, turning_radius)
    @assert e == Dubins.EDUBOK

    return Dubins.dubins_path_length(p)
end

"""
    DubinsRRTProblem(domain, turning_radius, wezes)

Construct a dubins RRT* problem. 

Parameters:
 - `domain`: a tuple defining the domain
 - `turning_radius`: turning radius for Dubins paths
 - `wezes`: A vector of wezes to avoid

Example:
```
using GatekeeperFormationFlight
using StaticArrays

min_domain = SVector{3}(0.0, 0.0, -1.0*π)
max_domain = SVector{3}(1.0, 1.0, 1.0*π)
domain = (min_domain, max_domain)

turning_radius = 0.1

wezes = [Cardioid(rand(), rand()) for i=1:10]

rrt_problem = DubinsRRTProblem(domain, turning_radius, wezes)
```
"""
struct DubinsRRTProblem{F,W} <: RRTStar.AbstractProblem{SVector{3,F}}
    domain::Tuple{SVector{3,F},SVector{3,F}}
    turning_radius::F
    wezes::Vector{W}
end

# create a default domain, with the wezes as an argument
function DubinsRRTProblem(wezes::VW) where {W<:AbstractWez,VW<:AbstractVector{W}}
    min_domain = SVector{3}(0.0, 0.0, -1.0 * π)
    max_domain = SVector{3}(1.0, 1.0, 1.0 * π)
    turning_radius = 0.1
    return DubinsRRTProblem((min_domain, max_domain), turning_radius, wezes)
end

function sample_domain(P::DubinsRRTProblem)
    v = @SVector rand(3)
    return P.domain[1] + (P.domain[2] - P.domain[1]) .* v
end

"""
    sample_free(problem::DubinsRRTProblem)

returns a random state within the problem domain
"""
function RRTStar.sample_free(problem::DubinsRRTProblem)

    while true
        q = sample_domain(problem)

        # check if q is in a wez
        r = Robot(q...)
        if !is_colliding(problem.wezes, r)
            return q
        end
    end

end

"""
    nearest(problem, nodes, x_rand)

returns the index of the nearest node within the list of `nodes` to the state `x_rand`, according to a dubins path distance.
"""
function RRTStar.nearest(P::DubinsRRTProblem, nodes, x_rand)
    nearest_ind = 0
    nearest_d = Inf

    for i in eachindex(nodes)
        d = dubins_distance(nodes[i].state, x_rand, P.turning_radius)
        if d < nearest_d
            nearest_ind = i
            nearest_d = d
        end
    end
    return nearest_ind
end

"""
        near(problem, nodes, x_new)

returns the set of indices of nearby nodes to a state `x_new`. For simplicity, it currently just returns all nodes.
"""
function RRTStar.near(problem::DubinsRRTProblem, nodes, x_new)
    # todo: be more judicious about which nodes are nearby
    return 1:length(nodes)
end

"""
    steer(problem, x_nearest, x_rand; max_travel_dist=0.2)

returns the state by travelling upto `max_travel_dist` towards `x_rand` starting from `x_nearest`.
"""
function RRTStar.steer(problem::DubinsRRTProblem, x_nearest, x_rand; max_travel_dist = 0.2)

    # first plan the full dubins path
    errcode, path = Dubins.dubins_shortest_path(x_nearest, x_rand, problem.turning_radius)
    @assert errcode == Dubins.EDUBOK

    L = Dubins.dubins_path_length(path)

    # get the node 20% of the waythrough
    s = min(L, max_travel_dist)

    errcode, x_new = Dubins.dubins_path_sample(path, s)
    @assert errcode == Dubins.EDUBOK

    return SVector{3}(x_new)

end

"""
    collision_free(problem, x_nearest, x_new; step_size=0.001)

checks if a path from `x_nearest` to `x_new` is collision free with respect to the wezes in `problem.wezes`.
"""
function RRTStar.collision_free(
    problem::DubinsRRTProblem,
    x_nearest,
    x_new;
    step_size = 0.001,
)

    # create the path
    errcode, path = Dubins.dubins_shortest_path(x_nearest, x_new, problem.turning_radius)
    @assert errcode == Dubins.EDUBOK

    L = Dubins.dubins_path_length(path)

    # sample path at a fine resolution
    xs = range(0.0, L, step = step_size)

    for x in xs
        errcode, q = Dubins.dubins_path_sample(path, x)
        #     errcode, samples = Dubins.dubins_path_sample_many(path, resolution)
        @assert errcode == Dubins.EDUBOK errcode

        # check each point for collision 
        r = Robot(q)
        if is_colliding(problem.wezes, r)
            return false
        end
    end
    return true

    # return !is_colliding(problem.wezes, path, step_size)
end

"""
    path_cost(problem, x_near, x_new)

returns the dubins path length from x_near to x_new.
"""
function RRTStar.path_cost(problem::DubinsRRTProblem, x_near, x_new)
    return dubins_distance(x_near, x_new, problem.turning_radius)
end
