using .RRTStar

using StaticArrays
using Dubins3D


"""
    dubins_distance_3d(q1, q2; turning_radius = 40, pitch_angle_constraints = [-15, 20])

Get the distance between two states connected by a 3D dubins path
"""
function dubins_distance_3d(
    q1,
    q2,
    turning_radius = 40,
    pitch_angle_constraints = [-15, 20],
)
    return DubinsManeuver3D(q1, q2, turning_radius, pitch_angle_constraints).length
end

"""
    in_domain(domain, q)

returns true if the state q is within the domain defined by the tuple domain
"""
function in_domain(domain::Tuple{SVector{6,F},SVector{6,F}}, q::Vector{F}) where {F}
    return all(q .>= domain[1][1:5]) && all(q .<= domain[2][1:5])
end


"""
    Dubins3DRRTProblem(domain, turning_radius, wezes)

Construct a 3D Dubins RRT* problem

Parameters:
    - `domain`: a tuple defining the domain
    - `turning_radius`: turning radius for Dubins paths
    - `obstacles`: A vector of 3D obstacles to avoid

Example:
```
using GatekeeperFormationFlight
using StaticArrays

# x, y, z, yaw, pitch, roll 
min_domain = SVector{5}(0.0, 0.0 0.0, -1.0*π, deg2rad(-15), deg2rad(-10)) 
max_domain = SVector{5}(1.0, 1.0, 1.0*π, deg2rad(20), deg2rad(10))
domain = (min_domain, max_domain)

turning_radius = 10

obstacles = [Sphere(rand(), rand(), rand(), rand()) for i=1:10]

rrt_problem = Dubins3DRRTProblem(domain, turning_radius, obstacles)
```
"""
struct Dubins3DRRTProblem{F,O} <: RRTStar.AbstractProblem{SVector{6,F}}
    domain::Tuple{SVector{6,F},SVector{6,F}}
    turning_radius::F
    obstacles::Vector{O}
end


# Creates a default domain, with abstract obstacles as an argument
function Dubins3DRRTProblem(obstacles::VO) where {O<:AbstractObstacle,VO<:AbstractVector{O}}
    min_domain = SVector{6}(0.0, 0.0, 0.0, -1.0 * π, deg2rad(-15), deg2rad(-10))
    max_domain = SVector{6}(100.0, 100.0, 100.0, 1.0 * π, deg2rad(20), deg2rad(10))
    turning_radius = 10.0

    return Dubins3DRRTProblem((min_domain, max_domain), turning_radius, obstacles)
end

"""
Returns a random vector lying in the problem domain
"""
function sample_domain(P::Dubins3DRRTProblem)
    @debug "Sample Domain"

    v = @SVector rand(6)
    sampled = P.domain[1] + (P.domain[2] - P.domain[1]) .* v

    return sampled
end

"""
    sample_free(problem::Dubins3DRRTProblem)

returns a random state within the problem domain
"""
function RRTStar.sample_free(problem::Dubins3DRRTProblem)
    @debug "sample_free"

    q = sample_domain(problem)
    while is_colliding(problem.obstacles, q[1:3])
        q = sample_domain(problem)
    end

    return q
end


"""
    nearest(problem, nodes, x_rand)

returns the index of the nearest nodew ithin the list of 'nodes' to the state 'x_rand', according to a dubins path distance.
"""
function RRTStar.nearest(P::Dubins3DRRTProblem, nodes, x_rand)
    @debug "nearest"

    pitchlims = [P.domain[1][5], P.domain[2][5]]

    dist =
        (from_node) ->
            DubinsManeuver3D(
                Vector(from_node.state),
                Vector(x_rand),
                P.turning_radius,
                pitchlims,
            ).length

    @debug "staring map"
    distances = map(dist, nodes)
    @debug "distances = $distances"

    return argmin(distances)
end

"""
    near(problem, nodes, x_new)

returns the set of indices of nearby nodes to a state x_new -- currently just returns all nodes

TODO -- implement better search
"""
function RRTStar.near(P::Dubins3DRRTProblem, nodes, x_new)
    @debug "near"
    return 1:length(nodes)
end

"""
    steer(problem, x_nearest, x_rand; max_travel_dist=0.2)

returns the state by traveling up to "max_travel_dist" along the dubins path between x_nearest and x_rand
"""
function RRTStar.steer(P::Dubins3DRRTProblem, x_nearest, x_rand; max_travel_dist = 2.0)
    @debug "steer"

    pitchlims = [P.domain[1][5], P.domain[2][5]]

    maneuver =
        DubinsManeuver3D(Vector(x_nearest), Vector(x_rand), P.turning_radius, pitchlims)

    num_samples = floor(Int, maneuver.length / 0.05)
    sampled_path = compute_sampling(maneuver; numberOfSamples = num_samples)

    # in the case of a very short path
    if max_travel_dist < maneuver.length
        steer_point = sampled_path[end]

        # wrap angles
        steer_point[4] = wrapToPi(steer_point[4])
        steer_point[5] = wrapToPi(steer_point[5])

        return SVector{6,Float64}(steer_point..., 0.0)
    end

    # Walk along the path, until distance traveled is greater than max_travel_dist
    # TODO -- this is inefficient, but it works

    cumulative_distance = 0.0
    x_idx = 1
    while cumulative_distance < max_travel_dist
        cumulative_distance += sqrt((sampled_path[x_idx+1] - sampled_path[x_idx])^2)
    end

    steer_point = sampled_path[x_idx]

    # wrap angles
    steer_point[4] = wrapToPi(steer_point[4])
    steer_point[5] = wrapToPi(steer_point[5])

    return SVector{6,Float64}(steer_point..., 0.0)
end


function RRTStar.collision_free(
    problem::Dubins3DRRTProblem,
    x_nearest,
    x_new;
    step_size = 0.01,
)
    @debug "collision_free"

    maneuver = DubinsManeuver3D(
        Vector(x_nearest),
        Vector(x_new),
        problem.turning_radius,
        [problem.domain[1][5], problem.domain[2][5]],
    )

    # Sampled path contains an array of points along the path
    num_samples = floor(Int, maneuver.length / step_size)
    sampled_path = compute_sampling(maneuver; numberOfSamples = num_samples)

    # check if any of the points in the path are colliding with the obstacles
    return !any(
        is_colliding(problem.obstacles, x[1:3]) && in_domain(problem.domain, x) for
        x in sampled_path
    )
end

"""
    path_cost(problem::Dubins3DRRTProblem, x_near, x_new)

returns the length of the dubins path from x_near to x_new
"""
function RRTStar.path_cost(problem::Dubins3DRRTProblem, x_near, x_new)
    @debug "path_cost between $x_near and $x_new"

    cost =
        DubinsManeuver3D(
            Vector(x_near),
            Vector(x_new),
            problem.turning_radius,
            [problem.domain[1][5], problem.domain[2][5]],
        ).length

    @debug "cost = $cost"

    return cost
end



