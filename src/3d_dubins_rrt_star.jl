using .RRTStar

using StaticArrays
using Dubins3D

using ..Obstacles

function Obstacles.is_colliding(
    obs::VAO,
    pos::VF,
) where {AO<:AbstractObstacle,VAO<:AbstractVector{AO},F<:Real,VF<:AbstractVector{F}}
    return any(o -> is_colliding(o, pos, 0.0, 0.0), obs)
end


"""
    dubins_distance_3d(q1, q2; turning_radius = 40, pitch_angle_constraints = [-15, 20])

Get the distance between two states connected by a 3D dubins path
"""
function dubins_distance_3d(
    q1::SVector{5,Float64},
    q2::SVector{5,Float64};
    turning_radius::Float64 = 40.0,
    pitch_angle_constraints::SVector{2,Float64},# = SVector{2,Float64}(-15, 20),
)
    return DubinsManeuver3D(q1, q2, turning_radius, pitch_angle_constraints).length
end

function safe_dubins_maneuver_3d(
    from_vec::SVector{5,Float64},
    to_vec::SVector{5,Float64},
    turning_radius::Float64,
    pitch_angle_constraints::SVector{2,Float64},# = SVector{2,Float64}(-15, 20),
    domain::Tuple{SVector{5,Float64},SVector{5,Float64}},
)
    if !in_domain(domain, from_vec) || !in_domain(domain, to_vec)
        return nothing
    end

    try
        return DubinsManeuver3D(from_vec, to_vec, turning_radius, pitch_angle_constraints)
    catch e
        if isa(e, DomainError) && e.val < 0 && abs(e.val) < 1e-6
            # Handle the case where the distance is very small
            perturbation = 1e-8 * randn(5)
            perturbed_to = from_vec + perturbation

            if !in_domain(domain, perturbed_to)
                # Clamp to domain
                perturbed_to =
                    SVector{5,Float64}(max.(min.(perturbed_to, domain[2]), domain[1]))
            end

            return safe_dubins_maneuver_3d(
                from_vec,
                perturbed_to,
                turning_radius,
                pitch_angle_constraints,
                domain,
            )
        else
            rethrow(e)
        end
    end
end

"""
    in_domain(domain, q)

returns true if the state q is within the domain defined by the tuple domain
"""
function in_domain(
    domain::Tuple{SVector{5,Float64},SVector{5,Float64}},
    q::SVector{5,Float64},
)::Bool
    id = all(q .>= domain[1]) && all(q .<= domain[2])
    if !id
        @debug "State $q is out of domain $domain"
    end
    return id
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
struct Dubins3DRRTProblem{F,VO} <: RRTStar.AbstractProblem{SVector{5,F}}
    domain::Tuple{SVector{5,Float64},SVector{5,Float64}}
    turning_radius::Float64
    obstacles::VO
end

function Dubins3DRRTProblem(
    domain::Tuple{SVector{5,Float64},SVector{5,Float64}},
    turning_radius::Float64,
    obstacles::VO,
) where {O<:AbstractObstacle,VO<:AbstractVector{O}}
    return Dubins3DRRTProblem{Float64,VO}(domain, turning_radius, obstacles)
end


# Creates a default domain, with abstract obstacles as an argument
function Dubins3DRRTProblem(obstacles::VO) where {O<:AbstractObstacle,VO<:AbstractVector{O}}
    min_domain = @SVector Float64[0.0, 0.0, 0.0, -1.0*π, deg2rad(-15)]
    max_domain = @SVector Float64[100.0, 100.0, 100.0, 1.0*π, deg2rad(20)]
    turning_radius = 10.0

    return Dubins3DRRTProblem((min_domain, max_domain), turning_radius, obstacles)
end

"""
Returns a random vector lying in the problem domain
"""
function sample_domain(P::Dubins3DRRTProblem)::SVector{5,Float64}
    @debug "Sample Domain"

    v = @SVector rand(5)
    return_vector = SVector{5,Float64}(P.domain[1] + (P.domain[2] - P.domain[1]) .* v)

    if !any(o -> o isa FPVGate, P.obstacles)
        return return_vector
    end

    # If the problem contains a "gate" obstacle, we sample points in the gate occasionally
    gate_obstacles = filter(o -> o isa FPVGate, P.obstacles)

    # TODO Fix this to be parameterizeable
    for gate in gate_obstacles
        # if the point is within the gate's rectangle bounds, then sample a point with the same x, yaw, pitch
        # but a random y and z within the gate's bounds
        if return_vector[1] >= gate.pos[1] - 0.45 && return_vector[1] <= gate.pos[1] + 0.45
            # Sample a random y and z within the gate's bounds
            new_y = rand((gate.pos[2]-0.125):0.01:(gate.pos[2]+0.125))
            new_z = rand((gate.pos[3]-0.125):0.01:(gate.pos[3]+0.125))

            yaw = clamp(return_vector[4], -π / 8, π / 8) + 0.001 * return_vector[4]# clamp to be mostly straight
            return SVector{5,Float64}(
                return_vector[1],
                new_y,
                new_z,
                yaw,
                return_vector[5], # pitch
            )
        end
    end

    return return_vector
end

"""
    sample_free(problem::Dubins3DRRTProblem)

returns a random state within the problem domain
"""
function RRTStar.sample_free(problem::Dubins3DRRTProblem)::SVector{5,Float64}
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
function RRTStar.steer(
    P::Dubins3DRRTProblem,
    x_nearest::SVector{5,Float64},
    x_rand::SVector{5,Float64};
    max_travel_dist = 20.0,
)::SVector{5,Float64}
    @debug "steer"

    pitchlims = @SVector Float64[P.domain[1][5], P.domain[2][5]]

    maneuver =
        safe_dubins_maneuver_3d(x_nearest, x_rand, P.turning_radius, pitchlims, P.domain)

    num_samples = max(10, floor(Int, maneuver.length / 0.1))
    sampled_path = compute_sampling(maneuver; numberOfSamples = num_samples)

    # in the case of a very short path
    if max_travel_dist >= maneuver.length
        return sampled_path[end] # steer_point is an SVector
    end

    # Walk along the path, until distance traveled is greater than max_travel_dist

    cumulative_distance = 0.0
    x_idx = 1
    while cumulative_distance < max_travel_dist && x_idx < length(sampled_path) - 1
        cumulative_distance +=
            norm(sampled_path[x_idx+1][1:3] .- sampled_path[x_idx][1:3], 2)
        x_idx += 1
    end

    return sampled_path[x_idx]
end


function RRTStar.collision_free(
    problem::Dubins3DRRTProblem,
    x_nearest,
    x_new;
    step_size = 0.05,
)::Bool
    @debug "collision_free"

    maneuver = safe_dubins_maneuver_3d(
        x_nearest,
        x_new,
        problem.turning_radius,
        SVector{2,Float64}(problem.domain[1][5], problem.domain[2][5]),
        problem.domain,
    )

    # Check if the maneuver is valid
    if isnothing(maneuver)
        @debug "Maneuver is invalid"
        return false
    end

    # Sampled path contains an array of points along the path
    num_samples = max(50, floor(Int, maneuver.length / step_size))
    sampled_path = compute_sampling(maneuver; numberOfSamples = num_samples)

    in_collision = any(x -> is_colliding(problem.obstacles, x[1:3]), sampled_path)
    path_in_domain = all(x -> in_domain(problem.domain, x), sampled_path)

    @debug "in_collision = $in_collision"
    @debug "path_in_domain = $path_in_domain"
    @debug "Returning collision free: $(!in_collision && path_in_domain)"

    return !in_collision && path_in_domain
end

"""
    path_cost(problem::Dubins3DRRTProblem, x_near, x_new)

returns the length of the dubins path from x_near to x_new
"""
function RRTStar.path_cost(
    problem::Dubins3DRRTProblem,
    x_near::SVector{5,Float64},
    x_new::SVector{5,Float64},
)::Float64
    @debug "path_cost between $x_near and $x_new"

    maneuver = safe_dubins_maneuver_3d(
        x_near,
        x_new,
        problem.turning_radius,
        SVector{2,Float64}(problem.domain[1][5], problem.domain[2][5]),
        problem.domain,
    )

    if isnothing(maneuver)
        return Inf
    end

    cost = maneuver.length

    @debug "cost = $cost"

    return cost
end



