using LinearAlgebra, StaticArrays
using Dubins
using RecipesBase

abstract type CollisionRegion{F} end

"""
    Circle(center, radius)

defines a circluar region by center and radius.
"""
struct Circle{F} <: CollisionRegion{F}
    center::SVector{2,F}
    radius::F
end

"""
    LineSegment(p1, p2)

defines a line segment connecting points `p1` and `p2`. Both `p1` and `p2` must be of type `SVector{2, F}`. 
"""
struct LineSegment{F} <: CollisionRegion{F}
    p1::SVector{2,F}
    p2::SVector{2,F}
end

"""
    minimum_distance(reg1::CollisionRegion, reg2::CollisionRegion)

return the minimum distance between `reg1` and `reg2`
"""
function minimum_distance(reg1::CollisionRegion, reg2::CollisionRegion)
    error("minimum_distance not implemented for $(typeof(reg1)) and $(typeof(reg2))")
end


"""
    is_colliding(reg1::CollisionRegion, reg2::CollisionRegion, tol=0)

return true if the distance between `reg1` and `reg2` is less than `tol`.
"""
function is_colliding(reg1::CollisionRegion, reg2::CollisionRegion, tol = 0)
    return minimum_distance(reg1, reg2) <= tol
end


# Distance between two circles
function minimum_distance(c1::Circle, c2::Circle)
    d = norm(c1.center - c2.center)
    return max(d - (c1.radius + c2.radius), 0.0)
end

# Distance between a circle and a line segment
function minimum_distance(c::Circle, seg::LineSegment)
    closest = closest_point_on_segment(c.center, seg.p1, seg.p2)
    d = norm(c.center - closest) - c.radius
    return max(d, 0.0)
end

# Distance is symmetric
minimum_distance(seg::LineSegment, c::Circle) = minimum_distance(c, seg)

# Distance between two line segments
function minimum_distance(s1::LineSegment, s2::LineSegment)
    return segment_segment_distance(s1.p1, s1.p2, s2.p1, s2.p2)
end

# closest point on segment from a point
function closest_point_on_segment(p, a, b)
    ab = b - a
    t = clamp(dot(p - a, ab) / dot(ab, ab), 0.0, 1.0)
    return a + t * ab
end

# segment-segment distance
function segment_segment_distance(p1, q1, p2, q2)
    # Check all pairwise distances from endpoints to opposite segments
    d1 = point_segment_distance(p1, p2, q2)
    d2 = point_segment_distance(q1, p2, q2)
    d3 = point_segment_distance(p2, p1, q1)
    d4 = point_segment_distance(q2, p1, q1)
    return minimum((d1, d2, d3, d4))
end

# distance from a point to a segment
function point_segment_distance(p, a, b)
    closest = closest_point_on_segment(p, a, b)
    return norm(p - closest)
end


"""
    create_collision_region(path::DubinsPath, segment_id)

creates a collision region for the `segment_id` section of a dubins path, assuming it has a known `segment type`
"""
function create_collision_region(path::DubinsPath, segment_id)

    # get the segment type
    segment_type = Dubins.DIRDATA[dubins_path_type(path)][segment_id]

    # get the state at the start of the segment
    ti = sum(path.params[SOneTo(segment_id - 1)]) * path.ρ
    tf = sum(path.params[SOneTo(segment_id)]) * path.ρ

    # get the initial point on this segment
    errcode, qi = dubins_path_sample(path, ti)
    @assert errcode == Dubins.EDUBOK

    # get the velocity vector
    v = @SVector [cos(qi[3]), sin(qi[3])]

    # depending on the segment type construct a different region
    if segment_type == Dubins.:L_SEG
        n = @SVector [-v[2], v[1]]
        center = qi[SOneTo(2)] + path.ρ * n
        radius = path.ρ
        return Circle(center, radius)
    elseif segment_type == Dubins.:R_SEG
        n = @SVector [v[2], -v[1]]
        center = qi[SOneTo(2)] + path.ρ * n
        radius = path.ρ
        return Circle(center, radius)
    else
        # segment_type == S_SEG
        L = (tf - ti)
        p1 = qi[SOneTo(2)]
        p2 = p1 + v * L
        return LineSegment(p1, p2)
    end
end


"""
    create_collision_region(c <: AbstractWez)

given a wez, create a maximum collision region around it. 
Currently only implemented for Cardioid.
"""
function create_collision_region(c::Cardioid)
    return Circle(SVector(c.x, c.y), c.Rmax)
end

"""
    is_colliding(wez::AbstractWez, path::DubinsPath, tol=1e-5)

returns true if the dubins path gets within `tol` distance of the wez. 
    
Checks by sampling every point `tol` apart along the path. 

To reduce computational cost, it checks bounding boxes first, and then also skips points too close to each other where there is no possibility of collision. 
"""
function is_colliding(wez::W, path::DubinsPath, tol = 1e-5) where {W<:AbstractWez}

    # get the critical times
    t0 = 0.0
    t1 = path.params[1] * path.ρ
    t2 = t1 + path.params[2] * path.ρ
    t3 = t2 + path.params[3] * path.ρ

    # construct the vector of critical times
    ts = SVector(t0, t1, t2, t3)

    # create the wez collision region
    wez_collision_region = create_collision_region(wez)

    # first check if the bounding regions are colliding
    for sub_idx = 1:3

        # create a bounding region for this subpath
        path_collision_region = create_collision_region(path, sub_idx)

        # check collisions with wez (by using the bounding boxes)
        if is_colliding(wez_collision_region, path_collision_region, tol)

            # check by sampling the trajectory
            last_τ = ts[sub_idx]
            last_distance = 0.0

            # loop through and check each point
            for τ in range(ts[sub_idx], ts[sub_idx+1], step = tol)

                # skip anypoints too close to the last sample
                if (τ - last_τ) < last_distance
                    continue
                end

                # sample the point
                errcode, s = dubins_path_sample(path, τ)
                @assert errcode == Dubins.EDUBOK

                if is_colliding(wez, s, tol)
                    return true
                end

                # since we checked a new point update the vars
                last_τ = τ
                last_distance = minimum_distance(wez, SVector(s[1], s[2]))
            end
        end
    end

    return false

end

"""
    is_colliding(wezes::Vector{AbstractWez}, path::DubinsPath, tol=1e-5)

returns true if the dubins path gets within `tol` distance of any wez. See docs for `is_colliding(wez, path, tol)`.
Sorts the wezes by distance before checking each one. 
"""
function is_colliding(
    wezes::VW,
    path::DubinsPath,
    tol = 1e-5,
) where {W<:AbstractWez,VW<:AbstractVector{W}}
    min_dists = [minimum_distance(wez, path.qi[SOneTo(2)]) for wez in wezes]
    idxs = sortperm(min_dists)

    for i in idxs
        if is_colliding(wezes[i], path, tol)
            return true
        end
    end
    return false
end

function is_colliding(
    wezes::VW, 
    paths::VP, 
    tol = 1e-5
) where {W <: AbstractWez, VW<: AbstractVector{W}, P <: DubinsPath, VP <: AbstractVector{P}}
    for p in paths
        if is_colliding(wezes, p, tol)
            return true
        end
    end
    return false
end


#################################################
## Plotting #####################################
#################################################

@recipe function plot_circle(c::Circle)

    N = 50

    x = [c.center[1] + c.radius * cos(t) for t in range(0, 2π, length = N)]
    y = [c.center[2] + c.radius * sin(t) for t in range(0, 2π, length = N)]

    @series begin
        label --> false
        aspect_ratio --> :equal
        x, y
    end
end


@recipe function plot_line(l::LineSegment)

    x = [l.p1[1], l.p2[1]]
    y = [l.p1[2], l.p2[2]]

    @series begin
        label --> false
        aspect_ratio --> :equal
        x, y
    end
end
