# Copyright (c) 2025 Devansh R Agrawal - All rights reserved.

using StaticArrays, LinearAlgebra
using RecipesBase

"""
    Robot(x, y, ψ)
    Robot([x, y, ψ])

Create a robot with some x, y position and heading ψ.
"""
struct Robot{F}
    x::F
    y::F
    ψ::F
end

function Robot(v::VF) where {F,VF<:AbstractVector{F}}
    @assert length(v) == 3
    return Robot(v...)
end

"""
    SVector(r::Robot)

returns a SVector{3, F} containing [x, y, ψ]
"""
function StaticArrays.SVector(r::Robot{F}) where {F}
    return SVector{3,F}(r.x, r.y, r.ψ)
end


#############################################################
### Plotting ################################################
#############################################################

@recipe function plot_robot!(r::Robot)

    # plot the origin marker
    @series begin
        seriestype := :scatter
        label --> false
        color --> :black
        marker --> :dot
        aspect_ratio --> :equal
        [r.x], [r.y]
    end

    # plot the arrow
    @series begin
        seriestype := :path
        label --> false
        color --> :black
        marker --> false
        arrow --> true
        aspect_ratio --> :equal
        [r.x, r.x + 0.1 * cos(r.ψ)], [r.y, r.y + 0.1 * sin(r.ψ)]
    end

end
