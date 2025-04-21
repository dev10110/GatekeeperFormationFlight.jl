# Copyright (c) 2025 Devansh R Agrawal - All rights reserved.

using RecipesBase
using Dubins

@userplot PlotScenario

@recipe function plot_scenario(h::PlotScenario, title=nothing)

    # check inputs
    if length(h.args) != 2 || !(typeof(h.args[1]) <: AbstractVector) || !(eltype(h.args[1]) <: AbstractWez)  || !(typeof(h.args[2]) <: AbstractVector) || !(eltype(h.args[2]) <: Robot)
        error("plot_scenario(wezes, robots) should be given two vectors, one of type abstract wez, and one of type robot.  Got: $(typeof(h.args))")
    end

    # extract the wezes and robots from the plot scenario
    wezes = h.args[1];
    robots = h.args[2];

    # extract the title
    if isnothing(title)
        # get the minimum collision distance
        mind = Inf
        for r in robots, w in wezes
            d = collision_distance(w, r)
            mind = min(mind, d)
        end

        title = "Min Wez Distance: $(round(mind; digits=2))"
        title_color = mind <= 0 ? :red : :black
    else
        title_color = :black
    end

    # plot the bounding box
    @series begin
        label --> false
        color --> :black
        marker --> false
        aspect_ratio --> :equal
        title := title
        plot_titlefontcolor --> title_color
        xlims --> (-0.4, 1.4)
        ylims --> (-0.1, 1.1)
        [0, 1, 1, 0, 0], [0, 0, 1, 1, 0]
    end

    # plot the wezes
    for wez in wezes, robot in robots
        @series begin
            (wez, robot)
        end
    end

    # plot the robot
    for robot in robots
        # color = is_colliding(wezes, robot) ? :red : :green
        # plot!(robot; color = color)
        @series begin
            robot
        end
    end

end

# """
#     plot_scenario(args...; kwargs...)

# runs `plot()` before calling `plot_scenario!(args...; kwargs...)`
# """
# function plot_scenario(args...; kwargs...)
#     plot()
#     plot_scenario!(args...; kwargs...)
# end


# function plot_scenario!(wezes::Vector{W}, robot::Robot; kwargs...) where {W}
#     return plot_scenario!(wezes, [robot]; kwargs...)
# end

# """
#     plot_scenario!(wezes, robots; draw_bbox=true, kwargs...)

# plots the wezes and the robots. if `draw_bbox=true`, it will draw a box starting at (0,0) and ending at (1,1). 
# if the `title` argument is set, it will use that as the plot title. else, the title will be `Min Wez Distance: xx`, and will be red if the min wez distance is negative.

# Both `wezes` and `robots` can either be a single `AbstractWez` or `Robot`, or it could be a vector of wezes or a vector of robots. 
# """
# function plot_scenario!(
#     wezes::Vector{W},
#     robots::Vector{R};
#     draw_bbox = true,
#     title = nothing,
#     kwargs...,
# ) where {W<:AbstractWez,R}
#     plot!(aspect_ratio = :equal)


#     if draw_bbox
#         # plot the bounding box
#         plot!([0, 1, 1, 0, 0], [0, 0, 1, 1, 0], label = false, color = :black)
#         plot!(xlims = (-0.4, 1.4), ylims = (-0.1, 1.1))
#     end

#     # plot the wezes
#     for wez in wezes, robot in robots
#         plot!(wez, robot; color = :gray)
#     end

#     # plot the robot
#     for robot in robots
#         color = is_colliding(wezes, robot) ? :red : :green
#         plot!(robot; color = color)
#     end

#     if isnothing(title)
#         # get the minimum collision distance
#         mind = Inf
#         for r in robots, w in wezes
#             d = collision_distance(w, r)
#             mind = min(mind, d)
#         end
#         plot!(title = "Min Wez Distance: $(round(mind; digits=2))")

#         if mind <= 0
#             plot!(titlefont = font(:red))
#         else
#             plot!(titlefont = font(:black))
#         end
#     else
#         plot!(title = title)
#     end

#     plot!()

# end


@recipe function plot_dubins_path(path::DubinsPath)

    N = 50
    errcode, samples = dubins_path_sample_many(path, dubins_path_length(path) / N)
    @assert errcode == Dubins.EDUBOK

    # push the final state too
    errcode, pt = Dubins.dubins_path_endpoint(path)
    @assert errcode == Dubins.EDUBOK
    push!(samples, pt)

    @series begin
        label --> false
        aspect_ratio --> :equal
        [s[1] for s in samples], [s[2] for s in samples]
    end
end

@recipe function plot_dubins_paths(path::Vector{DubinsPath})

    for p in path
        @series begin
            p
        end
    end
end
