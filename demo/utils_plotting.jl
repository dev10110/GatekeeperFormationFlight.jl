using Plots

# small function to help plot things
function plot_scenario!(
    wezes::VW,
    robots::VR;
    draw_bbox = true,
    title = nothing,
    kwargs...,
) where {W<:GFF.AbstractWez,VW<:AbstractVector{W},R<:Robot,VR<:AbstractVector{R}}

    if draw_bbox
        # plot the bounding box
        plot!([0, 1, 1, 0, 0], [0, 0, 1, 1, 0], label = false, color = :black)
        plot!(xlims = (-0.4, 1.4), ylims = (-0.4, 1.4))
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

    plot!(aspect_ratio = :equal) 

end