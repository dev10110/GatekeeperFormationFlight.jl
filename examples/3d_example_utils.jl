module ExampleUtils3D

using StaticArrays, LinearAlgebra, Plots, Combinatorics
using GatekeeperFormationFlight, Dubins3D
using Statistics


GK = GatekeeperFormationFlight.Gatekeeper
MGK = GatekeeperFormationFlight.MultiGatekeeper

function create_random_scenario(N_obstacles = 10, env_size = 100, radius_avg = 5)
    cylinders = [
        Cylinder(rand() * env_size, rand() * env_size, (rand() * radius_avg) + radius_avg) for i = 1:N_obstacles
    ]

    # Add some hemispherical obstacles
    hemispheres = [
        GroundedHalfDome(
            rand() * env_size,
            rand() * env_size,
            (rand() * radius_avg * 5) + radius_avg,
        ) for i = 1:5
    ]

    spheres = [
        Sphere(
            rand() * env_size,
            rand() * env_size,
            rand() * env_size,
            (rand() * radius_avg * 5) + radius_avg,
        ) for i = 1:5
    ]

    obstacles = vcat(cylinders, hemispheres, spheres)

    return obstacles
end

function solve_3d_rrt(x0, xgoal, rrt_problem; rrt_iterations = 100)
    nodes = [Node(x0)]
    nodes = rrt_star(rrt_problem, nodes, rrt_iterations)

    success, waypoints = get_best_path(rrt_problem, nodes, xgoal)
    return success, waypoints
end

function make_reference_trajectory(
    waypoints,
    TURNING_RADIUS::Float64,
    PITCH_LIMITS,
    x0,
    xg;
    X_PADDING = 10.0,
)::Vector{DubinsManeuver3D}
    padded_start = [@SVector [x0[1] - X_PADDING, x0[2], x0[3], 0.0, 0.0]]
    padded_goal = [@SVector [xg[1] + X_PADDING, xg[2], xg[3], 0.0, 0.0]]

    # padded_waypoints = hcat(padded_start, waypoints, padded_goal)
    padded_waypoints = [padded_start; waypoints; padded_goal]

    reference_trajectory = []

    for i = 1:(length(padded_waypoints)-1)
        x_curr = padded_waypoints[i]
        x_next = padded_waypoints[i+1]

        # Create a reference trajectory between the two waypoints
        maneuver = DubinsManeuver3D(x_curr, x_next, TURNING_RADIUS, PITCH_LIMITS)

        # Append the trajectory to the reference trajectory
        push!(reference_trajectory, maneuver)
    end

    return reference_trajectory
end

function visualize_baseline(obstacles, reference_trajectory)
    p = plot(aspect_ratio = :equal, size = (600, 600), legend = false)

    # Display the obstacles
    for obs in obstacles
        plot!(obs)
    end

    maneuver_lengths = [maneuver.length for maneuver in reference_trajectory]
    cumulative_lengths = cumsum(maneuver_lengths)

    total_length = sum(maneuver_lengths)

    function plot_at_t(t)
        # Find the maneuver that corresponds to the time t
        idx = findfirst(x -> x >= t, cumulative_lengths)
        if isnothing(idx)
            return
        end

        # Get the maneuver and its parameters
        maneuver = reference_trajectory[idx]
        t_maneuver = t - (idx == 1 ? 0 : cumulative_lengths[idx-1])
        t_maneuver = clamp(t_maneuver, 0.0, maneuver.length)

        return Dubins3D.compute_at_len(maneuver, t_maneuver)
    end

    plot!(
        τ -> plot_at_t(τ)[1],
        τ -> plot_at_t(τ)[2],
        τ -> plot_at_t(τ)[3],
        0.0,
        total_length,
        label = "Trajectory",
        color = :red,
        linewidth = 2,
        opacity = 0.5,
    )

    return p
end

function visualize_baseline_2d(
    obstacles,
    reference_trajectory;
    dims_max = nothing,
    dims_min = nothing,
)
    p = plot(aspect_ratio = :equal, size = (600, 600), legend = false)

    # Display the obstacles
    for obs in obstacles
        repr = get_2d_repr(obs)
        plot!(repr, label = false, linedwidth = 2)
    end

    maneuver_lengths = [maneuver.length for maneuver in reference_trajectory]
    cumulative_lengths = cumsum(maneuver_lengths)

    total_length = sum(maneuver_lengths)

    function plot_at_t(t)
        idx = findfirst(x -> x >= t, cumulative_lengths)
        if isnothing(idx)
            return [0.0, 0.0]
        end

        maneuver = reference_trajectory[idx]
        t_maneuver = t - (idx == 1 ? 0 : cumulative_lengths[idx-1])
        t_maneuver = clamp(t_maneuver, 0.0, maneuver.length)

        # Only take x, y for 2D
        pos = Dubins3D.compute_at_len(maneuver, t_maneuver)
        return pos[1:2]
    end

    if dims_min !== nothing && dims_max !== nothing
        # Set the limits based on the provided dimensions
        xlims!(p, dims_min[1], dims_max[1])
        ylims!(p, dims_min[2], dims_max[2])
    end
    plot!(
        τ -> plot_at_t(τ)[1],
        τ -> plot_at_t(τ)[2],
        0.0,
        total_length,
        label = "Trajectory",
        color = :red,
        linewidth = 2,
        opacity = 0.5,
    )

    return p
end

function plot_interagent_solution(solution, gk; dims_min = nothing, dims_max = nothing)
    # Assign a color for each agent
    n_agents = length(gk.problem.offset)

    pal = palette(:tab10)  # or :Set1, :Dark2, etc.
    agent_colors = [pal[mod1(i, length(pal))] for i = 1:n_agents]

    # Pass colors to plotting functions
    p1 = plot_3d_scenario(solution, gk; agent_colors = agent_colors, add_legend = true)

    if dims_min !== nothing && dims_max !== nothing
        # Set the limits based on the provided dimensions
        xlims!(p1, dims_min[1], dims_max[1])
        ylims!(p1, dims_min[2], dims_max[2])
        zlims!(p1, dims_min[3], dims_max[3])
    end

    p2 = plot_2d_projection(solution, gk; agent_colors = agent_colors)
    p3 = plot_min_interagent_distance(solution, gk)
    p4 = plot_deviation(solution, gk; agent_colors = agent_colors)

    lay = @layout [a b [c; d]]
    p = plot(p1, p2, p3, p4, layout = lay, size = (1800, 600))
    return p
end

function plot_3d_scenario(solution, gk; agent_colors = nothing, add_legend = false)
    p1 = plot(
        size = (600, 600),
        aspect_ratio = :equal,
        legend = add_legend ? :bottomleft : false,
    )
    title!("3D Dubins Vehicle Trajectory")
    xlabel!("X")
    ylabel!("Y")
    zlabel!("Z")

    n_agents = length(gk.problem.offset)
    colors = isnothing(agent_colors) ? distinguishable_colors(n_agents) : agent_colors

    # Plot the nominal trajectories
    for agent in eachindex(gk.problem.offset)
        subprob = MGK.get_single_agent_subproblem(gk.problem, agent)
        plot!(
            τ -> GK.get_reference_state_and_input(
                subprob,
                subprob.reference_path,
                subprob.offset,
                τ,
            )[1][1],
            τ -> GK.get_reference_state_and_input(
                subprob,
                subprob.reference_path,
                subprob.offset,
                τ,
            )[1][2],
            τ -> GK.get_reference_state_and_input(
                subprob,
                subprob.reference_path,
                subprob.offset,
                τ,
            )[1][3],
            0.0,
            solution.t[end],
            linestyle = :dash,
            label = false,
            linecolor = colors[agent],
            opacity = 0.5,
        )
    end

    # Plot the taken trajectories
    for agent in eachindex(gk.problem.offset)
        plot!(
            τ -> solution(τ)[agent, 1],
            τ -> solution(τ)[agent, 2],
            τ -> solution(τ)[agent, 3],
            0.0,
            solution.t[end],
            label = "Agent $agent",
            linewidth = 2,
            linecolor = colors[agent],
        )
    end

    # plot obstacles last so they occlude the trajectories
    for obs in GK.get_obstacles(gk.problem)
        plot!(obs)
    end

    return p1
end

function plot_2d_projection(solution, gk; agent_colors = nothing, add_legend = false)
    p2 = plot(
        size = (600, 600),
        aspect_ratio = :equal,
        legend = add_legend ? :bottomleft : false,
    )
    title!("2D Projection of the Trajectory")
    xlabel!("X")
    ylabel!("Y")

    for obs in GK.get_obstacles(gk.problem)
        repr = get_2d_repr(obs)
        plot!(repr, label = false, color = :black, linewidth = 2)
    end

    n_agents = length(gk.problem.offset)
    colors = isnothing(agent_colors) ? distinguishable_colors(n_agents) : agent_colors

    for agent in eachindex(gk.problem.offset)
        subprob = MGK.get_single_agent_subproblem(gk.problem, agent)
        plot!(
            τ -> GK.get_reference_state_and_input(
                subprob,
                subprob.reference_path,
                subprob.offset,
                τ,
            )[1][1],
            τ -> GK.get_reference_state_and_input(
                subprob,
                subprob.reference_path,
                subprob.offset,
                τ,
            )[1][2],
            0.0,
            solution.t[end],
            linestyle = :dash,
            label = false,
            linecolor = colors[agent],
            opacity = 0.5,
        )
    end

    # Plot the taken trajectories
    for agent in eachindex(gk.problem.offset)
        plot!(
            τ -> solution(τ)[agent, 1],
            τ -> solution(τ)[agent, 2],
            0.0,
            solution.t[end],
            label = "Agent $agent",
            linewidth = 2,
            linecolor = colors[agent],
        )
    end

    return p2
end

function plot_min_interagent_distance(solution, gk)
    pairs = combinations(collect(1:length(gk.problem.offset)), 2)
    function min_interagent_distance(t)
        s = solution(t)
        distances = [norm(s[i, 1:3] - s[j, 1:3]) for (i, j) in pairs]
        return minimum(distances)
    end

    p = plot(
        solution.t,
        min_interagent_distance.(solution.t),
        xlabel = "Time (t)",
        ylabel = "Distance",
        title = "Minimum Interagent Distance",
        label = "Distance",
        size = (600, 300),
    )
    ylims!(0, Inf)

    plot!(
        solution.t,
        [gk.problem.agent_radius * 2 for i = 1:length(solution.t)],
        label = "Collision Threshold",
        linestyle = :dash,
        color = :red,
    )

    return p
end

function plot_deviation(solution, gk; agent_colors = nothing, add_legend = false)
    p4 = plot(size = (600, 300), legend = add_legend ? :bottomleft : false)
    title!("Deviation from Reference Trajectory")
    xlabel!("Time (t)")
    ylabel!("Deviation")
    ylims!(0, Inf)

    n_agents = length(gk.problem.offset)
    colors = isnothing(agent_colors) ? distinguishable_colors(n_agents) : agent_colors

    for agent in eachindex(gk.problem.offset)
        subprob = MGK.get_single_agent_subproblem(gk.problem, agent)
        plot!(
            solution.t,
            [
                norm(
                    solution(t)[agent, 1:3] - GK.get_reference_state_and_input(
                        subprob,
                        subprob.reference_path,
                        subprob.offset,
                        t,
                    )[1][1:3],
                ) for t in solution.t
            ],
            label = "Deviation Agent $agent",
            linewidth = 2,
            linecolor = colors[agent],
        )
    end

    return p4
end

function plot_3d_at_time(solution, gk, end_time; agent_colors, add_legend = nothing)
    p1 = plot(
        size = (600, 600),
        aspect_ratio = :equal,
        legend = isnothing(add_legend) ? :bottomleft : false,
        title = "(t = $end_time)",
        titlelocation = :left,
        titlefontsize = 10,
    )
    xlabel!("X")
    ylabel!("Y")
    zlabel!("Z")

    n_agents = length(gk.problem.offset)
    colors = isnothing(agent_colors) ? distinguishable_colors(n_agents) : agent_colors

    # Plot the nominal trajectories
    for agent in eachindex(gk.problem.offset)
        subprob = MGK.get_single_agent_subproblem(gk.problem, agent)
        plot!(
            τ -> GK.get_reference_state_and_input(
                subprob,
                subprob.reference_path,
                subprob.offset,
                τ,
            )[1][1],
            τ -> GK.get_reference_state_and_input(
                subprob,
                subprob.reference_path,
                subprob.offset,
                τ,
            )[1][2],
            τ -> GK.get_reference_state_and_input(
                subprob,
                subprob.reference_path,
                subprob.offset,
                τ,
            )[1][3],
            0.0,
            solution.t[end],
            linestyle = :dash,
            label = false,
            linecolor = colors[agent],
            opacity = 0.5,
        )
    end

    # Plot the taken trajectories
    for agent in eachindex(gk.problem.offset)
        plot!(
            τ -> solution(τ)[agent, 1],
            τ -> solution(τ)[agent, 2],
            τ -> solution(τ)[agent, 3],
            0.0,
            end_time,
            label = "Agent $agent",
            linewidth = 2,
            linecolor = colors[agent],
        )
    end

    # Plot each agent at the end_time as an arrow
    for agent in eachindex(gk.problem.offset)
        pos = solution(end_time)[agent, :]
        # position = SVector{3,Float64}(pos[1:3])
        # rotation = @SVector [0.0, pos[4], pos[5]]
        # rob = Robot3(SVector{6,Float64}(pos..., 0.0))
        # rob = Robot3(position, rotation)
        rob = Robot3(pos)
        plot!(rob)
    end

    # plot obstacles last so they occlude the trajectories
    for obs in GK.get_obstacles(gk.problem)
        plot!(obs, opacity = 0.15)
    end

    return p1
end

function calculate_interagent_statistics(solution, gk)
    agent_errors = []
    for agent in eachindex(gk.problem.offset)
        subprob = MGK.get_single_agent_subproblem(gk.problem, agent)
        deviation = [
            norm(
                solution(t)[agent, 1:3] - GK.get_reference_state_and_input(
                    subprob,
                    subprob.reference_path,
                    subprob.offset,
                    t,
                )[1][1:3],
            ) for t in solution.t
        ]

        mean_error = mean(deviation)
        push!(agent_errors, mean_error)
    end

    agent_error = mean(agent_errors)


    pairs = combinations(collect(1:length(gk.problem.offset)), 2)
    function min_interagent_distance(t)
        s = solution(t)
        distances = [norm(s[i, 1:3] - s[j, 1:3]) for (i, j) in pairs]
        return minimum(distances)
    end

    interagent_dist = mean(min_interagent_distance.(solution.t))

    return true, agent_error, interagent_dist
end

function calculate_statistics(solution_vector, gk_vector)
    agent_errors = []

    for (solution, gk) in zip(solution_vector, gk_vector)
        deviation = [
            norm(
                solution(t)[1:3] - GK.get_reference_state_and_input(
                    gk.problem,
                    GK.get_reference_path(gk.problem),
                    GK.get_offset(gk.problem),
                    t,
                )[1][1:3],
            ) for t in solution.t
        ]

        mean_error = mean(deviation)
        push!(agent_errors, mean_error)
    end

    return true, mean(agent_errors)
end

function animate_interagent(solution, gk)

end

end # module ExampleUtils3D