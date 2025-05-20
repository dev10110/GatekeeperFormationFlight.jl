using GatekeeperFormationFlight
using Plots, LinearAlgebra, StaticArrays, Random

using Dubins3D

pyplot()

PITCH_LIMITS = SVector{2,Float64}(deg2rad(-15), deg2rad(20))
TURNING_RADIUS::Float64 = 10.0 # Minimum turning radius for the Dubins vehicle

function create_random_scenario(N_obstacles = 10, env_size = 100, radius_avg = 5)
    obstacles = [
        Cylinder(rand() * env_size, rand() * env_size, (rand() * radius_avg) + radius_avg) for i = 1:N_obstacles
    ]
    return obstacles
end

# Random.seed!(22331234)
Random.seed!(92541234)
# obstacles = create_random_scenario(25)
obstacles = create_random_scenario(25)

domain = (
    SVector(0.0, 0.0, 0.0, -1.0 * π, PITCH_LIMITS[1]),
    SVector(100.0, 100.0, 100.0, 1.0 * π, PITCH_LIMITS[2]),
)

# Initialize the 3D Dubins RRT* path planning problem
prob = Dubins3DRRTProblem(domain, TURNING_RADIUS, obstacles)

# Create the initial RRT node at the start position
start_pose = @SVector [1.0, 1.0, 1.0, 0.0, 0.0]
nodes = [Node(start_pose)]

# Run the RRT* algorithm with 400 iterations
nodes = rrt_star(prob, nodes, 1200)

# Define goal state and find path to it
goal = @SVector [100.0, 100.0, 70.0, 0.0, 0.0]
success, waypoints = get_best_path(prob, nodes, goal)

# Ensure a valid path was found
if !success
    # Plot the obstacles and display them
    p = plot(size = (800, 800), aspect_ratio = :equal, legend = :topright)
    for obs in obstacles
        plot!(PlotCircle(obs), label = false, color = :black, linewidth = 1.0)
    end

    # Plot the start and goal positions
    scatter!(
        [start_pose[1]],
        [start_pose[2]],
        label = "Start",
        color = :green,
        marker = :circle,
        markersize = 5,
    )
    scatter!(
        [goal[1]],
        [goal[2]],
        label = "Goal",
        color = :red,
        marker = :circle,
        markersize = 5,
    )
    display(p)

    @assert success
end

waypoints = reduce(hcat, waypoints)'  # Convert to matrix with points × 6 dimensions
reference_trajectory = Vector{Dubins3D.DubinsManeuver3D}()

pad_straight_start_pose = @SVector [-10.0, 1.0, 1.0, 0.0, 0.0]
pad_straight_end_pose = @SVector [110.0, 100.0, 70.0, 0.0, 0.0]

# Add a straight line segment to the path
start_straight_maneuver = DubinsManeuver3D(
    pad_straight_start_pose,
    start_pose,
    prob.turning_radius,
    SVector{2,Float64}(domain[1][5], domain[2][5]),  # Pitch constraints
)
end_straight_maneuver = DubinsManeuver3D(
    goal,
    pad_straight_end_pose,
    prob.turning_radius,
    SVector{2,Float64}(domain[1][5], domain[2][5]),  # Pitch constraints
)

# Construct the reference trajectory as a seriers of DubinsManeuvers
push!(reference_trajectory, start_straight_maneuver)
for i = 2:size(waypoints)[1]
    local_maneuver = DubinsManeuver3D(
        waypoints[i-1, :],
        waypoints[i, :],
        prob.turning_radius,
        [domain[1][5], domain[2][5]],  # Pitch constraints
    )

    push!(reference_trajectory, local_maneuver)
end
push!(reference_trajectory, end_straight_maneuver)

# offsets = [SVector(0.0, 0.0, 0.0), SVector(-3.0, -5.0, 0.0), SVector(-3.0, 5.0, 0.0)]

# follower_robots = [
#     SVector{5,Float64}(-10.0, 1.0, 1.0, 0.0, 0.0),
#     SVector{5,Float64}(-13.0, -4.0, 0.0, 0.0, 0.0),
#     SVector{5,Float64}(-13.0, 6.0, 0.0, 0.0, 0.0),
# ]
offsets = [SVector(0.0, 0.0, 0.0), SVector(-1.0, -3.0, 0.0), SVector(-1.0, 3.0, 0.0)]

follower_robots = [
    SVector{5,Float64}(-10.0, 1.0, 1.0, 0.0, 0.0),
    SVector{5,Float64}(-11.0, -2.0, 1.0, 0.0, 0.0),
    SVector{5,Float64}(-11.0, 4.0, 1.0, 0.0, 0.0),
]

coeffs = GatekeeperCoefficients(
    switch_step_size = 2.0,
    reconnection_step_size = 4.0,
    max_Ts_horizon = 50.0,
    integration_max_step_size = 0.1,
    integration_step_size = 0.01,
    collision_check_step_size = 0.1,
)

multi_gk_problem = GKDubinsObs3DInterAgent(;
    static_obstacles = obstacles,
    reference_path = reference_trajectory,
    offset = offsets,
    turning_radius = TURNING_RADIUS,
    pitch_limits = PITCH_LIMITS,
    v_min = 0.8,
    v_max = 1.0,
    agent_radius = 1.0,
)

multi_gk_instance = GatekeeperInstance(multi_gk_problem, coeffs)

tspan = [
    0.0,
    GatekeeperFormationFlight.path_length(multi_gk_instance.problem, reference_trajectory),
]

x0 = reduce(hcat, follower_robots)'
solution = simulate_closed_loop_gatekeeper(multi_gk_instance, x0, tspan)

########################################
# PLOTTING 3D SCENARIO
########################################

# Clear plot
p1 = plot(size = (800, 800), aspect_ratio = :equal, legend = :topright)
title!("3D Dubins Vehicle Trajectory")
xlabel!("X")
ylabel!("Y")
zlabel!("Z")

for obs in obstacles
    plot!(obs)
end

## PLOT NOMINAL TRAJECTORIES
using GatekeeperFormationFlight.Gatekeeper: get_reference_state_and_input
using GatekeeperFormationFlight.MultiGatekeeper: get_single_agent_subproblem
for agent in eachindex(multi_gk_instance.problem.offset)
    subproblem = get_single_agent_subproblem(multi_gk_instance.problem, agent)
    plot!(
        τ -> get_reference_state_and_input(
            subproblem,
            subproblem.reference_path,
            subproblem.offset,
            τ,
        )[1][1],
        τ -> get_reference_state_and_input(
            subproblem,
            subproblem.reference_path,
            subproblem.offset,
            τ,
        )[1][2],
        τ -> get_reference_state_and_input(
            subproblem,
            subproblem.reference_path,
            subproblem.offset,
            τ,
        )[1][3],
        tspan[1],
        tspan[2],
        linestyle = :dash,
        label = false,
        linecolor = :blue,
        opacity = 0.5,
    )
end

## PLOT TAKEN TRAJECTORIES
ts = solution.t
states = [solution(t) for t in ts]

# Convert to arrays for each agent
agent1_xyz = hcat([s[1, 1:3] for s in states]...)  # 3 x N
agent2_xyz = hcat([s[2, 1:3] for s in states]...)
agent3_xyz = hcat([s[3, 1:3] for s in states]...)

plot!(agent1_xyz[1, :], agent1_xyz[2, :], agent1_xyz[3, :], label = "Agent 1")
plot!(agent2_xyz[1, :], agent2_xyz[2, :], agent2_xyz[3, :], label = "Agent 2")
plot!(agent3_xyz[1, :], agent3_xyz[2, :], agent3_xyz[3, :], label = "Agent 3")

########################################
# PLOTTING 2D SCENARIO
########################################

p2 = plot(size = (800, 800), aspect_ratio = :equal, legend = :topright)
title!("2D Dubins Vehicle Trajectory")
xlabel!("X")
ylabel!("Y")

for obs in obstacles
    plot!(PlotCircle(obs), label = false, color = :black, linewidth = 1.0)
end

## PLOT NOMINAL TRAJECTORIES
using GatekeeperFormationFlight.Gatekeeper: get_reference_state_and_input
using GatekeeperFormationFlight.MultiGatekeeper: get_single_agent_subproblem
for agent in eachindex(multi_gk_instance.problem.offset)
    subproblem = get_single_agent_subproblem(multi_gk_instance.problem, agent)
    plot!(
        τ -> get_reference_state_and_input(
            subproblem,
            subproblem.reference_path,
            subproblem.offset,
            τ,
        )[1][1],
        τ -> get_reference_state_and_input(
            subproblem,
            subproblem.reference_path,
            subproblem.offset,
            τ,
        )[1][2],
        tspan[1],
        tspan[2],
        linestyle = :dash,
        label = false,
        linecolor = :blue,
        opacity = 0.5,
    )
end

## PLOT TAKEN TRAJECTORIES
ts = solution.t
states = [solution(t) for t in ts]

# Convert to arrays for each agent
agent1_xyz = hcat([s[1, 1:3] for s in states]...)  # 3 x N
agent2_xyz = hcat([s[2, 1:3] for s in states]...)
agent3_xyz = hcat([s[3, 1:3] for s in states]...)

plot!(agent1_xyz[1, :], agent1_xyz[2, :], label = "Agent 1")
plot!(agent2_xyz[1, :], agent2_xyz[2, :], label = "Agent 2")
plot!(agent3_xyz[1, :], agent3_xyz[2, :], label = "Agent 3")


########################################
# PLOT MINIMUM INTERAGENT DISTANCE
########################################
function min_interagent_distance(t)
    s = solution(t)

    distances = [
        norm(s[1, 1:3] - s[2, 1:3]),
        norm(s[1, 1:3] - s[3, 1:3]),
        norm(s[2, 1:3] - s[3, 1:3]),
    ]

    return minimum(distances)
end

p3 = plot(
    solution.t,
    min_interagent_distance.(solution.t),
    label = "Min Interagent Distance",
    xlabel = "Time",
    ylabel = "Distance",
    title = "Minimum Interagent Distance",
    size = (800, 400),
)

# plot horizontal line showing the minimum allowed distance
plot!(
    solution.t,
    [multi_gk_instance.problem.agent_radius * 2 for i = 1:length(solution.t)],
    label = "Min Allowed Distance",
    linestyle = :dash,
    color = :red,
)


lay = @layout [a{0.5w} [b{0.5h}; c{0.5h}]]
p = plot(p1, p2, p3, layout = lay, size = (1600, 800))
display(p)

savefig(p, "3d_gatekeeper_interagent.png")


# ## ANIMATE

@assert false # comment this line to run the animation

@show "Starting Animation"

sim_length = 100

modified_range = range(tspan[1], tspan[2], length = sim_length)
modified_range = vcat(modified_range, [tspan[2] for i = 1:floor(Int, sim_length * 0.15)])

anim = @animate for (idx, t) in enumerate(modified_range)
    println("Animation Time: $t")

    state_matrix = solution(t)

    # grab the states
    robots_ = [Robot3(r) for r in eachrow(state_matrix)]

    # start the plot
    plot()

    # plot the reference paths
    for agent in eachindex(robots_)
        subproblem = get_single_agent_subproblem(multi_gk_instance.problem, agent)
        plot!(
            τ -> get_reference_state_and_input(
                subproblem,
                subproblem.reference_path,
                subproblem.offset,
                τ,
            )[1][1],
            τ -> get_reference_state_and_input(
                subproblem,
                subproblem.reference_path,
                subproblem.offset,
                τ,
            )[1][2],
            τ -> get_reference_state_and_input(
                subproblem,
                subproblem.reference_path,
                subproblem.offset,
                τ,
            )[1][3],
            tspan[1],
            tspan[2],
            linestyle = :dash,
            label = false,
            linecolor = :blue,
            opacity = 0.5,
        )
    end

    # plot the trace of the robots upto this point in time
    for i = 1:length(robots_)
        plot!(
            τ -> solution(τ)[i, 1],
            τ -> solution(τ)[i, 2],
            τ -> solution(τ)[i, 3],
            tspan[1],
            tspan[2],
            label = "gk_sol_$(i)",
            color = (i == 1 ? :black : :green),
            linewidth = 2,
        )
    end

    # Plot the obstacles
    for obs in obstacles
        plot!(obs)
    end

    # plot the robots
    for rob in robots_
        plot!(rob)
    end

    plot!(legend = false, camera = (Int(floor((idx / length(modified_range)) * 70)), 30))
end

gif(anim, "3d_interagent_gatekeeper.gif")