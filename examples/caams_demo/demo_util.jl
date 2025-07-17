module DemoUtil

using GatekeeperFormationFlight
using StaticArrays

mutable struct EnvironmentConfig
    name::String
    obstacles::Vector{AbstractObstacle}
    agents::Vector{SVector{3, Float64}}
    offsets::Vector{SVector{3, Float64}}
    goal::SVector{3, Float64}
    domain::Tuple{SVector{5, Float64}, SVector{5, Float64}}
    pitch_limits::SVector{2, Float64}
    turn_radius::Float64
    v_min::Float64
    v_max::Float64
    agent_radius::Float64
    rrt_iterations::Int64
    coeffs::GatekeeperCoefficients
    x_padding::Float64
    leader_path
    solution
    composites
end

function load_env(env_file)::EnvironmentConfig
    """
    Loads in the environment from an environment description file
    ====
    Returns:
        env_id
        obstacles
        agents (including starting positions)
        goal positions
        environment dimensions
    """
    return
end

function solve_leader_path!(env::EnvironmentConfig)::Bool
    """
    Runs RRT* on the constructed environment. Stores the result in the variable leader_path. Returns true if successful.
    """

    # Construct the rrt problem instance
    rrt_problem = Dubins3DRRTProblem(
        env.domain, env.turn_radius, env.obstacles
    )

    # Padding on the start and end states
    x0 = @SVector [env.agents[1][1] + env.x_padding, env.agents[1][2], env.agents[1][3]]
    xg = @SVector [env.goal[1] - env.x_padding, env.goal[2], env.goals[1][3]]
    success, waypoints = solve_3d_rrt(x0, xg, rrt_problem; rrt_iterations=env.rrt_iterations)

    if !success
        return false
    end

    leader_path = make_reference_trajectory(waypoints, env.turn_radius, env.pitch_limits, agents[1], goals[1])

    # Set the leader path in the environment object
    env.leader_path = leader_path

    return true # Success
end


function solve_gk_problem!(env::EnvironmentConfig)::Bool
    multi_gk_problem = GKDubonsObs3DInterAgent(;
        static_obstacles = env.obstacles,
        reference_path = env.leader_path,
        offsets = env.offsets,
        turning_radius = env.turn_radius,
        pitch_limits = env.pitch_limits,
        v_min = env.v_min,
        v_max = env.v_max,
        agent_radius = env.agent_radius 
    )

    multi_gk_instance = GatekeeperInstance(multi_gk_problem, env.coeffs)

    initial_positions = []
    tspan = [0., sum(x->x.length, env.leader_path)]

    solution, composites = simulate_closed_loop_gatekeeper(multi_gk_instance, initial_positions, tspan);

    if (!solution)
        return false
    end

    env.solution = solution
    env.composites = composites

    return true
end

end