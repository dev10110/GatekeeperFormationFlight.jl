module DemoUtil

using GatekeeperFormationFlight
using StaticArrays
using YAML
using DataFrames
using CSV
using Polynomials

using ..FileUtil
using ..ExampleUtils3D

ASO = GatekeeperFormationFlight.Obstacles.AbstractStaticObstacle

@kwdef struct AgentSettings
    v_min::Float64 = 0.8
    v_max::Float64 = 1.0
    x_padding::Float64 = 0.1
    agent_radius::Float64 = 0.125
    turn_radius::Float64 = 0.5
    pitch_limits::SVector{2,Float64} = SVector(-pi / 4, pi / 4)
end

@kwdef struct SimAgent
    id::Int64 = 1
    offset::SVector{5,Float64} = SVector(0.0, 0.0, 0.0, 0.0, 0.0)
end

@kwdef struct SimScenario
    name::String = "default_scenario"
    start_pose::SVector{5,Float64} = SVector(0.0, 0.0, 0.0, 0.0, 0.0)
    goal_pose::SVector{5,Float64} = SVector(0.0, 0.0, 0.0, 0.0, 0.0)
    domain_min::SVector{3,Float64} = SVector(-1.0, -1.0, 0.0)
    domain_max::SVector{3,Float64} = SVector(1.0, 1.0, 2.0)
end
@kwdef mutable struct SimEnvironment
    scenario::SimScenario = SimScenario()
    agent_settings::AgentSettings = AgentSettings()
    agents::Vector{SimAgent} = Vector{SimAgent}()
    obstacles::Vector{ASO} = Vector{ASO}()
    gatekeeper_coefficients::GatekeeperCoefficients = GatekeeperCoefficients()
    leader_path = nothing
    solution = nothing
    composites = nothing
    gk = nothing # Gatekeeper instance
    data = nothing
end

function load_env(env_file)::SimEnvironment
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
    # Read in the parameter file
    obj = YAML.load_file(env_file; dicttype = Dict{Symbol,Any})

    # Convert the loaded dictionary to the appropriate types
    obj[:gatekeeper_coefficients] =
        GatekeeperCoefficients(; obj[:gatekeeper_coefficients]...)
    obj[:obstacles] = [FileUtil.dict_to_obstacle(d) for d in obj[:obstacles]]
    obj[:agent_settings] = AgentSettings(; obj[:agent_settings]...)
    obj[:agents] = [SimAgent(; d...) for d in obj[:agents]]
    obj[:scenario] = SimScenario(; obj[:scenario]...)

    # Create the SimEnvironment object
    sim_env = SimEnvironment(; obj...)

    return sim_env
end

function solve_leader_path!(env::SimEnvironment)::Bool
    """
    Runs RRT* on the constructed environment. Stores the result in the variable leader_path. Returns true if successful.
    """

    domain_min = SVector{5,Float64}(
        env.scenario.domain_min[1],
        env.scenario.domain_min[2],
        env.scenario.domain_min[3],
        -π,
        env.agent_settings.pitch_limits[1],
    )
    domain_max = SVector{5,Float64}(
        env.scenario.domain_max[1],
        env.scenario.domain_max[2],
        env.scenario.domain_max[3],
        π,
        env.agent_settings.pitch_limits[2],
    )

    rrt_domain = (domain_min, domain_max)

    # Construct the rrt problem instance
    rrt_problem =
        Dubins3DRRTProblem(rrt_domain, env.agent_settings.turn_radius, env.obstacles)

    # Padding on the start and end states -- pre-apply the x-padding
    x0 = @SVector [
        env.scenario.start_pose[1] + env.agent_settings.x_padding,
        env.scenario.start_pose[2],
        env.scenario.start_pose[3],
        env.scenario.start_pose[4],
        env.scenario.start_pose[5],
    ]

    xg = @SVector [
        env.scenario.goal_pose[1] - env.agent_settings.x_padding,
        env.scenario.goal_pose[2],
        env.scenario.goal_pose[3],
        env.scenario.goal_pose[4],
        env.scenario.goal_pose[5],
    ]

    success, waypoints =
        ExampleUtils3D.solve_3d_rrt(x0, xg, rrt_problem; rrt_iterations = 1200)

    if !success
        return false
    end

    leader_path = ExampleUtils3D.make_reference_trajectory(
        waypoints,
        env.agent_settings.turn_radius,
        env.agent_settings.pitch_limits,
        env.scenario.start_pose,
        env.scenario.goal_pose;
        X_PADDING = env.agent_settings.x_padding,
    )

    # Set the leader path in the environment object
    env.leader_path = leader_path

    return true # Success
end


function solve_gk_problem!(env::SimEnvironment)::Bool
    # put the agents in order by id
    sort!(env.agents, by = x -> x.id)
    offsets = [SVector{3,Float64}(agent.offset[1:3]...) for agent in env.agents]

    multi_gk_problem = GKDubinsObs3DInterAgent(;
        static_obstacles = env.obstacles,
        reference_path = env.leader_path,
        offset = offsets,
        turning_radius = env.agent_settings.turn_radius,
        pitch_limits = env.agent_settings.pitch_limits,
        v_min = env.agent_settings.v_min,
        v_max = env.agent_settings.v_max,
        agent_radius = env.agent_settings.agent_radius,
    )

    multi_gk_instance = GatekeeperInstance(multi_gk_problem, env.gatekeeper_coefficients)

    initial_positions = [
        env.scenario.start_pose + SVector{5,Float64}(offset..., 0.0, 0.0) for
        offset in offsets
    ]
    initial_positions = reduce(hcat, initial_positions)'
    tspan = [0.0, sum(x -> x.length, env.leader_path)]

    solution, composites =
        simulate_closed_loop_gatekeeper(multi_gk_instance, initial_positions, tspan)

    if (isnothing(solution))
        return false
    end

    env.solution = solution
    env.gk = multi_gk_instance
    env.composites = composites

    return true
end

function write_to_file(
    env::SimEnvironment,
    file_name::String,
    t_resolution::Float64 = 0.005,
)
    """
    Writes the results of the simulation to a file using a DataFrame.
    """
    data = Vector{Dict{Symbol,Any}}()

    t_start = env.solution.t[1]
    t_end = sum(x -> x.length, env.leader_path)
    # t_end = env.solution.t[end] # Use this if you want to use the full sim that may exceed
    # desired runtime

    t_length = t_end - t_start

    for t = t_start:t_resolution:t_end
        pos_at_time = env.solution(t)
        for i = 1:length(env.agents)
            push!(
                data,
                Dict(
                    :agent_id => env.agents[i].id,
                    :time => (t - t_start) / t_length, # normalize time to [0, 1]
                    :pos_x => pos_at_time[i, 1],
                    :pos_y => pos_at_time[i, 2],
                    :pos_z => pos_at_time[i, 3],
                    :heading => pos_at_time[i, 4],
                ),
            )
        end
    end

    df = DataFrame(data)
    CSV.write(file_name, df)

    env.data = df
end

function fit_polynomials(
    data::DataFrame;
    chunk_size::Int = 40,
    poly_degree::Int = 7,
    trajectory_time::Float64 = 30.0,
)
    agents = unique(data.agent_id)

    chunk_polys = []

    for agent in agents
        agent_data = filter(row -> row.agent_id == agent, eachrow(data))
        t_scaled = [row.time * trajectory_time for row in agent_data]

        for i = 1:(chunk_size - 5):length(agent_data)
            idx_end = min(i + chunk_size - 1 + 5, length(agent_data))
            t_chunk = t_scaled[i:idx_end]
            t_chunk = t_chunk .- t_chunk[1]  # Normalize to start at 0

            x = [agent_data[j].pos_x for j = i:idx_end]
            y = [agent_data[j].pos_y for j = i:idx_end]
            z = [agent_data[j].pos_z for j = i:idx_end]
            yaw = [agent_data[j].heading for j = i:idx_end]

            px = fit(t_chunk, x, poly_degree)
            py = fit(t_chunk, y, poly_degree)
            pz = fit(t_chunk, z, poly_degree)
            pyaw = fit(t_chunk, yaw, poly_degree)

            # println("Px: $px, Py: $py, Pz: $pz, Pyaw: $pyaw")

            names = [:agent_id, :duration]
            values = [agent, t_chunk[end]]

            for ind = 0:poly_degree
                coeff = ind <= degree(px) ? coeffs(px)[ind+1] : 0.0
                push!(names, Symbol("x^$ind"))
                push!(values, coeff)
            end
            for ind = 0:poly_degree
                coeff = ind <= degree(py) ? coeffs(py)[ind+1] : 0.0
                push!(names, Symbol("y^$ind"))
                push!(values, coeff)
            end
            for ind = 0:poly_degree
                coeff = ind <= degree(pz) ? coeffs(pz)[ind+1] : 0.0
                push!(names, Symbol("z^$ind"))
                push!(values, coeff)
            end
            for ind = 0:poly_degree
                coeff = ind <= degree(pyaw) ? coeffs(pyaw)[ind+1] : 0.0
                push!(names, Symbol("yaw^$ind"))
                push!(values, coeff)
            end

            res = (; zip(names, values)...)
            push!(chunk_polys, res)
        end

    end

    return chunk_polys
end

function write_polynomials_to_file(chunk_polys, file_name::String)
    """
    Writes the polynomial coefficients to a CSV file.
    """
    df = DataFrame(chunk_polys)
    CSV.write(file_name, df)
end

end # End Module