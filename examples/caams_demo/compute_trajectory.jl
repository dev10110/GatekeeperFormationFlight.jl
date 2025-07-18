using ArgParse

include("demo_util.jl")
using .DemoUtil

function parse_commandline()
    s = ArgParseSettings()


    @add_arg_table s begin
        "env"
        help = "File containing the environment description"
        required = true
    end

    return parse_args(s)
end

function main()
    parsed_args = parse_commandline()

    @info "Loading Environment..."
    env = DemoUtil.load_env(parsed_args["env"])
    @info "Environment loaded successfully."
    println("")

    @info "Solving for RRT* Path"
    if !(DemoUtil.solve_leader_path(env))
        @warn "Failed to solve an RRT* Trajectory. Exiting"
        return 1
    end
    @info "RRT Path Constructed successfully."
    println("")

    @info "Solving Inter-Agent Gatekeeper Problem...this may take a while."
    if !DemoUtil.solve_gk_problem!(env)
        @warn "Failed to solve gatekeeper problem."
        return 2
    end
    @info "Success!"
    println("")

    @info "Saving intermediate result to file..."

    result_file_name = "scenarios/$(env.scenario.name)_result.csv"
    DemoUtil.write_to_file(env, result_file_name)
    @info "Results written to $result_file_name"
    println("")

    @info "Fitting polynomials to the final trajectories."
    polynomials = DemoUtil.fit_polynomials!(env.data)
    @info "Success! Writing to file."


    trajectory_file_name = "scenarios/$(env.scenario.name)_trajectories.csv"
    DemoUtil.write_polynomials_to_file(polynomials, trajectory_file_name)
    @info "Trajectories written to $trajectory_file_name"
end


main()