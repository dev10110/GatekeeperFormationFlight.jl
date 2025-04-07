using GatekeeperFormationFlight
using LinearAlgebra, StaticArrays, Random

# use the dubins in GatekeeperFormationFlight
Dubins = GatekeeperFormationFlight.Dubins

@testset "GatekeeperFormationFlight.jl - Full Test" begin

    # create an environment
    function create_random_scenario(N_wezes = 24)
        wezes_1 = [Cardioid(rand(), rand()) for i = 1:N_wezes/2]
        wezes_2 = [Cardioid(0.5 + 0.2 * randn(), 0.5 + 0.2 * randn()) for i = 1:N_wezes/2]
        wezes = vcat(wezes_1, wezes_2)
        return wezes
    end
    Random.seed!(27182818)
    wezes = create_random_scenario(24)

    # create a set of robots
    leader_robot = Robot(-0.25, 0.0, 0.0)
    follower_robots = [Robot(-0.3, -0.05, 0.0), Robot(-0.3, 0.05, 0.0)]
    robots = vcat(leader_robot, follower_robots...)

    # plan a path for the leader using RRT*
    domain = ((@SVector [0, 0, -1.0 * π]), (@SVector [1, 1, 1.0 * π]))
    turning_radius = 0.1
    rrt_problem = DubinsRRTProblem(domain, turning_radius, wezes)

    # start the tree with the root node
    nodes = [Node(SVector(0, 0, 0.0))]

    # add 1000 nodes to the tree
    nodes = rrt_star(rrt_problem, nodes, 1000)

    # see if there is a path 
    success_code, waypoints = get_best_path(rrt_problem, nodes, @SVector [1.0, 1.0, 0])

    # while we havent found a path, add some nodes and check if we have a feasible path
    iter_counter = 0
    while !success_code && iter_counter < 10
        nodes = rrt_star(rrt_problem, nodes, 500)
        success_code, waypoints = get_best_path(rrt_problem, nodes, [1.0, 1.0, 0])
        iter_counter += 1
    end

    @assert success_code

    # prepend and append the start and the goal
    waypoints = [SVector(leader_robot), waypoints..., SVector(1.25, 1, 0.0)]

    # create all the dubins paths
    path = Dubins.DubinsPath[]
    # add all the waypoints to the path
    for i = 2:length(waypoints)
        e, p = Dubins.dubins_shortest_path(
            waypoints[i-1],
            waypoints[i],
            rrt_problem.turning_radius,
        )
        @assert e == Dubins.EDUBOK
        push!(path, p)
    end

    @show length(path)
    @test length(path) == 5


    # now test gatekeeper tracking on this trajectory
    offsets = [SVector(robot) - SVector(leader_robot) for robot in robots]

    prob = GatekeeperProblem(;
            wezes=wezes, 
            reference_path=path,
            offset=offsets[1], 
            switch_step_size=2e-3, 
            reconnection_step_size=0.01, 
            max_Ts_horizon=0.5, 
            integration_max_step_size=1e-3, 
            collision_check_step_size=1e-3, 
            ) 

 
    gk_problems = [
        GatekeeperProblem(;
            wezes=wezes, 
            reference_path=path, offset=offsets[i], 
            switch_step_size=2e-3, 
            reconnection_step_size=0.01, 
            max_Ts_horizon=0.5, 
            integration_max_step_size=1e-3,
            collision_check_step_size=1e-3,
            ) 
        for i=1:length(robots)
    ]

    tspan = [0.0, total_path_length(path)]
    gk_solutions = [
        simulate_closed_loop_gatekeeper(
            SVector(robots[i]), 
            tspan, 
            gk_problems[i])
        for i=1:length(robots)
    ]

    # TODO(dev): actually write a test that will check if gatekeeper is safe.

end
