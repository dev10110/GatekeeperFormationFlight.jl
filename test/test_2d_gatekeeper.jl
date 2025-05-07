using GatekeeperFormationFlight
using LinearAlgebra, StaticArrays
using Dubins


@testset "2D Gatekeeper - 2D Dubins Dynamics" begin
    wezes = [Cardioid(1.0, 1.0), Cardioid(2.0, 2.0), Cardioid(3.0, 3.0)]

    reference_path = [1]
    prob = GKDubinsWezes2D(
        wezes = wezes,
        reference_path = reference_path,
        turning_radius = 0.1,
        v_min = 0.8,
        v_max = 1.00,
    )

    @test prob.wezes == GatekeeperFormationFlight.get_obstacles(prob)
    @test prob.reference_path == GatekeeperFormationFlight.get_reference_path(prob)

    D = Vector([0.0, 0.1, 0.0])

    input = SVector(2.0, 0.5)
    input = GatekeeperFormationFlight.apply_input_bounds(prob, input)
    @test input[1] == 1.0
    @test input[2] == 0.5

    GatekeeperFormationFlight.state_dynamics!(
        prob,
        D,
        SVector(0.0, 0.0, 0.0), # Current State
        SVector(1.0, 0.5),
    )

    @test D[1] == 1.0
    @test D[2] == 0.0
    @test D[3] == 0.5
end

@testset "2D Gatekeeper - Dubins Shortest Path" begin
    prob = GKDubinsWezes2D(
        wezes = Vector{GatekeeperFormationFlight.AbstractWez}[],
        reference_path = [],
        turning_radius = 0.1,
        v_min = 0.8,
        v_max = 1.0,
    )

    from_state = Vector([0.0, 0.0, 0.0])
    to_state = Vector([1.0, 0.0, 0.0])

    path = GatekeeperFormationFlight.shortest_path(prob, from_state, to_state)

    @test typeof(path) == Dubins.DubinsPath
    @test path != nothing

    @test path.qi == from_state
    @test path.ρ == prob.turning_radius
    @test path.path_type == Dubins.LSL

    len = GatekeeperFormationFlight.path_length(prob, path)
    @test len == 1.0
end

@testset "2D Gatekeeper - Reconnection Sites" begin

    from_state = Vector([0.0, 0.0, 0.0])
    to_state = Vector([10.0, 0.0, 0.0])


    # one segment in the shortest path?
    reference_path = [dubins_shortest_path(from_state, to_state, 0.1)[2]]

    prob = GKDubinsWezes2D(
        wezes = [],
        reference_path = reference_path,
        turning_radius = 0.1,
        v_min = 0.8,
        v_max = 1.0,
    )

    reconnection_sites = GatekeeperFormationFlight.construct_reconnection_sites(prob, 0.1)

    @test reconnection_sites != nothing
    @test size(reconnection_sites)[1] == 102
end

@testset "2D Gatekeeper - Get Remaining Nominal" begin
    from_state = Vector([0.0, 0.0, 0.0])
    intermediate = Vector([5.0, 0.0, 0.0])
    to_state = Vector([10.0, 0.0, 0.0])

    # one segment in the shortest path?
    reference_path = [
        dubins_shortest_path(from_state, intermediate, 0.1)[2],
        dubins_shortest_path(intermediate, to_state, 0.1)[2],
    ]

    prob = GKDubinsWezes2D(
        wezes = [],
        reference_path = reference_path,
        turning_radius = 0.1,
        v_min = 0.8,
        v_max = 1.0,
    )

    connection_pt = Vector([2.0, 0.0, 0.0])
    path_idx = 1

    remaining_nominal =
        GatekeeperFormationFlight.get_remaining_nominal(prob, path_idx, connection_pt)

    @test remaining_nominal != nothing
    @test GatekeeperFormationFlight.path_length(prob, remaining_nominal) ≈ 8.0
end

@testset "2D Gatekeeper - Solve Problem, No Obstacles" begin
    from_state = Vector([0.0, 0.0, 0.0])
    intermediate = Vector([5.0, 0.0, 0.0])
    to_state = Vector([10.0, 0.0, 0.0])

    # one segment in the shortest path?
    reference_path = [
        dubins_shortest_path(from_state, intermediate, 0.1)[2],
        dubins_shortest_path(intermediate, to_state, 0.1)[2],
    ]

    prob = GKDubinsWezes2D(
        wezes = [],
        reference_path = reference_path,
        turning_radius = 0.1,
        v_min = 0.8,
        v_max = 1.0,
    )

    coeffs = GatekeeperCoefficients(
        switch_step_size = 0.01,
        reconnection_step_size = 0.01,
        max_Ts_horizon = 0.5,
        integration_max_step_size = 1e-3,
        collision_check_step_size = 1e-3,
    )

    gk_instance = GatekeeperInstance(prob, coeffs)

    tspan = [0.0, GatekeeperFormationFlight.path_length(prob, reference_path) / prob.v_max]
    gk_solution =
        simulate_closed_loop_gatekeeper(gk_instance, SVector{3}(from_state), tspan)
end