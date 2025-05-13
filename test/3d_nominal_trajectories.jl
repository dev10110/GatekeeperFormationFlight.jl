using GatekeeperFormationFlight


# @testset "GatekeeperFormationFlight.jl - 3D Nominal Trajectories - 1" begin
#     # Create a gatekeeperproblem3d

#     reference_path =
#         [SVector(10.0, 10.0, 10.0, 0.0, 0.0, 0.0), SVector(20.0, 10.0, 10.0, 0.0, 0.0, 0.0)] # Reference Path
#     reference_path = reduce(hcat, reference_path)' # Convert to matrix with points × 6 dimensions

#     @show size(reference_path)


#     prob = GatekeeperProblem3D(
#         obstacles = [Cylinder(5.0, 5.0, 1.0)], # obstacles
#         reference_path = reference_path, # Reference Path
#         offsets = [SVector(-1.0, 0.0, 0.0), SVector(1.0, 0.0, 0.0)], # Offsets
#     )

#     nominal = GatekeeperFormationFlight.compute_nominal_trajectories(prob)

#     @test nominal[1][1, :] == SVector(9.0, 10.0, 10.0, 0.0, 0.0, 0.0)
#     @test nominal[1][2, :] == SVector(19.0, 10.0, 10.0, 0.0, 0.0, 0.0)
#     @test nominal[2][1, :] == SVector(11.0, 10.0, 10.0, 0.0, 0.0, 0.0)
#     @test nominal[2][2, :] == SVector(21.0, 10.0, 10.0, 0.0, 0.0, 0.0)
# end

# @testset "GatekeeperFormationFlight.jl - 3D Nominal Trajectories - 2" begin
#     # Create a gatekeeperproblem3d

#     reference_path = [
#         SVector(10.0, 10.0, 10.0, π / 2, 0.0, 0.0),
#         SVector(10.0, 20.0, 10.0, π / 2, 0.0, 0.0),
#     ] # Reference Path
#     reference_path = reduce(hcat, reference_path)' # Convert to matrix with points × 6 dimensions

#     prob = GatekeeperProblem3D(
#         obstacles = [Cylinder(5.0, 5.0, 1.0)], # obstacles
#         reference_path = reference_path, # Reference Path
#         offsets = [SVector(-1.0, 0.0, 0.0), SVector(1.0, 0.0, 0.0)], # Offsets
#     )

#     nominal = GatekeeperFormationFlight.compute_nominal_trajectories(prob)
#     @test nominal[1][1, :] == SVector(10.0, 9.0, 10.0, π / 2, 0.0, 0.0)
#     @test nominal[1][2, :] == SVector(10.0, 19.0, 10.0, π / 2, 0.0, 0.0)
#     @test nominal[2][1, :] == SVector(10.0, 11.0, 10.0, π / 2, 0.0, 0.0)
#     @test nominal[2][2, :] == SVector(10.0, 21.0, 10.0, π / 2, 0.0, 0.0)
# end

@testset "GatekeeperFormationFlight.jl - 3D Rotation Matrices" begin

    @test isapprox(
        GatekeeperFormationFlight.get_transformation_matrix(π / 2, 0.0, 0.0),
        [
            0.0 -1.0 0.0
            1.0 0.0 0.0
            0.0 0.0 1.0
        ],
        rtol = 1e-6,
        atol = 1e-6,
    )

    @test isapprox(
        GatekeeperFormationFlight.get_transformation_matrix(0.0, π / 2, 0.0),
        [
            0.0 0.0 1.0
            0.0 1.0 0.0
            -1.0 0.0 0.0
        ],
        rtol = 1e-6,
        atol = 1e-6,
    )

    @test isapprox(
        GatekeeperFormationFlight.get_transformation_matrix(0.0, 0.0, π / 2),
        [
            1.0 0.0 0.0
            0.0 0.0 -1.0
            0.0 1.0 0.0
        ],
        rtol = 1e-6,
        atol = 1e-6,
    )
end


@testset "GatekeeperFormationFlight.jl - Reference State Transform" begin

    qi = @SVector Float64[1.0, 0.0, 0.0, 0.0, 0.0]
    qf = @SVector Float64[10.0, 0.0, 0.0, 0.0, 0.0]
    turning_radius = 10.0
    pitch_limits = @SVector Float64[deg2rad(-10), deg2rad(15)]

    maneuver = DubinsManeuver3D(qi, qf, turning_radius, pitch_limits)
    maneuver_v = [maneuver]

    @test isapprox(maneuver.length, 9.0, rtol = 1e-6, atol = 1e-6)
    @test isapprox(sum(x -> x.length, maneuver_v), 9.0, rtol = 1e-6, atol = 1e-6)


    # Get the reference state and input
    x_d, u_d =
        GatekeeperFormationFlight.Dubins3DTrackingController.get_reference_state_and_input(
            maneuver_v,
            0.0,
        )

    @test x_d == @SVector Float64[1.0, 0.0, 0.0, 0.0, 0.0]
    @test u_d == @SVector Float64[1.0, 0.0, 0.0]

    x_d, u_d =
        GatekeeperFormationFlight.Dubins3DTrackingController.get_reference_state_and_input(
            maneuver_v,
            9.0,
        )

    @test isapprox(x_d, @SVector Float64[10.0, 0.0, 0.0, 0.0, 0.0])
    @test isapprox(u_d, @SVector Float64[1.0, 0.0, 0.0])

    offset = @SVector Float64[-1.0, 0.0, 0.0]
    x_d, u_d =
        GatekeeperFormationFlight.Dubins3DTrackingController.get_reference_state_and_input(
            maneuver_v,
            0.0,
            offset,
        )

    @test x_d == @SVector Float64[0.0, 0.0, 0.0, 0.0, 0.0]
    @test u_d == @SVector Float64[1.0, 0.0, 0.0]


    # Now try a maneuver with a yaw
    qi = @SVector Float64[0.0, 1.0, 0.0, π/2.0, 0.0]
    qf = @SVector Float64[0.0, 10.0, 0.0, π/2.0, 0.0]

    maneuver = DubinsManeuver3D(qi, qf, turning_radius, pitch_limits)
    maneuver_v = [maneuver]

    @test isapprox(maneuver.length, 9.0, rtol = 1e-6, atol = 1e-6)
    @test isapprox(sum(x -> x.length, maneuver_v), 9.0, rtol = 1e-6, atol = 1e-6)

    offset = @SVector Float64[-1.0, 0.0, 0.0]
    x_d, u_d =
        GatekeeperFormationFlight.Dubins3DTrackingController.get_reference_state_and_input(
            maneuver_v,
            0.0,
            offset,
        )

    @test isapprox(x_d, @SVector Float64[0.0, 0.0, 0.0, π/2.0, 0.0])
    @test isapprox(u_d, @SVector Float64[1.0, 0.0, 0.0])

    offset = @SVector Float64[-1.0, -2.0, 0.0]
    x_d, u_d =
        GatekeeperFormationFlight.Dubins3DTrackingController.get_reference_state_and_input(
            maneuver_v,
            0.0,
            offset,
        )


    R = GatekeeperFormationFlight.Dubins3DTrackingController.get_transformation_matrix(
        π / 2,
        0.0,
        0.0,
    )

    @show R

    @test isapprox(x_d, @SVector Float64[-2.0, 0.0, 0.0, π/2.0, 0.0])
end


