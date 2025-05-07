using GatekeeperFormationFlight


@testset "GatekeeperFormationFlight.jl - 3D Nominal Trajectories - 1" begin
    # Create a gatekeeperproblem3d

    reference_path =
        [SVector(10.0, 10.0, 10.0, 0.0, 0.0, 0.0), SVector(20.0, 10.0, 10.0, 0.0, 0.0, 0.0)] # Reference Path
    reference_path = reduce(hcat, reference_path)' # Convert to matrix with points × 6 dimensions

    @show size(reference_path)


    prob = GatekeeperProblem3D(
        obstacles = [Cylinder(5.0, 5.0, 1.0)], # obstacles
        reference_path = reference_path, # Reference Path
        offsets = [SVector(-1.0, 0.0, 0.0), SVector(1.0, 0.0, 0.0)], # Offsets
    )

    nominal = GatekeeperFormationFlight.compute_nominal_trajectories(prob)

    @test nominal[1][1, :] == SVector(9.0, 10.0, 10.0, 0.0, 0.0, 0.0)
    @test nominal[1][2, :] == SVector(19.0, 10.0, 10.0, 0.0, 0.0, 0.0)
    @test nominal[2][1, :] == SVector(11.0, 10.0, 10.0, 0.0, 0.0, 0.0)
    @test nominal[2][2, :] == SVector(21.0, 10.0, 10.0, 0.0, 0.0, 0.0)
end

@testset "GatekeeperFormationFlight.jl - 3D Nominal Trajectories - 2" begin
    # Create a gatekeeperproblem3d

    reference_path = [
        SVector(10.0, 10.0, 10.0, π / 2, 0.0, 0.0),
        SVector(10.0, 20.0, 10.0, π / 2, 0.0, 0.0),
    ] # Reference Path
    reference_path = reduce(hcat, reference_path)' # Convert to matrix with points × 6 dimensions

    prob = GatekeeperProblem3D(
        obstacles = [Cylinder(5.0, 5.0, 1.0)], # obstacles
        reference_path = reference_path, # Reference Path
        offsets = [SVector(-1.0, 0.0, 0.0), SVector(1.0, 0.0, 0.0)], # Offsets
    )

    nominal = GatekeeperFormationFlight.compute_nominal_trajectories(prob)
    @test nominal[1][1, :] == SVector(10.0, 9.0, 10.0, π / 2, 0.0, 0.0)
    @test nominal[1][2, :] == SVector(10.0, 19.0, 10.0, π / 2, 0.0, 0.0)
    @test nominal[2][1, :] == SVector(10.0, 11.0, 10.0, π / 2, 0.0, 0.0)
    @test nominal[2][2, :] == SVector(10.0, 21.0, 10.0, π / 2, 0.0, 0.0)
end

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
