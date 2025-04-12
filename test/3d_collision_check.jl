using GatekeeperFormationFlight

using StaticArrays, Dubins3D

@testset "GatekeeperFormationFlight.jl - 3D Collision Check" begin

    # CHECK INFINITE HEIGHT COLLISIONS

    obstacle = Cylinder(5.0, 5.0, 1.0)

    pos1 = SVector(5.0, 5.0, 5.0)
    pos2 = SVector(5.0, 5.0, 50.0)
    pos3 = SVector(5.0, 5.0, 500.0)

    @test collision_distance(obstacle, pos1) <= 0.0
    @test collision_distance(obstacle, pos2) <= 0.0
    @test collision_distance(obstacle, pos3) <= 0.0

    positions = [
        SVector(5.0, 5.0, 5.0),
        SVector(4.8, 5.0, 5.0),
        SVector(4.6, 5.0, 5.0),
        SVector(4.4, 5.0, 5.0),
        SVector(4.2, 5.0, 5.0),
        SVector(4.0, 5.0, 5.0),
    ]

    # Check for collision
    for pos in positions
        @test collision_distance(obstacle, pos) <= 0.0
    end

    # Points exactly on the boundary of the cylinder
    boundary_positions = [
        SVector(6.0, 5.0, 5.0),  # On the boundary in +x direction
        SVector(4.0, 5.0, 5.0),  # On the boundary in -x direction
        SVector(5.0, 6.0, 5.0),  # On the boundary in +y direction
        SVector(5.0, 4.0, 5.0),   # On the boundary in -y direction
    ]

    for pos in boundary_positions
        @test abs(collision_distance(obstacle, pos)) <= 1e-6  # Should be very close to 0
    end

    # Points just outside the cylinder's radius
    outside_positions = [
        SVector(6.1, 5.0, 5.0),  # Slightly outside in +x direction
        SVector(3.9, 5.0, 5.0),  # Slightly outside in -x direction
        SVector(5.0, 6.1, 5.0),  # Slightly outside in +y direction
        SVector(5.0, 3.9, 5.0),   # Slightly outside in -y direction
    ]

    for pos in outside_positions
        @test collision_distance(obstacle, pos) > 0.0  # Should be positive
    end

    # Points far away from the cylinder
    far_positions =
        [SVector(10.0, 10.0, 5.0), SVector(-10.0, -10.0, 5.0), SVector(0.0, 0.0, 5.0)]

    for pos in far_positions
        @test collision_distance(obstacle, pos) > 0.0  # Should be positive
    end

    # Path that grazes the edge of the cylinder
    grazing_start = [4.0, 5.0, 5.0, 0.0, 0.0]
    grazing_goal = [6.0, 5.0, 5.0, 0.0, 0.0]

    grazing_maneuver =
        DubinsManeuver3D(grazing_start, grazing_goal, 1.0, [deg2rad(-15), deg2rad(20)])
    grazing_samples = compute_sampling(grazing_maneuver; numberOfSamples = 10)

    @test any(
        collision_distance(obstacle, SVector{3,Float64}(sample[1:3])) <= 0.0 for
        sample in grazing_samples
    ) == true  # Should detect collision
end

@testset "GatekeeperFormationFlight.jl - 3D Path Collision Check" begin

    start_pos = [0.0, 0.0, 0.0, 0.0, 0.0]
    goal_pos = [2.0, 0.0, 0.0, 0.0, 0.0]

    maneuver = DubinsManeuver3D(start_pos, goal_pos, 1.0, [deg2rad(-15), deg2rad(20)])

    @test abs(maneuver.length - 2.0) <= 1e-6

    samples = compute_sampling(maneuver; numberOfSamples = 10)
    @test length(samples) == 10

    # check for collision
    obstacle = Cylinder(1.0, 0.0, 0.5)

    @show samples

    @test any(
        collision_distance(obstacle, SVector{3,Float64}(sample[1:3])) < 0.0 for
        sample in samples
    ) == true

end