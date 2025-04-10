using Plots

@testset "GatekeeperFormationFlight.jl - 3D Robot Test" begin

    # Create a sample robot in 3D space

    robot = Robot3(SVector(1.0, 2.0, 3.0), SVector(0.1, 0.2, 0.3))
    @test robot.pos == SVector(1.0, 2.0, 3.0)
    @test robot.rot == SVector(0.1, 0.2, 0.3)

    # Test the SVector conversion
    sv = SVector(robot)
    @test sv == SVector(1.0, 2.0, 3.0, 0.1, 0.2, 0.3)

    # Initialize an Array of Obstacles
    obstacles =
        [Sphere(0.5, 0.5, 0.5, 0.1), Sphere(1.5, 1.5, 1.5, 0.2), Sphere(2.5, 2.5, 2.5, 0.3)]

    # Create a 3D RRT Problem
    problem = Dubins3DRRTProblem(obstacles)

    # Use the robot position as the start point
    nodes = [@SVector robot.pos]

    nodes = rrt_star(problem, nodes, 5)
end