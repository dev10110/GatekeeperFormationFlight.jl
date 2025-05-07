# Copyright (c) 2025 Devansh R Agrawal - All rights reserved.

using GatekeeperFormationFlight
using Test

@testset "GatekeeperFormationFlight" begin


    @testset "2D Tests" begin
        @testset "2D Gatekeeper" begin
            include("test_2d_gatekeeper.jl")
        end
    end

    @testset "3D Tests" begin
        @testset "#d Robot Test" begin
            include("3d_robot_test.jl")
        end

        @testset "3D Collision Check" begin
            include("3d_collision_check.jl")
        end

        @testset "3D Nominal Trajectories" begin
            include("3d_nominal_trajectories.jl")
        end
    end
end
