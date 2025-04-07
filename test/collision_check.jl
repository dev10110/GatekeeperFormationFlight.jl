# Copyright (c) 2025 Devansh R Agrawal - All rights reserved.

using GatekeeperFormationFlight
using LinearAlgebra, StaticArrays, Random

# use the dubins in GatekeeperFormationFlight
Dubins = GatekeeperFormationFlight.Dubins

@testset "GatekeeperFormationFlight.jl - Collision Check" begin

    wez = Cardioid(0.5, 0.5)

    q0 = SVector(0, 0, 0.0)
    q1 = SVector(1, 1, 0.0)
    q2 = SVector(1.0, 0.2, 0.0)
    turning_radius = 0.1

    errcode, path1 = Dubins.dubins_shortest_path(q0, q1, turning_radius)
    @test errcode == Dubins.EDUBOK

    errcode, path2 = Dubins.dubins_shortest_path(q0, q2, turning_radius)
    @test errcode == Dubins.EDUBOK

    # check collision 1 
    @test is_colliding(wez, path1) == true
    @test is_colliding(wez, path2) == false


end
