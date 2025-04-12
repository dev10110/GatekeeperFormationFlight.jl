# Copyright (c) 2025 Devansh R Agrawal - All rights reserved.

using GatekeeperFormationFlight
using Test

@testset "GatekeeperFormationFlight" begin

    # 3D Tests
    include("3d_robot_test.jl")
    include("3d_collision_check.jl")

    include("collision_check.jl")
    include("full_test.jl")
end
