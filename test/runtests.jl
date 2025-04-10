# Copyright (c) 2025 Devansh R Agrawal - All rights reserved.

using GatekeeperFormationFlight
using Test

@testset "GatekeeperFormationFlight" begin
    include("3d_robot_test.jl")
    include("collision_check.jl")
    include("full_test.jl")

end
