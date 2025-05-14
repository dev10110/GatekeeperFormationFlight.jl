"""
File: src/multi_gatekeeper.jl

Extension of gatekeeper.jl to simulate multiple agents at once
"""
using Dubins3D
using StaticArrays, LinearAlgebra

module MultiGatekeeper

struct MultiGatekeeperInstance{GK<:GatekeeperProblem} <: AbstractGatekeeperInstance
    problem::GK
    coefficients:GatekeeperCoefficients
end


end

