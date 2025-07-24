
module DemoUtilTypes

using StaticArrays

using GatekeeperFormationFlight

ASO = GatekeeperFormationFlight.Obstacles.AbstractStaticObstacle

export AgentSettings, SimAgent, SimScenario, SimEnvironment

@kwdef struct AgentSettings
    v_min::Float64 = 0.8
    v_max::Float64 = 1.0
    x_padding::Float64 = 0.1
    y_padding::Float64 = 0.1
    agent_radius::Float64 = 0.125
    turn_radius::Float64 = 0.5
    pitch_limits::SVector{2,Float64} = SVector(-pi / 4, pi / 4)
end

@kwdef struct SimAgent
    id::Int64 = 1
    offset::SVector{5,Float64} = SVector(0.0, 0.0, 0.0, 0.0, 0.0)
end

@kwdef struct SimScenario
    name::String = "default_scenario"
    reconstruction_time::Float64 = 30.0
    reconstruction_step_size::Float64 = 0.005
    start_pose::SVector{5,Float64} = SVector(0.0, 0.0, 0.0, 0.0, 0.0)
    goal_pose::SVector{5,Float64} = SVector(0.0, 0.0, 0.0, 0.0, 0.0)
    domain_min::SVector{3,Float64} = SVector(-1.0, -1.0, 0.0)
    domain_max::SVector{3,Float64} = SVector(1.0, 1.0, 2.0)
end
@kwdef mutable struct SimEnvironment
    scenario::SimScenario = SimScenario()
    agent_settings::AgentSettings = AgentSettings()
    agents::Vector{SimAgent} = Vector{SimAgent}()
    obstacles::Vector{ASO} = Vector{ASO}()
    gatekeeper_coefficients::GatekeeperCoefficients = GatekeeperCoefficients()
    leader_path = nothing
    solution = nothing
    composites = nothing
    gk = nothing # Gatekeeper instance
    data = nothing
end

end # module DemoUtilTypes

# Autoimport itself
using ..DemoUtilTypes
