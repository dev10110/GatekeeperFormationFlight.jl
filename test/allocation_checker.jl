using StaticArrays, LinearAlgebra
using Dubins3D




qi = @SVector [0.0, 0.0, 0.0, 0.0, 0.0]
qf = @SVector [1.0, 1.0, 1.0, 0.0, 0.0]
turning_radius = 1.0
pitch_limits = [-15, 20]

@time maneuver = DubinsManeuver3D(qi, qf, turning_radius, pitch_limits)


