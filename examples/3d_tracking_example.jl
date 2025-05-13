using Dubins3D
using LinearAlgebra, StaticArrays, OrdinaryDiffEq
using GatekeeperFormationFlight
using Plots

## Generate a 3D dubins path between two points

TURNING_RADIUS::Float64 = 4.0

qi = [0.0, 0.0, 0.0, 0.0, deg2rad(5.0)]
q1 = [20.0, 10.0, 5.0, π / 2.0, deg2rad(5.0)]
q2 = [10.0, 20.0, 10.0, -π, deg2rad(5.0)]
q3 = [0.0, 10.0, 15.0, -π / 2.0, deg2rad(5.0)]

m1 = DubinsManeuver3D(qi, q1, TURNING_RADIUS, [deg2rad(-30.0), deg2rad(30.0)])
m2 = DubinsManeuver3D(q1, q2, TURNING_RADIUS, [deg2rad(-30.0), deg2rad(30.0)])
m3 = DubinsManeuver3D(q2, q3, TURNING_RADIUS, [deg2rad(-30.0), deg2rad(30.0)])

combined_maneuver = [m1, m2, m3]

path_length = sum(x -> x.length, combined_maneuver)
@show path_length
path = compute_sampling.(combined_maneuver; numberOfSamples = 1000) #(3,) of (1000,) of (5)

# @show path
@show size(path)

path = vec(reduce(hcat, path))
path = reduce(hcat, path)

# @show path
@show size(path)


# function closed_loop_tracking!(D, state, params, time)
#     x, y, z, yaw, pitch = state

#     # Ensure idx stays within bounds 
#     idx = min(max(floor(Int, ((time / path_length) * 1000) + 1), 1), size(path, 2))

#     # Extract reference state
#     xr, yr, zr, yawr, pitchr = path[:, idx]
#     state_d = [xr, yr, zr, yawr]  # Note: only passing position and yaw reference

#     # Calculate reference inputs (approximate derivatives)
#     # If idx+1 is valid, use forward difference to approximate derivatives
#     if idx < size(path, 2)
#         dt = path_length / 1000  # Time between samples
#         next_state = path[:, idx+1]
#         v_ref = norm(next_state[1:3] - [xr, yr, zr]) / dt
#         ωyaw_ref = (next_state[4] - yawr) / dt
#         pitch_ref = next_state[5]  # Direct reference for pitch
#     else
#         # Use last known good values at the end of the path
#         v_ref = 1.0
#         ωyaw_ref = 0.0
#         pitch_ref = pitchr
#     end

#     input_d = [v_ref, ωyaw_ref, pitch_ref]

#     # Call tracking controller with proper references
#     v, ω_yaw, pitch_cmd =
#         GatekeeperFormationFlight.Dubins3DTrackingController.tracking_controller(
#             state,
#             state_d,
#             input_d,
#         )

#     # Apply input bounds
#     v = clamp(v, 0.8, 1.0)
#     ω_yaw = clamp(ω_yaw, -1, 1)
#     pitch_cmd = clamp(pitch_cmd, -deg2rad(30.0), deg2rad(30.0))

#     # Update state derivatives
#     D[1] = v * cos(yaw) * cos(pitch_cmd)
#     D[2] = v * sin(yaw) * cos(pitch_cmd)
#     D[3] = v * sin(pitch_cmd)
#     D[4] = ω_yaw
#     return
# end

nominal_states = []
reference_states = []
do_append = 1

function closed_loop_tracking!(D, state, params, time)
    global do_append, nominal_states
    x, y, z, yaw, pitch = state

    x_d, u_d =
        GatekeeperFormationFlight.Dubins3DTrackingController.get_reference_state_and_input(
            combined_maneuver,
            time,
            @SVector([0.0, 2.0, 0.0]),
            # @SVector([0.0, 0.0, 0.0]),
        )

    x_ref, u_ref =
        GatekeeperFormationFlight.Dubins3DTrackingController.get_reference_state_and_input(
            combined_maneuver,
            time,
        )

    if do_append % 10 == 0
        do_append = 0
        push!(reference_states, x_ref)
        push!(nominal_states, x_d)
    end
    do_append += 1

    v, ω, ψ = GatekeeperFormationFlight.Dubins3DTrackingController.tracking_controller(
        state,
        x_d,
        u_d;
        # k1 = 10000.0,
        # k1 = 1.0,
        # k1 = 250.0,
        # k1 = 450.0,
        # k2 = 100.0,
        # k3 = 20.0,
        # k4 = 5.0,
        # k3 = 10.0,
        # k4 = 1.0,
    )

    # Apply input bounds
    v = clamp(v, 0.5, 1.2)
    ω_max = v / TURNING_RADIUS
    # ω = clamp(ω, -1, 1)
    ω = clamp(ω, -ω_max, ω_max)
    pitch_cmd = clamp(ψ, -deg2rad(30.0), deg2rad(30.0))

    # update state derivatives
    D[1] = v * cos(yaw) * cos(pitch_cmd)
    D[2] = v * sin(yaw) * cos(pitch_cmd)
    D[3] = v * sin(pitch_cmd)
    D[4] = ω
    return
end

q0 = Vector([0.0, 2.0, 0.0, 0.0, 0.0])

## Solve for agent tracking controller usign odesolver
odeproblem = ODEProblem(closed_loop_tracking!, q0, (0.0, path_length - 0.5), nothing)
odesol = solve(odeproblem, Tsit5(), dtmax = 0.01, dt = 0.001)

## Plot the nominal path and path taken by the agent
# @show size(path)

# @show size(path)

# plot(
#     path[1, :],
#     path[2, :],
#     path[3, :],
#     label = "Leader Path",
#     legend = :topleft,
#     xlabel = "X",
#     ylabel = "Y",
#     zlabel = "Z",
#     title = "3D Dubins Path",
# )

# @show size(nominal_states)
# @show size(nominal_path)
# plot!(
#     nominal_path[1, :],
#     nominal_path[2, :],
#     nominal_path[3, :],
#     label = "Follower Path",
#     color = :blue,
# )
# plot!(
#     reference_path[1, :],
#     reference_path[2, :],
#     reference_path[3, :],
#     label = "Reference Path",
#     color = :green,
# )

# # Plot the path taken by the agent as an animation
# plot!(odesol, idxs = (1, 2, 3), label = "Tracked Path", color = :red)


nominal_path = hcat([Vector(x) for x in nominal_states]...)
reference_path = hcat([Vector(x) for x in reference_states]...)

# Create a layout with two subfigures
p1 = plot(
    path[1, :],
    path[2, :],
    path[3, :],
    label = "Leader Path",
    legend = :topleft,
    xlabel = "X",
    ylabel = "Y",
    zlabel = "Z",
    title = "3D Dubins Path",
)

plot!(
    p1,
    nominal_path[1, :],
    nominal_path[2, :],
    nominal_path[3, :],
    label = "Follower Path",
    color = :blue,
)
plot!(
    p1,
    reference_path[1, :],
    reference_path[2, :],
    reference_path[3, :],
    label = "Reference Path",
    color = :green,
)

# Plot the path taken by the agent as an animation
plot!(p1, odesol, idxs = (1, 2, 3), label = "Tracked Path", color = :red)

# Second subfigure: Reference yaw angle
# p2 = plot(
#     1:length(reference_path[4, :]),
#     reference_path[4, :],
#     label = "Reference Yaw",
#     xlabel = "Sample Index",
#     ylabel = "Yaw (rad)",
#     title = "Reference Yaw Angle",
#     color = :green,
# )

# # Combine plots into a layout
# plot(p1, p2, layout = (2, 1), size = (800, 800))