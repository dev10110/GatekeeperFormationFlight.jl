using Plots
using CSV, DataFrames
# using BSplineKit

# load the trajectories csv
df = CSV.read("gatekeeper_trajectories.csv", DataFrame)
println(df[1:5, :])

# create a spline for robot 1 on the x axis
robot_df = df[df.robot_id .== 1, :]
spline_x = interpolate(robot_df.time, robot_df.x, BSplineOrder(4))



# plot the t-x trajectory of each robot
println("Plotting Gatekeeper Trajectories...")
plots = [plot(), plot()]

for i=1:1
    # extract the df for this robot_id
    robot_df = df[df.robot_id .== i, :]
    
    # plot x vs t
    
    plot!(plots[1], 
        robot_df.time,
        robot_df.x,
        label = "Robot $(i) X",
        color = (i == 1 ? :blue : :green),
        linewidth = 2,
    )

    # plot y vs t
    
    plot!(plots[2], 
        robot_df.time,
        robot_df.y,
        label = "Robot $(i) Y",
        color = (i == 1 ? :blue : :green),
        linewidth = 2,
    )

end

plot!()

# load the polynomial representation

# plot the polynomial representation
println("Plotting Polynomial Representation of Robot 1 Trajectory...")
for i=1:1    

    df_poly = CSV.read("robot_$(i)_polynomials.csv", DataFrame)
    println("Polynomial Representation of Robot 1 Trajectory:")
    println(df_poly[1:5, :])

    ts = Float64[]
    xs = Float64[]
    ys = Float64[]

    current_t = 0.0
    for j in 1:size(df_poly, 1)
        duration = df_poly[j, :duration]
        sub_ts = 0:0.01:duration
        sub_xs = map(ti -> sum(df_poly[j, 1 + k] * ti^(k-1) for k in 1:8), sub_ts)
        sub_ys = map(ti -> sum(df_poly[j, 9 + k] * ti^(k-1) for k in 1:8), sub_ts)
        append!(ts, current_t .+ sub_ts)
        append!(xs, sub_xs)
        append!(ys, sub_ys)
        current_t += duration
    end

    # plot x vs t
    plot!(plots[1], 
        ts,
        xs,
        label = "Robot $(i) X",
    )

    # plot y vs t
    
    plot!(plots[2], 
        ts,
        ys,
        label = "Robot $(i) Y",
    )
end

plot()
plot!(plots...)

# struct MyPoly
#     t::Float64
#     cs::Vector{Float64}
# end

# function (p::MyPoly)(t)
#     return sum(p.cs[i] * (t-p.t)^(i-1) for i in 1:length(p.cs))
# end

# # load the csv of the trajectories
# df = CSV.read("gatekeeper_trajectories.csv", DataFrame)

# println(df[1:5, :])

# # rescale time so that it lines in [0, Tmax]
# Tmax = 30.0 # desired t_max 
# df.time .*= Tmax / maximum(df.time)

# # plot the trajectories
# println("Plotting Gatekeeper Trajectories...")
# plot()
# # for i = 1:1
# i = 1
#     # extract the df for this robot_id
#     robot_df = df[df.robot_id .== i, :]
    
#     # plot x vs t
#     plot!(
#         robot_df.time,
#         robot_df.x,
#         label = "Robot $(i) X",
#         color = (i == 1 ? :blue : :green),
#         linewidth = 2,
#     )

#     # subselect atmost N datapoints
#     N = 30
#     if length(robot_df.time) > N
#         robot_df = robot_df[1:round(Int, length(robot_df.time) / N):end, :]
#     end

#     # now interpolate the x trajectory
    
#     spline_x = interpolate(robot_df.time, robot_df.x, BSplineOrder(4), Natural())
#     Tmax = maximum(robot_df.time)
#     plot!(
#         t -> spline_x(t),
#         0.0,
#         Tmax,
#         label = "Interpolated Robot $(i) X",
#         linestyle = :dash,
#         linewidth = 4,
#     )

#     # try to create the polynomial representation
#     max_degree = 3
#     vs = []
#     for ti in 0:1.0:Tmax
#         v = zeros(max_degree + 1)
#         for degree in 0:max_degree
#             v[degree+1] = (1/factorial(degree)) * (Derivative(degree) * spline_x)(ti)
#         end
#         poly = MyPoly(ti, v)
#         push!(vs, poly)
#     end

#     # try plotting my poly
#     for poly in vs
#         plot!(
#             t -> poly(t),
#             poly.t,
#             poly.t + 1.0, 
#             # linestyle = :dot, 
#             label=false
#         )
#     end

#     # # plot the x and y trajectories
#     # plot!(
#     #     robot_df.x,
#     #     robot_df.y,
#     #     label = "Robot $(i) X",
#     #     color = (i == 1 ? :blue : :green),
#     #     linewidth = 2,
#     # )
# # end
# plot!()
