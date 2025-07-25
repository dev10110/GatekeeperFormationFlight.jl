using Plots
using CSV, DataFrames
using BSplineKit

struct MyPoly
    t::Float64
    cs::Vector{Float64}
end

function (p::MyPoly)(t)
    return sum(p.cs[i] * (t-p.t)^(i-1) for i in 1:length(p.cs))
end

# load the csv of the trajectories
df = CSV.read("gatekeeper_trajectories.csv", DataFrame)

println(df[1:5, :])

# rescale time so that it lines in [0, Tmax]
Tmax = 30.0 # desired t_max 
df.time .*= Tmax / maximum(df.time)

# plot the trajectories
println("Plotting Gatekeeper Trajectories...")
plot()
# for i = 1:1
i = 1
    # extract the df for this robot_id
    robot_df = df[df.robot_id .== i, :]
    
    # plot x vs t
    plot!(
        robot_df.time,
        robot_df.x,
        label = "Robot $(i) X",
        color = (i == 1 ? :blue : :green),
        linewidth = 2,
    )

    # subselect atmost N datapoints
    N = 30
    if length(robot_df.time) > N
        robot_df = robot_df[1:round(Int, length(robot_df.time) / N):end, :]
    end

    # now interpolate the x trajectory
    
    spline_x = interpolate(robot_df.time, robot_df.x, BSplineOrder(4), Natural())
    Tmax = maximum(robot_df.time)
    plot!(
        t -> spline_x(t),
        0.0,
        Tmax,
        label = "Interpolated Robot $(i) X",
        linestyle = :dash,
        linewidth = 4,
    )

    # try to create the polynomial representation
    max_degree = 3
    vs = []
    for ti in 0:1.0:Tmax
        v = zeros(max_degree + 1)
        for degree in 0:max_degree
            v[degree+1] = (1/factorial(degree)) * (Derivative(degree) * spline_x)(ti)
        end
        poly = MyPoly(ti, v)
        push!(vs, poly)
    end

    # try plotting my poly
    for poly in vs
        plot!(
            t -> poly(t),
            poly.t,
            poly.t + 1.0, 
            # linestyle = :dot, 
            label=false
        )
    end

    # # plot the x and y trajectories
    # plot!(
    #     robot_df.x,
    #     robot_df.y,
    #     label = "Robot $(i) X",
    #     color = (i == 1 ? :blue : :green),
    #     linewidth = 2,
    # )
# end
plot!()
