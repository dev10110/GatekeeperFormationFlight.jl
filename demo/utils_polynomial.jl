using BSplineKit

struct MyPolynomial
    t0::Float64 # where it is defined
    δt::Float64 # duration of validity
    cs::Vector{Float64} # coefficients of the polynomial
end

function (p::MyPolynomial)(t)
    return sum(p.cs[i] * (t - p.t0)^(i-1) for i in 1:length(p.cs))
end

function create_polynominal_representation(ts, xs, δt; degree=3, max_degree=7)
    # Fit a bspline of the given degree to the data
    spline = interpolate(ts, xs, BSplineOrder(degree+1))

    # approxiamte the spline at the chosen δt
    subts = range(minimum(ts), maximum(ts), step=δt)
    B = BSplineBasis(BSplineOrder(degree+1), subts)
    spline  = approximate(spline, B)

    tmin = minimum(ts)
    tmax = maximum(ts)
    # Create a polynomial representation
    ps = []
    for ti in tmin:δt:tmax
        δ = min( δt, tmax - ti) # ensure we do not go out of bounds
        if δ <= 1e-2 # if the duration is too small, skip
            continue 
        end
        c = zeros(max_degree + 1)
        for d in 0:degree # since we are using cubic splines, we only need up to degree 3
            c[d+1] = (1/factorial(d)) * (Derivative(d) * spline)(ti)
        end
        poly = MyPolynomial(ti, δ, c)
        push!(ps, poly)
    end
    return ps
end

function write_poly_file(filename, polys_x, polys_y, polys_z, polys_yaw)
    # check lengths
    @assert length(polys_x) == length(polys_y) == length(polys_z) == length(polys_yaw)
    N = length(polys_x) # number of rows
    open(filename, "w") do io
        # write the header
        println(io, "duration,x^0,x^1,x^2,x^3,x^4,x^5,x^6,x^7,y^0,y^1,y^2,y^3,y^4,y^5,y^6,y^7,z^0,z^1,z^2,z^3,z^4,z^5,z^6,z^7,yaw^0,yaw^1,yaw^2,yaw^3,yaw^4,yaw^5,yaw^6,yaw^7")
        for i=1:N
            # write the polynomial coefficients
            println(io, 
                polys_x[i].δt, ", ",
                join(polys_x[i].cs, ", "), ", ",
                join(polys_y[i].cs, ", "), ", ",
                join(polys_z[i].cs, ", "), ", ",
                join(polys_yaw[i].cs, ", ")
            )
        end
    end
end