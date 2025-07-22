module FileUtil

using GatekeeperFormationFlight

ASO = GatekeeperFormationFlight.Obstacles.AbstractStaticObstacle

function obstacle_to_dict(obs::ASO)
    if obs isa Cylinder
        return Dict("type" => "Cylinder", "radius" => obs.radius, "position" => obs.center)
    elseif obs isa Sphere
        return Dict("type" => "Sphere", "radius" => obs.R, "position" => obs.pos)
    elseif obs isa GroundedHalfDome
        return Dict("type" => "GroundedHalfDome", "radius" => obs.R, "position" => obs.pos)
    elseif obs isa FPVGate
        return Dict("type" => "FPVGate", "position" => obs.pos)
    else
        error("Unsupported obstacle type: $(typeof(obs))")
    end
end


function dict_to_obstacle(d)
    obs_type = d[:type]
    if obs_type == "Cylinder"
        return Cylinder(d[:position][1], d[:position][2], d[:radius])
    elseif obs_type == "Sphere"
        return Sphere(d[:position]..., d[:radius])
    elseif obs_type == "GroundedHalfDome"
        return GroundedHalfDome(d[:position]..., d[:radius])
    elseif obs_type == "FPVGate"
        return FPVGate(d[:position]...)
    else
        error("Unsupported obstacle type: $obs_type")
    end
end

end