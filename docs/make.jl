using GatekeeperFormationFlight
using Documenter

DocMeta.setdocmeta!(
    GatekeeperFormationFlight,
    :DocTestSetup,
    :(using GatekeeperFormationFlight);
    recursive = true,
)

makedocs(;
    modules = [GatekeeperFormationFlight],
    authors = "Devansh Agrawal <devansh@umich.edu> and contributors",
    sitename = "GatekeeperFormationFlight.jl",
    format = Documenter.HTML(;
        canonical = "https://dev10110.github.io/GatekeeperFormationFlight.jl",
        edit_link = "main",
        assets = String[],
    ),
    pages = [
        "Home" => "index.md",
        "Robots and Wezes" => "robots_and_wezes.md",
        "RRT*" => "rrt_star.md",
        "Tracking Controllers" => "tracking_controllers.md",
        "Gatekeeper" => "gatekeeper.md",
        "API" => "api.md",
    ],
    clean = false,
)

deploydocs(; repo = "github.com/dev10110/GatekeeperFormationFlight.jl", devbranch = "main")
