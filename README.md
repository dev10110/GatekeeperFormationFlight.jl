# GatekeeperFormationFlight

[![Stable](https://img.shields.io/badge/docs-stable-blue.svg)](https://dev10110.github.io/GatekeeperFormationFlight.jl/stable/)
[![Dev](https://img.shields.io/badge/docs-dev-blue.svg)](https://dev10110.github.io/GatekeeperFormationFlight.jl/dev/)
[![Build Status](https://github.com/dev10110/GatekeeperFormationFlight.jl/actions/workflows/CI.yml/badge.svg?branch=main)](https://github.com/dev10110/GatekeeperFormationFlight.jl/actions/workflows/CI.yml?query=branch%3Amain)


## Quickstart

Clone this repo to a good location. 

Install the dependencies
```julia
] instantiate
] precompile
```

If you have any issues with `Dubins.jl`, 
```julia
] rm Dubins 
] add https://github.com/dev10110/Dubins.jl.git
```
Tested with `Dubins.jl v1.2.3` on Apr 7 2025. 

Run the example in `examples/full_example.jl`: From the `examples` directory, run
```
julia --project full_example.jl
```
This will generate the gif that is currently in `examples/gatekeeper.gif`. 

## Documentation

If the docs are already built (check the `docs/build` directory), simply start a live server to see the docs. From this directory run
```
julia -e 'using LiveServer; serve(dir="docs/build")'
```
Navigate to [http://localhost:8000](http://localhost:8000) to see the documentation. 

To build the docs locally, from the `docs` directory run
```
julia --project make.jl
```
and then use the liveserver to serve the documentation. Building the docs locally will take a bit of time (~3 minutes).


## License

```
Copyright (c) 2025 Devansh R Agrawal. All Rights Reserved.
```