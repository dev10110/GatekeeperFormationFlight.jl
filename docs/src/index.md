```@meta
CurrentModule = GatekeeperFormationFlight
```

# GatekeeperFormationFlight

Documentation for [GatekeeperFormationFlight](https://github.com/dev10110/GatekeeperFormationFlight.jl).

## Installation

First, clone this repo to a desired location. `cd` into that directory.

Run the tests
```julia
] test
```

If you have issues with `Dubins.jl`, install Dev's fork of `Dubins.jl` :
```julia
] rm Dubins
] add http://github.com/dev10110/Dubins.jl
```

## Quickstart

From the `examples` directory, run
```bash
julia --project full_example.md
```

## Documentation

To build the documentation, cd into the `docs` directory, and run
```bash
julia --project make.jl
```

In a separate terminal (from the `docs` directory), start the liveserver:
```bash
julia -e 'using LiveServer; serve(dir="build")'
```
Now go to `localhost:8000` and you should see the docs. 

## Index

```@index
```

