```@meta
CurrentModule = GatekeeperFormationFlight
```

# GatekeeperFormationFlight

Documentation for [GatekeeperFormationFlight](https://github.com/dev10110/GatekeeperFormationFlight.jl).

## Installation

First, clone this repo to a desired location. `cd` into that directory.

Install Dev's fork of `Dubins.jl` :

```julia
] add http://github.com/dev10110/Dubins.jl
```
Run the tests
```julia
] test
```

Build the documentation
```julia
] activate docs/
include("docs/make.jl")
```

In a separate terminal, start the liveserver:
```bash
julia -e 'using LiveServer; serve(dir="docs/build")'
```
Now go to `localhost:8000` and you should see the docs. 

## Modifying

TODO(Dev): add docs on how/where to modify things.

## Index

```@index
```

