module RRTStar

abstract type AbstractProblem{T} end

"""
    Node{T}(state::T, parent_index::Int64, incremental_cost::Float64)

Defines a node in the RRT* Tree, with a defined `state`, `parent_index`, and the `incremental_cost` from the parent to the current node. 
Since the nodes are stored in a vector, the root node should have a `parent_index=0`.
"""
mutable struct Node{T}
    state::T
    parent_index::Int64
    incremental_cost::Float64
end   

Node(state) = Node(state, 0, 0.)

"""
    cost(i::Int64, nodes::Vector{Node})

returns the cost to get to node `i`. Computes this quantity recursively. 
"""
function cost(i::Int64, nodes::Vector{Node{T}}) where {T}
    if i == 0
        return 0.0
    else
        i_parent = nodes[i].parent_index
        incremental_cost = nodes[i].incremental_cost
        return cost( i_parent, nodes) + incremental_cost
    end
end

"""
    rrt_star(problem::AbstractProblem, nodes::Vector{Node}, max_iters::Int64; do_rewire=true)

Adds `max_iters` nodes to the RRT* star. Will modify the `nodes` vector.
"""
function rrt_star(problem, nodes, max_iters; do_rewire=true)

    # i -> index in nodes vector
    # x -> actual state
    # n -> full node
    
    for iter = 1:max_iters
        # grab a random state
        x_rand = sample_free(problem)

        # find the nearest node
        i_nearest = nearest(problem, nodes, x_rand)
        n_nearest = nodes[i_nearest]
        x_nearest = n_nearest.state

        # get the new point
        x_new = steer(problem, x_nearest, x_rand)

        # check that it is obstacle free
        if collision_free(problem, x_nearest, x_new)

            # get the set of nearby nodes (this should return an index set)
            I_near = near(problem, nodes, x_new) # todo - set a max radius

            # determine the best one to connect to 
            i_min = i_nearest
            x_min = nodes[i_min].state
            c_min = cost(i_nearest, nodes) + path_cost(problem, x_nearest, x_new)
            for i_near in I_near
                
                x_near = nodes[i_near].state
                c_near = cost(i_near, nodes) + path_cost(problem, x_near, x_new)
                
                if ( c_near < c_min ) && collision_free(problem, x_near, x_new)
                    i_min = i_near
                    x_min = x_near
                    c_min = c_near
                end
            end

            # add in the new edge
            n_new = Node(x_new, i_min, path_cost(problem, x_min, x_new))
            push!(nodes, n_new)

            # rewire
            if do_rewire
                i_new = length(nodes)
                for i_near in I_near
                    if nodes[i_near].parent_index != 0
                        x_near = nodes[i_near].state
                        pc = path_cost(problem, x_new, x_near) 
                        if (cost(i_new, nodes) + pc < cost(i_near, nodes) ) && collision_free(problem, x_new, x_near)
                            # change the parent of i_near to i_new
                            nodes[i_near].parent_index = i_new
                            nodes[i_near].incremental_cost = pc
                        end
                    end
                end
            end
        end

    end

    return nodes
end

"""
    get_best_path(problem::AbstractProblem, nodes::Vector{Node{T}}, x_goal::T)

returns the `(true, best_path)` if a path to `x_goal` is found. 
returns `(false, nothing)` if no path to `x_goal` is found. 

The best path is a list of nodes from the root to a node on the tree. returns the states along the nodes. 
"""
function get_best_path(problem::P, nodes::Vector{Node{T}}, x_goal) where {T, P <: AbstractProblem{T}}

    # for each node, try to connect it to the goal
    best_cost = Inf
    best_node = -1
    for i=1:length(nodes)
        node = nodes[i]
        node_cost = cost(i, nodes)
        incremental_cost = path_cost(problem, node.state, x_goal)

        if  node_cost + incremental_cost < best_cost
            # check if the path was collision free
            if collision_free(problem, node.state, x_goal)
                best_cost = node_cost + incremental_cost
                best_node = i
            end
        end
    end

    if best_node == -1
        return false, nothing
    else
        node = nodes[best_node]
        path = [x_goal, node.state]
        while node.parent_index != 0
            node = nodes[node.parent_index]
            push!(path, node.state)
        end
        return true, reverse(path)
    end
end

# the following functions need to be defined for your problem
function sample_free(problem::P) where {T, P <: AbstractProblem{T}}
    throw(MethodError(sample_free, (problem, )))
end

function nearest(problem::P, nodes, x_rand) where {T, P<: AbstractProblem{T}}
    throw(MethodError(nearest, (problem, nodes, x_rand)))
end

function near(problem::P, nodes,  x_new) where {T, P<: AbstractProblem{T}}
    throw(MethodError(near, (problem, nodes, x_new)))
end

function steer(problem::P, x_nearest, x_rand) where {T, P<: AbstractProblem{T}}
    throw(MethodError(steer, (problem, x_nearest, x_rand)))
end

function collision_free(problem::P, x_nearest, x_new) where {T, P<: AbstractProblem{T}}
    throw(MethodError(collision_free, (problem, x_nearest, x_new)))
end

function path_cost(problem::P, x_near, x_new) where {T, P <: AbstractProblem{T}}
    throw(MethodError(path_cost, (problem, x_near, x_new)))
end


end





