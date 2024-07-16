# Hardcode für die gegebenen Daten

using CPLEX
using JuMP
include("read_instance.jl")

# #################################################################################
#           solve_ILP
#
# Creates the integer linear programming model for task 1 without constraint 4
#
# input:
# - n: number of nodes
# - Q: delievery points
# - arcs: the possible connections of the nodes which can be used
# - pred: the predecessors of each node (one vector per node)
# - succ: the successors of each node (one vector per node)
# - costs: the costs for each arc
#
# output:
# - an integer linear model
# #################################################################################


function solve_ILP(n::Int64, Q::Vector{Int64}, arcs::Vector{Vector{Int64}}, pred::Vector{Vector{Int64}}, succ::Vector{Vector{Int64}}, costs::Matrix{Float64})

    #create model
    model = Model(CPLEX.Optimizer)
    set_silent(model)
    #Decision Variable
    @variable(model, x[1:n, 1:n] >= 0, Int)#here we already fullfill constraint 5

    #objective function
    #we want to minimize the total costs
    @objective(model, Min, sum(costs[i,j] * x[i,j] for (i, j) in arcs))
    
    #each node i has to be visited as often as i is left
    @constraint(model, [v=1:n], sum(x[i, v] for i in pred[v]) == sum(x[v, j] for j in succ[v]))
    
    # each location which has to be visited is visited at leaste once
    @constraint(model, [v in Q], sum(x[i, v] for i in pred[v]) >= 1)

  return model
end

# #################################################################################
#           find_all_tours
#
# finds all subtours of the current model solution
#
# input:
# - x: decision variables of the current model
# - n: number of nodes
# - Q: delievery points
#
# output:
# - all subtours
# #################################################################################

function find_all_tours(x::Matrix{Float64}, n::Int64, Q::Vector{Int64})
    #create a function which finds all nodes of the current subtours
    # #################################################################################
    #           find_connections
    #
    # looks for all connections of the visited nodes in the current solution
    #
    # input:
    # - c: the node we currently look at
    # - subtour: the subtour where the current node is in
    # - visited: the vector which describes if a node is already visited
    # - x: decision variables of the current model
    #
    # updates:
    # - the nodes get pushed to the subtours
    # - visited is set to true for the node which is currently looked at
    # #################################################################################
    function find_connections(c::Int64, subtour::Vector{Int64}, visited::BitVector, x::Matrix{Float64})
        push!(subtour, c)
        visited[c] = true
        for con in 1:n
            if !visited[con] && value(x[c,con]) > 0.5
                find_connections(con, subtour, visited, x)
            end
        end        
    end
    #now lets look for the subtours we currently have
    all_components = Vector{Vector{Int64}}()
    visited = falses(n)
    #decided to only iterate through delievery points as there are no tours without a node of the delievery points as this would just increase costs
    #but I know if I would exactly look at the explanation of step 3 it would have to be 
    #for u in 1:n
    for u in Q
        if !visited[u] # && any(used_arcs[u, :]) - we would need this if we iterate from 1:n so that it is more efficient
            #for all locations with connections in the current solution we compute the subtours
            subtour = Vector{Int64}(undef,0)
            find_connections(u, subtour, visited, x)
            push!(all_components, subtour)
        end
    end
    #we return the vector of all subtours which are also stored in a vector
    return all_components
end

# #################################################################################
#           connect_solution
#
# finds all subtours of the current model solution
#
# input:
# - model: the model we get from using solve_ILP
# - Q: delievery points
# - n: number of nodes
# - succ: The vectors of successors for each node
#
# output:
# - prints out the objective value
# - we could also return the decision variable x but is not needed in this case
# #################################################################################

#now we need to connect the subtours to one tour which delievers all delievery points
function connect_solution(model::Model, Q::Vector{Int64}, n::Int64, succ::Vector{Vector{Int64}})
    
    subtour_detected = true
    #as long as there is no tour which contains all delievery points we need to add connectivity cuts
    while subtour_detected
        #we optimize the model every time all connectivity cuts for all subtours had been added
        optimize!(model)
    
        xopt = value.(model[:x])
        #we need to find all current tours
        all_tours = find_all_tours(xopt, n, Q)

        #now we either iterate through all subtours or until one subtour contains all delievery points
        for subtour in all_tours
            #check if all delievery points are in the current subtour
            if !all(in(subtour), Q) && subtour_detected
                @constraint(model, sum(model[:x][i, j] for i in subtour for j in succ[i] if !(j in subtour[1:end])) >= 1) 
                subtour_detected = true
            else subtour_detected = false
            end
        end
        
    end
    #we again optimize the model to get the objective value and fullfill all constraints
    #optimize!(model)
    println("Total travelling distance: ", objective_value(model))
    
    #return(value.(model[:x]))
end
# n, m, Q, arcs, costs = read_instance(string("instances/", "inst_n-20_m-5_1.txt"))
# predecessors, successors = create_pre_succesors(n, arcs)
# model = solve_ILP(n, Q, arcs, predecessors, successors, costs)

# println(Q)
# current_solution = connect_solution(model, Q, n, successors)

