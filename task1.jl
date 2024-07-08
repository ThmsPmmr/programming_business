# Hardcode für die gegebenen Daten
using Random
using DelimitedFiles
using CPLEX
using JuMP

# #read the instance
# function read_instance_data(file_path)

#     # Lesen des gesamten Inhalts des Files
#     file_content = readdlm(file_path, ' ')

#     # extracts the number of nodes and number of delievery points
#     num_nodes = file_content[1, 1]
#     num_del_points = file_content[2, 1]

#     #get number of rows in instance
#     n= size(file_content, 1)

#     #initialise arrays
#     index_del_points = Array{Int64}(undef, num_del_points)
#     arcs = Array{Float64}(undef, (n-3), 3)

#     #fill the index points 
#     for i in 1:num_del_points
#         index_del_points[i] = file_content[3, i]
#     end

#     #fill the arcs 
#     for i in 1:n-3
#         for j in 1:3
#             arcs[i, j] = file_content[3 + i, j]
#         end
#     end

#     return num_nodes, num_del_points, index_del_points, arcs
# end

function read_instance(file_path)
    open(file_path, "r") do f
        n = parse(Int64, readline(f))
        m = parse(Int64, readline(f))
        delivery_points = parse.(Int64, split(readline(f)))
        arcs = Vector{Vector{Int64}}()
        costs = zeros(Float64, n, n)
        while !eof(f)
            linei = split(readline(f))
            i  = parse(Int64, linei[1])
            j  = parse(Int64, linei[2])
            c  = parse(Float64, linei[3])
            push!(arcs, [i,j])
            costs[i, j] = c
        end
        
        close(f)
    return n, m, delivery_points, arcs, costs
    end
end

#testing the task on this file
file_path = "instances/inst_n-20_m-5_1.txt"
test_file = read_instance(file_path)

###################starting with task 1################################

#getting the data needed in order to solve this constraint

n, m, Q, arcs, costs = read_instance(file_path)
#generating the data needed to solve the problem
predecessors = Vector{Vector{Int64}}(undef,n)
successors = Vector{Vector{Int64}}(undef, n)
for i in 1:n
    predecessors[i] = Vector{Int64}(undef, 0)
    successors[i] = Vector{Int64}(undef, 0)
end


for arc in arcs
    pre, suc = arc
    push!(predecessors[suc], pre)
    push!(successors[pre], suc)
end

function solve_ILP(n::Int64, Q::Vector{Int64}, arcs::Vector{Vector{Int64}}, pred::Vector{Vector{Int64}}, succ::Vector{Vector{Int64}}, costs::Matrix{Float64})

    #create model
    model = Model(CPLEX.Optimizer)
    set_silent(model)
    #Decision Variable
    @variable(model, x[1:n, 1:n] >= 0, Int)#here we already fullfill constraint 5

    #objective function
    #we want to minimize the total costs
    @objective(model, Min, sum(costs[i,j] * x[i,j] for arc in arcs for (i, j) in [(arc[1], arc[2])]))
    
    #each node has to be visited as often as i is left
    @constraint(model, [v=1:n], sum(x[i, v] for i in pred[v]) == sum(x[v, j] for j in succ[v]))
    
    # each location which has to be visited is visited at leaste once
    @constraint(model, [v in Q], sum(x[i, v] for i in pred[v]) >= 1)

    #optimize!(model)

    # termination_status = JuMP.termination_status(model)
    # println("Termination status: ", termination_status)

    # if termination_status != MOI.OPTIMAL
    #     println("The solver did not find an optimal solution.")
    #     return nothing
    # end
    # print(objective_value(model))
  return model
end
solve_ILP(n, Q, arcs, predecessors, successors, costs)

# ###################################################################################################
#           computeTour
#
# Compute the tour starting at startLocation in the optimal solution of the model.
#
# input:
# - model: JuMP model for the TSP
# - startLocation: location where the tour starts
#
# output:
# - Sequence of locations visited.
# ###################################################################################################

function computeTour(model::Model, startLocation::Int64)
    optimize!(model)
    xopt = round.(Int64,value.(model[:x])) # we retrieve the optimal x
    tour = [startLocation] # we initialize the tour with the start location
    
    # iteration 1
    currentLocation = startLocation
    nextLocation = findfirst(!isequal(0), xopt[currentLocation,:])
    push!(tour, nextLocation)

    # fill the rest of the tour
    while nextLocation != startLocation
        currentLocation = nextLocation
        nextLocation = findfirst(!isequal(0), xopt[currentLocation,:])
        push!(tour, nextLocation)
    end

    return tour
end

function connect_solution(model::Model, Q::Vector{Int64}, n::Int64, succ::Vector{Vector{Int64}})
    #solve the problem
    print("Optimiiiiiiiiiiiiert!")
    all_tours = Vector{Vector{Int64}}()
    subtour_detected = true
    while subtour_detected
        #set all nodes to unvisited again 
        visited = falses(n)
        subtour_detected = false
        for i in Q
            if !visited[i]
                sub = computeTour(model, i)
                push!(all_tours, sub)
                for j in sub[1:end-1]
                    visited[j] = true
                end
                if !all(in(sub), Q)
                     @constraint(model, [i in sub[1:end-1]], sum(model[:x][i, j] for j in succ[i] if !(j in sub[1:end-1])) >= 1) 
                     subtour_detected = true
                end
            end
        end
        
    end
    
    return all_tours[end]
end
model = solve_ILP(n, Q, arcs, predecessors, successors, costs)
current_solution = connect_solution(model, Q, n, successors)

