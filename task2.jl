using CPLEX
using JuMP

include("read_instance.jl")


####################################### Step 1 ##############################################

mutable struct Graph
    numberNodes::Int64
    successors::Vector{Vector{Int64}}
    predecessors::Vector{Vector{Int64}}
    dist::Matrix{Float64}
end

struct ShortestPathProblem
    startNode::Int64
    destinationNode::Int64
    G::Graph
end

mutable struct Label
    cost::Float64
    origin::Int64
    explored::Bool
end

# #################################################################################
#           findNextNode
#
# Finds the node that has the unexplored label with the smallest cost in the graph.
#
# input:
# - G: a graph
# - L: label for each node (/!\ L[i] is undef if there is no label at node i /!\)
# - hasLabel: hasLabel[i] is true if there is a label at node i, false otherwise
#
# output:
# - the index of the node with the smallest-cost unexplored label
#
# note:
# - returns -1 if there is no unexplored label in the graph
# #################################################################################
function findNextNode(G::Graph, L::Vector{Label}, hasLabel::BitVector)

    # initialize the minimum cost
    minCostLabel = -1
    minCost = 9999999

    # check the label at each node of the graph
    for i in 1:G.numberNodes
        if hasLabel[i] && !L[i].explored && L[i].cost < minCost # there should be a label and it should be unexplored
            minCostLabel = i
            minCost = L[i].cost
        end
    end

    return minCostLabel
end

# #################################################################################
#           testConnectivity
#
# Test wether there exists a path between origin and destination or not in the
# graph G.
#
# input:
# - G: a Graph
# - origin: index of the starting node
# - destination: index of the destination node
#
# output:
# - true if there exists a path bewteen origin and destination, false otherwise
# #################################################################################
function hasPath(G::Graph, origin::Int64, destination::Int64)

    # initialization
    visited = falses(G.numberNodes)
    L = Vector{Int64}(undef,0)

    # setting up the starting point (origin)
    push!(L, origin)
    visited[origin] = true

    # looping through the connected component containing origin
    while length(L) > 0

        # we visit a new node
        node = pop!(L)
        visited[node] = true

        # we put unvisited neighbors in the list of nodes to visit
        for neighbor in G.successors[node]
            if !visited[neighbor]
                push!(L, neighbor)
            end
        end
    end

    # checking if destination was reached
    return visited[destination]
end


# #################################################################################
#           DijsktraAlgorithm
#
# Calls Dijkstra's algorithm on the given instance of the shortest path SPP.
# Displays the optimal solution found at the end.
#
# input:
# - SPP: an instance of the shortest path problem
#
# note:
# - this assumes that there exists a path between the origin and the destination
# - this assumes that there is no negative cycle
# - this assumes that all costs are positive
# #################################################################################
function DijsktraAlgorithm(SPP::ShortestPathProblem)

    G = SPP.G # we extract the graph in a new variable for readability

    # intialization
    L = Vector{Label}(undef, G.numberNodes) # one label per node
    hasLabel = falses(G.numberNodes)

    # setup the starting point
    initialLabel = Label(0, -1, false) # default origin: -1
    hasLabel[SPP.startNode] = true
    L[SPP.startNode] = initialLabel

    # main loop
    while true
        
        # select the open label with the smallest cost
        node = findNextNode(G, L, hasLabel) # return -1 if there is no unexplored label

        # we exit the loop if the destination node was selected
        if node == SPP.destinationNode
            break
        end

        # set the selected label as explored
        currentLabel = L[node]
        currentLabel.explored = true

        # propagate the costs along the arcs
        for neighbor in G.successors[node]

            # if the neighbor has no label, create one
            if !hasLabel[neighbor]
                L[neighbor] = Label(currentLabel.cost + G.dist[node, neighbor], node, false)
                hasLabel[neighbor] = true

            # if there is a label, replace it if its cost is smaller
            elseif currentLabel.cost + G.dist[node, neighbor] < L[neighbor].cost
                L[neighbor] = Label(currentLabel.cost + G.dist[node, neighbor], node, false)
            end
        end
    end

    # display the shortest path's length
    return L[SPP.destinationNode].cost

end


# #################################################################################
#           computeShortestPath
#
# Calculates the shortest distance between each pair of delivery locations
#
#  input:
# - G: a Graph
# - Q: indices of the delivery locations
# - m: number of delivery locations
#
# output:
# - d: Distance matrix of delivery locations (m x m)
# #################################################################################
function computeShortestPath(G::Graph, Q::Vector, m::Int64)
    #initialize a matrix with only zeros
    d = zeros(m,m)

    #fill the matrix with always the smallest distance between each pair of delivery locations
    for i in 1:length(Q) 
        for j in 1:length(Q)
            if i != j
                #create shortest Path problem
                St = ShortestPathProblem(Q[i], Q[j], G)
                # solve the problem via the Dijkstra Algorithm and fill the distance matrix
                d[i,j] = DijsktraAlgorithm(St)
            end
        end 
    end
    #return the distance matrix
    return d
end


########################################### Step 2 ######################################################


# ###################################################################################################
#           buildTSPmodel
#
# Build a JuMP model for the TSP without sub-tour constraints.
#
# input:
# - n: number of nodes
# - d: distance matrix
# - startLocation: starting location for the tour
#
# ouput:
# - JuMP model for the correspond TSP problem
#
# note:
# - there is no sub-tour constraint
# ###################################################################################################
function buildTSPmodel(n::Int64, d::Matrix{Float64}, startLocation::Int64)

    # initialize the model
    model = Model(CPLEX.Optimizer)
    set_silent(model)

    # making the variables
    @variable(model, x[i=1:n,j=1:n], Bin)

    # making the objective function
    @objective(model, Min, sum(d[i,j] * x[i,j] for i in 1:n for j in 1:n if i != j))

    # making the constraints
    @constraint(model, incoming[j=1:n], sum(x[i,j] for i = 1:n if i != j) == 1)
    @constraint(model, outgoing[i=1:n], sum(x[i,j] for j = 1:n if j != i) == 1)

    return model
end

# ###################################################################################################
#           addConnectivityCut
#
# Add the conectivity cut corresponding to the tour provided to the JuMP model.
#
# input:
# - model: JuMP model for the TSP
# - tour: sequence of locations visited
# - n: number of delivery locations
# ###################################################################################################
function addConnectivityCut(model::Model, tour::Vector, n::Int64)

    # check which locations are visited in the tour
    visited = falses(n)
    for location in tour
        visited[location] = true
    end

    # construct the set of unvisited locations
    notInTour = [i for i = 1:n if !visited[i]]

    # add the cut to the model
    @constraint(model, sum(model[:x][i,j] for i in tour for j in notInTour) >= 1)

end

# ###################################################################################################
#           connectivityCutsAlgorithm
#
# Cutting plane algorithm for the TSP using connectivity cuts. Display the optimal solution
# when it is found.
#
# input:
# - model: JuMP model for the TSP
# - startLocation: location where the tour starts
# - n: number of delivery locations
# ###################################################################################################
function connectivityCutsAlgorithm(model::Model, n::Int64, startLocation::Int64, Q::Vector)
    # List to store all subtours
    sub = []
    # Flag to indicate whether subtours are detected
    subTourDetected = true

    # Continue until no subtours are detected
    while subTourDetected
        # Array to track visited nodes
        visited = falses(n)
        # Reset the flag
        subTourDetected = false

        # Iterate over all nodes
        for i in 1:n
            # Check if the node is not visited
            if !visited[i]
                # Compute the tour from this node i
                current_tour = computeTour(model, i)
                # Add the tour to the list of subtours
                push!(sub, current_tour)
                # Mark all nodes in the tour as visited
                for j in current_tour
                    visited[j] = true
                end
                # Check if there are still unvisited nodes
                if any(!, visited)
                    # Set the flag to continue the loop
                    subTourDetected = true
                    # Add a connectivity cut to the model based on the current tour
                    addConnectivityCut(model, current_tour, n)
                end
            end
        end
    end

    println("Total distance traveled: ", objective_value(model))
end

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
    #we always optimize in each subtour we create
    optimize!(model)
    xopt = round.(Int64,value.(model[:x])) # we retrieve the optimal x
    tour = [startLocation] # we initialize the tour with the start location
    
    # iteration 1
    currentLocation = startLocation
    nextLocation = findfirst(isequal(1), xopt[currentLocation,:])
    push!(tour, nextLocation)

    # fill the rest of the tour
    while nextLocation != startLocation
        currentLocation = nextLocation
        nextLocation = findfirst(isequal(1), xopt[currentLocation,:])
        push!(tour, nextLocation)
    end

    return tour
end


# main script 

#read in data
n, m, Q, arcs, costs = read_instance(string("instances/", "inst_n-20_m-5_1.txt"))

#initialize predecessors and successors
predecessors, successors = create_pre_succesors(n, arcs)
#set startLocation
startLocation = Q[1]
#create Graph regarding our data
G = Graph(n, successors, predecessors, costs)

#compute shortest paths between each pair of delivery location.
d = computeShortestPath(G, Q,m)

#build model
tsp = buildTSPmodel(m, d, startLocation)

# call to the cutting plane algorithm
connectivityCutsAlgorithm(tsp, m, startLocation, Q)

