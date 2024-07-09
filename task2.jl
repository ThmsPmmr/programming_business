# Hardcode für die gegebenen Daten
using Random
using DelimitedFiles
using CPLEX
using JuMP

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


file_path = "instances/inst_n-20_m-5_1.txt"


n, m, Q, arcs, costs = read_instance(file_path)




####################################### Step 1 ##############################################


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

G = Graph(n, successors, predecessors, costs)


function computeShortestPath(G::Graph, Q::Vector, m::Int64)
    d = zeros(m,m)
    for i in 1:length(Q) 
        for j in 1:length(Q)
            if i != j
                St = ShortestPathProblem(Q[i], Q[j], G)
                d[i,j] = DijsktraAlgorithm(St)
            end
        end 
    end
    return d
end


d = computeShortestPath(G, Q,m)


########################################### Step 2 ######################################################

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

    # display results
    deliveryTour = Vector{Int64}(undef,0)

    for node in sub[end]
        
        push!(deliveryTour, Q[node])
        
    end

    println(deliveryTour)
    println("Total distance traveled: ", objective_value(model))
end

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

startLocation = m[1]

tsp = buildTSPmodel(m, d, startLocation)

# call to the cutting plane algorithm
@time connectivityCutsAlgorithm(tsp, m, startLocation, Q)

