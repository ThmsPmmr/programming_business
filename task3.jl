#task 3
#want to observe the average time used for the cutting plane which in this case is the connect solution method
include("read_instance.jl")
include("task1.jl")
include("task2.jl")
directory = "instances/"
filenames = readdir(directory)

### Average Time for task 1
function calc_comp_time_task1(instances::Vector{String} )
    total_time = 0.0
    for file in instances
        n, m, Q, arcs, costs = read_instance(string(directory, file))
        predecessors, successors = create_pre_succesors(n, arcs)

        model = solve_ILP(n, Q, arcs, predecessors, successors, costs)
        t = @elapsed connect_solution(model, Q, n, successors)
        total_time += t
    end
    return total_time / length(instances)
end

    
avg_20_inst_task1 = calc_comp_time_task1(filenames[1:10])
avg_35_inst_task1 = calc_comp_time_task1(filenames[11:20])
avg_50_inst_task1 = calc_comp_time_task1(filenames[21:30])

### average time for task 2
function calc_comp_time_task2(instances::Vector{String} )
    total_time = 0.0
    for file in instances
        n, m, Q, arcs, costs = read_instance(string(directory, file))
        predecessors, successors = create_pre_succesors(n, arcs)
        startLocation = Q[1]
        G = Graph(n, successors, predecessors, costs)
        t1 = @elapsed d = computeShortestPath(G, Q, m)
        t2 = @elapsed tsp = buildTSPmodel(m, d, startLocation)
        t3 = @elapsed connectivityCutsAlgorithm(tsp, m, startLocation, Q)
        total_time += t1 + t2 + t3
    end
    return total_time / length(instances)
end

    
avg_20_inst_task1 = calc_comp_time_task2(filenames[1:10])
avg_35_inst_task1 = calc_comp_time_task2(filenames[11:20])
avg_50_inst_task1 = calc_comp_time_task2(filenames[21:30])