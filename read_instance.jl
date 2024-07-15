
using DelimitedFiles

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

function create_pre_succesors(n::Int64, arcs::Vector{Vector{Int64}})
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
    return predecessors, successors
end


#generating the data needed to solve the problem
