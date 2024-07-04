# # Hardcode für die gegebenen Daten
# using Random
# using DelimitedFiles

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

n, m, Q, W = read_instance_data(file_path)
V = 1:n
W[3,3]



