% dijkstra_shortest_path.m
% Solves the shortest path problem using Dijkstra's Algorithm.
% Returns shortest distance to each node in a distances vector. 
% Returns the path to the destination / end node in a path vector. 

function dijkstra_shortest_path
clc
disp('Graph 1')
% adjacency matrix containing distances from node to node
graph = [0 4 3 6 0 0 0 0 0 0 0;
         4 0 0 5 3 0 0 0 0 0 0;
         3 0 0 4 0 6 0 0 0 0 0;
         6 5 4 0 2 5 2 0 0 0 0;
         0 3 0 2 0 0 2 4 0 0 0;
         0 0 6 5 0 0 1 0 2 5 0;
         0 0 0 2 2 1 0 2 5 0 0;
         0 0 0 0 4 0 2 0 2 0 7;
         0 0 0 0 0 2 5 2 0 3 8;
         0 0 0 0 0 5 0 0 3 0 4;
         0 0 0 0 0 0 0 7 8 4 0;
        ];
% cell array containing the neighbors of each node
neighbors = {[2,3,4],[1,4,5],[1,4,6],[1,2,3,5,6,7],[2,4,7,8],[3,4,7,9,10],...
             [4,5,6,8,9],[5,7,9,11],[6,7,8,10,11],[6,9,11],[8,9,10]};
origin = 1;  % start node 
destination = 11;  % end node 
[distances,path]=dijkstra(graph,neighbors,origin,destination);
disp('Distances Vector: Shortest distance to each node')
disp(distances)
disp('Path Vector: Path to traverse the graph')
disp(path)
disp('')

disp('Graph 2')
% adjacency matrix
graph = [0 2 7 0 0 0 0
         2 0 4 8 10 0 0
         7 4 0 1 0 7 0
         0 8 1 0 1 0 3
         0 10 0 1 0 5 1
         0 0 7 0 5 0 5
         0 0 0 3 1 5 0
        ];
% cell array containing the neighbors of each node
neighbors = {[2,3],[1,3,4,5],[1,2,4,6],[2,3,5,7],[2,4,6,7],[3,5,7],[4,5,6]};
origin = 1;  % start node 
destination = 7;  % end node 
[distances,path]=dijkstra(graph,neighbors,origin,destination);
disp('Distances Vector: Shortest distance to each node')
disp(distances)
disp('Path Vector: Path to traverse the graph')
disp(path)
end 

function [distances,path]=dijkstra(graph,neighbors,origin,destination)
% Calculates the shortest distance to each node and returns the path for
% the selected node (destination). 
% Inputs: graph, neighbors, origin, destination
% Outputs: distances, path: 1xn matrix, 1xn matrix
n = size(graph,1);  % number of nodes
d = inf(1,n);  % initialize to infinity (distance not yet known)
d(origin) = 0;  % distance from origin to itself is 0
S = zeros(1,n);  % set of solved nodes, in order
closest = origin;  % the current node
count = 0;
while count < n
    % update the set of solved nodes according to i, the current node
    % set temporary variable A to the neighbors of i
    % d(i) = distance of current node (from origin)
    % i = current node 
    % j = neighbor of current node 
    % iterate over A 
        % take the minimum of the existing distance and the distance of the
        % current node plus distance from current node to the neighbor
        % Ex. min(inf, d(i) + graph(i,j))
        % set distance of j to resulting value. 
        % out of the neighbors which are not in the set of solved nodes,
        % find the closest neighbor. 
        % set i to this neighbor. 
    i = closest;
    count = count + 1;  % increment the counter for the while loop
    S(count) = i;  % update set of solved nodes 
    A = neighbors{i};  % temporary variable for neighbors of i
    unsolved = setdiff(A,S);  % unsolved nodes
    shortest_distance = inf;
    for j = 1:length(unsolved)
        neighbor = unsolved(j);
        d(neighbor) = min([d(neighbor) d(i) + graph(i,neighbor)]);
        if d(neighbor) < shortest_distance
            closest = neighbor;
            shortest_distance = d(neighbor);
        end
    end
end
% all nodes have now been solved and shortest distances have been found
distances = d;
path = S;
end 
