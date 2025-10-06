#include <iostream>
#include <vector>
#include <queue>
#include <climits>
#include <cmath>
#include <algorithm>

using namespace std;

// Structure to represent a node in the graph with its position
struct Node {
    int id;           // Node identifier
    int x, y;         // Coordinates for heuristic calculation
};

// Structure to represent a state in the priority queue
struct State {
    int node;         // Current node
    int g_cost;       // Cost from start to current node
    int f_cost;       // Total cost (g_cost + heuristic)
    
    // Constructor for easier initialization
    State(int n, int g, int f) : node(n), g_cost(g), f_cost(f) {}
    
    // Comparison operator for priority queue (min-heap based on f_cost)
    bool operator>(const State& other) const {
        return f_cost > other.f_cost;
    }
};

// Define a pair for adjacency list (weight, neighbor_node)
typedef pair<int, int> pii;

// Function to calculate Manhattan distance heuristic
int calculateHeuristic(const Node& current, const Node& goal) {
    return abs(current.x - goal.x) + abs(current.y - goal.y);
}

// Function to reconstruct the path from start to goal
void reconstructPath(vector<int>& parent, int start, int goal) {
    vector<int> path;
    int current = goal;
    
    // Backtrack from goal to start using parent array
    while (current != -1) {
        path.push_back(current);
        current = parent[current];
    }
    
    // Reverse the path to get start to goal order
    reverse(path.begin(), path.end());
    
    // Print the path
    cout << "Path from " << start << " to " << goal << ": ";
    for (int i = 0; i < path.size(); i++) {
        cout << path[i];
        if (i < path.size() - 1) cout << " -> ";
    }
    cout << endl;
}

// Function to implement A* algorithm
void aStar(int start, int goal, vector<vector<pii>>& graph, vector<Node>& nodes, int V) {
    // Create a g_cost vector initialized to infinity (cost from start to each node)
    vector<int> g_cost(V, INT_MAX);
    
    // Create a parent vector to reconstruct the path
    vector<int> parent(V, -1);
    
    // Create a closed set to track visited nodes
    vector<bool> closed(V, false);
    
    // Priority queue to store states (min-heap based on f_cost)
    priority_queue<State, vector<State>, greater<State>> open_set;
    
    // Initialize the start node
    g_cost[start] = 0;
    int h_cost = calculateHeuristic(nodes[start], nodes[goal]);
    open_set.push(State(start, 0, h_cost));
    
    cout << "Starting A* algorithm from node " << start << " to node " << goal << "...\n\n";
    
    // Main A* loop
    while (!open_set.empty()) {
        // Get the node with the lowest f_cost
        State current = open_set.top();
        open_set.pop();
        
        int current_node = current.node;
        
        // Skip if this node is already in closed set
        if (closed[current_node]) continue;
        
        // Mark current node as visited
        closed[current_node] = true;
        
        // Check if we reached the goal
        if (current_node == goal) {
            cout << "Goal reached!\n";
            cout << "Total cost: " << g_cost[goal] << "\n\n";
            reconstructPath(parent, start, goal);
            return;
        }
        
        // Explore all neighbors of the current node
        for (auto& edge : graph[current_node]) {
            int neighbor = edge.second;
            int edge_weight = edge.first;
            
            // Skip if neighbor is already processed
            if (closed[neighbor]) continue;
            
            // Calculate tentative g_cost for the neighbor
            int tentative_g_cost = g_cost[current_node] + edge_weight;
            
            // If we found a better path to the neighbor
            if (tentative_g_cost < g_cost[neighbor]) {
                // Update the g_cost and parent
                g_cost[neighbor] = tentative_g_cost;
                parent[neighbor] = current_node;
                
                // Calculate f_cost (g_cost + heuristic)
                int h_cost = calculateHeuristic(nodes[neighbor], nodes[goal]);
                int f_cost = tentative_g_cost + h_cost;
                
                // Add neighbor to open set
                open_set.push(State(neighbor, tentative_g_cost, f_cost));
            }
        }
    }
    
    // If we exit the loop without finding the goal
    cout << "No path found from " << start << " to " << goal << endl;
}

int main() {
    int V, E, u, v, w;
    
    // Input number of vertices and edges
    cout << "Enter number of vertices and edges: ";
    cin >> V >> E;
    
    // Create a graph as an adjacency list
    vector<vector<pii>> graph(V);
    
    // Create a vector to store node positions
    vector<Node> nodes(V);
    
    // Input node positions for heuristic calculation
    cout << "Enter coordinates for each node (x y):\n";
    for (int i = 0; i < V; i++) {
        cout << "Node " << i << ": ";
        cin >> nodes[i].x >> nodes[i].y;
        nodes[i].id = i;
    }
    
    // Input all edges
    cout << "Enter edges (u v w) where u and v are vertices and w is the weight:\n";
    for (int i = 0; i < E; i++) {
        cin >> u >> v >> w;
        graph[u].push_back({w, v});
        graph[v].push_back({w, u}); // If the graph is undirected
    }
    
    int start, goal;
    cout << "Enter the start vertex: ";
    cin >> start;
    cout << "Enter the goal vertex: ";
    cin >> goal;
    
    // Call A* algorithm
    aStar(start, goal, graph, nodes, V);
    
    return 0;
}
