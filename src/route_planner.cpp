#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    //  m_Model.FindClosestNode method is used to find the closest nodes to the starting and ending coordinates.
    start_node = &m_Model.FindClosestNode(start_x, start_x);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}


//  the CalculateHValue method.
// 
// - The distance to the end_node for the h value is used.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}


//  AddNeighbors method expands the current node by adding all unvisited neighbors to the open list.
// 
// - FindNeighbors() method of the current_node is used to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, the parent, the h_value, the g_value are set. 
// - CalculateHValue is used to implement the h-Value calculation.
// - For each node in current_node.neighbors, the neighbor is added to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for (auto &node : current_node->neighbors){
        node->parent = current_node;
        node->h_value = CalculateHValue(node);
        node->g_value = current_node->g_value + node->distance(*current_node);
        open_list.push_back(node);
        node->visited = true;
    }
}


//  5: NextNode method sorts the open list and return the next node.
// 
// - open_list is sorted according to the sum of the h value and g value.
// - A pointer to the node in the list with the lowest sum is created.
// - That node is returned from the open_list.
// - Return the pointer.

RouteModel::Node *RoutePlanner::NextNode() {
    // A lambda expression that compares two nodes based on their f value (f = g + h). 
    auto cmp_by_f = [](RouteModel::Node* node_lhs, RouteModel::Node* node_rhs){ 
        return node_lhs->g_value + node_lhs->h_value < node_rhs->g_value + node_rhs->h_value; };

    std::sort(this->open_list.begin(), this->open_list.end(), cmp_by_f);
    RouteModel::Node *next_node = *open_list.begin();
    open_list.erase(open_list.begin());
    return next_node;
}


// ConstructFinalPath method returns the final path found from A* search.
// 
// - This method takes the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector is in the correct order: the start node is the first element
//   of the vector, the end node is the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    while(current_node->parent != nullptr){
        distance += current_node->distance(*current_node->parent);
        path_found.push_back(*current_node);
        current_node = current_node->parent;
    }
    path_found.push_back(*current_node);

    std::reverse(path_found.begin(), path_found.end());
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


//  A* Search algorithm.
// 
// - AddNeighbors method is used to add all of the neighbors of the current node to the open_list.
// - NextNode() method is used to sort the open_list and return the next node.
// - When the search has reached the end_node, ConstructFinalPath method is used to return the final path that was found.
// - final path is stored in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    current_node = start_node;
    current_node->visited = true;
    open_list.push_back(current_node);

    while (open_list.size() > 0){
        current_node = NextNode();
        if (current_node == end_node){
            break;
        }
        AddNeighbors(current_node);
    }
    auto final_path = ConstructFinalPath(current_node);
    m_Model.path = final_path;

}