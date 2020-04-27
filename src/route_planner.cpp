#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}

// h-value by checking the distance from the current node to the end-node
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}

// Expand the current node by adding all unvisited neighbors to the open list.
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for (RouteModel::Node* neighbor: current_node->neighbors) {
        neighbor->parent = current_node;
        neighbor->g_value = current_node->g_value + neighbor->distance(*current_node);
        neighbor->h_value = CalculateHValue(neighbor);
        open_list.emplace_back(neighbor);
        neighbor->visited = true;
    }
}

// Gets the next best node (accoding to heuristic) to expand to
RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(open_list.begin(), open_list.end(), [](const RouteModel::Node *a, const RouteModel::Node *b) {
        return (a->g_value + a->h_value) > (b->g_value + b->h_value);
    });

    RouteModel::Node *next_node = open_list.back();
    open_list.pop_back();
    return next_node;
}

// Return the final path found from your A* search (required to print the path in the map tile)
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    while(current_node != start_node) {
        path_found.push_back((*current_node));
        distance += current_node->distance(*current_node->parent);
        current_node = current_node->parent;
    }
    path_found.push_back((*start_node));
    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}

// Search until end node is found using A* path-finding algorithm
void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    AddNeighbors(start_node);
    while (open_list.size() > 0) {
        current_node = NextNode();
        if (current_node == end_node) {
            m_Model.path = ConstructFinalPath(current_node);
            return;
        }
        AddNeighbors(current_node);
    }
}