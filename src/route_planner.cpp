#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    RouteModel::Node& start_node = m_Model.FindClosestNode(start_x, start_y);
    RouteModel::Node& end_node = m_Model.FindClosestNode(end_x, end_y);

    this->start_node = &start_node;
    this->end_node = &end_node;

}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*(this->end_node));
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for(auto neighbor : current_node->neighbors)
    {
        if(neighbor->visited)
        {
            continue;
        }
        neighbor->parent = current_node;
        float distance_current_node_neighbor = neighbor->distance(*current_node);
        neighbor->g_value = current_node->g_value + distance_current_node_neighbor;
        neighbor->h_value = CalculateHValue(neighbor);
        neighbor->visited = true;

        this->open_list.push_back(neighbor);
    }
}

RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(this->open_list.begin(), this->open_list.end(), [](const auto& lhs, const auto&rhs){
        return ((lhs->g_value + lhs->h_value) > (rhs->g_value + rhs->h_value));
    });

    RouteModel::Node* next_node = this->open_list.back();
    this->open_list.pop_back();

    return next_node;

}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    path_found.push_back(*current_node);
    RouteModel::Node* node = current_node;
    while(node != start_node)
    {
        path_found.push_back(*(node->parent));
        distance += node->distance(*(node->parent));
        node = node->parent;

    }
    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    start_node->visited = true;

    open_list.push_back(start_node);
    current_node = start_node;
    while(current_node != end_node)
    {
        this->AddNeighbors(current_node);
        current_node = this->NextNode();
    }
    m_Model.path = ConstructFinalPath(current_node);

}