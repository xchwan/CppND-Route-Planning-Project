#include "route_planner.h"
#include <algorithm>
#include <vector>
#include <iostream>

using std::cout;
using std::endl;

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}



float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}


void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for(RouteModel::Node *neighbor : current_node->neighbors){
        neighbor->parent = current_node;
        float g = current_node->g_value;
        float d = current_node->distance(*neighbor);
        neighbor->g_value = g + d;
        neighbor->h_value = CalculateHValue(neighbor);
        open_list.push_back(neighbor);
        neighbor->visited = true;
    }
}




RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(open_list.begin(), open_list.end(),[](auto const &a, auto const &b) {
                  return (a->g_value + a->h_value) > (b->g_value + b->h_value);
              });
    open_list.shrink_to_fit();
    auto node = open_list.back();
    open_list.pop_back();
    return node;
}




std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here.
    while (current_node->parent != nullptr) {
        path_found.insert(path_found.begin(), *current_node);
        distance += current_node->distance(*current_node->parent);
        current_node = current_node->parent;
    }
    path_found.insert(path_found.begin(), *start_node);

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}




void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
    
    // TODO: Implement your solution here.
    start_node->visited = true;
    open_list.push_back(start_node);
    while (open_list.size() > 0) {
        current_node = NextNode();
        if (current_node->distance(*end_node) == 0) {
            m_Model.path = ConstructFinalPath(current_node);
            return;
        } else {
            AddNeighbors(current_node);
        }
    }

}