#include "route_planner.h"
#include <algorithm>
using std::sort;

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);

}


// Implement the CalculateHValue method.


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
  
  return node ->distance(*end_node);

}


// AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {

    current_node ->FindNeighbors();

    for (auto node : current_node->neighbors){

      node->parent = current_node;
      node->h_value = CalculateHValue(node);
      node->g_value = current_node->g_value + current_node->distance(*node);

      node->visited = true;
      open_list.push_back(node);
    }
}


// Complete the NextNode method to sort the open list and return the next node.

bool CompareF(const RouteModel::Node* a, const RouteModel::Node* b){

int f1 = a->g_value + a->h_value;
int f2 = b->g_value + b->h_value;
return f1 > f2;
}


RouteModel::Node *RoutePlanner::NextNode() {

    sort(this->open_list.begin(), this->open_list.end(), CompareF);
    RouteModel::Node *lowest_pointer = this->open_list.back();
    this->open_list.pop_back();
    return lowest_pointer;
}


// Complete the ConstructFinalPath method to return the final path found from your A* search.
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    std::cout << "\nPathSize: " << path_found.size();
    RouteModel::Node *temp = current_node;

    while( temp->parent!= nullptr ){

        path_found.insert(path_found.begin(), *(temp));
        distance += (temp->distance(*temp->parent));
        temp = temp->parent;
        std::cout << "\nPathSize: " << path_found.size();
    }

    path_found.insert(path_found.begin(), *(temp));

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}


// A* Search algorithm.
void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    this->open_list.push_back(this->start_node);
    current_node = start_node;
    current_node ->visited = true;
    while(this->open_list.size() > 0){

      current_node = NextNode();  
      
      if ( (current_node->x == end_node->x) && 
           (current_node->y == end_node->y)){
        m_Model.path = ConstructFinalPath(current_node);
        return;
      }
      AddNeighbors(current_node);
    }
}
