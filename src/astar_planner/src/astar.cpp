#include "astar_planner/astar.hpp"
#include <iostream>
#include <limits>

namespace astar_planner
{

AStar::AStar()
: map_width_(0), map_height_(0)
{
}

AStar::~AStar()
{
}

void AStar::setMap(const std::vector<std::vector<int>>& map)
{
  map_ = map;
  if (!map_.empty()) {
    map_height_ = map_.size();
    map_width_ = map_[0].size();
  }
}

double AStar::calculateHeuristic(const GridCell& a, const GridCell& b) const
{
  //Euclidean distance
  double x_pow = static_cast<double>(pow(a.x - b.x, 2));
  double y_pow = static_cast<double>(pow(a.y - b.y, 2));
  double euclidean = sqrt(x_pow + y_pow);

  return euclidean;
}

bool AStar::isValid(const GridCell& cell) const
{
  if (cell.x < 0 || cell.x >= map_width_ || cell.y < 0 || cell.y >= map_height_) {
    return false;
  }
  // 0 means free space, 1 means obstacle
  return map_[cell.y][cell.x] == 0;
}

std::vector<GridCell> AStar::getNeighbors(const GridCell& cell) const
{
  std::vector<GridCell> neighbors;
  
  // 8-connected grid: up, down, left, right, and 4 diagonals
  std::vector<std::pair<int, int>> directions = {
    {0, 1},   // up
    {0, -1},  // down
    {1, 0},   // right
    {-1, 0},  // left
    {1, 1},   // up-right
    {1, -1},  // down-right
    {-1, 1},  // up-left
    {-1, -1}  // down-left
  };
  
  for (const auto& dir : directions) {
    GridCell neighbor = {cell.x + dir.first, cell.y + dir.second};
    if (isValid(neighbor)) {
      neighbors.push_back(neighbor);
    }
  }
  
  return neighbors;
}

std::vector<GridCell> AStar::reconstructPath(
  const std::unordered_map<GridCell, GridCell, GridCellHash>& came_from,
  const GridCell& start,
  const GridCell& goal) const
{
  std::vector<GridCell> path;
  GridCell current = goal;
  
  while (!(current == start)) {
    path.push_back(current);
    auto it = came_from.find(current);
    if (it == came_from.end()) {
      break;
    }
    current = it->second;
  }
  
  path.push_back(start);
  std::reverse(path.begin(), path.end());
  
  return path;
}

std::vector<GridCell> AStar::findPath(const GridCell& start, const GridCell& goal)
{
  std::vector<GridCell> empty_path;
  
  // Check if start and goal are valid
  if (!isValid(start)) {
    std::cerr << "Start position is invalid or occupied!" << std::endl;
    return empty_path;
  }
  
  if (!isValid(goal)) {
    std::cerr << "Goal position is invalid or occupied!" << std::endl;
    return empty_path;
  }
  
  // Priority queue for open set
  std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_set;
  
  // Track visited nodes
  std::unordered_map<GridCell, bool, GridCellHash> closed_set;
  
  // Track g_cost for each node
  std::unordered_map<GridCell, double, GridCellHash> g_score;
  
  // Track parent of each node
  std::unordered_map<GridCell, GridCell, GridCellHash> came_from;
  
  // Initialize start node
  Node start_node;
  start_node.cell = start;
  start_node.g_cost = 0.0;
  start_node.h_cost = calculateHeuristic(start, goal);
  start_node.f_cost = start_node.g_cost + start_node.h_cost;
  start_node.parent = start;
  
  open_set.push(start_node);
  g_score[start] = 0.0;
  
  while (!open_set.empty()) {
    // Get node with lowest f_cost
    Node current = open_set.top();
    open_set.pop();
    
    // Check if we reached the goal
    if (current.cell == goal) {
      return reconstructPath(came_from, start, goal);
    }
    
    // Skip if already processed
    if (closed_set[current.cell]) {
      continue;
    }
    
    closed_set[current.cell] = true;
    
    // Check all neighbors
    std::vector<GridCell> neighbors = getNeighbors(current.cell);
    
    for (const auto& neighbor : neighbors) {
      // Skip if already processed
      if (closed_set[neighbor]) {
        continue;
      }
      
      // Calculate tentative g_cost
      double dx = static_cast<double>(neighbor.x - current.cell.x);
      double dy = static_cast<double>(neighbor.y - current.cell.y);
      double movement_cost = std::sqrt(dx * dx + dy * dy);
      double tentative_g = current.g_cost + movement_cost;
      
      // Check if this path is better
      auto it = g_score.find(neighbor);
      if (it == g_score.end() || tentative_g < it->second) {
        // This path is better, record it
        came_from[neighbor] = current.cell;
        g_score[neighbor] = tentative_g;
        
        Node neighbor_node;
        neighbor_node.cell = neighbor;
        neighbor_node.g_cost = tentative_g;
        neighbor_node.h_cost = calculateHeuristic(neighbor, goal);
        neighbor_node.f_cost = neighbor_node.g_cost + neighbor_node.h_cost;
        neighbor_node.parent = current.cell;
        
        open_set.push(neighbor_node);
      }
    }
  }
  
  std::cerr << "No path found!" << std::endl;
  return empty_path;
}

}  // namespace astar_planner

