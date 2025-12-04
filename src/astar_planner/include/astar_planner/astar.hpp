#ifndef ASTAR_PLANNER__ASTAR_HPP_
#define ASTAR_PLANNER__ASTAR_HPP_

#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <unordered_map>

namespace astar_planner
{

struct GridCell
{
  int x;
  int y;
  
  bool operator==(const GridCell& other) const
  {
    return x == other.x && y == other.y;
  }
};

struct GridCellHash
{
  std::size_t operator()(const GridCell& cell) const
  {
    return std::hash<int>()(cell.x) ^ (std::hash<int>()(cell.y) << 1);
  }
};

struct Node
{
  GridCell cell;
  double g_cost;  // Cost from start to current node
  double h_cost;  // Heuristic cost from current node to goal
  double f_cost;  // Total cost: g_cost + h_cost
  GridCell parent;
  
  bool operator>(const Node& other) const
  {
    return f_cost > other.f_cost;
  }
};

class AStar
{
public:
  AStar();
  ~AStar();
  
  // Set the map
  void setMap(const std::vector<std::vector<int>>& map);
  
  // Find path using A* algorithm
  std::vector<GridCell> findPath(const GridCell& start, const GridCell& goal);
  
  // Get map dimensions
  int getMapWidth() const { return map_width_; }
  int getMapHeight() const { return map_height_; }
  
private:
  std::vector<std::vector<int>> map_;
  int map_width_;
  int map_height_;
  
  // Heuristic function (Euclidean distance)
  double calculateHeuristic(const GridCell& a, const GridCell& b) const;
  
  // Check if cell is valid (within bounds and not an obstacle)
  bool isValid(const GridCell& cell) const;
  
  // Get neighbors of a cell (8-connected grid)
  std::vector<GridCell> getNeighbors(const GridCell& cell) const;
  
  // Reconstruct path from goal to start
  std::vector<GridCell> reconstructPath(
    const std::unordered_map<GridCell, GridCell, GridCellHash>& came_from,
    const GridCell& start,
    const GridCell& goal) const;
};

}  // namespace astar_planner

#endif  // ASTAR_PLANNER__ASTAR_HPP_

