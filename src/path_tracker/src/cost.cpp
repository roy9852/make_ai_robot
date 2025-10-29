#include "cost.hpp"

void Cost::Reset() {
    map_arrived = false;
    const_indices.clear();
}

double Cost::GetShiftCost(const Action& u, const Action& v, const std::vector<double>& cov) {
    double cost = 1.0 / cov.at(0) * u.vx * v.vx + 1.0 / cov.at(1) * u.wz * v.wz;
    return cost;
}

double Cost::GetStateCost(const State& pred_state, const State& ref_state) {
    auto res_state = (pred_state - ref_state).toVector();
    double cost = 0.0;
    for (size_t i = 0; i < res_state.size(); i++) {
        cost += _Q.at(i) * std::pow(res_state.at(i), 2);
    }
    return cost;
}

double Cost::GetActionCost(const Action& action) {
    auto vec = action.toVector();
    double cost = 0.0;
    for (size_t i = 0; i < vec.size(); i++) {
        cost += _R.at(i) * std::pow(vec.at(i), 2);
    }
    return cost;
}

double Cost::GetVelocityCost(const State& pred_state, double target_vel) {
    double cost = _vel_cost_weight * std::abs(pred_state.v - target_vel);
    return cost;
}

double Cost::GetSubgoalCost(const State& pred_state, const PathStamp& subgoal) {
    double cost = _subgoal_cost_weight * (std::pow(subgoal.x - pred_state.x, 2) + std::pow(subgoal.y - pred_state.y, 2));
    return cost;
}

double Cost::GetHeadingCost(const geometry_msgs::msg::Point& pred_state, const geometry_msgs::msg::Point& subgoal, double robot_yaw) {
    double path_yaw = std::atan2(subgoal.y - pred_state.y, subgoal.x - pred_state.x);
    double cost = _heading_cost_weight * std::abs(path_yaw - robot_yaw);
    return cost;
}

double Cost::GetTerminalCost(const State& pred_state, const State& ref_state) {
    auto res_state = (pred_state - ref_state).toVector();
    double cost = 0.0;
    for (size_t i = 0; i < res_state.size(); i++) {
        cost += _Qf.at(i) * std::pow(res_state.at(i), 2);
    }
    return cost;
}

double Cost::GetMapCost(const geometry_msgs::msg::Point& pt) {
    if (!map_arrived) return 0.0;
    if (_CheckOutsideBound(pt)) return 1.0;
    return _GetValue(pt, _cost_map);
}

bool Cost::GetMapConst(const geometry_msgs::msg::Point& pt) {
    if (!map_arrived) return false;
    if (_CheckOutsideBound(pt)) return true;
    double constraint = _GetValue(pt, _cost_map);
    return constraint > _const_thres;
}

void Cost::SetConfig(rclcpp::Node* node) {
    // Declare parameters
    node->declare_parameter("Q", std::vector<double>{12.0, 12.0, 50.0, 0.0});
    node->declare_parameter("Qf", std::vector<double>{5.0, 5.0, 30.0, 0.0});
    node->declare_parameter("R", std::vector<double>{0.1, 0.6});
    node->declare_parameter("subgoal_cost_weight", 0.0);
    node->declare_parameter("velocity_cost_weight", 0.0);
    node->declare_parameter("heading_cost_weight", 0.0);
    node->declare_parameter("map_cost_weight", 0.0);
    node->declare_parameter("use_const", false);
    node->declare_parameter("const_thres", 0.5);
    node->declare_parameter("origin_x", 0.0);
    node->declare_parameter("origin_y", 0.0);
    node->declare_parameter("map_x_size", 0.0);
    node->declare_parameter("map_y_size", 0.0);

    // Get parameters
    node->get_parameter("Q", _Q);
    node->get_parameter("Qf", _Qf);
    node->get_parameter("R", _R);
    node->get_parameter("subgoal_cost_weight", _subgoal_cost_weight);
    node->get_parameter("velocity_cost_weight", _vel_cost_weight);
    node->get_parameter("heading_cost_weight", _heading_cost_weight);
    node->get_parameter("map_cost_weight", _map_cost_weight);
    node->get_parameter("use_const", _use_const);
    node->get_parameter("const_thres", _const_thres);
    node->get_parameter("origin_x", _origin_x);
    node->get_parameter("origin_y", _origin_y);
    node->get_parameter("map_x_size", _map_x_size);
    node->get_parameter("map_y_size", _map_y_size);

    _bounds = {_origin_x, _origin_x + _map_x_size, _origin_y, _origin_y + _map_y_size};
}

void Cost::PrintConfig() {
    std::cout << "COST : " << std::endl;
    std::cout << "\tQ : " << _Q << std::endl;
    std::cout << "\tQf : " << _Qf << std::endl;
    std::cout << "\tR : " << _R << std::endl;
    std::cout << "\tsubgoal_cost_weight : " << _subgoal_cost_weight << std::endl;
    std::cout << "\tvelocity_cost_weight : " << _vel_cost_weight << std::endl;
    std::cout << "\theading_cost_weight : " << _heading_cost_weight << std::endl;
    std::cout << "\tmap_cost_weight : " << _map_cost_weight << std::endl;
    std::cout << "\tuse_const : " << _use_const << std::endl;
}

void Cost::SetQ(std::vector<double> Q) {
    _Q = std::move(Q);
}

void Cost::SetQf(std::vector<double> Qf) {
    _Qf = std::move(Qf);
}

void Cost::SetR(std::vector<double> R) {
    _R = std::move(R);
}

void Cost::SetCost(const nav_msgs::msg::OccupancyGrid& cost_map) {
    _cost_map = cost_map;
    map_arrived = true;
}

bool Cost::_CheckOutsideBound(const geometry_msgs::msg::Point& point) {
    return (point.x < _bounds[0] || point.x > _bounds[1] || 
            point.y < _bounds[2] || point.y > _bounds[3]);
}

std::vector<int> Cost::_PointToGrid(const geometry_msgs::msg::Point& point, const nav_msgs::msg::MapMetaData& info) {
    int grid_x = static_cast<int>((point.x - info.origin.position.x) / info.resolution);
    int grid_y = static_cast<int>((point.y - info.origin.position.y) / info.resolution);
    
    grid_x = std::clamp(grid_x, 0, static_cast<int>(info.width) - 1);
    grid_y = std::clamp(grid_y, 0, static_cast<int>(info.height) - 1);

    return {grid_x, grid_y};
}

double Cost::_GetValue(const geometry_msgs::msg::Point& point, const nav_msgs::msg::OccupancyGrid& map) {
    std::vector<int> grid_xy = _PointToGrid(point, map.info);
    size_t grid_idx = grid_xy[0] + map.info.width * grid_xy[1];
    
    if (grid_idx >= map.data.size()) {
        throw CustomException::ErrorType::NotFoundMapValue;
    }
    
    return std::max(static_cast<double>(map.data[grid_idx]) / 100.0, 0.0);
}