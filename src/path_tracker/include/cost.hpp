#ifndef COST_HPP
#define COST_HPP

#include <vector>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "misc.hpp"
#include "state.hpp"
#include "action.hpp"

class Cost {
    public:
        Cost() = default;
        ~Cost() = default;

        bool map_arrived{false};
        std::vector<int> const_indices;

        void Reset();
        double GetShiftCost(const Action& u, const Action& v, const std::vector<double>& cov);
        double GetStateCost(const State& pred_state, const State& ref_state);
        double GetActionCost(const Action& action);
        double GetVelocityCost(const State& pred_state, double target_vel);
        double GetSubgoalCost(const State& pred_state, const PathStamp& subgoal);
        double GetHeadingCost(const geometry_msgs::msg::Point& pred_state, const geometry_msgs::msg::Point& subgoal, double robot_yaw);
        double GetTerminalCost(const State& pred_state, const State& ref_state);
        double GetMapCost(const geometry_msgs::msg::Point& pt);
        bool GetMapConst(const geometry_msgs::msg::Point& pt);

        void SetConfig(rclcpp::Node* node);
        void PrintConfig();

        std::vector<double> GetQ() const { return _Q; }
        std::vector<double> GetQf() const { return _Qf; }
        std::vector<double> GetR() const { return _R; }
        std::vector<double> GetBound() const { return _bounds; }
        double GetMapCostWeight() const { return _map_cost_weight; }
        bool UseMapConst() const { return _use_const; }

        void SetQ(std::vector<double> Q);
        void SetQf(std::vector<double> Qf);
        void SetR(std::vector<double> R);
        void SetCost(const nav_msgs::msg::OccupancyGrid& cost_map);

    private:
        std::vector<double> _Q;
        std::vector<double> _Qf;
        std::vector<double> _R;
        double _const_thres{0.5};
        double _subgoal_cost_weight{0.0};
        double _vel_cost_weight{0.0};
        double _heading_cost_weight{0.0};
        double _map_cost_weight{0.0};
        bool _use_const{false};

        double _origin_x{0.0}, _origin_y{0.0};
        double _map_x_size{0.0}, _map_y_size{0.0};
        std::vector<double> _bounds;

        nav_msgs::msg::OccupancyGrid _cost_map;

        bool _CheckOutsideBound(const geometry_msgs::msg::Point& point);
        std::vector<int> _PointToGrid(const geometry_msgs::msg::Point& point, const nav_msgs::msg::MapMetaData& info);
        double _GetValue(const geometry_msgs::msg::Point& point, const nav_msgs::msg::OccupancyGrid& map);
};

#endif