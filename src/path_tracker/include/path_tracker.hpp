#ifndef PATH_TRACKER_HPP
#define PATH_TRACKER_HPP

#include <chrono>
#include <vector>
#include <random>
#include <numeric>
#include <iostream>
#include <algorithm>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/parameter_events_filter.hpp"

#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "builtin_interfaces/msg/time.hpp"

#include "misc.hpp"
#include "cost.hpp"
#include "action.hpp"
#include "state.hpp"
#include "dynamics.hpp"
#include "transform.hpp"
#include "dummy_path.hpp"
#include "regression.hpp"

using namespace std::chrono_literals;

class PathTracker : public rclcpp::Node {
    public:
        PathTracker(const std::string& pkg_loc);
        ~PathTracker();

        void MainLoop();
        void PrintConfig();
        void ResetPathTracker();
        void PubTrackingInfo(double cte);
        void PubZeroAction();
        double GetCTE();

    private:
        // ROS2 publishers
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _gear_pub;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _hist_pub;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _poly_pub;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _twist_pub;
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr _twist_stamped_pub;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _segment_pub;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _reference_pub;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _candidate_pub;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _dummy_path_pub;
        rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr _tracking_info_pub;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _lp_subgoal_pub;

        // ROS2 subscribers
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr _local_path_sub;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _localization_sub;
        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr _twist_sub;
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr _clicked_goal_sub;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _emergency_sub;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr _cost_map_sub;

        // Timer for main loop
        rclcpp::TimerBase::SharedPtr _timer;

        // Parameters
        std::string _gear_topic, _local_path_topic, _localization_topic;
        std::string _cmd_ctrl_topic, _cmd_ctrl_stamped_topic, _clicked_goal_topic, _emergency_topic, _cost_map_topic;
        std::string _frame_id;
        std::string _custom_path_name;
        std::string _Regressor_Type;

        bool _verbose;
        bool _preserved_path;
        bool _reconfigure;
        bool _emergency_situation;
        bool _subgoal_changed;
        bool _is_forward;
        bool _vis_segment;
        bool _vis_reference;
        bool _vis_candidate;
        bool _vis_hist;
        bool _vis_poly;
        bool _vis_subgoal;
        bool _debug_cost;
        bool _enable_stamped_cmd_vel;
        int _num_vis;

        int _average;

        int _K;
        int _Window;
        int _Order;
        int _Knot;
        double _Lamb;

        double _target_speed;
        int _n_ind_search;
        double _DL;

        double _Alp;
        double _Gam;
        double _Lam;
        std::vector<double> _Cov;

        int _target_ind;
        int _prev_ind;

        double _tracking_thres;
        double _goal_thres;

        // Class members
        Dynamics _dynamics;
        Cost _cost;
        Transform _trans;
        
        std::vector<PathStamp> _path;

        State _state;
        State _avg_state;
        double _avg_cte{0.0};
        std::vector<geometry_msgs::msg::PoseStamped> _hist;
        std::vector<PathStamp> _path_hist;
        std::default_random_engine _gen;
        std::vector<Action> _us;
        Action _prev_u;
        std::unique_ptr<DummyPathGenerator> _path_gen;
        std::unique_ptr<Regressor> _path_reg;
        
        // For twist calculation from pose
        geometry_msgs::msg::PoseStamped _prev_pose;
        bool _prev_pose_initialized;
        rclcpp::Time _prev_timestamp;

        PathStamp _subgoal;
        geometry_msgs::msg::Point _global_subgoal;

        // Callback methods
        void _LocalPathCallback(const nav_msgs::msg::Path::SharedPtr msg);
        void _LocalizationCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void _ClickedGoalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
        void _EmergencyCallback(const std_msgs::msg::Bool::SharedPtr msg);
        void _CostMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
        

        // Helper methods
        void _PubAction(const Action& solved_action);
        void _PubActionStamped(const Action& solved_action);
        void _SetConfig();
        void _CheckForward();
        void _UpdateState();
        void _GeneratePath();
        void _UpdateSolution(const std::vector<Action>& weighted_noise);
        void _UpdateSubgoal(const std::vector<PathStamp>& ref);
        bool _IsGoal();
        void _ClipAction(Action& action);
        
        // Path processing methods
        std::vector<PathStamp> _GetPathSegmentBasedOnTgtVel(const std::vector<PathStamp>& path, int target_ind);
        std::vector<PathStamp> _GetLocalPathSegment(const State& state);
        State _GetRefState(const std::vector<PathStamp>& ref, const State& cur_state, int t);
        int _GetNearestIndex(const State& state, const std::vector<PathStamp>& path, const int prev_ind);
        

        // Coordinate transformation methods
        std::vector<PathStamp> _ChangeGlobal2Local(const std::vector<PathStamp>& path);
        std::vector<PathStamp> _ChangeGlobal2LocalPath(const std::vector<PathStamp>& path, const State& state);
        std::vector<PathStamp> _ChangeLocal2GlobalPath(const std::vector<PathStamp>& path);
        
        // Twist calculation methods
        geometry_msgs::msg::Twist _CalculateTwistFromPose(const geometry_msgs::msg::PoseStamped::SharedPtr& msg);
        double _GetYawFromQuaternion(const geometry_msgs::msg::Quaternion& q);

        
        geometry_msgs::msg::Point _ChangeLocal2Global(const PathStamp& point);

        std::vector<geometry_msgs::msg::Point> _ChangeLocal2Global(const std::vector<State>& states);
        std::vector<geometry_msgs::msg::Point> _ChangeLocal2Global(const std::vector<PathStamp>& path);
        std::vector<geometry_msgs::msg::Point> _ChangeGlobal2Local(const std::vector<State>& states);
        PathStamp _ChangeGlobal2Local(const geometry_msgs::msg::Point& point);
        


        // MPPI algorithm methods
        std::vector<std::vector<Action>> _GetNoise();
        std::vector<double> _GetCost(const std::vector<PathStamp>& ref, const std::vector<std::vector<Action>>& noise);
        std::vector<double> _GetWeight(const std::vector<double>& costs);
        std::vector<Action> _GetWeightedNoise(const std::vector<double>& weights, const std::vector<std::vector<Action>>& noise);
        
        // Visualization methods
        void _VisPath(const std::vector<State>& path_segment);
        void _VisPath(const std::vector<PathStamp>& path_segment);
        void _VisPath(const std::vector<geometry_msgs::msg::Point>& path_segment);
        void _VisPoly(const std::vector<geometry_msgs::msg::Point>& path_segment);
        void _VisSegment(const std::vector<PathStamp>& path_segment);
        void _VisReference(const std::vector<geometry_msgs::msg::Point>& ref_xs);
        void _VisCandidate(const std::vector<geometry_msgs::msg::Point>& pred_xs, const std::vector<double>& costs, const bool constraint);
        void _VisLpSubgoal(const geometry_msgs::msg::Point& lp_subgoal);
        void _VisHist();

        std_msgs::msg::Header _GetHeader();
        OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
        rcl_interfaces::msg::SetParametersResult ParametersCallback(const std::vector<rclcpp::Parameter>& parameters);
        // void DeclareParameters();
        void UpdateParams(const rclcpp::Parameter& param);
        void ProcessDummyDeactivation();

    };

#endif