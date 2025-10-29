#include "path_tracker.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

PathTracker::PathTracker(const std::string& pkg_loc) : Node("path_tracker") 
{
    _SetConfig();

    // Initialize path generator and regressor
    _path_gen = std::make_unique<DummyPathGenerator>(ament_index_cpp::get_package_share_directory("path_tracker"), _DL);
    
    if (_Regressor_Type == "Poly") {
        _path_reg = std::make_unique<Regressor>(_Order, _Window, _DL, _verbose);
    } else if (_Regressor_Type == "Spline") {
        _path_reg = std::make_unique<SplineRegressor>(_Knot, _Order, _Window, _DL, _verbose);
    } else if (_Regressor_Type == "SmoothSpline") {
        _path_reg = std::make_unique<SmoothingSplineRegressor>(_Lamb, _Knot, _Order, _Window, _DL, _verbose);
    } else if (_Regressor_Type == "PenalizedBSpline") {
        _path_reg = std::make_unique<PenalizedBSplineRegressor>(_Lamb, _Knot, _Order, _Window, _DL, _verbose);
    } else {
        throw CustomException::ErrorType::NotFoundRegressorType;
    }

    _local_path_sub     = create_subscription<nav_msgs::msg::Path>(_local_path_topic, 1, std::bind(&PathTracker::_LocalPathCallback, this, std::placeholders::_1));
    _localization_sub   = create_subscription<geometry_msgs::msg::PoseStamped>(_localization_topic, 1, std::bind(&PathTracker::_LocalizationCallback, this, std::placeholders::_1));
    _clicked_goal_sub   = create_subscription<geometry_msgs::msg::PointStamped>(_clicked_goal_topic, 10, std::bind(&PathTracker::_ClickedGoalCallback, this, std::placeholders::_1));
    _emergency_sub      = create_subscription<std_msgs::msg::Bool>(_emergency_topic, 1, std::bind(&PathTracker::_EmergencyCallback, this, std::placeholders::_1));
    _cost_map_sub       = create_subscription<nav_msgs::msg::OccupancyGrid>(_cost_map_topic, 10, std::bind(&PathTracker::_CostMapCallback, this, std::placeholders::_1));

    _gear_pub           = create_publisher<std_msgs::msg::Bool>(_gear_topic, 1);
    _twist_pub          = create_publisher<geometry_msgs::msg::Twist>(_cmd_ctrl_topic, 1);
    _twist_stamped_pub  = create_publisher<geometry_msgs::msg::TwistStamped>(_cmd_ctrl_stamped_topic, 1);
    _hist_pub           = create_publisher<nav_msgs::msg::Path>(get_name() + std::string("/history"), 1);
    _poly_pub           = create_publisher<nav_msgs::msg::Path>(get_name() + std::string("/polyline"), 1);
    _segment_pub        = create_publisher<nav_msgs::msg::Path>(get_name() + std::string("/segment"), 1);
    _reference_pub      = create_publisher<visualization_msgs::msg::MarkerArray>(get_name() + std::string("/reference"), 10);
    _candidate_pub      = create_publisher<visualization_msgs::msg::MarkerArray>(get_name() + std::string("/candidate"), 10);
    _dummy_path_pub     = create_publisher<nav_msgs::msg::Path>(get_name() + std::string("/dummy_path"), 1);
    _tracking_info_pub  = create_publisher<geometry_msgs::msg::Point>(get_name() + std::string("/tracking_info"), 1);
    _lp_subgoal_pub     = create_publisher<visualization_msgs::msg::Marker>(get_name() + std::string("/lp_subgoal"), 1);
    
    _timer              = create_wall_timer(std::chrono::milliseconds(100), std::bind(&PathTracker::MainLoop, this));

    ResetPathTracker();

    PrintConfig();

    if (_reconfigure) {
        param_callback_handle_ = add_on_set_parameters_callback(std::bind(&PathTracker::ParametersCallback, this, std::placeholders::_1));
    }
}

PathTracker::~PathTracker() {
    ResetPathTracker();
    std::cout << "Path tracker is successfully deleted" << std::endl;
}

void PathTracker::_SetConfig(){
    // Parameter descriptor 생성을 위한 helper 함수들
    auto declare_bool_param = [this](const std::string& name, bool default_value, const std::string& description) {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = description;
        descriptor.additional_constraints = "";
        descriptor.read_only = false;
        descriptor.dynamic_typing = false;
        declare_parameter(name, default_value, descriptor);
    };

    auto declare_double_param = [this](const std::string& name, double default_value, const std::string& description, double from, double to) {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        rcl_interfaces::msg::FloatingPointRange range;
        range.from_value = from;
        range.to_value = to;
        descriptor.floating_point_range.push_back(range);
        descriptor.description = description;
        declare_parameter(name, default_value, descriptor);
    };

    auto declare_int_param = [this](const std::string& name, int default_value, const std::string& description, int from, int to) {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        rcl_interfaces::msg::IntegerRange range;
        range.from_value = from;
        range.to_value = to;
        descriptor.integer_range.push_back(range);
        descriptor.description = description;
        declare_parameter(name, default_value, descriptor);
    };

    // Dummy Path Parameters
    declare_bool_param("dummy_path", false, "test tracker with generated dummy path");
    declare_bool_param("dummy_activate", false, "Activate to generate dummy path");
    declare_double_param("dummy_curvature", 1.0, "dummy path curvature", 0.1, 5.0);
    declare_double_param("dummy_length", 5.0, "dummy path length", 0.0, 50.0);
    declare_int_param("dummy_type", 0, "dummy path type", 0, 20);
    declare_bool_param("enable_stamped_cmd_vel", false, "Enable/disable cmd_vel_stamped output (reconfigurable via rqt)");
    declare_bool_param("reconfigure", true, "use reconfigure");
    declare_bool_param("preserved_path", true, "preserved_path");
    
    declare_bool_param("verbose", false, "print debug information");
    declare_bool_param("vis_hist", false, "visualization history");
    declare_bool_param("vis_segment", false, "visualization segment");
    declare_bool_param("vis_reference", false, "visualization reference");
    declare_bool_param("vis_candidate", false, "visualization candidate");
    declare_bool_param("vis_poly", false, "visualization poly");
    declare_bool_param("vis_subgoal", false, "visualization subgoal");
    declare_bool_param("debug_cost", false, "debug cost");
    declare_int_param("num_vis", 50, "number of visualization", 1, 200);
                                                                                                                                                                                                                                                                                                                                                                                        

    declare_int_param("average", 10, "moving avg window size", 1, 20);
    declare_int_param("K", 500, "number of samples", 1, 3000);
    declare_int_param("T", 20, "number of predicted timesteps", 5, 50);
    declare_int_param("n_ind_search", 30, "search range for reference path", 5, 100);
    declare_double_param("DT", 0.1, "system timestep", 0.05, 0.2);
    declare_double_param("target_speed", 1.0, "desired vehicle velocity [m/s]", 0.0, 5.0);
    declare_double_param("Lam", 12.0, "scaling factor for noise cost", 0.1, 100.0);
    declare_double_param("Gam", 0.4, "scaling factor for time", 0.0, 1.5);

    // Covariance Parameters
    std::vector<double> default_cov = {0.06, 0.10};
    _Cov = default_cov;
    declare_double_param("Cov_Lin_Vel", _Cov[0], "covariance for linear velocity", 0.0, 5.0);
    declare_double_param("Cov_Ang_Vel", _Cov[1], "covariance for angular velocity", 0.0, 5.0);

    // Cost Parameters
    declare_double_param("Q_Pos", 12.0, "state cost for position", 0.0, 200.0);
    declare_double_param("Q_Vel", 50.0, "state cost for velocity", 0.0, 200.0);
    declare_double_param("Q_Yaw", 0.0, "state cost for heading", 0.0, 20.0);

    // Terminal Cost Parameters
    declare_double_param("Qf_Pos", 5.0, "terminal cost for position", 0.0, 200.0);
    declare_double_param("Qf_Vel", 30.0, "terminal cost for velocity", 0.0, 200.0);
    declare_double_param("Qf_Yaw", 0.0, "terminal cost for heading", 0.0, 20.0);

    // Action Cost Parameters
    declare_double_param("R_Lin_Vel", 0.1, "action cost for linear velocity", 0.0, 10.0);
    declare_double_param("R_Ang_Vel", 0.6, "action cost for angular velocity", 0.0, 10.0);



    // Declare and get parameters
    this->declare_parameter("frame_id", "base_link");
    this->declare_parameter("gear_topic", "/gear");
    this->declare_parameter("local_path_topic", "/local_path");
    this->declare_parameter("localization_topic", "zed_front/zed/pose");//topic name
    this->declare_parameter("cmd_ctrl_topic", "/cmd_vel");
    this->declare_parameter("cmd_ctrl_stamped_topic", "/cmd_vel_stamped");
    this->declare_parameter("clicked_goal_topic", "/clicked_point");
    this->declare_parameter("emergency_topic", "/emergency");
    this->declare_parameter("cost_map_topic", "/cost_map");


    this->declare_parameter("DL", 0.1);
    this->declare_parameter("Window", 5);
    this->declare_parameter("Order", 2);
    this->declare_parameter("Knot", 5);
    this->declare_parameter("Lamb", 0.0001);
    this->declare_parameter("Regressor_Type", "Spline");

    this->declare_parameter("Cov", std::vector<double>{0.06, 0.10});

    this->declare_parameter("custom_path_name", "custom_path.csv");

    this->declare_parameter("tracking_thres", 5.0);
    this->declare_parameter("goal_thres", 0.5);


    // Get parameters
    this->get_parameter("frame_id", _frame_id);
    this->get_parameter("gear_topic", _gear_topic);
    this->get_parameter("local_path_topic", _local_path_topic);
    this->get_parameter("localization_topic", _localization_topic);
    this->get_parameter("cmd_ctrl_topic", _cmd_ctrl_topic);
    this->get_parameter("cmd_ctrl_stamped_topic", _cmd_ctrl_stamped_topic);
    this->get_parameter("clicked_goal_topic", _clicked_goal_topic);
    this->get_parameter("emergency_topic", _emergency_topic);
    this->get_parameter("cost_map_topic", _cost_map_topic);

    this->get_parameter("verbose", _verbose);
    this->get_parameter("average", _average);
    this->get_parameter("vis_segment", _vis_segment);
    this->get_parameter("vis_reference", _vis_reference);
    this->get_parameter("vis_candidate", _vis_candidate);
    this->get_parameter("vis_hist", _vis_hist);
    this->get_parameter("vis_poly", _vis_poly);
    this->get_parameter("vis_subgoal", _vis_subgoal);
    this->get_parameter("num_vis", _num_vis);
    this->get_parameter("debug_cost", _debug_cost);

    this->get_parameter("K", _K);
    this->get_parameter("Window", _Window);
    this->get_parameter("Order", _Order);
    this->get_parameter("Knot", _Knot);
    this->get_parameter("Lamb", _Lamb);

    this->get_parameter("Regressor_Type", _Regressor_Type);
    this->get_parameter("target_speed", _target_speed);
    this->get_parameter("n_ind_search", _n_ind_search);

    this->get_parameter("DL", _DL);
    this->get_parameter("Gam", _Gam);
    this->get_parameter("Lam", _Lam);
    this->get_parameter("Cov", _Cov);

    this->get_parameter("custom_path_name", _custom_path_name);

    this->get_parameter("tracking_thres", _tracking_thres);
    this->get_parameter("goal_thres", _goal_thres);

    this->get_parameter("preserved_path", _preserved_path);
    this->get_parameter("reconfigure", _reconfigure);
    this->get_parameter("enable_stamped_cmd_vel", _enable_stamped_cmd_vel);
    
    _Alp = 1 - _Gam / _Lam;

    _dynamics.SetConfig(this);
    _cost.SetConfig(this);

    _subgoal_changed = false;
    _emergency_situation = false;
}

void PathTracker::MainLoop() {

    try {
        if (_emergency_situation) {
            throw CustomException::ErrorType::EmergencySituation;
        }

        const auto begin = std::chrono::steady_clock::now();

        _UpdateState();

        if (_path_gen->GetActive()) {
            _GeneratePath();
            if (_path.size() > 1) {
                _VisHist();
                _VisPath(_path);
            }
        }

        _CheckForward();

        if (_path.size() < 2) {
            throw CustomException::ErrorType::NotImplementedPath;
        }

        const auto is_goal = _IsGoal();
        if (is_goal) {
            _path.clear();
            throw CustomException::ErrorType::GoalArrived;
        }

        const auto cte = GetCTE();
        PubTrackingInfo(cte);
        if (cte > _tracking_thres) {
            throw CustomException::ErrorType::NotFoundTargetPath;
        }

        auto path_segment = _GetLocalPathSegment(_state);
        auto noise = _GetNoise();
        auto costs = _GetCost(path_segment, noise);
        auto weights = _GetWeight(costs);
        auto weighted_noise = _GetWeightedNoise(weights, noise);

        _UpdateSolution(weighted_noise);

        const auto end = std::chrono::steady_clock::now();
        const auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();

        if (_verbose) {
            std::cout << "Current State:" << std::endl;
            std::cout << _state << std::endl;

            std::cout << "Average State:" << std::endl;
            std::cout << _avg_state << std::endl;

            std::cout << std::setw(20) << "Current Index" << ": " << _target_ind << " / " << _path.size() - 1 << std::endl;
            std::cout << std::setw(20) << "Elapsed Time" << ": " << elapsed_time << " [ms]" << std::endl;
            std::cout << "---" << std::endl;
        }

    } catch (const CustomException::ErrorType& e) {
        CustomException::report(e);
        ResetPathTracker();
    } catch (const std::exception& e) {
        std::cout << typeid(e).name() << std::endl;
        std::cerr << e.what() << std::endl;
    }
}

std::vector<PathStamp> PathTracker::_GetPathSegmentBasedOnTgtVel(const std::vector<PathStamp>& path, int target_ind) {
    
    std::vector<PathStamp> path_seg;
    double cum_tgt_vel_dist = _target_speed * _dynamics.GetDT();
    int idx = target_ind;
    
    PathStamp new_stamp = path[idx];
    path_seg.push_back(new_stamp);
    
    double cum_dist = std::sqrt(std::pow(path[idx+1].x - path[idx].x, 2.0) + 
                           std::pow(path[idx+1].y - path[idx].y, 2.0));
                           
    while(idx < path.size()) {
        if (idx < path.size() - 1) {
            if (cum_tgt_vel_dist < cum_dist) {
                double dist = std::sqrt(std::pow(path[idx+1].x - path[idx].x, 2.0) + 
                                  std::pow(path[idx+1].y - path[idx].y, 2.0));
                double residual = dist - cum_dist + cum_tgt_vel_dist;
                
                new_stamp.x = path[idx].x + residual/dist * (path[idx+1].x - path[idx].x);
                new_stamp.y = path[idx].y + residual/dist * (path[idx+1].y - path[idx].y);
                new_stamp.yaw = path[idx].yaw;
                new_stamp.gear = path[idx].gear;
                
                path_seg.push_back(new_stamp);
                cum_tgt_vel_dist += _target_speed * _dynamics.GetDT();
            } else {
                idx += 1;
                cum_dist += std::sqrt(std::pow(path[idx+1].x - path[idx].x, 2.0) + 
                                std::pow(path[idx+1].y - path[idx].y, 2.0));
            }
        } else {
            path_seg.push_back(path[idx]);
        }
        if (path_seg.size() == _dynamics.GetT()) break;
    }
    return path_seg;
}

std::vector<PathStamp> PathTracker::_GetLocalPathSegment(const State& state) {
    if (!_preserved_path) {
        _prev_ind = 0;
    }

    _target_ind = _GetNearestIndex(state, _path, _prev_ind);
    
    auto path_segment = _GetPathSegmentBasedOnTgtVel(_path, _target_ind);
    auto local_path_segment = _ChangeGlobal2Local(path_segment);

    if (_vis_segment) {
        _VisSegment(path_segment);
    }
    _prev_ind = _target_ind;
    return local_path_segment;
}

std::vector<PathStamp> PathTracker::_ChangeGlobal2LocalPath(const std::vector<PathStamp>& path, const State& state) {
    
    std::vector<PathStamp> local_path;
    local_path.reserve(path.size());
    
    EulerAngle e{0.0, 0.0, -state.yaw};
    auto rot = _trans.GetRotationFromEuler(e);
    
    std::transform(path.begin(), path.end(), std::back_inserter(local_path), [&](const PathStamp& stamp) -> PathStamp {
        PathStamp local_stamp;
        geometry_msgs::msg::Point pt = _trans.ChangePointCoordinate(stamp.x - state.x, stamp.y - state.y, stamp.z - state.z, rot);
        local_stamp.x = pt.x;
        local_stamp.y = pt.y;
        local_stamp.z = pt.z;
        local_stamp.yaw = Pi2Pi(stamp.yaw - state.yaw);
        local_stamp.gear = stamp.gear;
        return local_stamp;
    });
    return local_path;
}

std::vector<PathStamp> PathTracker::_ChangeGlobal2Local(const std::vector<PathStamp>& path) {
    
    std::vector<PathStamp> local_path;
    local_path.reserve(path.size());
    
    EulerAngle e{0.0, 0.0, -_state.yaw};
    auto rot = _trans.GetRotationFromEuler(e);
    
    std::transform(path.begin(), path.end(), std::back_inserter(local_path),[&](const PathStamp& stamp) -> PathStamp {
        PathStamp local_stamp;
        geometry_msgs::msg::Point pt = _trans.ChangePointCoordinate(stamp.x - _state.x, stamp.y - _state.y, stamp.z - _state.z, rot);
        local_stamp.x = pt.x;
        local_stamp.y = pt.y;
        local_stamp.z = pt.z;
        local_stamp.yaw = Pi2Pi(stamp.yaw - _state.yaw);
        local_stamp.gear = stamp.gear;
        return local_stamp;
    });
    return local_path;
}

std::vector<geometry_msgs::msg::Point> PathTracker::_ChangeGlobal2Local(const std::vector<State>& states) {
    
    std::vector<geometry_msgs::msg::Point> local_path;
    local_path.reserve(states.size());
    
    EulerAngle e{0.0, 0.0, -_state.yaw};
    auto rot = _trans.GetRotationFromEuler(e);
    
    std::transform(states.begin(), states.end(), std::back_inserter(local_path), [&](const State& s) -> geometry_msgs::msg::Point {
        return _trans.ChangePointCoordinate(s.x - _state.x, s.y - _state.y, s.z - _state.z, rot);
    });
    return local_path;
}

std::vector<PathStamp> PathTracker::_ChangeLocal2GlobalPath(const std::vector<PathStamp>& path) {
    
    std::vector<PathStamp> global_path;
    global_path.reserve(path.size());
    
    EulerAngle e{0.0, 0.0, _state.yaw};
    auto rot = _trans.GetRotationFromEuler(e);
    
    std::transform(path.begin(), path.end(), std::back_inserter(global_path), [&](const PathStamp& stamp) -> PathStamp {
        PathStamp global_stamp;
        geometry_msgs::msg::Point pt = _trans.ChangePointCoordinate(stamp.x, stamp.y, stamp.z, rot);
        global_stamp.x = pt.x + _state.x;
        global_stamp.y = pt.y + _state.y;
        global_stamp.z = pt.z;
        global_stamp.yaw = Pi2Pi(stamp.yaw + _state.yaw);
        global_stamp.gear = stamp.gear;
        return global_stamp;
    });
    return global_path;
}

std::vector<geometry_msgs::msg::Point> PathTracker::_ChangeLocal2Global(const std::vector<PathStamp>& path) {
    
    std::vector<geometry_msgs::msg::Point> global_path;
    global_path.reserve(path.size());
    
    EulerAngle e{0.0, 0.0, _state.yaw};
    auto rot = _trans.GetRotationFromEuler(e);
    
    std::transform(path.begin(), path.end(), std::back_inserter(global_path),[&](const PathStamp& stamp) -> geometry_msgs::msg::Point {
        geometry_msgs::msg::Point global_point;
        auto pt = _trans.ChangePointCoordinate(stamp.x, stamp.y, stamp.z, rot);
        global_point.x = pt.x + _state.x;
        global_point.y = pt.y + _state.y;
        global_point.z = pt.z;
        return global_point;
    });
    return global_path;
}

std::vector<geometry_msgs::msg::Point> PathTracker::_ChangeLocal2Global(const std::vector<State>& states) {
    
    std::vector<geometry_msgs::msg::Point> global_path;
    global_path.reserve(states.size());
    
    EulerAngle e{0.0, 0.0, _state.yaw};
    auto rot = _trans.GetRotationFromEuler(e);
    
    std::transform(states.begin(), states.end(), std::back_inserter(global_path), [&](const State& s) -> geometry_msgs::msg::Point {
        geometry_msgs::msg::Point global_point;
        auto pt = _trans.ChangePointCoordinate(s.x, s.y, s.z, rot);
        global_point.x = pt.x + _state.x;
        global_point.y = pt.y + _state.y;
        global_point.z = pt.z;
        return global_point;
    });
    return global_path;
}

geometry_msgs::msg::Point PathTracker::_ChangeLocal2Global(const PathStamp& point) {
    EulerAngle e{0.0, 0.0, _state.yaw};
    auto rot = _trans.GetRotationFromEuler(e);

    auto pt = _trans.ChangePointCoordinate(point.x, point.y, point.z, rot);
    geometry_msgs::msg::Point global_point;
    global_point.x = pt.x + _state.x;
    global_point.y = pt.y + _state.y;
    global_point.z = pt.z;

    return global_point;
}

PathStamp PathTracker::_ChangeGlobal2Local(const geometry_msgs::msg::Point& point) {
    EulerAngle e{0.0, 0.0, -_state.yaw};
    auto rot = _trans.GetRotationFromEuler(e);

    PathStamp local_stamp;
    auto pt = _trans.ChangePointCoordinate(
        point.x - _state.x,
        point.y - _state.y,
        point.z - _state.z,
        rot);

    local_stamp.x = pt.x;
    local_stamp.y = pt.y;
    local_stamp.z = pt.z;

    return local_stamp;
}


int PathTracker::_GetNearestIndex(const State& state, const std::vector<PathStamp>& path, const int prev_ind) {
    
    int start_ind = prev_ind;
    int end_ind = std::min(static_cast<int>(path.size()), prev_ind + _n_ind_search);
    double min_distance = std::numeric_limits<double>::max();
    int min_idx = start_ind;

    for (int idx = start_ind; idx < end_ind; idx++) {
        double distance = state.GetDistance(path[idx]);
        if (distance < min_distance) {
            min_distance = distance;
            min_idx = idx;
        }
    }
    return min_idx;
}

void PathTracker::_CheckForward() {
    if (_path.size() > 0) {
        _is_forward = _path.at(0).gear >= 0;
    }
    
    auto gear_msg = std::make_unique<std_msgs::msg::Bool>();
    gear_msg->data = _is_forward;
    _gear_pub->publish(std::move(gear_msg));
}

void PathTracker::_GeneratePath() {
    if (_path_gen->flag == false) {
        if (_verbose) {
            std::cout << "Path Type: " << static_cast<int>(_path_gen->GetType()) << std::endl;
        }
        
        std::vector<PathStamp> dummy_path;
        
        switch (static_cast<PathType>(_path_gen->GetType())) {
            case PathType::ForwardStraight:
                dummy_path = _path_gen->GetForwardStraightPath();
                break;

            case PathType::BackwardStraight:
                dummy_path = _path_gen->GetBackwardStraightPath();
                break;

            case PathType::ForwardTurnRight:
                dummy_path = _path_gen->GetForwardTurnRightPath();
                break;

            case PathType::ForwardTurnLeft:
                dummy_path = _path_gen->GetForwardTurnLeftPath();
                break;

            case PathType::BackwardTurnRight:
                dummy_path = _path_gen->GetBackwardTurnRightPath();
                break;

            case PathType::BackwardTurnLeft:
                dummy_path = _path_gen->GetBackwardTurnLeftPath();
                break;

            case PathType::ForwardInfty:
                dummy_path = _path_gen->GetForwardInftyPath();
                break;

            case PathType::BackwardInfty:
                dummy_path = _path_gen->GetBackwardInftyPath();
                break;

            case PathType::ForwardRandom:
                dummy_path = _path_gen->GetForwardRandomPath();
                break;

            case PathType::BackwardRandom:
                dummy_path = _path_gen->GetBackwardRandomPath();
                break;

            case PathType::ForwardTriangle:
                dummy_path = _path_gen->GetForwardTrianglePath();
                break;

            case PathType::ForwardSquare:
                dummy_path = _path_gen->GetForwardSquarePath();
                break;

            case PathType::ForwardPentagon:
                dummy_path = _path_gen->GetForwardPentagonPath();
                break;

            case PathType::ForwardHourGlass:
                dummy_path = _path_gen->GetForwardHourGlassPath();
                break;

            case PathType::BackwardTriangle:
                dummy_path = _path_gen->GetBackwardTrianglePath();
                break;

            case PathType::BackwardSquare:
                dummy_path = _path_gen->GetBackwardSquarePath();
                break;

            case PathType::BackwardPentagon:
                dummy_path = _path_gen->GetBackwardPentagonPath();
                break;

            case PathType::BackwardHourGlass:
                dummy_path = _path_gen->GetBackwardHourGlassPath();
                break;

            case PathType::ReturnStraight:
                dummy_path = _path_gen->GetReturnStraightPath();
                break;

            case PathType::SaveCustomPath:
                PathTracker::_VisHist();
                break;

            case PathType::CustomPath:
                dummy_path = _path_gen->LoadCustomPath(_custom_path_name);
                break;
            default:
                std::cout << "Unknown Path Type: " << static_cast<int>(_path_gen->GetType()) << std::endl;
                break;
        }

        if (static_cast<PathType>(_path_gen->GetType()) != PathType::SaveCustomPath) {
            _path = _ChangeLocal2GlobalPath(dummy_path);
            _path_gen->flag = true;
        }
    }
}

void PathTracker::_UpdateState() {
    auto pos = _trans.GetPos();
    auto orient = _trans.GetOrient();
    auto twist = _trans.GetTwist();

    auto new_state = _trans.GetState(pos, orient, twist);
    
    // Output velocity
    std::cout << "Current velocity: " << twist.linear.x << " m/s, angular.z: " << twist.angular.z << " rad/s" << std::endl;
    std::cout << "  - pose: (" << pos.x << ", " << pos.y << ", " << pos.z << ")" << std::endl;
    std::cout << "  - twist original: linear.x=" << twist.linear.x << ", angular.z=" << twist.angular.z << std::endl;
    
    auto distance = _state.GetDistance(new_state);

    if (distance > _tracking_thres) {
        _state = new_state;
        throw CustomException::ErrorType::OutOdometry;
    }
    _state = new_state;
}

std::vector<std::vector<Action>> PathTracker::_GetNoise() {
    std::normal_distribution<double> dist(0.0, 1.0);
    std::vector<std::vector<Action>> noise;
    noise.reserve(_K);
    
    for (int k = 0; k < _K; k++) {
        std::vector<Action> noise_sequence;
        noise_sequence.reserve(_dynamics.GetT());
        
        for (int i = 0; i < _dynamics.GetT(); i++) {
            Action action;
            action.vx = std::sqrt(_Cov.at(0)) * dist(_gen);
            action.wz = std::sqrt(_Cov.at(1)) * dist(_gen);
            noise_sequence.push_back(action);
        }
        noise.push_back(std::move(noise_sequence));
    }
    return noise;
}

std::vector<double> PathTracker::_GetWeight(const std::vector<double>& costs) {
    double rho = *std::min_element(costs.begin(), costs.end());
    double eta = 0.0;
    
    std::vector<double> weights;
    weights.reserve(costs.size());
    auto weight_fn = [&](double cost) -> double {
        double weight = std::exp(- 1 / _Lam * (cost - rho));
        eta += weight;
        return weight;
    };

    // Calculate raw weights
    std::transform(costs.begin(), costs.end(), std::back_inserter(weights), weight_fn);

    // Normalize weights
    std::vector<double> normalized_weights;
    normalized_weights.reserve(weights.size());
    auto normalize_fn = [&](double weight) -> double {
        return  weight / eta;
    };

    std::transform(weights.begin(), weights.end(), std::back_inserter(normalized_weights), normalize_fn);

    // Apply constraints if needed
    if (_cost.UseMapConst()) {
        for (const auto& idx : _cost.const_indices) {
            normalized_weights[idx] = 0.0;
        }
    }
    
    return normalized_weights;
}

std::vector<Action> PathTracker::_GetWeightedNoise(const std::vector<double>& weights, const std::vector<std::vector<Action>>& noise) {
    
    std::vector<Action> weighted_noise;
    weighted_noise.reserve(_dynamics.GetT());

    for (int t = 0; t < _dynamics.GetT(); t++) {
        Action weight_noise_t;
        
        for (int k = 0; k < _K; k++) {
            weight_noise_t += noise[k][t] * weights[k];
        }
        weighted_noise.push_back(weight_noise_t);
    }
    
    return weighted_noise;
}

std::vector<double> PathTracker::_GetCost(const std::vector<PathStamp>& ref, const std::vector<std::vector<Action>>& noise) {
    
    State local_state = _state.GetLocalState();

    std::vector<double> sample_costs;
    std::vector<geometry_msgs::msg::Point> pred_xs;
    std::vector<geometry_msgs::msg::Point> ref_xs;

    std::vector<double> state_costs;
    std::vector<double> action_costs;
    std::vector<double> shift_costs;
    std::vector<double> terminal_costs;
    std::vector<double> map_costs;
    std::vector<double> subgoal_costs;
    std::vector<double> velocity_costs;
    std::vector<double> heading_costs;

    _UpdateSubgoal(ref);
    auto robot_orient = _trans.GetEulerFromQuaternion(_state.orient);

    if (!_path_reg->CheckDOF(ref.size())) {
        throw CustomException::ErrorType::NotInitalizeRegressor;
    }
    
    _path_reg->Fit(ref);
    
    if (!_path_reg->CheckSolved()) {
        throw CustomException::ErrorType::NotSolvedRegressor;
    }

    _cost.const_indices.clear();
    
    for (int k = 0; k < _K; k++) {
        State cur_state = local_state;
        State ref_state;

        double state_cost = 0.0;
        double velocity_cost = 0.0;
        double action_cost = 0.0;
        double shift_cost = 0.0;
        double map_cost = 0.0;
        bool map_const = false;

        int end_ind = std::min(static_cast<int>(_dynamics.GetT()), static_cast<int>(ref.size()));
                              
        for (int t = 0; t < end_ind; t++) {
            Action v;
            const auto& u = _us[t];
            const auto& e = noise[k][t];

            if (k < (1 - _Alp) * _K){
                v = u + e;
            } else {
                v = e;
            }

            _ClipAction(v);

            cur_state = _dynamics.UpdateState(cur_state, v);
            ref_state = _GetRefState(ref, cur_state, t);

            state_cost += _cost.GetStateCost(cur_state, ref_state);
            action_cost += _cost.GetActionCost(v);
            shift_cost += _Gam * _cost.GetShiftCost(u, v, _Cov);
            velocity_cost += _cost.GetVelocityCost(cur_state, _target_speed);

            auto global_ref_state = ref_state.toGlobalPoint(_state.yaw, _state.toGeoPoint());
            ref_xs.push_back(global_ref_state);
            auto global_state = cur_state.toGlobalPoint(_state.yaw, _state.toGeoPoint());
            pred_xs.push_back(global_state);

            map_cost += _cost.GetMapCost(global_state);
            map_const = map_const || _cost.GetMapConst(global_state);
        }
        
        if (map_const) _cost.const_indices.push_back(k);

        auto global_cur_state = cur_state.toGlobalPoint(_state.yaw, _state.toGeoPoint());
        double heading_cost = _cost.GetHeadingCost(global_cur_state, _global_subgoal, robot_orient.yaw);
        double terminal_cost = _cost.GetTerminalCost(cur_state, ref_state);
        double subgoal_cost = _cost.GetSubgoalCost(cur_state, _subgoal);
    
        map_cost = _cost.GetMapCostWeight() * map_cost;

        state_costs.push_back(state_cost);
        action_costs.push_back(action_cost);
        shift_costs.push_back(shift_cost);
        terminal_costs.push_back(terminal_cost);
        map_costs.push_back(map_cost);
        subgoal_costs.push_back(subgoal_cost);
        velocity_costs.push_back(velocity_cost);
        heading_costs.push_back(heading_cost);

        const double total_cost = state_cost + action_cost + shift_cost + 
                                terminal_cost + map_cost + subgoal_cost + 
                                velocity_cost + heading_cost;

        sample_costs.push_back(total_cost);
    }

    if (_vis_candidate) _VisCandidate(pred_xs, subgoal_costs, _cost.UseMapConst());
    if (_vis_reference) _VisReference(ref_xs);
    if (_vis_poly) {
        auto line = _path_reg->Getline(ref);
        auto global_line = _ChangeLocal2Global(line);
        _VisPoly(global_line);
    }
    
    if (_debug_cost && _verbose) {
        std::cout << std::fixed << std::setprecision(2);
        std::cout << prColor::Cyan << "Cost Information:\n" << prColor::End;
        std::cout << std::left;
        
        const auto print_cost = [](const std::string& name, const std::vector<double>& costs) {
            std::cout << prColor::Cyan << std::setw(20) << "\t" + name << ": (" << GetMean(costs) << ", " << GetStd(costs) << ")"<< prColor::End << '\n';
        };

        print_cost("State Cost", state_costs);
        print_cost("Action Cost", action_costs);
        print_cost("Shift Cost", shift_costs);
        print_cost("Terminal Cost", terminal_costs);
        print_cost("Map Cost", map_costs);
        print_cost("Subgoal Cost", subgoal_costs);
        print_cost("Velocity Cost", velocity_costs);
        print_cost("Heading Cost", heading_costs);
        std::cout << prColor::Cyan << std::setw(20) << "\tConstrained Candidates" << ": " << _cost.const_indices << prColor::End << '\n';
    }

    return sample_costs;
}

void PathTracker::_UpdateSolution(const std::vector<Action>& weighted_noise) {
    std::vector<Action> solution(_us.size());
    
    for (int t = 0; t < _dynamics.GetT(); t++) {
        solution[t] = _us[t] + weighted_noise[t];
    }

    _PubAction(solution[0]);

    for (int t = 1; t < _dynamics.GetT(); t++) {
        _us[t-1] = solution[t];
    }

    _us[_dynamics.GetT() - 1] = solution[_dynamics.GetT() - 1];
    _prev_u = _us[0];
}

void PathTracker::_ClipAction(Action& action) {
    const double maxV = _dynamics.GetMaxLinVel();
    const double maxW = _dynamics.GetMaxAngVel();
    
    action.vx = std::clamp(action.vx, -maxV, maxV);
    action.wz = std::clamp(action.wz, -maxW, maxW);
}

State PathTracker::_GetRefState(const std::vector<PathStamp>& ref, const State& cur_state, int t) {
    
    auto target_yaw = _path_reg->GetTangentialAngle(ref[t].x);

    State ref_state;
    ref_state.x = ref[t].x;
    ref_state.y = ref[t].y;
    ref_state.v = ref[t].gear * _target_speed;
    ref_state.yaw = target_yaw;

    return ref_state;
}

double PathTracker::GetCTE() {
    constexpr double eps = 1e-6;
    
    int next_target_ind = std::min(_target_ind + 1, static_cast<int>(_path.size()) - 1);
    
    // Current segment direction
    std::vector<double> direction{
        _path[next_target_ind].x - _path[_target_ind].x,
        _path[next_target_ind].y - _path[_target_ind].y
    };
    double direction_norm = std::hypot(direction[0], direction[1]);
    
    // Vector to robot
    std::vector<double> to_robot{
        _state.x - _path[_target_ind].x,
        _state.y - _path[_target_ind].y
    };
    
    double cte = std::abs(to_robot[0] * direction[1] - to_robot[1] * direction[0]) / (direction_norm + eps);

    // Previous segment check
    int prev_target_ind = std::max(_target_ind - 1, 0);
    std::vector<double> prev_direction{
        _path[_target_ind].x - _path[prev_target_ind].x,
        _path[_target_ind].y - _path[prev_target_ind].y
    };
    double prev_direction_norm = std::hypot(prev_direction[0], prev_direction[1]);
    
    std::vector<double> prev_to_robot{
        _state.x - _path[prev_target_ind].x,
        _state.y - _path[prev_target_ind].y
    };
    
    double prev_cte = std::abs(prev_to_robot[0] * prev_direction[1] - 
                              prev_to_robot[1] * prev_direction[0]) /
                      (prev_direction_norm + eps);

    double error = std::min(cte, prev_cte);

    return error;
}

bool PathTracker::_IsGoal() {
    return std::abs(static_cast<int>(_path.size()) - 1 - _target_ind) <= _goal_thres;
}

void PathTracker::ResetPathTracker() {

    _target_ind = 0;
    _prev_ind = 0;
    _is_forward = true;

    _cost.Reset();

    _us.clear();
    _us.reserve(_dynamics.GetT());

    for (int t = 0; t < _dynamics.GetT(); t++) {
        _us.emplace_back(0, 0);
    }

    if (!_path_gen->GetActive()) {
        _path.clear();
    }
    
    _prev_u = 0;
    _avg_cte = 0;
    _PubAction(_prev_u);

    _hist.clear();
    _path_hist.clear();

    // Reset twist calculation variables
    _prev_pose_initialized = false;
    _prev_timestamp = this->now();

    _path_reg->Reset();
}

void PathTracker::_UpdateSubgoal(const std::vector<PathStamp>& ref){
    auto lp_subgoal = ref.back();
    auto lp_global_subgoal = PathTracker::_ChangeLocal2Global(lp_subgoal);

    _subgoal = lp_subgoal;
    _global_subgoal = lp_global_subgoal;

    // cylinder -> target
    if (_vis_subgoal) {
        PathTracker::_VisLpSubgoal(lp_global_subgoal);    // red
    }
}

void PathTracker::PrintConfig() {
    std::cout << "======================================================" << std::endl;
    std::cout << "Path Tracker:" << std::endl;

    std::cout << "\tframe_id : " << _frame_id << std::endl;
    std::cout << "\tverbose : " << _verbose << std::endl;
    std::cout << "\taverage : " << _average << std::endl;

    std::cout << "\tvis_segment : " << _vis_segment << std::endl;
    std::cout << "\tvis_reference : " << _vis_reference << std::endl;
    std::cout << "\tvis_candidate : " << _vis_candidate << std::endl;
    std::cout << "\tvis_hist : " << _vis_hist << std::endl;
    std::cout << "\tvis_poly : " << _vis_poly << std::endl;
    std::cout << "\tvis_subgoal : " << _vis_subgoal << std::endl;
    std::cout << "\tnum_vis : " << _num_vis << std::endl;
    std::cout << "\tdebug_cost : " << _debug_cost << std::endl;

    std::cout << "MPPI:" << std::endl;
    std::cout << "\tK : " << _K << std::endl;
    std::cout << "\tWindow : " << _Window << std::endl;
    std::cout << "\tRegressor_Type : " << _Regressor_Type << std::endl;

    std::cout << "\ttarget_speed : " << _target_speed << std::endl;
    std::cout << "\tn_ind_search : " << _n_ind_search << std::endl;
    std::cout << "\tDL : " << _DL << std::endl;
    std::cout << "\tAlp : " << _Alp << std::endl;
    std::cout << "\tGam : " << _Gam << std::endl;
    std::cout << "\tLam : " << _Lam << std::endl;
    std::cout << "\tCov : " << _Cov << std::endl;

    std::cout << "\tcustom_path_name : " << _custom_path_name << std::endl;

    std::cout << "\ttracking_thres : " << _tracking_thres << std::endl;
    std::cout << "\tgoal_thres : " << _goal_thres << std::endl;

    std::cout << "\tpreserved_path : " << _preserved_path << std::endl;
    std::cout << "\tdynamic_config : " << _reconfigure << std::endl;

    _dynamics.PrintConfig();
    _cost.PrintConfig();
    _path_reg->PrintConfig();
    
    std::cout << "======================================================" << std::endl;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    std::string pkg_loc = ament_index_cpp::get_package_share_directory("path_tracker");
    
    rclcpp::executors::SingleThreadedExecutor executor;
    auto node = std::make_shared<PathTracker>(pkg_loc);
    
    executor.add_node(node);

    RCLCPP_INFO(rclcpp::get_logger("path_tracker"), "%sPath Tracker is successfully initialized%s", prColor::Cyan, prColor::End);

    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}