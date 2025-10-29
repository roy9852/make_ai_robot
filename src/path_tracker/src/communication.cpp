#include "path_tracker.hpp"

void PathTracker::_LocalPathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    _path.clear();
    for (const auto& pose : msg->poses) {
        PathStamp stamp;
        stamp.x = pose.pose.position.x;
        stamp.y = pose.pose.position.y;
        stamp.z = pose.pose.position.z;

        auto angle = _trans.GetEulerFromQuaternion(pose.pose.orientation);
        stamp.yaw = angle.yaw;
        
        if (pose.header.frame_id == "D") {
            stamp.gear = 1.0;
        } else if (pose.header.frame_id == "R") {
            stamp.gear = -1.0;
        } else if (pose.header.frame_id == "N") {
            stamp.gear = 0.0;
        } else {
            auto init_yaw = _trans.GetEulerFromQuaternion(msg->poses[0].pose.orientation).yaw;
            if (std::cos(_state.yaw - init_yaw) > 0) {
                stamp.gear = 1.0;
            } else if (std::cos(_state.yaw - init_yaw) == 0) {
                stamp.gear = 0.0;
            } else {
                stamp.gear = -1.0;
            }
        }
        _path.push_back(stamp);
    }
}

void PathTracker::_LocalizationCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    _trans.SetPos(msg->pose.position);
    _trans.SetOrient(msg->pose.orientation);
    
    // Check if this is the first call
    RCLCPP_INFO(get_logger(), "=== Localization Callback ===");
    RCLCPP_INFO(get_logger(), "  Previous pose initialized: %s", (_prev_pose_initialized ? "true" : "false"));
    RCLCPP_INFO(get_logger(), "  Current position: (%.3f, %.3f, %.3f)", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    RCLCPP_INFO(get_logger(), "  Header stamp: %d.%09d", msg->header.stamp.sec, msg->header.stamp.nanosec);
    
    // Calculate twist from pose changes
    geometry_msgs::msg::Twist calculated_twist = _CalculateTwistFromPose(msg);
    
    // NAN check and safe twist setting
    if (std::isnan(calculated_twist.linear.x) || std::isnan(calculated_twist.linear.y) || std::isnan(calculated_twist.linear.z) ||
        std::isnan(calculated_twist.angular.x) || std::isnan(calculated_twist.angular.y) || std::isnan(calculated_twist.angular.z) ||
        std::isinf(calculated_twist.linear.x) || std::isinf(calculated_twist.linear.y) || std::isinf(calculated_twist.linear.z) ||
        std::isinf(calculated_twist.angular.x) || std::isinf(calculated_twist.angular.y) || std::isinf(calculated_twist.angular.z)) {
        
        geometry_msgs::msg::Twist safe_twist;
        safe_twist.linear.x = 0.0; safe_twist.linear.y = 0.0; safe_twist.linear.z = 0.0;
        safe_twist.angular.x = 0.0; safe_twist.angular.y = 0.0; safe_twist.angular.z = 0.0;
        
        _trans.SetTwist(safe_twist);
        RCLCPP_WARN(get_logger(), "NAN/INF WARNING from calculated velocity! Using safe default values.");
    } else {
        _trans.SetTwist(calculated_twist);
    }

    if (_path_gen->GetActive()) {
        _hist.push_back(*msg);
    } else {
        if (_vis_hist && (_path.size() > 2)) {
            _hist.push_back(*msg);
            _path_hist.push_back(_path.at(_target_ind));
            _VisPath(_path_hist);
            _VisHist();
        } else {
            _hist.clear();
            _path_hist.clear();
        }
    }
}

void PathTracker::_ClickedGoalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    _subgoal_changed = true;
    if (_subgoal_changed) {
        ResetPathTracker();
        _subgoal_changed = false;
    }
    std::cout << prColor::Yellow << "Goal Changed" << prColor::End << std::endl;
}

void PathTracker::_EmergencyCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    _emergency_situation = msg->data;
}

void PathTracker::_CostMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    _cost.SetCost(*msg);
    _cost.map_arrived = true;
}


rcl_interfaces::msg::SetParametersResult PathTracker::ParametersCallback(const std::vector<rclcpp::Parameter>& parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto& param : parameters) {
        if (param.get_name() == "dummy_path") {
            bool dummy_path = param.as_bool();
            if (dummy_path) {
                bool dummy_activate = get_parameter("dummy_activate").as_bool();
                _path_gen->SetActive(dummy_activate);
                if (!dummy_activate) {
                    ProcessDummyDeactivation();
                }
            }
        }
        else if (param.get_name() == "dummy_activate") {
            bool dummy_activate = param.as_bool();
            bool dummy_path = get_parameter("dummy_path").as_bool();
            if (dummy_path) {
                _path_gen->SetActive(dummy_activate);
                if (!dummy_activate) {
                    ProcessDummyDeactivation();
                }
            }
        }
        else if (param.get_name() == "dummy_type" && _path_gen) {
            _path_gen->SetType(param.as_int());
        }
        else if (param.get_name() == "dummy_curvature" && _path_gen) {
            _path_gen->SetCurvature(param.as_double());
        }
        else if (param.get_name() == "dummy_length" && _path_gen) {
            _path_gen->SetLength(param.as_double());
        }
        else {
            UpdateParams(param);
        }
    }

    _path_gen->PrintConfig();
    PrintConfig();

    return result;
}

void PathTracker::ProcessDummyDeactivation() {
    PathTracker::ResetPathTracker();
    if (static_cast<PathType>(_path_gen->GetType()) == PathType::SaveCustomPath) {
        if (!_hist.empty()) {
            auto path_info = _path_gen->ProcessCustomPath(_trans, _hist);
            auto custom_path = _ChangeGlobal2LocalPath(std::get<0>(path_info), std::get<1>(path_info));
            _path_gen->SaveCustomPath(custom_path, _custom_path_name);
        } else {
            RCLCPP_INFO(get_logger(), "No Path to Save");
        }
    }
    _hist.clear();
}


void PathTracker::UpdateParams(const rclcpp::Parameter& param) {
    const auto& name = param.get_name();
    
    if (name == "verbose") {
        _verbose = param.as_bool();
        _path_reg->SetVerbose(param.as_bool());
    }
    else if (name == "vis_hist") _vis_hist = param.as_bool();
    else if (name == "K") _K = param.as_int();
    else if (name == "average") _average = param.as_int();
    else if (name == "T") _dynamics.SetT(param.as_int());
    else if (name == "n_ind_search") _n_ind_search = param.as_int();
    else if (name == "DT") _dynamics.SetDT(param.as_double());
    else if (name == "target_speed") _target_speed = param.as_double();
    else if (name == "Lam") {
        _Lam = param.as_double();
        _Alp = 1 - _Gam / _Lam;
    }
    else if (name == "Gam") {
        _Gam = param.as_double();
        _Alp = 1 - _Gam / _Lam;
    }
    else if (name == "Cov_Lin_Vel") _Cov[0] = param.as_double();
    else if (name == "Cov_Ang_Vel") _Cov[1] = param.as_double();
    else if (name == "Q_Pos") {
        std::vector<double> Q = _cost.GetQ();
        Q[0] = Q[1] = param.as_double();
        _cost.SetQ(Q);
    }
    else if (name == "Q_Vel") {
        std::vector<double> Q = _cost.GetQ();
        Q[2] = param.as_double();
        _cost.SetQ(Q);
    }
    else if (name == "Q_Yaw") {
        std::vector<double> Q = _cost.GetQ();
        Q[3] = param.as_double();
        _cost.SetQ(Q);
    }
    else if (name == "Qf_Pos") {
        std::vector<double> Qf = _cost.GetQf();
        Qf[0] = Qf[1] = param.as_double();
        _cost.SetQf(Qf);
    }
    else if (name == "Qf_Vel") {
        std::vector<double> Qf = _cost.GetQf();
        Qf[2] = param.as_double();
        _cost.SetQf(Qf);
    }
    else if (name == "Qf_Yaw") {
        std::vector<double> Qf = _cost.GetQf();
        Qf[3] = param.as_double();
        _cost.SetQf(Qf);
    }
    else if (name == "R_Lin_Vel") {
        std::vector<double> R = _cost.GetR();
        R[0] = param.as_double();
        _cost.SetR(R);
    }
    else if (name == "R_Ang_Vel") {
        std::vector<double> R = _cost.GetR();
        R[1] = param.as_double();
        _cost.SetR(R);
    }
    else if (name == "enable_stamped_cmd_vel") {
        _enable_stamped_cmd_vel = param.as_bool();
    }
}
void PathTracker::_PubAction(const Action& solved_action) {
    // cmd_vel은 항상 출력
    auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
    twist_msg->linear.x = solved_action.vx;
    twist_msg->angular.z = solved_action.wz;
    _twist_pub->publish(std::move(twist_msg));
    
    // cmd_vel_stamped는 플래그로 조절
    if (_enable_stamped_cmd_vel) {
        auto twist_stamped_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
        twist_stamped_msg->header.stamp = this->now();
        twist_stamped_msg->header.frame_id = _frame_id;
        twist_stamped_msg->twist.linear.x = solved_action.vx;
        twist_stamped_msg->twist.angular.z = solved_action.wz;
        _twist_stamped_pub->publish(std::move(twist_stamped_msg));
    }
}

void PathTracker::PubTrackingInfo(double cte) {
    auto tracking_info_msg = std::make_unique<geometry_msgs::msg::Point>();
    tracking_info_msg->x = cte;
    tracking_info_msg->y = _target_speed;
    tracking_info_msg->z = _state.v;
    _tracking_info_pub->publish(std::move(tracking_info_msg));

    if (_verbose) {
        _avg_state = _avg_state * (_average - 1) / _average + _state * 1 / _average;
        _avg_cte = _avg_cte * (_average - 1) / _average + cte * 1 / _average;

        std::cout << prColor::Yellow << "Tracking Info: " << prColor::End << std::endl;
        std::cout << prColor::Yellow << std::setw(20) << "\tCur CTE" << ": " << cte << prColor::End << std::endl;
        std::cout << prColor::Yellow << std::setw(20) << "\tCur Vel" << ": " << _state.v << prColor::End << std::endl;
        std::cout << prColor::Yellow << std::setw(20) << "\tAvg CTE" << ": " << _avg_cte << prColor::End << std::endl;
        std::cout << prColor::Yellow << std::setw(20) << "\tAvg Vel" << ": " << _avg_state.v << prColor::End << std::endl;
    }
}

void PathTracker::PubZeroAction() {
    Action zero(0, 0);
    _PubAction(zero);
}

geometry_msgs::msg::Twist PathTracker::_CalculateTwistFromPose(
    const geometry_msgs::msg::PoseStamped::SharedPtr& msg) {

    geometry_msgs::msg::Twist twist;  // 기본 0 초기화
    auto curr = rclcpp::Time(msg->header.stamp);

    if (!_prev_pose_initialized) {
        _prev_pose = *msg;
        _prev_timestamp = curr;
        _prev_pose_initialized = true;
        return twist;
    }

    // rclcpp::Time/Duration 연산으로 dt 계산 (초 단위)
    const double dt = (curr - _prev_timestamp).seconds();
    
    // 시간 변화 디버그 출력 추가
    RCLCPP_INFO(get_logger(), "=== Twist 계산 디버그 ===");
    RCLCPP_INFO(get_logger(), "  현재 시간: %.6f 초", curr.seconds());
    RCLCPP_INFO(get_logger(), "  이전 시간: %.6f 초", _prev_timestamp.seconds());
    RCLCPP_INFO(get_logger(), "  시간 차이 (dt): %.6f 초", dt);
    RCLCPP_INFO(get_logger(), "  현재 위치: (%.3f, %.3f, %.3f)", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    RCLCPP_INFO(get_logger(), "  이전 위치: (%.3f, %.3f, %.3f)", _prev_pose.pose.position.x, _prev_pose.pose.position.y, _prev_pose.pose.position.z);

    // dt 가드 (역행/정지/이상치)
    if (!std::isfinite(dt) || dt <= 0.0 || dt < 1e-6 || dt > 1.0) {
        // 필요하면 dt>1.0 임계값은 센서 Hz에 맞게 조정(예: 0.2~0.5 등)
        // 역행/정지에서는 속도 0으로 리턴하고 상태만 갱신
        _prev_pose = *msg;
        _prev_timestamp = curr;
        return twist;
    }

    // 선속도
    const double dx = msg->pose.position.x - _prev_pose.pose.position.x;
    const double dy = msg->pose.position.y - _prev_pose.pose.position.y;
    const double dz = msg->pose.position.z - _prev_pose.pose.position.z;
    twist.linear.x = dx / dt;
    twist.linear.y = dy / dt;
    twist.linear.z = dz / dt;

    Eigen::Matrix3d rotation_matrix = _trans.GetRotationFromQuaternion(_prev_pose.pose.orientation).block<3,3>(0,0);
    Eigen::Vector3d world_velocity(twist.linear.x, twist.linear.y, twist.linear.z);
    Eigen::Vector3d robot_velocity = rotation_matrix.transpose() * world_velocity;

    twist.linear.x = robot_velocity.x();
    twist.linear.y = robot_velocity.y();
    twist.linear.z = robot_velocity.z();
    
    RCLCPP_INFO(get_logger(), "  위치 변화: dx=%.6f, dy=%.6f, dz=%.6f", dx, dy, dz);
    RCLCPP_INFO(get_logger(), "  계산된 선속도: vx=%.6f, vy=%.6f, vz=%.6f", twist.linear.x, twist.linear.y, twist.linear.z);

    // 각속도 (yaw 래핑 처리)
    const double cy = _GetYawFromQuaternion(msg->pose.orientation);
    const double py = _GetYawFromQuaternion(_prev_pose.pose.orientation);
    double dyaw = cy - py;
    if (dyaw > M_PI)  dyaw -= 2*M_PI;
    if (dyaw < -M_PI) dyaw += 2*M_PI;
    twist.angular.z = dyaw / dt;
    
    RCLCPP_INFO(get_logger(), "  현재 yaw: %.6f rad", cy);
    RCLCPP_INFO(get_logger(), "  이전 yaw: %.6f rad", py);
    RCLCPP_INFO(get_logger(), "  yaw 변화: %.6f rad", dyaw);
    RCLCPP_INFO(get_logger(), "  계산된 각속도: wz=%.6f rad/s", twist.angular.z);
    RCLCPP_INFO(get_logger(), "=========================");

    _prev_pose = *msg;
    _prev_timestamp = curr;
    return twist;
}


double PathTracker::_GetYawFromQuaternion(const geometry_msgs::msg::Quaternion& q) {
    // Convert quaternion to yaw (rotation around z-axis)
    return atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}