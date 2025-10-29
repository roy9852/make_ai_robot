#include "path_tracker.hpp"

void PathTracker::_VisHist() {
    auto path_msg = std::make_unique<nav_msgs::msg::Path>();
    
    for (const auto& stamp : _hist) {
        geometry_msgs::msg::PoseStamped pose_stamp;
        pose_stamp.pose.position = stamp.pose.position;
        path_msg->poses.push_back(pose_stamp);
    }
    
    path_msg->header = _GetHeader();
    _hist_pub->publish(std::move(path_msg));
}

void PathTracker::_VisPath(const std::vector<State>& path_segment) {
    auto path_msg = std::make_unique<nav_msgs::msg::Path>();
    
    for (const auto& stamp : path_segment) {
        geometry_msgs::msg::PoseStamped pose_stamp;
        pose_stamp.pose.position.x = stamp.x;
        pose_stamp.pose.position.y = stamp.y;
        pose_stamp.pose.position.z = stamp.z;
        path_msg->poses.push_back(pose_stamp);
    }
    
    path_msg->header = _GetHeader();
    _dummy_path_pub->publish(std::move(path_msg));
}

void PathTracker::_VisPath(const std::vector<PathStamp>& path_segment) {
    auto path_msg = std::make_unique<nav_msgs::msg::Path>();
    
    for (const auto& stamp : path_segment) {
        geometry_msgs::msg::PoseStamped pose_stamp;
        pose_stamp.pose.position.x = stamp.x;
        pose_stamp.pose.position.y = stamp.y;
        pose_stamp.pose.position.z = stamp.z;
        path_msg->poses.push_back(pose_stamp);
    }
    
    path_msg->header = _GetHeader();
    _dummy_path_pub->publish(std::move(path_msg));
}

void PathTracker::_VisPath(const std::vector<geometry_msgs::msg::Point>& path_segment) {
    auto path_msg = std::make_unique<nav_msgs::msg::Path>();
    
    for (const auto& stamp : path_segment) {
        geometry_msgs::msg::PoseStamped pose_stamp;
        pose_stamp.pose.position = stamp;
        path_msg->poses.push_back(pose_stamp);
    }
    
    path_msg->header = _GetHeader();
    _dummy_path_pub->publish(std::move(path_msg));
}

void PathTracker::_VisPoly(const std::vector<geometry_msgs::msg::Point>& path_segment) {
    auto path_msg = std::make_unique<nav_msgs::msg::Path>();
    
    for (const auto& stamp : path_segment) {
        geometry_msgs::msg::PoseStamped pose_stamp;
        pose_stamp.pose.position = stamp;
        path_msg->poses.push_back(pose_stamp);
    }
    
    path_msg->header = _GetHeader();
    _poly_pub->publish(std::move(path_msg));
}

void PathTracker::_VisSegment(const std::vector<PathStamp>& path_segment) {
    auto path_msg = std::make_unique<nav_msgs::msg::Path>();
    
    for (const auto& stamp : path_segment) {
        geometry_msgs::msg::PoseStamped pose_stamp;
        pose_stamp.pose.position.x = stamp.x;
        pose_stamp.pose.position.y = stamp.y;
        pose_stamp.pose.position.z = stamp.z;
        path_msg->poses.push_back(pose_stamp);
    }
    
    path_msg->header = _GetHeader();
    _segment_pub->publish(std::move(path_msg));
}

void PathTracker::_VisReference(const std::vector<geometry_msgs::msg::Point>& ref_xs) {
    int T = static_cast<int>(_dynamics.GetT());
    int num_path = std::min(_num_vis, _K);
    
    std::vector<std::vector<geometry_msgs::msg::Point>> resized_xs;
    std::vector<geometry_msgs::msg::Point> point_seq;
    
    for (const auto& xs : ref_xs) {
        point_seq.push_back(xs);
        if (point_seq.size() == T) {
            resized_xs.push_back(point_seq);
            point_seq.clear();
            if (resized_xs.size() == num_path) {
                break;
            }
        }
    }

    auto ref_msg = std::make_unique<visualization_msgs::msg::MarkerArray>();
    auto header = _GetHeader();
    int id = 0;
    
    for (const auto& xs : resized_xs) {
        visualization_msgs::msg::Marker line_strip;
        line_strip.header = header;
        line_strip.id = id++;
        line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_strip.action = visualization_msgs::msg::Marker::ADD;
        line_strip.scale.x = 0.12;
        line_strip.color.r = 1.0;
        line_strip.color.g = 0.4;
        line_strip.color.b = 0.8;
        line_strip.color.a = 0.5;
        line_strip.lifetime = rclcpp::Duration(1, 0);
        line_strip.pose.orientation.w = 1.0;
        line_strip.points = xs;
        
        ref_msg->markers.push_back(line_strip);
    }
    
    _reference_pub->publish(std::move(ref_msg));
}

void PathTracker::_VisCandidate(
    const std::vector<geometry_msgs::msg::Point>& pred_xs,
    const std::vector<double>& costs,
    const bool constraint) {
    
    int T = static_cast<int>(_dynamics.GetT());
    int num_path = std::min(_num_vis, _K);
    
    std::vector<std::vector<geometry_msgs::msg::Point>> resized_xs;
    std::vector<geometry_msgs::msg::Point> point_seq;
    
    for (const auto& xs : pred_xs) {
        point_seq.push_back(xs);
        if (point_seq.size() == T) {
            resized_xs.push_back(point_seq);
            point_seq.clear();
            if (resized_xs.size() == num_path) {
                break;
            }
        }
    }

    auto candidate_msg = std::make_unique<visualization_msgs::msg::MarkerArray>();
    auto header = _GetHeader();
    int id = 0;
    
    for (const auto& xs : resized_xs) {
        visualization_msgs::msg::Marker line_strip;
        line_strip.header = header;
        line_strip.id = id++;
        line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_strip.action = visualization_msgs::msg::Marker::ADD;
        line_strip.scale.x = 0.01;
        line_strip.color.r = 0.706;
        line_strip.color.g = 1.0;
        line_strip.color.b = 0.475;
        line_strip.color.a = 0.5;
        line_strip.lifetime = rclcpp::Duration(1, 0);
        line_strip.pose.orientation.w = 1.0;
        line_strip.points = xs;
        
        candidate_msg->markers.push_back(line_strip);
    }

    if (constraint) {
        for (auto& marker : candidate_msg->markers) {
            if (std::find(_cost.const_indices.begin(),
                         _cost.const_indices.end(),
                         marker.id) != _cost.const_indices.end()) {
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
            }
        }
    }
    
    _candidate_pub->publish(std::move(candidate_msg));
}

void PathTracker::_VisLpSubgoal(const geometry_msgs::msg::Point& lp_subgoal) {
    auto goal = std::make_unique<visualization_msgs::msg::Marker>();
    goal->header = _GetHeader();
    goal->type = visualization_msgs::msg::Marker::CYLINDER;
    goal->action = visualization_msgs::msg::Marker::ADD;
    goal->scale.x = 0.5;
    goal->scale.y = 0.5;
    goal->scale.z = 0.2;
    goal->pose.position = lp_subgoal;
    goal->color.r = 1.0;
    goal->color.a = 1.0;
    
    _lp_subgoal_pub->publish(std::move(goal));
}

std_msgs::msg::Header PathTracker::_GetHeader() {
    std_msgs::msg::Header header;
    header.stamp = this->now();
    header.frame_id = _frame_id;
    return header;
}