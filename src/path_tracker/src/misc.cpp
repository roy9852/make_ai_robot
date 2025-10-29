#include "misc.hpp"

const char* prColor::Red = "\033[91m";
const char* prColor::Green = "\033[92m";
const char* prColor::Yellow = "\033[93m";
const char* prColor::LightPurple = "\033[94m";
const char* prColor::Purple = "\033[95m";
const char* prColor::Cyan = "\033[96m";
const char* prColor::LightGray = "\033[97m";
const char* prColor::End = "\033[00m";

double GetNorm(geometry_msgs::msg::Vector3& vec) {
    return std::sqrt(std::pow(vec.x, 2) + std::pow(vec.y, 2) + std::pow(vec.z, 2));
}

std::ostream& operator<<(std::ostream& os, const PathStamp& pt) {
    os << "[" << pt.x << ", " << pt.y << ", " << pt.z << "]";
    return os;
}

void CustomException::report(const ErrorType& e) {
    switch (e) {
        case ErrorType::NotImplementedPath:
            RCLCPP_INFO(rclcpp::get_logger("path_tracker"), "%sNotImplementedPath Error%s", prColor::Red, prColor::End);
            break;
        case ErrorType::NotFoundTargetPath:
            RCLCPP_INFO(rclcpp::get_logger("path_tracker"), "%sNotFoundTargetPath Error%s", prColor::Red, prColor::End);
            break;
        case ErrorType::NotFoundReference:
            RCLCPP_INFO(rclcpp::get_logger("path_tracker"), "%sNotFoundReference Error%s", prColor::Red, prColor::End);
            break;
        case ErrorType::GoalArrived:
            RCLCPP_INFO(rclcpp::get_logger("path_tracker"), "%sGoalArrived%s", prColor::Green, prColor::End);
            break;
        case ErrorType::NotFoundMapValue:
            RCLCPP_INFO(rclcpp::get_logger("path_tracker"), "%sNotFoundMapValue Error%s", prColor::Red, prColor::End);
            break;
        case ErrorType::NotFoundRegressorType:
            RCLCPP_INFO(rclcpp::get_logger("path_tracker"), "%sNotFoundRegressorType Error%s", prColor::Red, prColor::End);
            break;
        case ErrorType::NotInitalizeRegressor:
            RCLCPP_INFO(rclcpp::get_logger("path_tracker"), "%sNotInitalizeRegressor Error%s", prColor::Red, prColor::End);
            break;
        case ErrorType::NotSolvedRegressor:
            RCLCPP_INFO(rclcpp::get_logger("path_tracker"), "%sNotSolvedRegressor Error%s", prColor::Red, prColor::End);
            break;
        case ErrorType::EmergencySituation:
            RCLCPP_INFO(rclcpp::get_logger("path_tracker"), "%sEmergencySituation%s", prColor::Red, prColor::End);
            break;
        case ErrorType::OutOdometry:
            RCLCPP_INFO(rclcpp::get_logger("path_tracker"), "%sOutOdometry%s", prColor::Red, prColor::End);
            break;
        case ErrorType::OutTargetPath:
            RCLCPP_INFO(rclcpp::get_logger("path_tracker"), "%sOutTargetPath%s", prColor::Red, prColor::End);
            break;
    }
}

double GetMean(const std::vector<double>& vec) {
    if (vec.empty()) return 0.0;
    return std::accumulate(vec.begin(), vec.end(), 0.0) / static_cast<double>(vec.size());
}

double GetStd(const std::vector<double>& vec, const double mean) {
    if (vec.empty()) return 0.0;
    double sum = std::accumulate(vec.begin(), vec.end(), 0.0,
        [mean](double acc, double val) { return acc + std::pow(val - mean, 2); });
    return std::sqrt(sum / static_cast<double>(vec.size()));
}

double GetStd(const std::vector<double>& vec) {
    double mean = GetMean(vec);
    return GetStd(vec, mean);
}

double Pi2Pi(double angle) {
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}