#ifndef MISC_HPP
#define MISC_HPP

#include <cmath>
#include <string>
#include <vector>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#include "state.hpp"

class CustomException : public std::exception {
    public:
        enum class ErrorType {
            NotImplementedPath,
            GoalArrived,
            NotFoundTargetPath,
            NotFoundReference,
            NotFoundMapValue,
            NotFoundRegressorType,
            NotInitalizeRegressor,
            NotSolvedRegressor,
            EmergencySituation,
            OutOdometry,
            OutTargetPath,
        };

        static void report(const ErrorType& e);
};

struct EulerAngle {
    double roll{0.0}, pitch{0.0}, yaw{0.0};
};

class PathStamp {
    public:
        double x{0.0};
        double y{0.0};
        double z{0.0};
        double yaw{0.0};
        double gear{0.0};

        friend std::ostream& operator<<(std::ostream& os, const PathStamp& pt);
};

namespace prColor {
    extern const char* Red;
    extern const char* Green;
    extern const char* Yellow;
    extern const char* LightPurple;
    extern const char* Purple;
    extern const char* Cyan;
    extern const char* LightGray;
    extern const char* End;
}

template <typename T>
std::ostream& operator<<(std::ostream& os, const std::vector<T>& vec) {
    os.precision(3);
    os << "[";
    int cnt = 1;
    for(auto it = vec.begin(); it != vec.end(); it++) {
        os << *it;
        if(it != vec.end() - 1) {
            os << ", ";
        }
        if(cnt % 5 == 0 && cnt != static_cast<int>(vec.size())) {
            os << "\n";
        }
        cnt = cnt + 1;
    }
    os << "]";
    return os;
}

template <typename T>
std::vector<T> slicing(const std::vector<T>& arr, int begin_ind, int end_ind) {
    end_ind = std::min(end_ind, static_cast<int>(arr.size()));
    auto start = arr.begin() + begin_ind;
    auto end = arr.begin() + end_ind;
    std::vector<T> result(end_ind - begin_ind);
    std::copy(start, end, result.begin());
    return result;
}

double GetNorm(geometry_msgs::msg::Vector3& vec);
double GetMean(const std::vector<double>& vec);
double GetStd(const std::vector<double>& vec, const double mean);
double GetStd(const std::vector<double>& vec);
double Pi2Pi(double angle);

#endif