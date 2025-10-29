#ifndef STATE_HPP
#define STATE_HPP

#include <cmath>
#include <vector>
#include <Eigen/Dense>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "misc.hpp"
#include "action.hpp"
#include "transform.hpp"

class PathStamp;

class State {
    public:
        State() = default;
        
        State(double x, double y, double z, double yaw, double v, double w, geometry_msgs::msg::Quaternion q)
        : x(x), y(y), z(z), yaw(yaw), v(v), w(w), orient(q) {}

        State(double x, double y, double v, double yaw)
        : x(x), y(y), v(v), yaw(yaw) {}

        State(const std::vector<double>& vecState)
        : x(vecState.at(0)), y(vecState.at(1)), v(vecState.at(2)), yaw(vecState.at(3)) {}

        ~State() = default;

        double x{0}, y{0}, z{0}, yaw{0}, v{0}, w{0};
        geometry_msgs::msg::Quaternion orient;

        // Distance calculations
        double GetDistance(const State& state) const;
        double GetDistance(const geometry_msgs::msg::Point& pt) const;
        double GetDistance(const PathStamp& stamp) const;
        double GetDistance(const double& p, const double& q, const double& r) const;

        // Conversion methods
        std::vector<double> toVector();
        std::vector<double> toVector() const;

        geometry_msgs::msg::Point toGeoPoint();
        geometry_msgs::msg::Point toGeoPoint() const;
        geometry_msgs::msg::Point toGlobalPoint(const double robot_yaw, const geometry_msgs::msg::Point& robot_pos);
        geometry_msgs::msg::Point toGlobalPoint(const double robot_yaw, const geometry_msgs::msg::Point& robot_pos) const;
        
        PathStamp toStamp();
        PathStamp toStamp() const;
        State GetLocalState();

        // Operator overloads with scalar
        State operator+(const double& num) const;
        State operator-(const double& num) const;
        State operator*(const double& num) const;
        State operator/(const double& num) const;

        // Operator overloads with State
        State operator+(const State& state) const;
        State operator-(const State& state) const;
        State operator*(const State& state) const;
        State operator/(const State& state) const;

        // Assignment operators
        State& operator=(const double& num);
        State& operator=(const State& state);

        // Compound assignment operators with scalar
        State& operator+=(const double& num);
        State& operator-=(const double& num);
        State& operator*=(const double& num);
        State& operator/=(const double& num);

        // Compound assignment operators with State
        State& operator+=(const State& state);
        State& operator-=(const State& state);
        State& operator*=(const State& state);
        State& operator/=(const State& state);

        friend std::ostream& operator<<(std::ostream& os, const State& state);
};

#endif