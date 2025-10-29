#ifndef TRANSFORM_HPP
#define TRANSFORM_HPP

#include <cmath>
#include <vector>
#include <Eigen/Dense>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include "state.hpp"
#include "misc.hpp"

struct EulerAngle;
class State;

class Transform {
public:
    geometry_msgs::msg::Point ChangePointCoordinate(double x, double y, double z, Eigen::Matrix4d rot);
    geometry_msgs::msg::Point ChangePointCoordinate(double x, double y, double z, Eigen::Matrix3d rot);

    std::vector<geometry_msgs::msg::Point> TransformWithPosQuaternion(std::vector<geometry_msgs::msg::Point> pts, geometry_msgs::msg::Point p, geometry_msgs::msg::Quaternion q);
    std::vector<geometry_msgs::msg::Point> TransformWithPosEuler(std::vector<geometry_msgs::msg::Point> pts, geometry_msgs::msg::Point p, EulerAngle e);

    std::vector<geometry_msgs::msg::Point> RotateWithQuaternion(std::vector<geometry_msgs::msg::Point> pts, geometry_msgs::msg::Quaternion q);
    std::vector<geometry_msgs::msg::Point> RotateWithEuler(std::vector<geometry_msgs::msg::Point> pts, EulerAngle e);

    EulerAngle GetEulerFromQuaternion(geometry_msgs::msg::Quaternion q);
    geometry_msgs::msg::Quaternion GetQuaternionFromEuler(EulerAngle e);

    EulerAngle GetEulerFromRotation(Eigen::Matrix3d rot);
    geometry_msgs::msg::Quaternion GetQuaternionFromRotation(Eigen::Matrix3d rot);

    Eigen::Matrix3d GetRotationFromEuler(EulerAngle e);
    Eigen::Matrix4d GetRotationFromQuaternion(geometry_msgs::msg::Quaternion q);

    geometry_msgs::msg::Quaternion MultiplyQuaternion(geometry_msgs::msg::Quaternion p, geometry_msgs::msg::Quaternion q);

    // Getters
    geometry_msgs::msg::Point GetPos() const { return _pos; }
    geometry_msgs::msg::Quaternion GetOrient() const { return _orient; }
    geometry_msgs::msg::Twist GetTwist() const { return _twist; }

    // Setters
    void SetPos(geometry_msgs::msg::Point p) { _pos = p; }
    void SetOrient(geometry_msgs::msg::Quaternion q) { _orient = q; }
    void SetTwist(geometry_msgs::msg::Twist t) { _twist = t; }
    void SetTwist(const geometry_msgs::msg::TwistStamped& t) { _twist = t.twist; }

    State GetState(geometry_msgs::msg::Point p, geometry_msgs::msg::Quaternion q, geometry_msgs::msg::Twist t);

private:
    geometry_msgs::msg::Point _pos;
    geometry_msgs::msg::Quaternion _orient;
    geometry_msgs::msg::Twist _twist;
};

#endif