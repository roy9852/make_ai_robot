#ifndef DUMMY_PATH_HPP
#define DUMMY_PATH_HPP

#include <random>
#include <fstream>
#include <filesystem>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "misc.hpp"

enum class PathType {
    ForwardStraight = 0,
    BackwardStraight = 1,
    ForwardTurnRight = 2,
    ForwardTurnLeft = 3,
    BackwardTurnRight = 4,
    BackwardTurnLeft = 5,
    ForwardInfty = 6,
    BackwardInfty = 7,
    ForwardRandom = 8,
    BackwardRandom = 9,
    ForwardTriangle = 10,
    ForwardSquare = 11,
    ForwardPentagon = 12,
    ForwardHourGlass = 13,
    BackwardTriangle = 14,
    BackwardSquare = 15,
    BackwardPentagon = 16,
    BackwardHourGlass = 17,
    ReturnStraight = 18,
    SaveCustomPath = 19,
    CustomPath = 20
};

class DummyPathGenerator {
    public:
        DummyPathGenerator(const std::string& pkg_loc, double dl = 0.1, bool active = false, double curvature = 1.0, double length = 5.0, int type = 0);
        DummyPathGenerator() = default;
        ~DummyPathGenerator() = default;

        // Path generation methods
        std::vector<PathStamp> GetForwardStraightPath();
        std::vector<PathStamp> GetBackwardStraightPath();
        std::vector<PathStamp> GetForwardTurnRightPath();
        std::vector<PathStamp> GetForwardTurnLeftPath();
        std::vector<PathStamp> GetBackwardTurnRightPath();
        std::vector<PathStamp> GetBackwardTurnLeftPath();
        std::vector<PathStamp> GetForwardInftyPath();
        std::vector<PathStamp> GetBackwardInftyPath();
        std::vector<PathStamp> GetForwardRandomPath();
        std::vector<PathStamp> GetBackwardRandomPath();
        std::vector<PathStamp> GetForwardTrianglePath();
        std::vector<PathStamp> GetForwardSquarePath();
        std::vector<PathStamp> GetForwardPentagonPath();
        std::vector<PathStamp> GetForwardHourGlassPath();
        std::vector<PathStamp> GetBackwardTrianglePath();
        std::vector<PathStamp> GetBackwardSquarePath();
        std::vector<PathStamp> GetBackwardPentagonPath();
        std::vector<PathStamp> GetBackwardHourGlassPath();
        std::vector<PathStamp> GetReturnStraightPath();

        void SaveCustomPath(const std::vector<PathStamp>& path, const std::string& file_name);
        std::vector<PathStamp> LoadCustomPath(const std::string& file_name);
        std::tuple<std::vector<PathStamp>, State> ProcessCustomPath(Transform& trans, const std::vector<geometry_msgs::msg::PoseStamped>& history);

        PathStamp OdometryToStamp(Transform& trans, const nav_msgs::msg::Odometry& odom);
        PathStamp PoseStampedToStamp(Transform& trans, const geometry_msgs::msg::PoseStamped& pose);

        // Setters & Getters
        void SetActive(bool active);
        void SetType(int type) { _type = type; }
        void SetCurvature(double curvature) { _curvature = curvature; }
        void SetLength(double length) { _length = length; }

        bool GetActive() const { return _active; }
        int GetType() const { return _type; }
        double GetCurvature() const { return _curvature; }
        double GetLength() const { return _length; }

        std::string GetPkgPath() const { return _pkg_loc; }

        void PrintConfig();

        bool flag{false};

    private:
        std::string _pkg_loc;
        bool _active{false};
        double _dl{0.1};
        double _curvature{1.0};
        double _length{5.0};
        int _type{0};
        
        std::default_random_engine _gen;
};

#endif