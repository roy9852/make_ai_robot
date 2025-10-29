#include "dummy_path.hpp"

DummyPathGenerator::DummyPathGenerator(const std::string& pkg_loc, double dl, bool active, double curvature, double length, int type)
: _pkg_loc(pkg_loc), _dl(dl), _active(active), _curvature(curvature), _length(length), _type(type) {}

std::vector<PathStamp> DummyPathGenerator::GetForwardStraightPath() {
    std::vector<PathStamp> dummy_path;
    int n_step = _length / _dl;

    PathStamp stamp{};
    stamp.gear = 1;
    dummy_path.push_back(stamp);

    for (int step = 0; step < n_step; step++) {
        stamp.x = stamp.x + stamp.gear * _dl * std::cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * std::sin(stamp.yaw);
        dummy_path.push_back(stamp);
    }
    return dummy_path;
}

std::vector<PathStamp> DummyPathGenerator::GetBackwardStraightPath() {
    std::vector<PathStamp> dummy_path;
    int n_step = _length / _dl;

    PathStamp stamp{};
    stamp.gear = -1;
    dummy_path.push_back(stamp);

    for (int step = 0; step < n_step; step++) {
        stamp.x = stamp.x + stamp.gear * _dl * std::cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * std::sin(stamp.yaw);
        dummy_path.push_back(stamp);
    }
    return dummy_path;
}

std::vector<PathStamp> DummyPathGenerator::GetForwardTurnRightPath() {
    std::vector<PathStamp> dummy_path;
    int n_step = _length / _dl;

    PathStamp stamp{};
    stamp.gear = 1;
    dummy_path.push_back(stamp);

    for (int step = 0; step < n_step; step++) {
        stamp.x = stamp.x + stamp.gear * _dl * std::cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * std::sin(stamp.yaw);
        stamp.yaw = stamp.yaw - stamp.gear * _dl * _curvature;
        dummy_path.push_back(stamp);
    }
    return dummy_path;
}

std::vector<PathStamp> DummyPathGenerator::GetForwardTurnLeftPath() {
    std::vector<PathStamp> dummy_path;
    int n_step = _length / _dl;

    PathStamp stamp{};
    stamp.gear = 1;
    dummy_path.push_back(stamp);

    for (int step = 0; step < n_step; step++) {
        stamp.x = stamp.x + stamp.gear * _dl * std::cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * std::sin(stamp.yaw);
        stamp.yaw = stamp.yaw + stamp.gear * _dl * _curvature;
        dummy_path.push_back(stamp);
    }
    return dummy_path;
}

std::vector<PathStamp> DummyPathGenerator::GetBackwardTurnRightPath() {
    std::vector<PathStamp> dummy_path;
    int n_step = static_cast<int>(_length / _dl);

    PathStamp stamp{};
    stamp.x = 0.0;
    stamp.y = 0.0;
    stamp.z = 0.0;
    stamp.yaw = 0.0;
    stamp.gear = -1.0;
    dummy_path.push_back(stamp);

    for (int step = 0; step < n_step; step++) {
        stamp.x = stamp.x + stamp.gear * _dl * std::cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * std::sin(stamp.yaw);
        stamp.yaw = stamp.yaw - stamp.gear * _dl * _curvature;
        dummy_path.push_back(stamp);
    }
    return dummy_path;
}

std::vector<PathStamp> DummyPathGenerator::GetBackwardTurnLeftPath() {
    std::vector<PathStamp> dummy_path;
    int n_step = static_cast<int>(_length / _dl);

    PathStamp stamp{};
    stamp.x = 0.0;
    stamp.y = 0.0;
    stamp.z = 0.0;
    stamp.yaw = 0.0;
    stamp.gear = -1.0;
    dummy_path.push_back(stamp);

    for (int step = 0; step < n_step; step++) {
        stamp.x = stamp.x + stamp.gear * _dl * std::cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * std::sin(stamp.yaw);
        stamp.yaw = stamp.yaw + stamp.gear * _dl * _curvature;
        dummy_path.push_back(stamp);
    }
    return dummy_path;
}


std::vector<PathStamp> DummyPathGenerator::GetForwardInftyPath() {
    std::vector<PathStamp> dummy_path;
    int n_step = 2 * M_PI / _curvature / _dl;

    PathStamp stamp{};
    stamp.gear = 1;
    dummy_path.push_back(stamp);

    // First curve
    for (int step = 0; step < n_step; step++) {
        stamp.x = stamp.x + stamp.gear * _dl * std::cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * std::sin(stamp.yaw);
        stamp.yaw = stamp.yaw + stamp.gear * _dl * _curvature;
        dummy_path.push_back(stamp);
    }

    // Second curve
    for (int step = 0; step < n_step; step++) {
        stamp.x = stamp.x + stamp.gear * _dl * std::cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * std::sin(stamp.yaw);
        stamp.yaw = stamp.yaw - stamp.gear * _dl * _curvature;
        dummy_path.push_back(stamp);
    }
    return dummy_path;
}

std::vector<PathStamp> DummyPathGenerator::GetBackwardInftyPath() {
    std::vector<PathStamp> dummy_path;
    int n_step = static_cast<int>(2.0 * M_PI / _curvature / _dl);

    PathStamp stamp{};
    stamp.x = 0.0;
    stamp.y = 0.0;
    stamp.z = 0.0;
    stamp.yaw = 0.0;
    stamp.gear = -1.0;
    dummy_path.push_back(stamp);

    // First half
    for (int step = 0; step < n_step; step++) {
        stamp.x = stamp.x + stamp.gear * _dl * std::cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * std::sin(stamp.yaw);
        stamp.yaw = stamp.yaw + stamp.gear * _dl * _curvature;
        dummy_path.push_back(stamp);
    }

    // Second half
    for (int step = 0; step < n_step; step++) {
        stamp.x = stamp.x + stamp.gear * _dl * std::cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * std::sin(stamp.yaw);
        stamp.yaw = stamp.yaw - stamp.gear * _dl * _curvature;
        dummy_path.push_back(stamp);
    }
    return dummy_path;
}

std::vector<PathStamp> DummyPathGenerator::GetForwardRandomPath() {
    std::normal_distribution<double> dist(0.0, 1.0);
    double length = std::clamp(2.0 * dist(_gen) + M_PI, 1.0, 2.0 * M_PI);
    int n_step = static_cast<int>(length / _curvature / _dl);

    std::vector<PathStamp> dummy_path;
    PathStamp stamp{};
    stamp.x = 0.0;
    stamp.y = 0.0;
    stamp.z = 0.0;
    stamp.yaw = 0.0;
    stamp.gear = 1.0;
    dummy_path.push_back(stamp);

    // First half
    for (int step = 0; step < n_step; step++) {
        stamp.x = stamp.x + stamp.gear * _dl * std::cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * std::sin(stamp.yaw);
        stamp.yaw = stamp.yaw + stamp.gear * _dl * _curvature;
        dummy_path.push_back(stamp);
    }

    // Second half
    for (int step = 0; step < n_step; step++) {
        stamp.x = stamp.x + stamp.gear * _dl * std::cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * std::sin(stamp.yaw);
        stamp.yaw = stamp.yaw - stamp.gear * _dl * _curvature;
        dummy_path.push_back(stamp);
    }
    return dummy_path;
}

std::vector<PathStamp> DummyPathGenerator::GetBackwardRandomPath() {
    std::normal_distribution<double> dist(0.0, 1.0);
    double length = std::clamp(2.0 * dist(_gen) + M_PI, 1.0, 2.0 * M_PI);
    int n_step = static_cast<int>(length / _curvature / _dl);

    std::vector<PathStamp> dummy_path;
    PathStamp stamp{};
    stamp.x = 0.0;
    stamp.y = 0.0;
    stamp.z = 0.0;
    stamp.yaw = 0.0;
    stamp.gear = -1.0;
    dummy_path.push_back(stamp);

    // First half
    for (int step = 0; step < n_step; step++) {
        stamp.x = stamp.x + stamp.gear * _dl * std::cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * std::sin(stamp.yaw);
        stamp.yaw = stamp.yaw + stamp.gear * _dl * _curvature;
        dummy_path.push_back(stamp);
    }

    // Second half
    for (int step = 0; step < n_step; step++) {
        stamp.x = stamp.x + stamp.gear * _dl * std::cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * std::sin(stamp.yaw);
        stamp.yaw = stamp.yaw - stamp.gear * _dl * _curvature;
        dummy_path.push_back(stamp);
    }
    return dummy_path;
}

std::vector<PathStamp> DummyPathGenerator::GetForwardTrianglePath() {
    std::vector<PathStamp> dummy_path;
    int n_step = static_cast<int>(_length / _dl);

    PathStamp stamp{};
    stamp.x = 0.0;
    stamp.y = 0.0;
    stamp.z = 0.0;
    stamp.yaw = 0.0;
    stamp.gear = 1.0;
    dummy_path.push_back(stamp);

    // First edge
    for (int step = 0; step < n_step; step++) {
        stamp.x = stamp.x + stamp.gear * _dl * std::cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * std::sin(stamp.yaw);
        dummy_path.push_back(stamp);
    }

    // Second edge
    stamp.yaw = stamp.yaw - 2.0 * M_PI / 3.0;
    for (int step = 0; step < n_step; step++) {
        stamp.x = stamp.x + stamp.gear * _dl * std::cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * std::sin(stamp.yaw);
        dummy_path.push_back(stamp);
    }

    // Third edge
    stamp.yaw = stamp.yaw - 2.0 * M_PI / 3.0;
    for (int step = 0; step < n_step; step++) {
        stamp.x = stamp.x + stamp.gear * _dl * std::cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * std::sin(stamp.yaw);
        dummy_path.push_back(stamp);
    }

    return dummy_path;
}

std::vector<PathStamp> DummyPathGenerator::GetForwardSquarePath() {
    std::vector<PathStamp> dummy_path;
    int n_step = static_cast<int>(_length / _dl);

    PathStamp stamp{};
    stamp.x = 0.0;
    stamp.y = 0.0;
    stamp.z = 0.0;
    stamp.yaw = 0.0;
    stamp.gear = 1.0;
    dummy_path.push_back(stamp);

    // First side
    for (int step = 0; step < n_step; step++) {
        stamp.x = stamp.x + stamp.gear * _dl * std::cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * std::sin(stamp.yaw);
        dummy_path.push_back(stamp);
    }

    // Second side
    stamp.yaw = stamp.yaw + M_PI / 2;
    for (int step = 0; step < n_step; step++) {
        stamp.x = stamp.x + stamp.gear * _dl * std::cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * std::sin(stamp.yaw);
        dummy_path.push_back(stamp);
    }

    // Third side
    stamp.yaw = stamp.yaw + M_PI / 2;
    for (int step = 0; step < n_step; step++) {
        stamp.x = stamp.x + stamp.gear * _dl * std::cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * std::sin(stamp.yaw);
        dummy_path.push_back(stamp);
    }

    // Fourth side
    stamp.yaw = stamp.yaw + M_PI / 2;
    for (int step = 0; step < n_step; step++) {
        stamp.x = stamp.x + stamp.gear * _dl * std::cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * std::sin(stamp.yaw);
        dummy_path.push_back(stamp);
    }

    return dummy_path;
}

std::vector<PathStamp> DummyPathGenerator::GetForwardPentagonPath() {
    std::vector<PathStamp> dummy_path;
    int n_step = static_cast<int>(_length / _dl);

    PathStamp stamp{};
    stamp.x = 0.0;
    stamp.y = 0.0;
    stamp.z = 0.0;
    stamp.yaw = 0.0;
    stamp.gear = 1.0;
    dummy_path.push_back(stamp);

    // Generate five sides
    for (int side = 0; side < 5; side++) {
        for (int step = 0; step < n_step; step++) {
            stamp.x = stamp.x + stamp.gear * _dl * std::cos(stamp.yaw);
            stamp.y = stamp.y + stamp.gear * _dl * std::sin(stamp.yaw);
            dummy_path.push_back(stamp);
        }
        stamp.yaw = stamp.yaw + 2.0 * M_PI / 5.0;
    }

    return dummy_path;
}

std::vector<PathStamp> DummyPathGenerator::GetForwardHourGlassPath() {
    std::vector<PathStamp> dummy_path;
    int n_step = static_cast<int>(_length / _dl);

    PathStamp stamp{};
    stamp.x = 0.0;
    stamp.y = 0.0;
    stamp.z = 0.0;
    stamp.yaw = 0.0;
    stamp.gear = 1.0;
    dummy_path.push_back(stamp);

    // First segment
    for (int step = 0; step < n_step; step++) {
        stamp.x = stamp.x + stamp.gear * _dl * std::cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * std::sin(stamp.yaw);
        dummy_path.push_back(stamp);
    }

    // Diagonal segments
    const double angle_step = 2.0 * M_PI / 3.0;
    for (int i = 0; i < 2; i++) {
        stamp.yaw = stamp.yaw - angle_step;
        for (int step = 0; step < n_step; step++) {
            stamp.x = stamp.x + stamp.gear * _dl * std::cos(stamp.yaw);
            stamp.y = stamp.y + stamp.gear * _dl * std::sin(stamp.yaw);
            dummy_path.push_back(stamp);
        }
    }

    // Connecting segment
    for (int step = 0; step < n_step; step++) {
        stamp.x = stamp.x + stamp.gear * _dl * std::cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * std::sin(stamp.yaw);
        dummy_path.push_back(stamp);
    }

    // Return segments
    for (int i = 0; i < 2; i++) {
        stamp.yaw = stamp.yaw + angle_step;
        for (int step = 0; step < n_step; step++) {
            stamp.x = stamp.x + stamp.gear * _dl * std::cos(stamp.yaw);
            stamp.y = stamp.y + stamp.gear * _dl * std::sin(stamp.yaw);
            dummy_path.push_back(stamp);
        }
    }

    return dummy_path;
}

std::vector<PathStamp> DummyPathGenerator::GetBackwardTrianglePath() {
    std::vector<PathStamp> dummy_path;
    int n_step = static_cast<int>(_length / _dl);

    PathStamp stamp{};
    stamp.x = 0.0;
    stamp.y = 0.0;
    stamp.z = 0.0;
    stamp.yaw = 0.0;
    stamp.gear = -1.0;
    dummy_path.push_back(stamp);

    // Generate three sides
    for (int side = 0; side < 3; side++) {
        for (int step = 0; step < n_step; step++) {
            stamp.x = stamp.x + stamp.gear * _dl * std::cos(stamp.yaw);
            stamp.y = stamp.y + stamp.gear * _dl * std::sin(stamp.yaw);
            dummy_path.push_back(stamp);
        }
        stamp.yaw = stamp.yaw - 2.0 * M_PI / 3.0;
    }

    return dummy_path;
}

std::vector<PathStamp> DummyPathGenerator::GetBackwardSquarePath() {
    std::vector<PathStamp> dummy_path;
    int n_step = static_cast<int>(_length / _dl);

    PathStamp stamp{};
    stamp.x = 0.0;
    stamp.y = 0.0;
    stamp.z = 0.0;
    stamp.yaw = 0.0;
    stamp.gear = -1.0;
    dummy_path.push_back(stamp);

    // Generate four sides
    for (int side = 0; side < 4; side++) {
        for (int step = 0; step < n_step; step++) {
            stamp.x = stamp.x + stamp.gear * _dl * std::cos(stamp.yaw);
            stamp.y = stamp.y + stamp.gear * _dl * std::sin(stamp.yaw);
            dummy_path.push_back(stamp);
        }
        stamp.yaw = stamp.yaw + M_PI / 2.0;
    }

    return dummy_path;
}

std::vector<PathStamp> DummyPathGenerator::GetBackwardPentagonPath() {
    std::vector<PathStamp> dummy_path;
    int n_step = static_cast<int>(_length / _dl);

    PathStamp stamp{};
    stamp.x = 0.0;
    stamp.y = 0.0;
    stamp.z = 0.0;
    stamp.yaw = 0.0;
    stamp.gear = -1.0;
    dummy_path.push_back(stamp);

    // Generate five sides
    for (int side = 0; side < 5; side++) {
        for (int step = 0; step < n_step; step++) {
            stamp.x = stamp.x + stamp.gear * _dl * std::cos(stamp.yaw);
            stamp.y = stamp.y + stamp.gear * _dl * std::sin(stamp.yaw);
            dummy_path.push_back(stamp);
        }
        stamp.yaw = stamp.yaw + 2.0 * M_PI / 5.0;
    }

    return dummy_path;
}

std::vector<PathStamp> DummyPathGenerator::GetBackwardHourGlassPath() {
    std::vector<PathStamp> dummy_path;
    int n_step = static_cast<int>(_length / _dl);

    PathStamp stamp{};
    stamp.x = 0.0;
    stamp.y = 0.0;
    stamp.z = 0.0;
    stamp.yaw = 0.0;
    stamp.gear = -1.0;
    dummy_path.push_back(stamp);

    // First segment
    for (int step = 0; step < n_step; step++) {
        stamp.x = stamp.x + stamp.gear * _dl * std::cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * std::sin(stamp.yaw);
        dummy_path.push_back(stamp);
    }

    // Diagonal segments
    const double angle_step = 2.0 * M_PI / 3.0;
    for (int i = 0; i < 2; i++) {
        stamp.yaw = stamp.yaw - angle_step;
        for (int step = 0; step < n_step; step++) {
            stamp.x = stamp.x + stamp.gear * _dl * std::cos(stamp.yaw);
            stamp.y = stamp.y + stamp.gear * _dl * std::sin(stamp.yaw);
            dummy_path.push_back(stamp);
        }
    }

    // Return path segments
    for (int i = 0; i < 2; i++) {
        stamp.yaw = stamp.yaw + angle_step;
        for (int step = 0; step < n_step; step++) {
            stamp.x = stamp.x + stamp.gear * _dl * std::cos(stamp.yaw);
            stamp.y = stamp.y + stamp.gear * _dl * std::sin(stamp.yaw);
            dummy_path.push_back(stamp);
        }
    }

    return dummy_path;
}

std::vector<PathStamp> DummyPathGenerator::GetReturnStraightPath() {
    std::vector<PathStamp> dummy_path;
    int n_step = static_cast<int>(_length / _dl);

    PathStamp stamp{};
    stamp.x = 0.0;
    stamp.y = 0.0;
    stamp.z = 0.0;
    stamp.yaw = 0.0;
    stamp.gear = 1.0;
    dummy_path.push_back(stamp);

    // Forward path
    for (int step = 0; step < n_step; step++) {
        stamp.x = stamp.x + stamp.gear * _dl * std::cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * std::sin(stamp.yaw);
        dummy_path.push_back(stamp);
    }

    // Change direction
    stamp.gear = -1.0;
    dummy_path.push_back(stamp);

    // Return path
    for (int step = 0; step < n_step; step++) {
        stamp.x = stamp.x + stamp.gear * _dl * std::cos(stamp.yaw);
        stamp.y = stamp.y + stamp.gear * _dl * std::sin(stamp.yaw);
        dummy_path.push_back(stamp);
    }

    return dummy_path;
}

void DummyPathGenerator::SaveCustomPath(const std::vector<PathStamp>& path, const std::string& file_name) {
    std::filesystem::path dir_path(_pkg_loc + "/path");
    if (!std::filesystem::exists(dir_path)) {
        std::filesystem::create_directories(dir_path);
    }

    std::string file_path = dir_path / file_name;
    std::ofstream file(file_path);
    
    file << "x,y,z,yaw,gear\n";
    for (const auto& stamp : path) {
        file << stamp.x << "," << stamp.y << "," << stamp.z << ","
             << stamp.yaw << "," << stamp.gear << "\n";
    }
    file.close();

    RCLCPP_INFO(rclcpp::get_logger("dummy_path_generator"), "Custom Path saved to: %s", file_path.c_str());
}

std::vector<PathStamp> DummyPathGenerator::LoadCustomPath(const std::string& file_name) {
    std::vector<PathStamp> dummy_path;
    std::filesystem::path file_path(_pkg_loc + "/path/" + file_name);

    if (!std::filesystem::exists(file_path)) {
        RCLCPP_ERROR(rclcpp::get_logger("dummy_path_generator"), "File not found: %s", file_path.c_str());
        return dummy_path;
    }

    std::ifstream file(file_path);
    std::string line;
    bool first_line = true;

    while (std::getline(file, line)) {
        if (first_line) {
            first_line = false;
            continue;
        }

        std::stringstream ss(line);
        std::string value;
        std::vector<double> values;

        while (std::getline(ss, value, ',')) {
            values.push_back(std::stod(value));
        }

        if (values.size() == 5) {
            PathStamp stamp;
            stamp.x = values[0];
            stamp.y = values[1];
            stamp.z = values[2];
            stamp.yaw = values[3];
            stamp.gear = values[4];
            dummy_path.push_back(stamp);
        }
    }
    return dummy_path;
}

std::tuple<std::vector<PathStamp>, State> DummyPathGenerator::ProcessCustomPath(Transform& trans, const std::vector<geometry_msgs::msg::PoseStamped>& history) {
    std::vector<PathStamp> dummy_path;
    State base_state;

    for (size_t idx = 0; idx < history.size(); idx++) {
        if (idx == 0) {
            dummy_path.push_back(PoseStampedToStamp(trans, history[idx]));
            base_state = trans.GetState(history[idx].pose.position,
                                      history[idx].pose.orientation,
                                      geometry_msgs::msg::Twist()); // Default twist
            continue;
        }

        geometry_msgs::msg::Vector3 res;
        res.x = history[idx].pose.position.x - dummy_path.back().x;
        res.y = history[idx].pose.position.y - dummy_path.back().y;
        res.z = 0;

        auto dist = std::sqrt(res.x * res.x + res.y * res.y + res.z * res.z);
        if (dist >= _dl) {
            dummy_path.push_back(PoseStampedToStamp(trans, history[idx]));
        }
    }
    return {dummy_path, base_state};
}

PathStamp DummyPathGenerator::OdometryToStamp(Transform& trans, const nav_msgs::msg::Odometry& odom) {
    auto state = trans.GetState(odom.pose.pose.position, odom.pose.pose.orientation, odom.twist.twist);
    return state.toStamp();
}

PathStamp DummyPathGenerator::PoseStampedToStamp(Transform& trans, const geometry_msgs::msg::PoseStamped& pose) {
    auto state = trans.GetState(pose.pose.position, pose.pose.orientation, geometry_msgs::msg::Twist());
    return state.toStamp();
}

void DummyPathGenerator::SetActive(bool active) {
    _active = active;
    if (!active) {
        flag = false;
    }
}

void DummyPathGenerator::PrintConfig() {
    std::cout << "======================================================" << std::endl;
    std::cout << "Dummy Path Generator : " << std::endl;

    std::cout << "\tActive : " << _active << std::endl;
    std::cout << "\tType : " << _type << std::endl;
    std::cout << "\tCurvature : " << _curvature << std::endl;
    std::cout << "\tLength : " << _length << std::endl;
    std::cout << "======================================================" << std::endl;
}