#include "action.hpp"

std::vector<double> Action::toVector() {
    return {vx, wz};
}

std::vector<double> Action::toVector() const {
    return {vx, wz};
}

Action Action::operator+(const double& num) const {
    return Action(vx + num, wz + num);
}

Action Action::operator-(const double& num) const {
    return Action(vx - num, wz - num);
}

Action Action::operator*(const double& num) const {
    return Action(vx * num, wz * num);
}

Action Action::operator/(const double& num) const {
    return Action(vx / num, wz / num);
}

Action Action::operator+(const Action& action) const {
    return Action(vx + action.vx, wz + action.wz);
}

Action Action::operator-(const Action& action) const {
    return Action(vx - action.vx, wz - action.wz);
}

Action Action::operator*(const Action& action) const {
    return Action(vx * action.vx, wz * action.wz);
}

Action Action::operator/(const Action& action) const {
    return Action(vx / action.vx, wz / action.wz);
}

Action& Action::operator=(const Action& action) = default;

Action& Action::operator=(const double& num) {
    vx = num;
    wz = num;
    return *this;
}

Action& Action::operator+=(const double& num) {
    *this = *this + num;
    return *this;
}

Action& Action::operator-=(const double& num) {
    *this = *this - num;
    return *this;
}

Action& Action::operator*=(const double& num) {
    *this = *this * num;
    return *this;
}

Action& Action::operator/=(const double& num) {
    *this = *this / num;
    return *this;
}

Action& Action::operator+=(const Action& action) {
    *this = *this + action;
    return *this;
}

Action& Action::operator-=(const Action& action) {
    *this = *this - action;
    return *this;
}

Action& Action::operator*=(const Action& action) {
    *this = *this * action;
    return *this;
}

Action& Action::operator/=(const Action& action) {
    *this = *this / action;
    return *this;
}

std::ostream& operator<<(std::ostream& os, const Action& action) {
    os << "[" << action.vx << ", " << action.wz << "]";
    return os;
}