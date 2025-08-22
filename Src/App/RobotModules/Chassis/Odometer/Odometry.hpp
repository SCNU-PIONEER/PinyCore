#pragma once

namespace CHASSIS {
class Odometry {
public:
    /**
     * @note Axis plot:
     *       ^ +x
     *       |
     *       |
     *       |
     *       +------> -y
     *      <---
     *      |  |  +theta
     *      ---> 
     * @note The coordinate system is the same as the robot.
     * @note theta is 0 when the robot is facing the positive direction of the x-axis.
     * 
     */
    struct State_s {
        float x;
        float y;
        float theta;
    };

    Odometry() = default;

    void reset() { state_ = State_s{}; }
    const State_s &state() const { return state_; }

protected:
    State_s state_{};
};
} // namespace CHASSIS
