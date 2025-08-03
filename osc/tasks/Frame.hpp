
#pragma once

#include "osc/Task.hpp"

namespace osc {

struct FrameTarget {
    FrameTarget()
        : pose(SE3::Identity()),
          velocity(Motion::Zero()),
          acceleration(Motion::Zero()) {}

    FrameTarget(const SE3 &pose, const Motion &velocity = Motion::Zero(),
                const Motion &acceleration = Motion::Zero())
        : pose(pose), velocity(velocity), acceleration(acceleration) {}

    SE3 pose;
    Motion velocity;
    Motion acceleration;
};

/**
 * @brief Task for tracking a specified frame
 *
 */
class FrameTask : public Task<FrameTarget> {
    static constexpr Size DIMENSION = 6;

   public:
    FrameTask(const String &frame);

    const String &frame() const { return frame_; }

    void setTargetFromState(const State &state) {
        setTarget(FrameTarget(state.getTransformFrameToWorld(frame_)));
    }

    void computeError(const State &state, Eigen::Ref<Vector> e,
                      Eigen::Ref<Vector> dot_e) const override;

    void computeJacobian(const State &state,
                         Eigen::Ref<Matrix> jac) const override;

    void computeAccelerationBias(const State &state,
                                 Eigen::Ref<Vector> bias) const override;

   private:
    String frame_;
};

}  // namespace osc