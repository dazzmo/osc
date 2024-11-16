#pragma once

#include <bopt/program.h>

#include <Eigen/Core>
#include <bopt/ad/casadi/casadi.hpp>
#include <bopt/constraints.hpp>
#include <bopt/costs.hpp>
#include <bopt/variable.hpp>
#include <casadi/casadi.hpp>

#include "osc/common.hpp"

namespace osc {

template <class T>
struct task_traits {
    typedef typename T::value_type value_type;
};

struct task_attributes {};

/**
 * @brief State of a given system
 *
 * @tparam Vector
 */
template <typename Vector>
struct state {
    typename Vector vector_t;

    state(index_type &nq, index_type &nv = nq)
        : position(vector_t(nq)),
          velocity(vector_t(nv)),
          acceleration(vector_t(nv)) {}

    vector_t position;
    vector_t velocity;
    vector_t acceleration;
};

template <typename Vector>
struct pid_error {
    typedef Vector vector_t;

    vector_t positional;
    vector_t integral;
    vector_t derivative;
};

typedef pid_error<Eigen::Vector3d> task_error;

template <typename Vector>
struct pid_gains {
    typename Vector vector_t;

    pid_gains(index_type &sz)
        : p(vector_t(sz)), i(vector_t(sz)), d(vector_t(sz)) {}

    constexpr vector_t compute(const pid_error<Vector> &error) {
        return p * error.positional + i * error.integral + d * error.derivative;
    }

    vector_t p;
    vector_t i;
    vector_t d;
};

template <typename Scalar>
struct eigen_to_casadi {
    typedef std::size_t index_type;
    typedef casadi::Matrix<Scalar> casadi_vector_t;
    typedef Eigen::VectorX<casadi::Matrix<Scalar>> eigen_vector_t;

    static inline casadi_vector_t convert(const eigen_vector_t &v) {
        casadi_vector_t c(v.rows(), 1);
        for (index_type i = 0; i < v.size(); ++i) {
            // Only fill in non-zero entries
            if (!::casadi::is_zero(v[i]->at(0))) {
                c(i) = v[i]->at(0);
            }
        }
        return c;
    }
};

template <typename Scalar>
struct task_variables {
    // Model accelerations
    std::vector<Scalar> a;
    // Actuation signals
    std::vector<Scalar> u;
    // Constraint forces
    std::vector<Scalar> lambda;
};

/**
 * @brief Parameters within an OSC program for a given task
 *
 */
template <typename Scalar>
struct task_parameters {
    // Task weighting vector
    std::vector<Scalar> w;
    // Desired task acceleration
    std::vector<Scalar> desired_task_acceleration;
    // Model configuration
    std::vector<Scalar> q;
    // Model velocity
    std::vector<Scalar> v;

    // Additional parameters that can be added
    std::vector<Scalar> additional;
};

/**
 * @brief The state of a frame in the group SE3
 *
 * @tparam Scalar
 */
template <typename Scalar>
struct frame_state {
    // Convert parameters to useable vectors
    typedef Eigen::VectorX<Scalar> vector_t;
    typedef pinocchio::SE3Tpl<Scalar> se3_t;
    typedef pinocchio::MotionTpl<Scalar> motion_t;

    se3_t pos;
    motion_t vel;
    motion_t acc;
};

template <typename Scalar>
frame_state<Scalar> get_frame_state(
    const pinocchio::ModelTpl<Scalar> &model, Eigen::VectorX<Scalar> &q,
    Eigen::VectorX<Scalar> &v, Eigen::VectorX<Scalar> &a,
    const std::string &target,
    const std::string &reference_frame = "universe") {
    frame_state<Scalar> frame;

    pinocchio::DataTpl<Scalar> data(model);

    // Compute the kinematic tree state of the system
    pinocchio::forwardKinematics(model, data, q, v, a);
    pinocchio::updateFramePlacements(model, data);

    // Target frame wrt world frame
    se3_t oMf = data.oMf[model.getFrameId(target)];
    // Reference frame wrt world frame
    se3_t oMb = data.oMf[model.getFrameId(reference_frame)];

    motion_t o_xvel = pinocchio::getFrameVelocity(
        model, data, model.getFrameId(target), pinocchio::LOCAL_WORLD_ALIGNED);

    motion_t o_xacc = pinocchio::getFrameClassicalAcceleration(
        model, data, model.getFrameId(frame), pinocchio::LOCAL_WORLD_ALIGNED);

    frame.pos = oMb.actInv(oMf);
    frame.vel = oMb.actInv(o_xvel);
    frame.acc = oMb.actInv(o_xacc);

    return frame;
}

/**
 * @brief
 *
 */
class Task {
   public:
    typedef double value_type;
    typedef std::size_t index_type;
    typedef int integer_type;

    // Convert parameters to useable vectors
    typedef Eigen::VectorX<sym_t> sym_vector_t;
    typedef pinocchio::SE3Tpl<sym_t> se3_t;
    typedef pinocchio::MotionTpl<sym_t> motion_t;

    typedef typename Eigen::VectorX<value_type> vector_t;

    Task(const index_type &nx, const index_type &ndx) : pid(nx) {}

    Task() : dimension_(index_type(0)) {}

    index_type dimension() const { return dimension_; }

    value_type weight() {}

    integer_type pid_gains(vector_t &p, vector_t &i, vector_t &d) {
        pid.p = p;
        pid.i = i;
        pid.d = d;
    }

    virtual integer_type get_state(const model_state &state,
                                   task_state &task_state) {
        // Create parameter input
        std::vector<value_type *> in;
        in = {state.position.data(), state.velocity.data(),
              task_state.position.data()};
        xpos({}, in);
        xvel({});
    }

    virtual integer_type get_error(const task_state &task_state,
                                   task_error &error,
                                   const value_type &dt = 0.0) {
        error.positional = task_state.position - reference.position;
        error.derivative = task_state.velocity - reference.velocity;
        error.integral += dt * error.positional;
        return integer_type(0);
    }

    virtual bopt::quadratic_cost<value_type>::shared_ptr to_task_cost() = 0;

    virtual void add_to_program(
        bopt::mathematical_program<value_type> &program) {
        auto cost = to_task_cost();

        // program.add_parameter();

        // Bind to program
        program.add_quadratic_cost(
            cost, {{}},
            {{parameters_v.q.data(), parameters_v.v.data(),
              parameters_v.desired_task_acceleration.data()}});
    }

    // std::vector<value_type> weight() {}
    pid_gains<eigen_vector_t> pid;

    task_parameters<value_type> &parameters() { return parameters_d; }

    task_parameters<value_type> parameters_d;

   protected:
    // Parameter variables
    task_parameters<bopt::variable> parameters_v;

   private:
    // Dimension of the task
    index_type dimension_;
};

/**
 * @brief Position task
 *
 */
class PositionTask : public Task {
    PositionTask(const model_sym_t &model, const std::string &frame_name,
                 const std::string &reference_frame);

    bopt::quadratic_cost<value_type>::shared_ptr to_task_cost() override;

    const std::string &reference_frame();

    integer_type get_state(const model_state &state,
                           task_state &task_state) override;
    integer_type get_error(const task_state &task_state,
                           task_error &error) override;

    task_state<double> reference;

   private:
    typename bopt::casadi::expression_evaluator<value_type>
        expression_evaluator_t;
    std::unique_ptr<expression_evaluator_t> xpos;
    std::unique_ptr<expression_evaluator_t> xvel;
    std::unique_ptr<expression_evaluator_t> xacc;
};

class OrientationTask : public Task {
    OrientationTask(const model_sym_t &model, const std::string &frame_name,
                    const std::string &reference_frame);

    bopt::quadratic_cost<value_type>::shared_ptr to_task_cost() override;

    const std::string &reference_frame();

    // void set_target(){}

   private:
    bopt::casadi::expression_evaluator<value_type> xpos;
    bopt::casadi::expression_evaluator<value_type> xvel;
    bopt::casadi::expression_evaluator<value_type> xacc;
};

class CentreOfMassTask : public Task {
    CentreOfMassTask(const model_sym_t &model, const std::string &frame_name,
                     const std::string &reference_frame);

    bopt::quadratic_cost<value_type>::shared_ptr to_task_cost() override;

    const std::string &reference_frame();

    // void set_target(){}

   private:
    bopt::casadi::expression_evaluator<value_type> xpos;
    bopt::casadi::expression_evaluator<value_type> xvel;
    bopt::casadi::expression_evaluator<value_type> xacc;
};

class SE3Task : public Task {
    SE3Task(const model_sym_t &model, const std::string &frame_name,
            const std::string &reference_frame);

    bopt::quadratic_cost<value_type>::shared_ptr to_task_cost() override;

    const std::string &reference_frame();

    // void set_target(){}

   private:
    bopt::casadi::expression_evaluator<value_type> xpos;
    bopt::casadi::expression_evaluator<value_type> xvel;
    bopt::casadi::expression_evaluator<value_type> xacc;
};

class JointTrackingTask : public Task {
    JointTrackingTask(const model_sym_t &model);

    bopt::quadratic_cost<value_type>::shared_ptr to_task_cost() override;

    task_state<double> reference;

   private:
    typename bopt::casadi::expression_evaluator<value_type>
        expression_evaluator_t;
    std::unique_ptr<expression_evaluator_t> xpos;
    std::unique_ptr<expression_evaluator_t> xvel;
    std::unique_ptr<expression_evaluator_t> xacc;
};

}  // namespace osc