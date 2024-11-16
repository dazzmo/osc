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
    typedef typename T::index_type index_type;
    typedef typename T::reference_t reference_t;
};

struct task_attributes {};

/**
 * @brief State of a given system
 *
 * @tparam Vector
 */
template <typename VectorType>
struct state {
    state(const std::size_t &nq, const std::size_t &nv = nq)
        : position(nq), velocity(nv), acceleration(nv) {}

    VectorType position;
    VectorType velocity;
    VectorType acceleration;
};

template <typename VectorType>
struct error {
    state(const std::size_t &sz)
        : positional(sz), integral(sz), derivative(sz) {}

    VectorType positional;
    VectorType integral;
    VectorType derivative;
};

template <typename VectorType>
struct pid_gains {
    pid_gains(const std::size_t &sz) : p(sz), i(sz), d(sz) {}

    constexpr VectorType compute(const pid_error<VectorType> &error) {
        return p * error.positional + i * error.integral + d * error.derivative;
    }

    VectorType p;
    VectorType i;
    VectorType d;
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
template <typename VectorType>
struct task_parameters {
    // Task weighting vector
    VectorType w;
    // Desired task acceleration
    VectorType desired_task_acceleration;
    // Model configuration
    VectorType q;
    // Model velocity
    VectorType v;

    // Additional parameters that can be added
    VectorType additional;
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
    // Typedefs
    typedef double value_type;
    typedef std::size_t index_type;
    typedef int integer_type;

    typedef pinocchio::SE3 se3_t;
    typedef pinocchio::SE3Tpl<sym_t> se3_sym_t;

    typedef pinocchio::MotionTpl<sym_t> motion_sym_t;

    typedef state<eigen_vector_t> state_t;
    typedef state<eigen_vector_sym_t> state_sym_t;

    typedef state_t task_state_t;
    typedef error<eigen_vector_t> task_error_t;

    Task(const index_type &dim) : dimension_(dim), pid(dim) {}

    Task() : dimension_(index_type(0)), pid(index_type(0)) {}

    index_type dimension() const { return dimension_; }

    virtual integer_type get_task_state(const state_t &state,
                                   task_state_t &task_state) = 0;

    virtual integer_type get_task_error(const task_state_t &task_state,
                                   task_error_t &error,
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

    /**
     * @brief PID gains for task tracking
     * 
     */
    pid_gains<eigen_vector_t> pid;

    /**
     * @brief Task parameters
     *
     * @return task_parameters<value_type>&
     */
    task_parameters<eigen_vector_t> &parameters() { return parameters_d; }

   protected:
    // Parameter variables
    task_parameters<eigen_vector_t> parameters_d;
    task_parameters<eigen_vector_var_t> parameters_v;

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

    struct reference {
        eigen_vector_t position;
        eigen_vector_t velocity;
    };

    typedef reference reference_t;

    integer_type get_error(const task_state_t &task_state, task_error_t &error,
                           const value_type &dt = 0.0) {
        error.positional = task_state.position - reference.position;
        error.derivative = task_state.velocity - reference.velocity;
        error.integral += dt * error.positional;
        return integer_type(0);
    }

    reference_t reference;

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

    struct reference {
        eigen_vector_t rotation;
        eigen_vector_t velocity;
    };

    typedef reference reference_t;

    integer_type get_error(const task_state_t &task_state, task_error_t &error,
                           const value_type &dt = 0.0) {
        error.positional = task_state.position - reference.rotation;
        error.derivative = task_state.velocity - reference.velocity;
        error.integral += dt * error.positional;
        return integer_type(0);
    }

    reference_t reference;

    // void set_target(){}

   private:
    bopt::casadi::expression_evaluator<value_type> xpos;
    bopt::casadi::expression_evaluator<value_type> xvel;
    bopt::casadi::expression_evaluator<value_type> xacc;
};

class CentreOfMassTask : public PositionTask {
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

    struct reference {
        se3_t pose;
        twist_t twist;
    };

    typedef reference reference_t;

    integer_type get_error(const task_state_t &task_state, task_error_t &error,
                           const value_type &dt = 0.0) {
        error.positional = task_state.position - reference.pose;
        error.derivative = task_state.velocity - reference.twist;
        error.integral += dt * error.positional;
        return integer_type(0);
    }

    reference_t reference;

    // void set_target(){}

   private:
    bopt::casadi::expression_evaluator<value_type> xpos;
    bopt::casadi::expression_evaluator<value_type> xvel;
    bopt::casadi::expression_evaluator<value_type> xacc;
};

class JointTrackingTask : public Task {
    JointTrackingTask(const model_sym_t &model);

    bopt::quadratic_cost<value_type>::shared_ptr to_task_cost() override;

    struct reference {
        eigen_vector_t joint_position;
        eigen_vector_t joint_velocity;
    };

    typedef reference reference_t;

    integer_type get_error(const task_state_t &task_state, task_error_t &error,
                           const value_type &dt = 0.0) {
        error.positional = task_state.position - reference.joint_position;
        error.derivative = task_state.velocity - reference.joint_velocity;
        error.integral += dt * error.positional;
        return integer_type(0);
    }

    reference_t reference;

   private:
    typename bopt::casadi::expression_evaluator<value_type>
        expression_evaluator_t;
    std::unique_ptr<expression_evaluator_t> xpos;
    std::unique_ptr<expression_evaluator_t> xvel;
    std::unique_ptr<expression_evaluator_t> xacc;
};

}  // namespace osc