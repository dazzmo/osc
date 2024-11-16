#pragma once

#include <bopt/program.h>

#include <Eigen/Core>
#include <bopt/ad/casadi/casadi.hpp>
#include <bopt/constraints.hpp>
#include <bopt/costs.hpp>
#include <bopt/variable.hpp>
#include <casadi/casadi.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "osc/common.hpp"

namespace osc {

template <class T>
struct task_traits {
    typedef typename T::value_type value_type;
    typedef typename T::index_type index_type;
    typedef typename T::integer_type integer_type;

    typedef typename T::task_state_t task_state_t;
    typedef typename T::task_error_t task_error_t;
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
    state(const std::size_t &nq, const std::size_t &nv)
        : position(nq), velocity(nv), acceleration(nv) {}

    VectorType position;
    VectorType velocity;
    VectorType acceleration;
};

template <typename VectorType>
struct pid_error {
    pid_error(const std::size_t &sz)
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
struct eigen_to_std_vector {
    static inline std::vector<Scalar> convert(
        const eigen_vector_tpl_t<Scalar> &v) {
        return std::vector<Scalar>(v.data(), v.data() + v.rows() * v.cols());
    }
};

template <typename VectorType>
struct task_variables {
    // Model accelerations
    VectorType a;
    // Actuation signals
    VectorType u;
    // Constraint forces
    VectorType lambda;
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

    typedef pinocchio::SE3Tpl<Scalar> se3_t;
    typedef pinocchio::MotionTpl<Scalar> motion_t;

    pinocchio::DataTpl<Scalar> data(model);

    // Compute the kinematic tree state of the system
    pinocchio::forwardKinematics(model, data, q, v, a);
    pinocchio::updateFramePlacements(model, data);

    // Target frame wrt world frame
    se3_t oMf = data.oMf[model.getFrameId(target)];
    // Reference frame wrt world frame
    se3_t oMb = data.oMf[model.getFrameId(reference_frame)];

    motion_t o_xvel = pinocchio::getFrameVelocity(
        model, data, model.getFrameId(target), pinocchio::WORLD);

    motion_t o_xacc = pinocchio::getFrameClassicalAcceleration(
        model, data, model.getFrameId(target), pinocchio::WORLD);

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

    typedef state<eigen_vector_t> model_state_t;
    typedef pid_error<eigen_vector_t> task_error_t;

    Task(const index_type &dim) : dimension_(dim), pid(dim) {}

    Task() : dimension_(index_type(0)), pid(index_type(0)) {}

    index_type dimension() const { return dimension_; }

    virtual bopt::quadratic_cost<value_type>::shared_ptr to_task_cost() = 0;

    void add_to_program(bopt::mathematical_program<value_type> &program) {
        auto cost = to_task_cost();

        // program.add_parameter();

        // Bind to program
        program.add_quadratic_cost(
            cost,
            // Variables
            {eigen_to_std_vector<bopt::variable>::convert(variables.a)},
            // Parameters
            {eigen_to_std_vector<bopt::variable>::convert(parameters_v.q),
             eigen_to_std_vector<bopt::variable>::convert(parameters_v.v),
             eigen_to_std_vector<bopt::variable>::convert(parameters_v.w),
             eigen_to_std_vector<bopt::variable>::convert(
                 parameters_v.desired_task_acceleration),
             eigen_to_std_vector<bopt::variable>::convert(
                 parameters_v.additional)});
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
    task_variables<eigen_vector_var_t> variables;
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
   public:
    PositionTask(const model_sym_t &model, const std::string &frame_name,
                 const std::string &reference_frame);

    bopt::quadratic_cost<value_type>::shared_ptr to_task_cost() override;

    struct task_state {
        eigen_vector_t position;
        eigen_vector_t velocity;
    };

    typedef task_state task_state_t;
    typedef task_state_t reference_t;

    integer_type get_task_state(const model_state_t &model_state,
                                task_state_t &task_state) const {
        // (*xpos)({model_state.position, model_state.velocity},
        //         {task_state.position.data()});
        // (*xvel)({model_state.position, model_state.velocity},
        //         {task_state.velocity.data()});
        return integer_type(0);
    }

    integer_type get_task_error(const task_state_t &task_state,
                                task_error_t &error,
                                const value_type &dt = 0.0) const {
        error.positional = task_state.position - reference.position;
        error.derivative = task_state.velocity - reference.velocity;
        error.integral += dt * error.positional;
        return integer_type(0);
    }

    reference_t reference;
    string_t reference_frame;

   private:
    typedef bopt::casadi::expression_evaluator<value_type>
        expression_evaluator_t;
    std::unique_ptr<expression_evaluator_t> xpos;
    std::unique_ptr<expression_evaluator_t> xvel;
    std::unique_ptr<expression_evaluator_t> xacc;
};

class OrientationTask : public Task {
   public:
    OrientationTask(const model_sym_t &model, const std::string &frame_name,
                    const std::string &reference_frame);

    bopt::quadratic_cost<value_type>::shared_ptr to_task_cost() override;

    struct task_state {
        eigen_matrix_t rotation;
        eigen_vector_t velocity;
    };

    typedef task_state task_state_t;
    typedef task_state_t reference_t;

    integer_type get_task_state(const model_state_t &model_state,
                                task_state_t &task_state) const {
        // (*xpos)({model_state.position, model_state.velocity},
        //         {task_state.position.data()});
        // (*xvel)({model_state.position, model_state.velocity},
        //         {task_state.velocity.data()});
        return integer_type(0);
    }

    integer_type get_task_error(const task_state_t &task_state,
                                task_error_t &error,
                                const value_type &dt = 0.0) const {
        Eigen::Matrix<double, 3, 3> Jlog;
        error.positional =
            pinocchio::log3(reference.rotation.inverse() * task_state.rotation);
        pinocchio::Jlog3(reference.rotation.inverse() * task_state.rotation,
                         Jlog);

        error.derivative = Jlog * task_state.velocity;

        error.integral += dt * error.positional;
        return integer_type(0);
    }

    reference_t reference;
    string_t reference_frame;

    // void set_target(){}

   private:
    bopt::casadi::expression_evaluator<value_type> xpos;
    bopt::casadi::expression_evaluator<value_type> xvel;
    bopt::casadi::expression_evaluator<value_type> xacc;
};

class CentreOfMassTask : public PositionTask {
   public:
    CentreOfMassTask(const model_sym_t &model, const std::string &frame_name,
                     const std::string &reference_frame);

    bopt::quadratic_cost<value_type>::shared_ptr to_task_cost() override;

    const std::string &reference_frame();

    integer_type get_task_state(const model_state_t &model_state,
                                task_state_t &task_state) const {
        // (*xpos)({model_state.position, model_state.velocity},
        //         {task_state.position.data()});
        // (*xvel)({model_state.position, model_state.velocity},
        //         {task_state.velocity.data()});
        return integer_type(0);
    }

   private:
    bopt::casadi::expression_evaluator<value_type> xpos;
    bopt::casadi::expression_evaluator<value_type> xvel;
    bopt::casadi::expression_evaluator<value_type> xacc;
};

class SE3Task : public Task {
   public:
    SE3Task(const model_sym_t &model, const std::string &frame_name,
            const std::string &reference_frame);

    bopt::quadratic_cost<value_type>::shared_ptr to_task_cost() override;

    typedef pinocchio::SE3 se3_t;
    typedef pinocchio::Motion twist_t;

    struct task_state {
        se3_t pose;
        twist_t twist;
    };

    typedef task_state task_state_t;
    typedef task_state_t reference_t;

    integer_type get_task_state(const model_state_t &model_state,
                                task_state_t &task_state) const {
        // (*xpos)({model_state.position, model_state.velocity},
        //         {task_state.position.data()});
        // (*xvel)({model_state.position, model_state.velocity},
        //         {task_state.velocity.data()});
        return integer_type(0);
    }

    integer_type get_task_error(const task_state_t &task_state,
                                task_error_t &error,
                                const value_type &dt = 0.0) const {
        Eigen::Matrix<double, 6, 6> Jlog;
        error.positional =
            pinocchio::log6(reference.pose.actInv(task_state.pose)).toVector();
        pinocchio::Jlog6(reference.pose.actInv(task_state.pose), Jlog);

        error.derivative = Jlog * task_state.twist.toVector();

        error.integral += dt * error.positional;
        return integer_type(0);
    }

    reference_t reference;

    string_t reference_frame;



   private:
    bopt::casadi::expression_evaluator<value_type> xpos;
    bopt::casadi::expression_evaluator<value_type> xvel;
    bopt::casadi::expression_evaluator<value_type> xacc;
};

class JointTrackingTask : public Task {
    JointTrackingTask(const model_sym_t &model);

    bopt::quadratic_cost<value_type>::shared_ptr to_task_cost() override;

    struct task_state {
        eigen_vector_t joint_position;
        eigen_vector_t joint_velocity;
    };

    typedef task_state task_state_t;
    typedef task_state_t reference_t;

    integer_type get_task_state(const model_state_t &model_state,
                                task_state_t &task_state) const {
        task_state.joint_position = model_state.position;
        task_state.joint_velocity = model_state.velocity;
        return integer_type(0);
    }

    integer_type get_task_error(const task_state_t &task_state,
                                task_error_t &error,
                                const value_type &dt = 0.0) const {
        error.positional = task_state.joint_position - reference.joint_position;
        error.derivative = task_state.joint_position - reference.joint_velocity;
        error.integral += dt * error.positional;
        return integer_type(0);
    }

    reference_t reference;

   private:
    typedef bopt::casadi::expression_evaluator<value_type>
        expression_evaluator_t;
    std::unique_ptr<expression_evaluator_t> xpos;
    std::unique_ptr<expression_evaluator_t> xvel;
    std::unique_ptr<expression_evaluator_t> xacc;
};

}  // namespace osc