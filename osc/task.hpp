#pragma once

#include <bopt/program.hpp>

#define GLOG_USE_GLOG_EXPORT
#include <glog/logging.h>

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

// Forward declaration of OSC
class OSC;

template <class T>
struct task_traits {
    typedef typename T::value_type value_type;
    typedef typename T::index_type index_type;
    typedef typename T::integer_type integer_type;

    typedef typename T::reference_t reference_t;
};

struct task_attributes {};

template <typename VectorType>
struct task_error {
    task_error(const std::size_t &n) : error(n), error_dot(n), error_int(n) {}
    // Error
    VectorType error;
    // Time derivative of the error
    VectorType error_dot;
    // Integral of the error
    VectorType error_int;
};

template <typename VectorType>
struct pid_gains {
    pid_gains(const std::size_t &sz) : p(sz), i(sz), d(sz) {}

    void resize(const std::size_t &sz) {
        p.resize(sz);
        i.resize(sz);
        d.resize(sz);
    }

    constexpr VectorType compute(const task_error<VectorType> &error) {
        return p.asDiagonal() * error.error + i.asDiagonal() * error.error_int +
               d.asDiagonal() * error.error_dot;
    }

    VectorType p;
    VectorType i;
    VectorType d;
};

/**
 * @brief Parameters specific to a given task
 *
 */
template <typename VectorType>
struct task_parameters {
    // Task weighting vector
    VectorType w;
    // Desired task acceleration
    VectorType xacc_d;
    // todo - Additional parameters that can be added
};

/**
 * @brief
 *
 */
template <typename ValueType, typename IndexType = std::size_t,
          typename IntegerType = int>
class TaskTpl {
    friend class OSC;

   public:
    // Typedefs
    typedef ValueType value_type;
    typedef IndexType index_type;
    typedef IntegerType integer_type;

    typedef task_error<eigen_vector_t> error_t;

    TaskTpl(const index_type &dim) : dimension_(dim), pid(dim) {
        set_dimension(dim);
    }

    TaskTpl() : dimension_(index_type(0)), pid(index_type(0)) {}

    index_type dimension() const { return dimension_; }

    /**
     * @brief PID gains for task tracking
     *
     */
    pid_gains<eigen_vector_t> pid;

    /**
     * @brief Add the task to the program
     *
     * @param program
     */
    virtual void add_to_program(const model_sym_t &model, OSC &program) = 0;

    /**
     * @brief Compute the error of the task given the current state of the
     * model.
     *
     * @param model
     * @param data
     * @param e
     */
    virtual void compute_task_error(const model_t &model, const data_t &data,
                                    error_t &e) = 0;

    /**
     * @brief Task parameters
     *
     * @return task_parameters<value_type>&
     */
    task_parameters<eigen_vector_t> &parameters() { return parameters_d; }

   protected:
    /**
     * @brief Set the dimension of the task, such that the dimension is the
     * dimension of the error of the system when considering the task (e.g.
     * typically the dimension of the Lie algebra)
     *
     * @param dimension
     */
    void set_dimension(const index_type &dimension) {
        dimension_ = dimension;
        // Create parameters
        parameters_v.w = create_variable_vector("w", dimension);
        parameters_d.w = eigen_vector_t::Ones(dimension);

        parameters_v.xacc_d = create_variable_vector("xacc_d", dimension);
        parameters_d.xacc_d = eigen_vector_t::Zero(dimension);

        pid.resize(dimension);
        pid.p.setOnes();
        pid.i.setOnes();
        pid.d.setZero();
    }

    // Parameter variables
    task_parameters<eigen_vector_t> parameters_d;
    task_parameters<eigen_vector_var_t> parameters_v;

   private:
    // Dimension of the task
    index_type dimension_;
};

typedef TaskTpl<double> Task;

class FrameTask : public Task {
    friend class OSC;

   public:
    /**
     * @brief Types of frame tasks.
     *
     */
    enum class Type { Position = 0, Orientation, Full };

    typedef pinocchio::SE3 se3_t;
    typedef pinocchio::Motion twist_t;

    FrameTask(const model_sym_t &model, const std::string &frame_name,
              const Type &type = Type::Full,
              const std::string &reference_frame = "universe");

    struct task_state {
        se3_t pose = se3_t::Identity();
        twist_t twist = twist_t::Zero();
    };

    typedef task_state task_state_t;
    typedef task_state_t reference_t;

    void compute_task_error(const model_t &model, const data_t &data,
                            error_t &e) override;

    reference_t target;

   protected:
    void add_to_program(const model_sym_t &model, OSC &program) override;

   private:
    Type type;
    string_t frame;
    string_t reference_frame;
};

class CentreOfMassTask : public Task {
    friend class OSC;

   public:
    CentreOfMassTask(const model_sym_t &model,
                     const std::string &reference_frame);

    struct task_state {
        eigen_vector3_t position = eigen_vector3_t::Zero();
        eigen_vector3_t velocity = eigen_vector3_t::Zero();
    };

    typedef task_state reference_t;

    void compute_task_error(const model_t &model, const data_t &data,
                            error_t &e) override;

    reference_t reference;

   protected:
    void add_to_program(const model_sym_t &model, OSC &program) override;

   private:
    string_t reference_frame;
};

class JointTrackingTask : public Task {
    friend class OSC;

   public:
    JointTrackingTask(const model_sym_t &model);

    struct task_state {
        eigen_vector_t joint_position;
        eigen_vector_t joint_velocity;
    };

    typedef task_state task_state_t;
    typedef task_state_t reference_t;

    void compute_task_error(const model_t &model, const data_t &data,
                            error_t &e) override;

    reference_t reference;

   protected:
    void add_to_program(const model_sym_t &model, OSC &program) override;

   private:
};

}  // namespace osc