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

typedef state<Eigen::VectorXd> model_state;
typedef state<Eigen::Vector3d> task_state;

template <typename Vector>
struct pid_error {
    typename Vector vector_t;

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

    static inline casadi_vector_t operator()(const eigen_vector_t &v) {
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

/**
 * @brief Parameters within an OSC program for a given task
 *
 */
struct task_parameters {
    // Task weighting vector
    std::vector<bopt::variable> w;
    // Desired task acceleration
    std::vector<bopt::variable> desired_task_acceleration;
    std::vector<bopt::variable> q;
    std::vector<bopt::variable> v;
};

/**
 * @brief Expressed as a quadratic cost
 *
 */
struct task_program_data {
    bopt::quadratic_cost<double>::shared_ptr cost;
};

// /**
//  * @brief Expressed as a linear constraint and cost
//  *
//  */
// struct task_program_data {
//     bopt::linear_constraint<double>::shared_ptr cost;
//     bopt::quadratic_cost<double>::shared_ptr cost;

//     std::vector<bopt::variable> epsilon;
// };

/**
 * @brief
 *
 */
class Task {
   public:
    typedef double value_type;
    typedef std::size_t index_type;
    typedef int integer_type;

    typedef typename Eigen::VectorX<value_type> vector_t;

    Task() : dimension_(index_type(0)) {}

    index_type dimension() const { return dimension_; }

    value_type weight() {}

    integer_type pid_gains(vector_t &p, vector_t &i, vector_t &d) {
        pid.p = p;
        pid.i = i;
        pid.d = d;
    }

    virtual integer_type get_state(const model_state &state,
                                   task_state &task_state) {}
    virtual integer_type get_error(const task_state &task_state,
                                   task_error &error) {}

    virtual bopt::quadratic_cost<value_type>::shared_ptr to_task_cost() = 0;
    virtual void add_task_cost(
        bopt::mathematical_program<value_type> &program) = 0;

    // std::vector<value_type> weight() {}
    task_parameters parameters;
    pid_gains<Eigen::Vector3d> pid;

   private:
    index_type dimension_;
};

class PositionTask : public Task {
    PositionTask(const model_sym_t &model, const std::string &frame_name,
                 const std::string &reference_frame);

    bopt::quadratic_cost<value_type>::shared_ptr to_task_cost() override;
    
    void add_task_cost(
        bopt::mathematical_program<value_type> &program) override;

    const std::string &reference_frame();

    // void set_target(){}

   private:
    bopt::casadi::expression_evaluator<value_type> x_pos;
    bopt::casadi::expression_evaluator<value_type> x_vel;
    bopt::casadi::expression_evaluator<value_type> x_acc;
};

// class CentreOfMassTask : public Task {
//     // todo - choose a reference frame
//     CentreOfMassTask(const model_sym_t &model, const std::string &frame_name,
//                      const std::string &reference_frame);

//     const std::string &reference_frame();

//     // void set_target(){}
// };

// class OrientationTask : public Task {
//     // todo - choose a reference frame
//     PositionTask(const model_sym_t &model, const std::string &frame_name,
//                  const std::string &reference_frame);

//     const std::string &reference_frame();

//     // void set_target(){}

// };

// class SE3Task : public Task {
//     // todo - choose a reference frame
//     PositionTask(const model_sym_t &model, const std::string &frame_name,
//                  const std::string &reference_frame);

//     const std::string &reference_frame();
//     // void set_target(){}
// };
}  // namespace osc