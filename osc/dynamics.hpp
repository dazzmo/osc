

#include "osc/common.hpp"
#include "osc/contact.hpp"

namespace osc {

struct dynamics_parameters {};

struct dynamics_constraint_handler {
    void add(const sym_t &jacobian, const sym_t &lambda) {}

    // Constraint Jacobian
    sym_t jacobian;
    // Constraint forces
    sym_t lambda;

    // Variables
    sym_t variables;
    // Parameters
    sym_t parameters;
};

template<typename Scalar>
struct dynamics_variables {
    std::vector<Scalar> a;
    std::vector<Scalar> u;
    std::vector<Scalar> lambda;
};

template <typename Scalar>
struct dynamics_parameters {
    std::vector<Scalar> q;
    std::vector<Scalar> v;

    std::vector<Scalar> other;
};

class Dynamics {
   public:
    Dynamics(model_sym_t &model);

    void register_contact_point(const ContactTask &task);

    void register_actuation(const sym_t &u);

    void to_constraint();

    void add_constraint();

    // dynamics_parameters<bopt::variable> parameters;
    // dynamics_parameters<double> parameters_d;

   private:
    dynamics_variables<sym_t> variables_;
    dynamics_parameters<sym_t> parameters_;
    sym_t dynamics;
};

}  // namespace osc