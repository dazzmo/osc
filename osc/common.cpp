#include "osc/common.hpp"

namespace osc {

eigen_vector_sym_t create_symbolic_vector(const std::string &name,
                                          const std::size_t &sz) {
    eigen_vector_sym_t res(sz);
    for (std::size_t i = 0; i < sz; ++i) {
        res[i] = sym_t::sym(name + std::to_string(i));
    }
    return res;
}

eigen_vector_var_t create_variable_vector(const std::string &name,
                                          const std::size_t &sz) {
    eigen_vector_var_t res(sz);
    for (std::size_t i = 0; i < sz; ++i) {
        res[i] = bopt::variable(name + std::to_string(i));
    }
    return res;
}

}  // namespace osc