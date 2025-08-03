#pragma once

#include "osc/Types.hpp"

namespace osc {

class Variable {
   public:
    Variable(const String& name) : name_(name), id_(setId()) {}

    const String& name() const { return name_; }
    const Size& id() const { return id_; }

   private:
    String name_;
    Size id_;

    Size setId() const {
        static Size id = 0;
        return id++;
    }
};

using VariableVector = VectorX<Variable>;

void createVariableVector(const String& name, const Size& n) {}

}  // namespace osc
