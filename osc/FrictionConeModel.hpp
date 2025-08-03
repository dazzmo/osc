#pragma once

#include "osc/Fwd.hpp"
#include "osc/State.hpp"

namespace osc {

/**
 * @brief Friction cone properties for a friction cone defined locally within
 * the contact frame with a normal `n` and orthogonal basis axes `t` and `b`.
 * With these models is an associated parameterisation of the force that is
 * contained within them.
 *
 */
class FrictionConeModelAbstract {
   public:
    using SharedPtr = std::shared_ptr<FrictionConeModelAbstract>;

    void setCoefficient(const Real &mu) { mu_ = mu; }
    const Real &getCoefficient() const { return mu_; }

    virtual Size numLPConstraints() const { return 0; }

    virtual void toLPConstraints(Eigen::Ref<Matrix> A, Eigen::Ref<Vector> ubA,
                                 Eigen::Ref<Vector> lbA, Eigen::Ref<Vector> ubx,
                                 Eigen::Ref<Vector> lbx) const {}

    virtual Size numSOCPConstraints() const { return 0; }

    virtual void toSOCPConstraints(Eigen::Ref<Matrix> H, Eigen::Ref<Matrix> A,
                                   Eigen::Ref<Vector> g) const {}

    /**
     * @brief Returns the map that transforms the parameterisation of the
     * contact force within the cone to a contact force in 3D.
     *
     * @return Matrix
     */
    virtual Matrix parameterisationToForceMap() const = 0;

    virtual Size numParameters() const = 0;

   protected:
    FrictionConeModelAbstract() = default;
    ~FrictionConeModelAbstract() = default;

   private:
    Real mu_;
};

class LinearisedFrictionConeModel : public FrictionConeModelAbstract {
   public:
    using SharedPtr = std::shared_ptr<LinearisedFrictionConeModel>;

    LinearisedFrictionConeModel(const Size &n)
        : FrictionConeModelAbstract(), n_(n) {}

    Size numEdges() const { return n_; }

    Size numLPConstraints() const override { return n_; }

    void toLPConstraints(Eigen::Ref<Matrix> A, Eigen::Ref<Vector> ubA,
                         Eigen::Ref<Vector> lbA, Eigen::Ref<Vector> ubx,
                         Eigen::Ref<Vector> lbx) const override {
        // Unilaterality
        lbx[2] = 0.0;
        ubx[2] = 1e9;
        // Edges
        Real theta = 0;
        for (Size i = 0; i < n_; ++i) {
            A.row(i) << cos(theta), sin(theta), -getCoefficient() / sqrt(2.0);
            lbA(i) = -1e9;
            ubA(i) = Real(0);
            theta += 2 * M_PI / n_;
        }
    }

    Matrix parameterisationToForceMap() const override {
        return Matrix3::Identity();
    }

   protected:
   private:
    Size n_;
};

/**
 * @brief Linearised friction cone based on the generator model such that a
 * contact force is represented by the linear combination of its generators.
 *
 */
class LinearisedFrictionConeGeneratorModel : public FrictionConeModelAbstract {
   public:
    using SharedPtr = std::shared_ptr<LinearisedFrictionConeGeneratorModel>;

    LinearisedFrictionConeGeneratorModel(const Size &n)
        : FrictionConeModelAbstract(), n_(n) {
        assert(n >= 4 && "Need sufficient edges to approximate");
    }

    Size numEdges() const { return n_; }

    Size numLPConstraints() const override { return 0; }

    void toLPConstraints(Eigen::Ref<Matrix> A, Eigen::Ref<Vector> ubA,
                         Eigen::Ref<Vector> lbA, Eigen::Ref<Vector> ubx,
                         Eigen::Ref<Vector> lbx) const override {
        lbx.setZero();
        ubx.setConstant(1e9);
    }

    /**
     * @brief Returns a 3 x n matrix of the generators representing the friction
     * cone in the contact frame.
     *
     * @return Matrix
     */
    Matrix getGenerators() const {
        Matrix generators = Matrix::Zero(3, n_);
        Real theta = (2 * M_PI / n_) / 2.0;
        Real mu = getCoefficient();
        Real mu_s = mu / sqrt(2.0);
        for (Size i = 0; i < n_; ++i) {
            generators.col(i) =
                Vector3(mu_s * cos(theta), mu_s * sin(theta), 1.0);
            theta += 2 * M_PI / n_;
        }
        return generators;
    }

    Matrix parameterisationToForceMap() const override {
        // Construct force through generators
        return getGenerators();
    }

   protected:
   private:
    Size n_;
};

// todo - wrench abstract
}  // namespace osc