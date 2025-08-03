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
    using SharedPtr = std::shared_ptr<FrictionConeAbstract>;

    void setCoefficient(const Real &mu) { mu_ = mu; }
    const Real &getCoefficient() const { return mu_; }

    virtual void toLPConstraint(Eigen::Ref<Matrix> A, Eigen::Ref<Vector> ubA,
                                Eigen::Ref<Vector> lbA) {}

    virtual void toSOCPConstraint(Eigen::Ref<Matrix> H, Eigen::Ref<Matrix> A,
                                  Eigen::Ref<Vector> g) {}

    /**
     * @brief Returns the map that transforms the parameterisation of the
     * contact force within the cone to a contact force in 3D.
     *
     * @return Matrix
     */
    virtual Matrix parameterisationToForceMap() const = 0;

   protected:
    ContactAbstract() : dimension_(0), task_dimension_(0) {}
    ContactAbstract(const Size &dimension)
        : dimension_(dimension), task_dimension_(dimension) {}

    void setDimension(const Size &dimension) { dimension_ = dimension; }

   private:
    Size dimension_;
    Real mu_;
};

// class FrictionConeModel : public FrictionConeModelAbstract {
//    public:
//     using SharedPtr = std::shared_ptr<LinearisedFrictionConeConstraint>;

//     LinearisedFrictionConeModel(const Size &n) {}

//     void toSOCPConstraint(Eigen::Ref<Matrix> A, Eigen::Ref<Vector> ubA,
//                         Eigen::Ref<Vector> lbA) override {

//     }

//     Matrix parameterisationToForceMap() const override {
//         return Matrix3::Identity();
//     }

//    protected:
//     ContactAbstract() : dimension_(0) {}
//     ContactAbstract(const Size &dimension) : dimension_(dimension) {}

//     void setDimension(const Size &dimension) { dimension_ = dimension; }

//    private:
//     Size dimension_;
// };

class LinearisedFrictionConeModel : public FrictionConeModelAbstract {
   public:
    using SharedPtr = std::shared_ptr<LinearisedFrictionConeConstraint>;

    LinearisedFrictionConeModel(const Size &n)
        : FrictionConeModelAbstract(), n_(n) {}

    Size numEdges() const { return n_; }

    void toLPConstraint(Eigen::Ref<Matrix> A, Eigen::Ref<Vector> ubA,
                        Eigen::Ref<Vector> lbA) override {
        // Unilaterality
        A.row(0) << 0, 0, 1.0;
        lbA(0) << 0.0;
        ubA(0) << inf;
        // Edges
        Real theta = 0;
        for (Size i = 1; i < n_ + 1; ++i) {
            A.row(i) << cos(theta), sin(theta), -mu_ / sqrt(2.0);
            lbA(i) = -inf;
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
class LinearisedFrictionConeGeneratorModel {
   public:
    using SharedPtr = std::shared_ptr<LinearisedFrictionConeGeneratorModel>;

    LinearisedFrictionConeGeneratorModel(const Size &n) : n_(n) {
        assert(n >= 4 && "Need sufficient edges to approximate");
    }

    void toLPConstraint(Eigen::Ref<Matrix> A, Eigen::Ref<Vector> ubA,
                        Eigen::Ref<Vector> lbA) override {
        A.setIdentity();
        lbA.setZero();
        ubA.setConstant(inf);
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
    void setDimension(const Size &dimension) { dimension_ = dimension; }

   private:
    Size dimension_;
};

// todo - wrench abstract
};  // namespace osc