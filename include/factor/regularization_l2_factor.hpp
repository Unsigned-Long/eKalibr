// eKalibr, Copyright 2024, the School of Geodesy and Geomatics (SGG), Wuhan University, China
// https://github.com/Unsigned-Long/eKalibr.git
// Author: Shuolong Chen (shlchen@whu.edu.cn)
// GitHub: https://github.com/Unsigned-Long
//  ORCID: 0000-0002-5283-9057
// Purpose: See .h/.hpp file.
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * The names of its contributors can not be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef REGULARIZATION_L2_FACTOR_HPP
#define REGULARIZATION_L2_FACTOR_HPP

#include "ctraj/utils/sophus_utils.hpp"
#include "ctraj/spline/spline_segment.h"
#include "ctraj/spline/ceres_spline_helper.h"
#include "ceres/dynamic_autodiff_cost_function.h"
#include "util/utils.h"
#include "config/configor.h"
#include "ceres/problem.h"
#include "sophus/so3.hpp"

namespace ns_ekalibr {
template <int Dime>
struct RegularizationL2Factor {
private:
    double _weight;

public:
    explicit RegularizationL2Factor(double weight)
        : _weight(weight) {}

    static auto Create(double weight) {
        return new ceres::DynamicAutoDiffCostFunction<RegularizationL2Factor>(
            new RegularizationL2Factor(weight));
    }

    static void AddToSolver(ceres::Problem *prob, Eigen::Vector<double, Dime> *vec, double weight) {
        auto costFunc = RegularizationL2Factor::Create(weight);
        costFunc->AddParameterBlock(Dime);
        costFunc->SetNumResiduals(3);
        prob->AddResidualBlock(costFunc, nullptr, std::vector<double *>{vec->data()});
    }

    static std::size_t TypeHashCode() { return typeid(RegularizationL2Factor).hash_code(); }

public:
    /**
     * param blocks:
     * [ VECTOR ]
     */
    template <class T>
    bool operator()(T const *const *pars, T *sResiduals) const {
        Eigen::Map<const Eigen::Vector<T, Dime>> vec(pars[0]);
        Eigen::Map<Eigen::Vector<T, Dime>> residuals(sResiduals);
        residuals = T(_weight) * vec;
        return true;
    }

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct So3RegularizationL2Factor {
private:
    double _weight;

public:
    explicit So3RegularizationL2Factor(double weight)
        : _weight(weight) {}

    static auto Create(double weight) {
        return new ceres::DynamicAutoDiffCostFunction(new So3RegularizationL2Factor(weight));
    }

    static void AddToSolver(ceres::Problem *prob,
                            Sophus::SO3d *so3,
                            ceres::Manifold *so3Manifold,
                            double weight) {
        auto costFunc = So3RegularizationL2Factor::Create(weight);
        costFunc->AddParameterBlock(4);
        costFunc->SetNumResiduals(3);
        prob->AddResidualBlock(costFunc, nullptr, std::vector<double *>{so3->data()});
        prob->SetManifold(so3->data(), so3Manifold);
    }

    static std::size_t TypeHashCode() { return typeid(So3RegularizationL2Factor).hash_code(); }

public:
    /**
     * param blocks:
     * [ VECTOR ]
     */
    template <class T>
    bool operator()(T const *const *pars, T *sResiduals) const {
        Eigen::Map<const Sophus::SO3<T>> vec(pars[0]);
        Eigen::Map<Eigen::Vector3<T>> residuals(sResiduals);
        residuals = T(_weight) * vec.log();
        return true;
    }

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

extern template struct RegularizationL2Factor<3>;

}  // namespace ns_ekalibr

#endif  // REGULARIZATION_L2_FACTOR_HPP