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

#ifndef HAND_EYE_POS_ALIGN_FACTOR_HPP
#define HAND_EYE_POS_ALIGN_FACTOR_HPP

#include "ctraj/utils/sophus_utils.hpp"
#include "ctraj/spline/spline_segment.h"
#include "ctraj/spline/ceres_spline_helper_jet.h"
#include "ceres/dynamic_autodiff_cost_function.h"
#include "util/utils.h"
#include "config/configor.h"

namespace ns_ekalibr {
template <int Order>
struct HandEyeTransformAlignFactor {
private:
    ns_ctraj::SplineMeta<Order> _so3Meta, _scaleMeta;

    Sophus::SE3d SE3_CurCjToLastCj;
    double _tLastByCj, _tCurByCj;

    double _so3DtInv, _scaleDtInv;
    double _weight;

public:
    explicit HandEyeTransformAlignFactor(const ns_ctraj::SplineMeta<Order> &rotMeta,
                                         const ns_ctraj::SplineMeta<Order> &linScaleMeta,
                                         double tLastByLk,
                                         double tCurByLk,
                                         const Sophus::SE3d &se3CurCjToLastCj,
                                         double weight)
        : _so3Meta(rotMeta),
          _scaleMeta(linScaleMeta),
          SE3_CurCjToLastCj(se3CurCjToLastCj),
          _tLastByCj(tLastByLk),
          _tCurByCj(tCurByLk),
          _so3DtInv(1.0 / rotMeta.segments.front().dt),
          _scaleDtInv(1.0 / linScaleMeta.segments.front().dt),
          _weight(weight) {}

    static auto Create(const ns_ctraj::SplineMeta<Order> &rotMeta,
                       const ns_ctraj::SplineMeta<Order> &linScaleMeta,
                       double tLastByCj,
                       double tCurByCj,
                       const Sophus::SE3d &se3CurCjToLastCj,
                       double weight) {
        return new ceres::DynamicAutoDiffCostFunction<HandEyeTransformAlignFactor>(
            new HandEyeTransformAlignFactor(rotMeta, linScaleMeta, tLastByCj, tCurByCj,
                                            se3CurCjToLastCj, weight));
    }

    static std::size_t TypeHashCode() { return typeid(HandEyeTransformAlignFactor).hash_code(); }

public:
    /**
     * param blocks:
     * [ SO3 | ... | SO3 | LIN_SCALE | ... | LIN_SCALE | SO3_CjToBr | POS_CjInBr | TO_CjToBr ]
     */
    template <class T>
    bool operator()(T const *const *sKnots, T *sResiduals) const {
        // array offset
        std::size_t SO3_CjToBr_OFFSET = _so3Meta.NumParameters() + _scaleMeta.NumParameters();
        std::size_t POS_CjInBr_OFFSET = SO3_CjToBr_OFFSET + 1;
        std::size_t TO_CjToBr_OFFSET = POS_CjInBr_OFFSET + 1;

        Eigen::Map<Sophus::SO3<T> const> const SO3_CjToBr(sKnots[SO3_CjToBr_OFFSET]);
        Eigen::Map<Eigen::Vector3<T> const> const POS_CjInBr(sKnots[POS_CjInBr_OFFSET]);
        T TO_CjToBr = sKnots[TO_CjToBr_OFFSET][0];

        auto tLastByBr = _tLastByCj + TO_CjToBr;
        auto tCurByBr = _tCurByCj + TO_CjToBr;

        Sophus::SE3<T> SE3_LastBrToBr0, SE3_CurBrToBr0;
        {
            std::pair<std::size_t, T> iuSo3Last, iuPosLast;
            _so3Meta.ComputeSplineIndex(tLastByBr, iuSo3Last.first, iuSo3Last.second);
            _scaleMeta.ComputeSplineIndex(tLastByBr, iuPosLast.first, iuPosLast.second);
            std::size_t LAST_SO3_OFFSET = iuSo3Last.first;
            std::size_t LAST_POS_OFFSET = iuPosLast.first + _so3Meta.NumParameters();

            ns_ctraj::CeresSplineHelperJet<T, Order>::EvaluateLie(
                sKnots + LAST_SO3_OFFSET, iuSo3Last.second, _so3DtInv, &SE3_LastBrToBr0.so3());
            ns_ctraj::CeresSplineHelperJet<T, Order>::template Evaluate<3, 0>(
                sKnots + LAST_POS_OFFSET, iuPosLast.second, _scaleDtInv,
                &SE3_LastBrToBr0.translation());
        }
        {
            std::pair<std::size_t, T> iuSo3Cur, iuPosCur;
            _so3Meta.ComputeSplineIndex(tCurByBr, iuSo3Cur.first, iuSo3Cur.second);
            _scaleMeta.ComputeSplineIndex(tCurByBr, iuPosCur.first, iuPosCur.second);
            std::size_t CUR_SO3_OFFSET = iuSo3Cur.first;
            std::size_t CUR_POS_OFFSET = iuPosCur.first + _so3Meta.NumParameters();

            ns_ctraj::CeresSplineHelperJet<T, Order>::EvaluateLie(
                sKnots + CUR_SO3_OFFSET, iuSo3Cur.second, _so3DtInv, &SE3_CurBrToBr0.so3());
            ns_ctraj::CeresSplineHelperJet<T, Order>::template Evaluate<3, 0>(
                sKnots + CUR_POS_OFFSET, iuPosCur.second, _scaleDtInv,
                &SE3_CurBrToBr0.translation());
        }
        Sophus::SE3<T> SE3_CjToBr(SO3_CjToBr, POS_CjInBr);

        Sophus::SE3<T> left = SE3_CjToBr * SE3_CurCjToLastCj;
        Sophus::SE3<T> right = (SE3_LastBrToBr0.inverse() * SE3_CurBrToBr0) * SE3_CjToBr;

        Eigen::Map<Eigen::Vector6<T>> residuals(sResiduals);
        residuals = T(_weight) * (right.inverse() * left).log();

        return true;
    }

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

extern template struct HandEyeTransformAlignFactor<Configor::Prior::SplineOrder>;
}  // namespace ns_ekalibr

#endif  // HAND_EYE_POS_ALIGN_FACTOR_HPP
