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

#ifndef VISUAL_PROJECTION_CIRCLE_BASED_FACTOR_HPP
#define VISUAL_PROJECTION_CIRCLE_BASED_FACTOR_HPP

#include "factor/visual_projection_factor.hpp"
#include "sensor/event.h"
#include "ceres/jet.h"

#include <util/status.hpp>

namespace ns_ekalibr {
struct Circle3D {
    using Ptr = std::shared_ptr<Circle3D>;
    Eigen::Vector3d center;
    double radius;
    // [ xVec | yVec | zVec ]
    Eigen::Matrix3d orientation;
    /**
     * a point on this circle can be obtained by:
     * p = center + radius * xVec * cos(theta) + radius * yVec * sin(theta)
     */

    Circle3D(const Eigen::Vector3d &center, double radius, const Eigen::Matrix3d &orientation)
        : center(center),
          radius(radius),
          orientation(orientation) {}

    static Ptr CreateFromGridPattern(const Eigen::Vector3d &center, const double radius) {
        return std::make_shared<Circle3D>(center, radius, Eigen::Matrix3d::Identity());
    }
};

struct VisualProjectionCircleBasedPair {
public:
    using Ptr = std::shared_ptr<VisualProjectionCircleBasedPair>;

    Circle3D::Ptr c;
    Event::Ptr ev;
    Eigen::Vector3d p;

    VisualProjectionCircleBasedPair(const Circle3D::Ptr &circle,
                                    const Event::Ptr &ev,
                                    double cosTheta,
                                    double sinTheta)
        : c(circle),
          ev(ev) {
        /**
         * a point on this circle can be obtained by:
         * p = cenInCam + radius * xInCam * cos(theta) + radius * yInCam * sin(theta)
         */
        p = c->center +  // center
            c->radius * cosTheta * c->orientation.col(0) +
            c->radius * sinTheta * c->orientation.col(1);
    }

    static Ptr Create(const Circle3D::Ptr &circle,
                      const Event::Ptr &ev,
                      double cosTheta,
                      double sinTheta) {
        return std::make_shared<VisualProjectionCircleBasedPair>(circle, ev, cosTheta, sinTheta);
    }
};

template <int Order>
struct VisualProjectionCircleBasedFactor {
private:
    ns_ctraj::SplineMeta<Order> _so3Meta, _scaleMeta;
    VisualProjectionCircleBasedPair::Ptr _pair;
    double _so3DtInv, _scaleDtInv;
    double _weight;

public:
    explicit VisualProjectionCircleBasedFactor(ns_ctraj::SplineMeta<Order> rotMeta,
                                               ns_ctraj::SplineMeta<Order> linScaleMeta,
                                               VisualProjectionCircleBasedPair::Ptr pair,
                                               double weight)
        : _so3Meta(rotMeta),
          _scaleMeta(std::move(linScaleMeta)),
          _pair(std::move(pair)),
          _so3DtInv(1.0 / rotMeta.segments.front().dt),
          _scaleDtInv(1.0 / _scaleMeta.segments.front().dt),
          _weight(weight) {}

    static auto Create(const ns_ctraj::SplineMeta<Order> &rotMeta,
                       const ns_ctraj::SplineMeta<Order> &linScaleMeta,
                       const VisualProjectionCircleBasedPair::Ptr &pair,
                       double weight) {
        return new ceres::DynamicAutoDiffCostFunction<VisualProjectionCircleBasedFactor>(
            new VisualProjectionCircleBasedFactor(rotMeta, linScaleMeta, pair, weight));
    }

    static std::size_t TypeHashCode() {
        return typeid(VisualProjectionCircleBasedFactor).hash_code();
    }

public:
    /**
     * param blocks:
     * [ SO3 | ... | SO3 | LIN_SCALE | ... | LIN_SCALE | SO3_CjToCr | POS_CjInCr | TO_CjToCr |
     *   FX | FY | CX | CY | DIST_COEFFS ]
     */
    template <class T>
    bool operator()(T const *const *sKnots, T *sResiduals) const {
        std::size_t SO3_OFFSET;
        std::size_t LIN_SCALE_OFFSET;

        std::size_t SO3_CjToBr_OFFSET = _so3Meta.NumParameters() + _scaleMeta.NumParameters();
        std::size_t POS_CjInBr_OFFSET = SO3_CjToBr_OFFSET + 1;
        std::size_t TO_CjToBr_OFFSET = POS_CjInBr_OFFSET + 1;
        std::size_t FX_OFFSET = TO_CjToBr_OFFSET + 1;
        std::size_t FY_OFFSET = FX_OFFSET + 1;
        std::size_t CX_OFFSET = FY_OFFSET + 1;
        std::size_t CY_OFFSET = CX_OFFSET + 1;
        std::size_t DIST_COEFFS_OFFSET = CY_OFFSET + 1;

        Eigen::Map<const Sophus::SO3<T>> SO3_CjToBr(sKnots[SO3_CjToBr_OFFSET]);
        Eigen::Map<const Eigen::Vector3<T>> POS_CjInBr(sKnots[POS_CjInBr_OFFSET]);
        Sophus::SE3<T> SE3_CjToBr(SO3_CjToBr, POS_CjInBr);

        T TO_CjToBr = sKnots[TO_CjToBr_OFFSET][0];

        T FX = sKnots[FX_OFFSET][0];
        T FY = sKnots[FY_OFFSET][0];
        T CX = sKnots[CX_OFFSET][0];
        T CY = sKnots[CY_OFFSET][0];

        // this is for pinhole brow t2 [k1, k2, k3, p1, p2]
        Eigen::Map<const Eigen::Vector5<T>> DIST_COEFFS(sKnots[DIST_COEFFS_OFFSET]);

        T timeByBr = static_cast<T>(_pair->ev->GetTimestamp()) + TO_CjToBr;

        // calculate the so3 and lin scale offset
        std::pair<std::size_t, T> iuSo3, iuScale;
        _so3Meta.ComputeSplineIndex(timeByBr, iuSo3.first, iuSo3.second);
        _scaleMeta.ComputeSplineIndex(timeByBr, iuScale.first, iuScale.second);

        SO3_OFFSET = iuSo3.first;
        LIN_SCALE_OFFSET = iuScale.first + _so3Meta.NumParameters();

        Sophus::SO3<T> SO3_BrToW;
        ns_ctraj::CeresSplineHelperJet<T, Order>::EvaluateLie(sKnots + SO3_OFFSET, iuSo3.second,
                                                              _so3DtInv, &SO3_BrToW);
        Eigen::Vector3<T> POS_BrInW;
        ns_ctraj::CeresSplineHelperJet<T, Order>::template Evaluate<3, 0>(
            sKnots + LIN_SCALE_OFFSET, iuScale.second, _scaleDtInv, &POS_BrInW);

        Sophus::SE3<T> SE3_BrToW(SO3_BrToW, POS_BrInW);

        Sophus::SE3<T> SE3_CjToW = SE3_BrToW * SE3_CjToBr;

        // from world frame to camera frame
        Eigen::Vector3<T> pInCam = SE3_CjToW.inverse() * _pair->p.cast<T>();
        // from camera frame to camera normalized plane
        Eigen::Vector2<T> pInCamPlane(pInCam(0) / pInCam(2), pInCam(1) / pInCam(2));
        using Helper = VisualProjectionFactor<Configor::Prior::SplineOrder>;
        // add distortion
        pInCamPlane = Helper::AddDistortion<T>(DIST_COEFFS, pInCamPlane);

        Eigen::Vector2<T> pixelPred;
        Helper::TransformCamToImg<T>(&FX, &FY, &CX, &CY, pInCamPlane, &pixelPred);

        Eigen::Map<Eigen::Vector2<T>> residuals(sResiduals);
        residuals = pixelPred - _pair->ev->GetPos().cast<T>();
        residuals = T(_weight) * residuals;

        return true;
    }

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

extern template struct VisualProjectionCircleBasedFactor<Configor::Prior::SplineOrder>;
}  // namespace ns_ekalibr

#endif  // VISUAL_PROJECTION_CIRCLE_BASED_FACTOR_HPP
