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

#ifndef VISUAL_PROJECTION_FACTOR_HPP
#define VISUAL_PROJECTION_FACTOR_HPP

#include "ctraj/utils/eigen_utils.hpp"
#include "ctraj/utils/sophus_utils.hpp"
#include "ctraj/spline/spline_segment.h"
#include "ctraj/spline/ceres_spline_helper.h"
#include "ctraj/spline/ceres_spline_helper_jet.h"
#include "ceres/dynamic_autodiff_cost_function.h"
#include "util/utils.h"
#include "config/configor.h"

namespace ns_ekalibr {
struct VisualProjectionPair {
    using Ptr = std::shared_ptr<VisualProjectionPair>;

    double timestamp;
    Eigen::Vector3d point3d;
    Eigen::Vector2d pixel2d;

    VisualProjectionPair(double timestamp,
                         const Eigen::Vector3d &point3d,
                         const Eigen::Vector2d &pixel2d)
        : timestamp(timestamp),
          point3d(point3d),
          pixel2d(pixel2d) {}

    static Ptr Create(double timestamp,
                      const Eigen::Vector3d &point3d,
                      const Eigen::Vector2d &pixel2d) {
        return std::make_shared<VisualProjectionPair>(timestamp, point3d, pixel2d);
    }
};

template <int Order>
struct VisualProjectionFactor {
private:
    ns_ctraj::SplineMeta<Order> _so3Meta, _scaleMeta;
    VisualProjectionPair::Ptr _pair;
    double _so3DtInv, _scaleDtInv;
    double _weight;

public:
    explicit VisualProjectionFactor(ns_ctraj::SplineMeta<Order> rotMeta,
                                    ns_ctraj::SplineMeta<Order> linScaleMeta,
                                    VisualProjectionPair::Ptr pair,
                                    double weight)
        : _so3Meta(rotMeta),
          _scaleMeta(std::move(linScaleMeta)),
          _pair(std::move(pair)),
          _so3DtInv(1.0 / rotMeta.segments.front().dt),
          _scaleDtInv(1.0 / _scaleMeta.segments.front().dt),
          _weight(weight) {}

    static auto Create(const ns_ctraj::SplineMeta<Order> &rotMeta,
                       const ns_ctraj::SplineMeta<Order> &linScaleMeta,
                       const VisualProjectionPair::Ptr &pair,
                       double weight) {
        return new ceres::DynamicAutoDiffCostFunction<VisualProjectionFactor>(
            new VisualProjectionFactor(rotMeta, linScaleMeta, pair, weight));
    }

    static std::size_t TypeHashCode() { return typeid(VisualProjectionFactor).hash_code(); }

    template <class T>
    static void TransformCamToImg(const T *FX,
                                  const T *FY,
                                  const T *CX,
                                  const T *CY,
                                  const Eigen::Vector2<T> &P,
                                  Eigen::Vector2<T> *feat) {
        feat->operator()(0) = *FX * P(0) + *CX;
        feat->operator()(1) = *FY * P(1) + *CY;
    }

    template <typename T>
    static Eigen::Vector2<T> DistortionFunction(
        const Eigen::Map<const Eigen::Vector5<T>> &distoParams, const Eigen::Vector2<T> &p) {
        const T k1 = distoParams(0), k2 = distoParams(1), k3 = distoParams(2);
        const T t1 = distoParams(3), t2 = distoParams(4);
        const T r2 = p(0) * p(0) + p(1) * p(1);
        const T r4 = r2 * r2;
        const T r6 = r4 * r2;
        const T k_diff = k1 * r2 + k2 * r4 + k3 * r6;
        const T t_x = t2 * (r2 + T(2) * p(0) * p(0)) + T(2) * t1 * p(0) * p(1);
        const T t_y = t1 * (r2 + T(2) * p(1) * p(1)) + T(2) * t2 * p(0) * p(1);
        return {p(0) * k_diff + t_x, p(1) * k_diff + t_y};
    }

    template <typename T>
    static Eigen::Vector2<T> AddDistortion(const Eigen::Map<const Eigen::Vector5<T>> &distoParams,
                                           const Eigen::Vector2<T> &p) {
        return p + DistortionFunction(distoParams, p);
    }

public:
    /**
     * param blocks:
     * [ SO3 | ... | SO3 | LIN_SCALE | ... | LIN_SCALE | SO3_CjToBr | POS_CjInBr | TO_CjToBr |
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

        T timeByBr = static_cast<T>(_pair->timestamp) + TO_CjToBr;

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
        Eigen::Vector3<T> pInCam = SE3_CjToW.inverse() * _pair->point3d.cast<T>();
        // from camera frame to camera normalized plane
        Eigen::Vector2<T> pInCamPlane(pInCam(0) / pInCam(2), pInCam(1) / pInCam(2));
        // add distortion
        pInCamPlane = AddDistortion<T>(DIST_COEFFS, pInCamPlane);

        Eigen::Vector2<T> pixelPred;
        TransformCamToImg<T>(&FX, &FY, &CX, &CY, pInCamPlane, &pixelPred);

        Eigen::Map<Eigen::Vector2<T>> residuals(sResiduals);
        residuals = pixelPred - _pair->pixel2d.cast<T>();
        residuals = T(_weight) * residuals;

        return true;
    }

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

extern template struct VisualProjectionFactor<Configor::Prior::SplineOrder>;

struct VisualDiscreteProjectionFactor {
private:
    VisualProjectionPair::Ptr _pair{};
    double _weight;

public:
    explicit VisualDiscreteProjectionFactor(const VisualProjectionPair::Ptr &pair, double weight)
        : _pair(pair),
          _weight(weight) {}

    static auto Create(const VisualProjectionPair::Ptr &pair, double weight) {
        return new ceres::DynamicAutoDiffCostFunction<VisualDiscreteProjectionFactor>(
            new VisualDiscreteProjectionFactor(pair, weight));
    }

    static std::size_t TypeHashCode() { return typeid(VisualDiscreteProjectionFactor).hash_code(); }

    template <class T>
    static void TransformCamToImg(const T *FX,
                                  const T *FY,
                                  const T *CX,
                                  const T *CY,
                                  const Eigen::Vector2<T> &P,
                                  Eigen::Vector2<T> *feat) {
        feat->operator()(0) = *FX * P(0) + *CX;
        feat->operator()(1) = *FY * P(1) + *CY;
    }

    template <typename T>
    static Eigen::Vector2<T> DistortionFunction(
        const Eigen::Map<const Eigen::Vector5<T>> &distoParams, const Eigen::Vector2<T> &p) {
        const T k1 = distoParams(0), k2 = distoParams(1), k3 = distoParams(2);
        const T t1 = distoParams(3), t2 = distoParams(4);
        const T r2 = p(0) * p(0) + p(1) * p(1);
        const T r4 = r2 * r2;
        const T r6 = r4 * r2;
        const T k_diff = k1 * r2 + k2 * r4 + k3 * r6;
        const T t_x = t2 * (r2 + T(2) * p(0) * p(0)) + T(2) * t1 * p(0) * p(1);
        const T t_y = t1 * (r2 + T(2) * p(1) * p(1)) + T(2) * t2 * p(0) * p(1);
        return {p(0) * k_diff + t_x, p(1) * k_diff + t_y};
    }

    template <typename T>
    static Eigen::Vector2<T> AddDistortion(const Eigen::Map<const Eigen::Vector5<T>> &distoParams,
                                           const Eigen::Vector2<T> &p) {
        return p + DistortionFunction(distoParams, p);
    }

public:
    /**
     * param blocks:
     * [ SO3_CjToW | POS_CjInW | FX | FY | CX | CY | DIST_COEFFS ]
     */
    template <class T>
    bool operator()(T const *const *sKnots, T *sResiduals) const {
        std::size_t SO3_CjToW_OFFSET = 0;
        std::size_t POS_CjInW_OFFSET = SO3_CjToW_OFFSET + 1;
        std::size_t FX_OFFSET = POS_CjInW_OFFSET + 1;
        std::size_t FY_OFFSET = FX_OFFSET + 1;
        std::size_t CX_OFFSET = FY_OFFSET + 1;
        std::size_t CY_OFFSET = CX_OFFSET + 1;
        std::size_t DIST_COEFFS_OFFSET = CY_OFFSET + 1;

        Eigen::Map<const Sophus::SO3<T>> SO3_CjToW(sKnots[SO3_CjToW_OFFSET]);
        Eigen::Map<const Eigen::Vector3<T>> POS_CjInW(sKnots[POS_CjInW_OFFSET]);
        Sophus::SE3<T> SE3_CjToW(SO3_CjToW, POS_CjInW);

        T FX = sKnots[FX_OFFSET][0];
        T FY = sKnots[FY_OFFSET][0];
        T CX = sKnots[CX_OFFSET][0];
        T CY = sKnots[CY_OFFSET][0];

        // this is for pinhole brow t2 [k1, k2, k3, p1, p2]
        Eigen::Map<const Eigen::Vector5<T>> DIST_COEFFS(sKnots[DIST_COEFFS_OFFSET]);

        // from world frame to camera frame
        Eigen::Vector3<T> pInCam = SE3_CjToW.inverse() * _pair->point3d.cast<T>();
        // from camera frame to camera normalized plane
        Eigen::Vector2<T> pInCamPlane(pInCam(0) / pInCam(2), pInCam(1) / pInCam(2));
        // add distortion
        pInCamPlane = AddDistortion<T>(DIST_COEFFS, pInCamPlane);

        Eigen::Vector2<T> pixelPred;
        TransformCamToImg<T>(&FX, &FY, &CX, &CY, pInCamPlane, &pixelPred);

        Eigen::Map<Eigen::Vector2<T>> residuals(sResiduals);
        residuals = pixelPred - _pair->pixel2d.cast<T>();
        residuals = T(_weight) * residuals;

        return true;
    }

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace ns_ekalibr

#endif  // VISUAL_PROJECTION_FACTOR_HPP
