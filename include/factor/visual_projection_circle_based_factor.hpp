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
    Eigen::Vector2d pixCenter;

    VisualProjectionCircleBasedPair(const Circle3D::Ptr &circle,
                                    const Event::Ptr &ev,
                                    const Eigen::Vector2d &pixCenter)
        : c(circle),
          ev(ev),
          pixCenter(pixCenter) {}

    static Ptr Create(const Circle3D::Ptr &circle,
                      const Event::Ptr &ev,
                      const Eigen::Vector2d &pixCenter) {
        return std::make_shared<VisualProjectionCircleBasedPair>(circle, ev, pixCenter);
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

    // Function to calculate the shortest distance from a point to a line segment
    template <typename T>
    static Eigen::Vector2<T> PointToSegDistVec(const Eigen::Vector2<T> &point,
                                               const Eigen::Vector2<T> &A,
                                               const Eigen::Vector2<T> &B) {
        Eigen::Vector2<T> AB = B - A;      // Vector from A to B
        Eigen::Vector2<T> AP = point - A;  // Vector from A to point
        Eigen::Vector2<T> BP = point - B;  // Vector from B to point

        T AB_AB = AB.squaredNorm();  // Square of the length of AB
        if (AB_AB == T(0.0)) {
            // If A and B are the same point, return distance from point to A
            return point - A;
        }

        // Compute the projection coefficient
        T t = AP.dot(AB) / AB_AB;
        if (t < T(0.0)) {
            // Projection falls outside A, return distance from point to A
            return AP;
        } else if (t > T(1.0)) {
            // Projection falls outside B, return distance from point to B
            return BP;
        } else {
            // Projection falls on the line segment, return perpendicular distance
            Eigen::Vector2<T> projection = A + t * AB;
            return point - projection;
        }
    }

    // Function to calculate the shortest distance from a point to a convex polygon
    template <typename T, std::size_t Size>
    static Eigen::Vector2<T> PointToConvexPolygonDistance(
        const Eigen::Vector2<T> &point, const std::array<Eigen::Vector2<T>, Size> &verts) {
        // Initialize the minimum distance to a very large value
        double minDistSquared = std::numeric_limits<double>::max();
        Eigen::Vector2<T> minDistVec = Eigen::Vector2<T>::Zero();

        // Iterate over each edge of the polygon
        for (size_t i = 0; i < verts.size(); ++i) {
            // Current vertex
            Eigen::Vector2<T> A = verts[i];
            // Next vertex (wraps around to the first vertex)
            Eigen::Vector2<T> B = verts[(i + 1) % verts.size()];

            // Calculate the distance from the point to the current edge (line segment)
            Eigen::Vector2<T> distVec = PointToSegDistVec(point, A, B);
            T squaredNorm = distVec.squaredNorm();
            double squaredDist = 0.0;
            if constexpr (std::is_same<T, double>::value) {
                squaredDist = squaredNorm;
            } else if (std::is_same<T, ceres::Jet<double, 4>>::value) {
                squaredDist = squaredNorm.a;
            } else {
                throw Status(Status::CRITICAL, "Unsupported type, 'PointToConvexPolygonDistance'");
            }

            // Update the minimum distance
            if (squaredDist < minDistSquared) {
                minDistSquared = squaredDist;
                minDistVec = distVec;
            }
        }

        return minDistVec;  // Return the shortest distance to the polygon
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

        std::size_t SO3_CjToCr_OFFSET = _so3Meta.NumParameters() + _scaleMeta.NumParameters();
        std::size_t POS_CjInCr_OFFSET = SO3_CjToCr_OFFSET + 1;
        std::size_t TO_CjToCr_OFFSET = POS_CjInCr_OFFSET + 1;
        std::size_t FX_OFFSET = TO_CjToCr_OFFSET + 1;
        std::size_t FY_OFFSET = FX_OFFSET + 1;
        std::size_t CX_OFFSET = FY_OFFSET + 1;
        std::size_t CY_OFFSET = CX_OFFSET + 1;
        std::size_t DIST_COEFFS_OFFSET = CY_OFFSET + 1;

        Eigen::Map<const Sophus::SO3<T>> SO3_CjToCr(sKnots[SO3_CjToCr_OFFSET]);
        Eigen::Map<const Eigen::Vector3<T>> POS_CjInCr(sKnots[POS_CjInCr_OFFSET]);
        Sophus::SE3<T> SE3_CjToCr(SO3_CjToCr, POS_CjInCr);

        T TO_CjToCr = sKnots[TO_CjToCr_OFFSET][0];

        T FX = sKnots[FX_OFFSET][0];
        T FY = sKnots[FY_OFFSET][0];
        T CX = sKnots[CX_OFFSET][0];
        T CY = sKnots[CY_OFFSET][0];

        // this is for pinhole brow t2 [k1, k2, k3, p1, p2]
        Eigen::Map<const Eigen::Vector5<T>> DIST_COEFFS(sKnots[DIST_COEFFS_OFFSET]);

        T timeByCr = static_cast<T>(_pair->ev->GetTimestamp()) + TO_CjToCr;

        // calculate the so3 and lin scale offset
        std::pair<std::size_t, T> iuSo3, iuScale;
        _so3Meta.ComputeSplineIndex(timeByCr, iuSo3.first, iuSo3.second);
        _scaleMeta.ComputeSplineIndex(timeByCr, iuScale.first, iuScale.second);

        SO3_OFFSET = iuSo3.first;
        LIN_SCALE_OFFSET = iuScale.first + _so3Meta.NumParameters();

        Sophus::SO3<T> SO3_CrToW;
        ns_ctraj::CeresSplineHelperJet<T, Order>::EvaluateLie(sKnots + SO3_OFFSET, iuSo3.second,
                                                              _so3DtInv, &SO3_CrToW);
        Eigen::Vector3<T> POS_CrInW;
        ns_ctraj::CeresSplineHelperJet<T, Order>::template Evaluate<3, 0>(
            sKnots + LIN_SCALE_OFFSET, iuScale.second, _scaleDtInv, &POS_CrInW);

        Sophus::SE3<T> SE3_CrToW(SO3_CrToW, POS_CrInW);

        Sophus::SE3<T> SE3_WToCj = (SE3_CrToW * SE3_CjToCr /*SE3_CjToW*/).inverse();

        // project the circle to image plane and calculate the 2d residual
        Eigen::Vector3<T> cenInCam = SE3_WToCj * _pair->c->center.cast<T>();
        Eigen::Vector3<T> xInCam = SE3_WToCj.so3() * _pair->c->orientation.col(0).cast<T>();
        Eigen::Vector3<T> yInCam = SE3_WToCj.so3() * _pair->c->orientation.col(1).cast<T>();
        /**
         * a point on this circle can be obtained by:
         * p = cenInCam + radius * xInCam * cos(theta) + radius * yInCam * sin(theta)
         */

        constexpr int COUNT_PER_CIRCLE = 8;
        constexpr double DELTA_ANG = M_PI * 2.0 / COUNT_PER_CIRCLE;

        std::array<Eigen::Vector2<T>, COUNT_PER_CIRCLE> pixOnCircle;
        for (int i = 0; i < COUNT_PER_CIRCLE; i++) {
            const double ang = DELTA_ANG * static_cast<double>(i);
            const double c = std::cos(ang), s = std::sin(ang);
            const double r = _pair->c->radius;
            Eigen::Vector3<T> pInCam = cenInCam + T(c) * xInCam * r + T(s) * yInCam * r;
            // from camera frame to camera normalized plane
            Eigen::Vector2<T> pInCamPlane(pInCam(0) / pInCam(2), pInCam(1) / pInCam(2));
            // add distortion
            using Helper = VisualProjectionFactor<Configor::Prior::SplineOrder>;
            pInCamPlane = Helper::AddDistortion<T>(DIST_COEFFS, pInCamPlane);

            Helper::TransformCamToImg<T>(&FX, &FY, &CX, &CY, pInCamPlane, &pixOnCircle.at(i));
        }

        Eigen::Map<Eigen::Vector2<T>> residuals(sResiduals);
        residuals = PointToConvexPolygonDistance<T, COUNT_PER_CIRCLE>(_pair->ev->GetPos().cast<T>(),
                                                                      pixOnCircle);
        residuals = T(_weight) * residuals;

        return true;
    }

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

extern template struct VisualProjectionCircleBasedFactor<Configor::Prior::SplineOrder>;
}  // namespace ns_ekalibr

#endif  // VISUAL_PROJECTION_CIRCLE_BASED_FACTOR_HPP
