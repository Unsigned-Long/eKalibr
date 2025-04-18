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

#include "factor/imu_gyro_factor.hpp"
#include "factor/hand_eye_rot_align_factor.hpp"
#include "factor/so3_spline_world_align_factor.hpp"
#include "factor/event_inertial_align_factor.hpp"
#include "factor/imu_acce_factor.hpp"
#include "factor/lin_scale_factor.hpp"
#include "factor/so3_factor.hpp"
#include "factor/visual_projection_factor.hpp"
#include "factor/visual_projection_circle_based_factor.hpp"
#include "factor/hand_eye_transform_align_factor.hpp"
#include "factor/regularization_l2_factor.hpp"

namespace ns_ekalibr {
template struct IMUGyroFactor<Configor ::Prior::SplineOrder>;
template struct HandEyeRotationAlignFactor<Configor::Prior::SplineOrder>;
template struct So3SplineAlignToWorldFactor<Configor::Prior::SplineOrder>;
template struct EventInertialAlignHelper<Configor::Prior::SplineOrder>;
template struct EventInertialAlignFactor<Configor::Prior::SplineOrder>;
template struct IMUAcceFactor<Configor::Prior::SplineOrder, 2>;
template struct LinearScaleDerivFactor<Configor::Prior::SplineOrder, 2>;
template struct LinearScaleDerivFactor<Configor::Prior::SplineOrder, 1>;
template struct LinearScaleDerivFactor<Configor::Prior::SplineOrder, 0>;
template struct So3Factor<Configor::Prior::SplineOrder>;
template struct VisualProjectionFactor<Configor::Prior::SplineOrder>;
template struct VisualProjectionCircleBasedFactor<Configor::Prior::SplineOrder>;
template struct HandEyeTransformAlignFactor<Configor::Prior::SplineOrder>;
template struct RegularizationL2Factor<3>;
}  // namespace ns_ekalibr