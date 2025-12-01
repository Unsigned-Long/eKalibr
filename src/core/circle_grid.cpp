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

#include "core/circle_grid.h"
#include "util/status.hpp"
#include "cereal/types/polymorphic.hpp"
#include "cereal/types/list.hpp"
#include "cereal/types/vector.hpp"
#include <opencv2/imgproc.hpp>

namespace ns_ekalibr {
CircleGrid2D::CircleGrid2D(int id,
                           double timestamp,
                           const std::vector<cv::Point2f>& centers,
                           const std::vector<uint8_t>& cenValidity,
                           bool isComplete)
    : id(id),
      timestamp(timestamp),
      centers(centers),
      cenValidity(cenValidity),
      isComplete(isComplete) {}

CircleGrid2D::Ptr CircleGrid2D::Create(int id,
                                       double timestamp,
                                       const std::vector<cv::Point2f>& centers,
                                       const std::vector<uint8_t>& cenValidity,
                                       bool isComplete) {
    return std::make_shared<CircleGrid2D>(id, timestamp, centers, cenValidity, isComplete);
}

void CircleGrid2D::DrawCenters(cv::Mat& image, cv::Size patternSize) const {
    int type = image.type();
    int cn = CV_MAT_CN(type);
    CV_CheckType(type, cn == 1 || cn == 3 || cn == 4, "Number of channels must be 1, 3 or 4");

    int depth = CV_MAT_DEPTH(type);
    CV_CheckType(type, depth == CV_8U || depth == CV_16U || depth == CV_32F,
                 "Only 8-bit, 16-bit or floating-point 32-bit images are supported");
    cv::InputArray _corners = centers;
    if (_corners.empty()) return;
    cv::Mat corners = _corners.getMat();
    const cv::Point2f* corners_data = corners.ptr<cv::Point2f>(0);
    CV_DbgAssert(corners_data);
    int nelems = corners.checkVector(2, CV_32F, true);
    CV_Assert(nelems >= 0);

    const int shift = 0;
    const int radius = 4;
    const int r = radius * (1 << shift);

    double scale = 1;
    switch (depth) {
        case CV_8U:
            scale = 1;
            break;
        case CV_16U:
            scale = 256;
            break;
        case CV_32F:
            scale = 1. / 255;
            break;
        default: {
        }
    }

    int line_type = (type == CV_8UC1 || type == CV_8UC3) ? cv::LINE_AA : cv::LINE_8;

    const int line_max = 7;
    static const int line_colors[line_max][4] = {
        {0, 0, 255, 0},   {0, 128, 255, 0}, {0, 200, 200, 0}, {0, 255, 0, 0},
        {200, 200, 0, 0}, {255, 0, 0, 0},   {255, 0, 255, 0}};

    cv::Point2i prev_pt;
    bool prev_valid = false;
    for (int row = 0, i = 0; row < patternSize.height; row++) {
        const int* line_color = &line_colors[row % line_max][0];
        cv::Scalar color(line_color[0], line_color[1], line_color[2], line_color[3]);
        if (cn == 1) color = cv::Scalar::all(200);
        color *= scale;

        for (int col = 0; col < patternSize.width; col++, i++) {
            if (!cenValidity[i]) {
                prev_valid = false;
                continue;
            }
            cv::Point2i pt(cvRound(corners_data[i].x * (1 << shift)),
                           cvRound(corners_data[i].y * (1 << shift)));

            if (i != 0 && prev_valid) {
                line(image, prev_pt, pt, color, 1, line_type, shift);
            }

            line(image, cv::Point(pt.x - r, pt.y - r), cv::Point(pt.x + r, pt.y + r), color, 1,
                 line_type, shift);
            line(image, cv::Point(pt.x - r, pt.y + r), cv::Point(pt.x + r, pt.y - r), color, 1,
                 line_type, shift);
            circle(image, pt, r + (1 << shift), color, 1, line_type, shift);
            prev_pt = pt;
            prev_valid = true;
        }
    }
}

CircleGrid3D::CircleGrid3D(std::size_t rows,
                           std::size_t cols,
                           double spacing,
                           CirclePatternType type)
    : rows(rows),
      cols(cols),
      spacing(spacing),
      type(type),
      points(rows * cols) {
    switch (type) {
        case CirclePatternType::SYMMETRIC_GRID:
            for (unsigned int r = 0; r < rows; r++) {
                for (unsigned int c = 0; c < cols; c++) {
                    GridCoordinatesToPoint(r, c) = cv::Point3d(spacing * r, spacing * c, 0.0);
                }
            }
            break;
        case CirclePatternType::ASYMMETRIC_GRID:
            for (unsigned int r = 0; r < rows; r++) {
                for (unsigned int c = 0; c < cols; c++) {
                    GridCoordinatesToPoint(r, c) =
                        cv::Point3d((2 * c + r % 2) * spacing, r * spacing, 0.0);
                }
            }
            break;
        default:
            throw Status(Status::ERROR,
                         "unsupported circle pattern!!! only 'SYMMETRIC_GRID' and "
                         "'ASYMMETRIC_GRID' are supported!!!");
    }
}
cv::Size CircleGrid3D::GetPatternSize() const {
    return cv::Size(static_cast<int>(cols), static_cast<int>(rows));
}

CircleGrid3D::Ptr CircleGrid3D::Create(std::size_t rows,
                                       std::size_t cols,
                                       double spacing,
                                       CirclePatternType type) {
    return std::make_shared<CircleGrid3D>(rows, cols, spacing, type);
}

std::size_t CircleGrid3D::GridCoordinatesToPointIdx(std::size_t r, std::size_t c) const {
    return cols * r + c;
}

const cv::Point3f& CircleGrid3D::GridCoordinatesToPoint(std::size_t r, std::size_t c) const {
    return points.at(GridCoordinatesToPointIdx(r, c));
}

cv::Point3f& CircleGrid3D::GridCoordinatesToPoint(std::size_t r, std::size_t c) {
    return points.at(GridCoordinatesToPointIdx(r, c));
}

CircleGridPattern::CircleGridPattern(CircleGrid3D::Ptr grid3d,
                                     double timeBias,
                                     const std::list<CircleGrid2D::Ptr>& grid2d)
    : _timeBias(timeBias),
      _grid3d(std::move(grid3d)),
      _grid2d(grid2d) {}

CircleGridPattern::Ptr CircleGridPattern::Create(const CircleGrid3D::Ptr& grid3d,
                                                 double timeBias,
                                                 const std::list<CircleGrid2D::Ptr>& grid2d) {
    return std::make_shared<CircleGridPattern>(grid3d, timeBias, grid2d);
}

void CircleGridPattern::AddGrid2d(const CircleGrid2D::Ptr& grid2d) { _grid2d.push_back(grid2d); }

const std::list<CircleGrid2D::Ptr>& CircleGridPattern::GetGrid2d() const { return _grid2d; }

std::list<CircleGrid2D::Ptr>& CircleGridPattern::GetGrid2d() { return _grid2d; }

std::list<int> CircleGridPattern::RemoveGrid2DOutOfTimeRange(double st, double et) {
    std::list<int> idsOfRemoved;
    for (auto it = _grid2d.cbegin(); it != _grid2d.cend();) {
        auto t = (*it)->timestamp + _timeBias;
        if (t < st || t > et) {
            idsOfRemoved.push_back((*it)->id);
            it = _grid2d.erase(it);
        } else {
            ++it;
        }
    }
    return idsOfRemoved;
}

const CircleGrid3D::Ptr& CircleGridPattern::GetGrid3d() const { return _grid3d; }

CircleGridPattern::Ptr CircleGridPattern::Load(const std::string& filename,
                                               double newTimeBias,
                                               CerealArchiveType::Enum archiveType) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        return nullptr;
    }
    auto pattern = Create(nullptr, {});
    try {
        auto archive = GetInputArchiveVariant(file, archiveType);
        SerializeByInputArchiveVariant(archive, archiveType,
                                       cereal::make_nvp("grid3d_with_grid2ds", *pattern));
    } catch (const cereal::Exception& exception) {
        throw Status(Status::CRITICAL,
                     "Can not load 'CircleGridPattern' file '{}' into eKalibr using cereal!!! "
                     "Detailed cereal exception information: \n'{}'",
                     filename, exception.what());
    }
    for (const auto& grid2d : pattern->GetGrid2d()) {
        grid2d->timestamp = grid2d->timestamp + pattern->_timeBias - newTimeBias;
    }
    pattern->_timeBias = newTimeBias;
    return pattern;
}

bool CircleGridPattern::Save(const std::string& filename, CerealArchiveType::Enum archiveType) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        return false;
    }
    auto archive = GetOutputArchiveVariant(file, archiveType);
    SerializeByOutputArchiveVariant(archive, archiveType,
                                    cereal::make_nvp("grid3d_with_grid2ds", *this));
    return true;
}

std::string CircleGridPattern::InfoString() const {
    std::stringstream ss;
    ss << "CircleGridPattern [" << *this << ']';
    return ss.str();
}

std::ostream& operator<<(std::ostream& os, const CircleGrid3D& obj) {
    return os << "rows: " << obj.rows << ", cols: " << obj.cols << ", spacing: " << obj.spacing
              << " (m), type: '" << CirclePattern::ToString(obj.type)
              << "', points size: " << obj.points.size();
}

std::ostream& operator<<(std::ostream& os, const CircleGridPattern& obj) {
    return os << "time bias: " << obj._timeBias << " (sec), grid3d: [" << *obj._grid3d
              << "], grid2d size: " << obj._grid2d.size();
}
}  // namespace ns_ekalibr