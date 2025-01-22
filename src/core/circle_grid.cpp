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