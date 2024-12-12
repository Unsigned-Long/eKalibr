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

#ifndef CIRCLE_GRID_H
#define CIRCLE_GRID_H

#include "opencv4/opencv2/core/types.hpp"
#include "util/cereal_archive_helper.hpp"
#include "sensor/sensor_model.h"
#include "list"

#include <ostream>

namespace cv {
template <class Archive, typename ScaleType>
void serialize(Archive& archive, Point_<ScaleType>& m) {
    archive(cereal::make_nvp("x", m.x), cereal::make_nvp("y", m.y));
}
template <class Archive, typename ScaleType>
void serialize(Archive& archive, Point3_<ScaleType>& m) {
    archive(cereal::make_nvp("x", m.x), cereal::make_nvp("y", m.y), cereal::make_nvp("z", m.z));
}
}  // namespace cv

namespace ns_ekalibr {

struct CircleGrid2D {
    using Ptr = std::shared_ptr<CircleGrid2D>;

    double timestamp;
    std::vector<cv::Point2f> centers;

    CircleGrid2D(double timestamp = 0.0, const std::vector<cv::Point2f>& centers = {});

    static Ptr Create(double timestamp, const std::vector<cv::Point2f>& centers);

public:
    template <class Archive>
    void serialize(Archive& ar) {
        ar(CEREAL_NVP(timestamp), CEREAL_NVP(centers));
    }
};

struct CircleGrid3D {
    using Ptr = std::shared_ptr<CircleGrid3D>;

    std::size_t rows;
    std::size_t cols;
    double spacing;
    CirclePatternType type;
    std::vector<cv::Point3f> points;

    CircleGrid3D(std::size_t rows = 0,
                 std::size_t cols = 0,
                 double spacing = 0.0,
                 CirclePatternType type = CirclePatternType::ASYMMETRIC_GRID);

    static Ptr Create(std::size_t rows, std::size_t cols, double spacing, CirclePatternType type);

    std::size_t GridCoordinatesToPointIdx(std::size_t r, std::size_t c) const;

    const cv::Point3f& GridCoordinatesToPoint(std::size_t r, std::size_t c) const;

    cv::Point3f& GridCoordinatesToPoint(std::size_t r, std::size_t c);

    friend std::ostream& operator<<(std::ostream& os, const CircleGrid3D& obj);

    template <class Archive>
    void serialize(Archive& ar) {
        ar(CEREAL_NVP(rows), CEREAL_NVP(cols), CEREAL_NVP(spacing), CEREAL_NVP(type),
           CEREAL_NVP(points));
    }
};

class CircleGridPattern {
public:
    using Ptr = std::shared_ptr<CircleGridPattern>;

protected:
    double _timeBias;
    CircleGrid3D::Ptr _grid3d;
    std::list<CircleGrid2D::Ptr> _grid2d;

public:
    explicit CircleGridPattern(CircleGrid3D::Ptr grid3d,
                               double timeBias,
                               const std::list<CircleGrid2D::Ptr>& grid2d = {});

    static Ptr Create(const CircleGrid3D::Ptr& grid3d,
                      double timeBias,
                      const std::list<CircleGrid2D::Ptr>& grid2d = {});

    void AddGrid2d(const CircleGrid2D::Ptr& grid2d);

    const std::list<CircleGrid2D::Ptr>& GetGrid2d() const;

    const CircleGrid3D::Ptr& GetGrid3d() const;

    // load configure information from file
    static Ptr Load(const std::string& filename,
                    double newTimeBias,
                    CerealArchiveType::Enum archiveType = CerealArchiveType::Enum::YAML);

    // save configure information to file
    bool Save(const std::string& filename,
              CerealArchiveType::Enum archiveType = CerealArchiveType::Enum::YAML);

    friend std::ostream& operator<<(std::ostream& os, const CircleGridPattern& obj);

    std::string InfoString() const;

public:
    template <class Archive>
    void serialize(Archive& ar) {
        ar(cereal::make_nvp("time_bias", _timeBias), cereal::make_nvp("grid_3d", _grid3d),
           cereal::make_nvp("grid_2d", _grid2d));
    }
};
}  // namespace ns_ekalibr

#endif  // CIRCLE_GRID_H
