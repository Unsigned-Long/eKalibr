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

#include "util/utils.h"
#include "spdlog/spdlog.h"
#include "iostream"
#include "opencv2/imgproc.hpp"
#include "iomanip"
#include "filesystem"
#include "list"

namespace ns_ekalibr {
void ConfigSpdlog() {
    // [log type]-[thread]-[time] message
    spdlog::set_pattern("%^[%L]%$-[%t]-[%H:%M:%S.%e] %v");

    // set log level
    spdlog::set_level(spdlog::level::debug);
}

void PrintIKalibrLibInfo() {
    std::cout << "+----------------------------------------------------------+\n"
                 "|   ▓█████  ██ ▄█▀▄▄▄       ██▓     ██▓ ▄▄▄▄    ██▀███     |\n"
                 "|   ▓█   ▀  ██▄█▒▒████▄    ▓██▒    ▓██▒▓█████▄ ▓██ ▒ ██▒   |\n"
                 "|   ▒███   ▓███▄░▒██  ▀█▄  ▒██░    ▒██▒▒██▒ ▄██▓██ ░▄█ ▒   |\n"
                 "|   ▒▓█  ▄ ▓██ █▄░██▄▄▄▄██ ▒██░    ░██░▒██░█▀  ▒██▀▀█▄     |\n"
                 "|   ░▒████▒▒██▒ █▄▓█   ▓██▒░██████▒░██░░▓█  ▀█▓░██▓ ▒██▒   |\n"
                 "|   ░░ ▒░ ░▒ ▒▒ ▓▒▒▒   ▓▒█░░ ▒░▓  ░░▓  ░▒▓███▀▒░ ▒▓ ░▒▓░   |\n"
                 "|    ░ ░  ░░ ░▒ ▒░ ▒   ▒▒ ░░ ░ ▒  ░ ▒ ░▒░▒   ░   ░▒ ░ ▒░   |\n"
                 "|      ░   ░ ░░ ░  ░   ▒     ░ ░    ▒ ░ ░    ░   ░░   ░    |\n"
                 "|      ░  ░░  ░        ░  ░    ░  ░ ░   ░         ░        |\n"
                 "|                                            ░             |\n"
                 "+----------+-----------------------------------------------+\n"
                 "|  eKalibr | https://github.com/Unsigned-Long/eKalibr.git  |\n"
                 "+----------+----------------+--------+---------------------+\n"
                 "|  Author  | Shuolong Chen  | E-Mail | shlchen@whu.edu.cn  |\n"
                 "+----------+----------------+--------+---------------------+"
              << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
}

bool TryCreatePath(const std::string &path) {
    if (!std::filesystem::exists(path) && !std::filesystem::create_directories(path)) {
        spdlog::warn("create directory failed: '{}'", path);
        return false;
    } else {
        return true;
    }
}

std::string UpperString(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](char c) { return std::toupper(c); });
    return s;
}

std::string LowerString(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](char c) { return std::tolower(c); });
    return s;
}

bool IsNotWhiteSpace(int character) {
    return character != ' ' && character != '\n' && character != '\r' && character != '\t';
}

void StringLeftTrim(std::string *str) {
    str->erase(str->begin(), std::find_if(str->begin(), str->end(), IsNotWhiteSpace));
}

void StringRightTrim(std::string *str) {
    str->erase(std::find_if(str->rbegin(), str->rend(), IsNotWhiteSpace).base(), str->end());
}

void StringTrim(std::string *str) {
    StringLeftTrim(str);
    StringRightTrim(str);
}

std::string GetIndexedFilename(int idx, int num) {
    std::stringstream stream;
    std::string filename;
    stream << std::setfill('0') << std::setw(static_cast<int>(std::log10(num)) + 1) << idx;
    stream >> filename;
    return filename;
}

Eigen::MatrixXd TangentBasis(const Eigen::Vector3d &g0) {
    Eigen::Vector3d b, c;
    Eigen::Vector3d a = g0.normalized();
    Eigen::Vector3d tmp(0, 0, 1);
    if (a == tmp) tmp << 1, 0, 0;
    b = (tmp - a * (a.transpose() * tmp)).normalized();
    c = a.cross(b);
    Eigen::MatrixXd bc(3, 2);
    bc.block<3, 1>(0, 0) = b;
    bc.block<3, 1>(0, 1) = c;
    return bc;
}

std::vector<std::size_t> SamplingWoutReplace(std::default_random_engine &engine,
                                             std::size_t num,
                                             std::size_t start,
                                             std::size_t end,
                                             std::size_t step) {
    // create the pool for sampling
    std::vector<std::size_t> idxPool((end - start) / step + 1);
    for (int i = 0; i != static_cast<int>(idxPool.size()); ++i) {
        idxPool.at(i) = start + i * step;
    }
    std::vector<std::size_t> res(num);
    // the engine
    for (std::size_t i = 0; i != num; ++i) {
        // generate the random index
        std::uniform_int_distribution<std::size_t> ui(0, idxPool.size() - 1);
        std::size_t ridx = ui(engine);
        // record it
        res.at(i) = idxPool.at(ridx);
        // remove it
        idxPool.at(ridx) = idxPool.back();
        idxPool.pop_back();
    }
    return res;
}

std::vector<std::size_t> SamplingWithReplace(std::default_random_engine &engine,
                                             std::size_t num,
                                             std::size_t start,
                                             std::size_t end,
                                             std::size_t step) {
    // create the pool for sampling
    std::vector<std::size_t> idxPool((end - start) / step + 1);
    for (int i = 0; i != static_cast<int>(idxPool.size()); ++i) {
        idxPool.at(i) = start + i * step;
    }
    std::vector<std::size_t> res(num);
    // the engine
    std::uniform_int_distribution<std::size_t> ui(0, idxPool.size() - 1);
    for (std::size_t i = 0; i != num; ++i) {
        // generate the random index
        std::size_t ridx = ui(engine);
        // record it
        res.at(i) = idxPool.at(ridx);
    }
    return res;
}

Eigen::Vector3d RotMatToYPR(const Eigen::Matrix3d &R) {
    Eigen::Vector3d n = R.col(0);
    Eigen::Vector3d o = R.col(1);
    Eigen::Vector3d a = R.col(2);

    Eigen::Vector3d ypr(3);
    double y = atan2(n(1), n(0));
    double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
    double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));

    ypr(0) = y;
    ypr(1) = p;
    ypr(2) = r;
    return ypr / M_PI * 180.0;
}

double NormalizeAngle(double ang_degree) {
    if (ang_degree > 180.0) ang_degree -= 360.0;
    if (ang_degree < -180.0) ang_degree += 360.0;
    return ang_degree;
}

std::vector<std::string> FilesInDir(const std::string &directory, bool sort) {
    std::list<std::string> files;
    for (const auto &elem : std::filesystem::directory_iterator(directory))
        if (elem.status().type() != std::filesystem::file_type::directory)
            files.emplace_back(std::filesystem::canonical(elem.path()).c_str());
    if (sort) {
        files.sort();
    }
    return std::vector(files.begin(), files.end());
}

std::vector<std::string> FilesInDirRecursive(const std::string &directory, bool sort) {
    std::list<std::string> files;
    for (const auto &elem : std::filesystem::recursive_directory_iterator(directory))
        if (elem.status().type() != std::filesystem::file_type::directory)
            files.emplace_back(std::filesystem::canonical(elem.path()).c_str());
    if (sort) {
        files.sort();
    }
    return std::vector(files.begin(), files.end());
}

std::vector<std::string> SplitString(const std::string &str, char splitor, bool ignoreEmpty) {
    std::vector<std::string> vec;
    auto iter = str.cbegin();
    while (true) {
        auto pos = std::find(iter, str.cend(), splitor);
        auto elem = std::string(iter, pos);
        if (!(elem.empty() && ignoreEmpty)) {
            vec.push_back(elem);
        }
        if (pos == str.cend()) {
            break;
        }
        iter = ++pos;
    }
    return vec;
}

void DrawKeypointOnCVMat(cv::Mat &img,
                         const Eigen::Vector2d &feat,
                         bool withBox,
                         const cv::Scalar &color) {
    DrawKeypointOnCVMat(img, cv::Point2d(feat(0), feat(1)), withBox, color);
}

void DrawKeypointOnCVMat(cv::Mat &img,
                         const cv::Point2d &feat,
                         bool withBox,
                         const cv::Scalar &color) {
    // square
    if (withBox) {
        cv::drawMarker(img, feat, color, cv::MarkerTypes::MARKER_SQUARE, 10, 1);
    }
    // key point
    cv::drawMarker(img, feat, color, cv::MarkerTypes::MARKER_SQUARE, 2, 2);
}

void DrawLineOnCVMat(cv::Mat &img,
                     const Eigen::Vector2d &p1,
                     const Eigen::Vector2d &p2,
                     const cv::Scalar &color) {
    DrawLineOnCVMat(img, cv::Point2d(p1(0), p1(1)), cv::Point2d(p2(0), p2(1)), color);
}

void DrawLineOnCVMat(cv::Mat &img,
                     const cv::Point2d &p1,
                     const cv::Point2d &p2,
                     const cv::Scalar &color) {
    cv::line(img, p1, p2, color, 1, cv::LINE_AA);
}
void PutTextOnCVMat(cv::Mat &img,
                    const std::string &str,
                    const cv::Point2d &pt,
                    double xBias,
                    double yBias,
                    const cv::Scalar &color) {
    cv::putText(img, str, cv::Point2d(pt.x + xBias, pt.y + yBias),
                cv::HersheyFonts::FONT_HERSHEY_PLAIN, 1.0, color, 2);
}
void PutTextOnCVMat(cv::Mat &img,
                                const std::string &str,
                                const Eigen::Vector2d &pt,
                                double xBias,
                                double yBias,
                                const cv::Scalar &color) {
    PutTextOnCVMat(img, str, cv::Point2d(pt(0), pt(1)), xBias, yBias, color);
}
}
