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

#ifndef STATUS_HPP
#define STATUS_HPP

#include <exception>
#include <string>
#include <iostream>
#include <util/enum_cast.hpp>
#include "spdlog/fmt/fmt.h"

namespace ns_ekalibr {
#define EKALIBR_CODE_POS fmt::format("[line: '{}', file: '{}']", __LINE__, __FILE__)

#define EKALIBR_DEBUG                                                          \
spdlog::warn(fmt::format(fmt::emphasis::italic | fmt::fg(fmt::color::red), \
"(EKALIBR-DEBUG) {}", EKALIBR_CODE_POS));

enum class Status : std::uint8_t { FINE, WARNING, ERROR, CRITICAL };

struct EKalibrStatus : std::exception {
public:
    Status flag;
    std::string what;

public:
    EKalibrStatus(Status flag, std::string what)
        : flag(flag),
          what(std::move(what)) {}

    friend std::ostream &operator<<(std::ostream &os, const EKalibrStatus &status) {
        os << "[" << EnumCast::enumToString(status.flag) << "]-[" << status.what << "]";
        return os;
    }
};

template <typename... T>
EKalibrStatus Status(Status flag, fmt::format_string<T...> fmt, T &&...args) {
    return EKalibrStatus(flag, fmt::format(fmt, args...));
}
}

#endif //STATUS_HPP
