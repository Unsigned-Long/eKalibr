# eKalibr, Copyright 2024, the School of Geodesy and Geomatics (SGG), Wuhan University, China
# https://github.com/Unsigned-Long/eKalibr.git
# Author: Shuolong Chen (shlchen@whu.edu.cn)
# GitHub: https://github.com/Unsigned-Long
#  ORCID: 0000-0002-5283-9057
# Purpose: See .h/.hpp file.
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * The names of its contributors can not be
#   used to endorse or promote products derived from this software without
#   specific prior written permission.
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# the origin of the pattern must be uniquely identifiable, and thus should not have 180-degree ambiguity.
# for more information, please refer to https://www.mathworks.com/help/vision/ug/calibration-patterns.html
#---------------------------#
# Symmetric (not recommend) #
#---------------------------#
# (1) Circles are arranged evenly in rows and columns
# (2) Dimensions are measured in number of circles as [height width], where height is the number of circles in one row and width is the number of circles in one column.
# (3) Cannot be used to calibrate stereo-visual/inertial-visual suites due to 180-degree ambiguity.
#------------------------#
# Asymmetric (recommend) #
#------------------------#
# (1) Every second row of circles is offset by half the column distance between neighboring row elements.
# (2) Dimensions are measured in number of circles as [dim1 dim2], where dim1 is the number of circles along
#     the dimension that contains the same number of circles in each row or column and dim2 is the number
#     of circles across two adjacent columns (or rows) in the dimension where the two columns (or rows)
#     contain an unequal number of circles.
# (2) Greater density of points for the same circle radius.
# (3) Can be used to calibrate stereo-visual/inertial-visual suites. No 180-degree ambiguity

python3 gen_pattern.py -o acircleboard-7x3-50mm.svg \
        --rows            7                         \
        --columns         3                         \
        --type acircles                             \
        --square_size     50                        \
        --radius_rate     2.5                       \
        --page_width      400                       \
        --page_height     450                       \
        --units           mm

python3 gen_pattern.py -o acircleboard-9x4-50mm.svg \
        --rows            9                         \
        --columns         4                         \
        --type acircles                             \
        --square_size     50                        \
        --radius_rate     2.5                       \
        --page_width      500                       \
        --page_height     550                       \
        --units           mm

python3 gen_pattern.py -o acircleboard-11x4-50mm.svg \
        --rows            11                         \
        --columns         4                          \
        --type acircles                              \
        --square_size     50                         \
        --radius_rate     2.5                        \
        --page_width      500                        \
        --page_height     650                        \
        --units           mm