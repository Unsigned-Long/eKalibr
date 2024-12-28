#
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
#

#roslaunch dvs_renderer davis_mono_events_only.launch
if [ $# -ne 1 ]; then
    echo -e "please provide a number (0, 1, or 2) to specify the folder:\n(0): acircle-7x3-50mm\n(1): acircle-9x4-50mm\n(2): acircle-11x4-50mm"
    exit 1
fi

case $1 in
    0)
        Category="acircle-7x3-50mm"
        ;;
    1)
        Category="acircle-9x4-50mm"
        ;;
    2)
        Category="acircle-11x4-50mm"
        ;;
    *)
        echo -e "please provide a number (0, 1, or 2) to specify the folder:\n(0): acircle-7x3-50mm\n(1): acircle-9x4-50mm\n(2): acircle-11x4-50mm"
        exit 1
        ;;
esac

# launch driver first
# roslaunch dvs_renderer davis_mono_events_only.launch

# EKALIBR_ROOT_PATH=$(rospack find ekalibr)
EKALIBR_ROOT_PATH=/media/csl/samsung/eKalibr
OutputPath=$EKALIBR_ROOT_PATH/dataset/$Category

mkdir -p ${OutputPath}
BagPath=$OutputPath/eKalibr-data-$(date +%Y-%m-%d-%H-%M-%S).bag
echo "record davis data [/dvs/events;/dvs/imu] as ros bag [$BagPath]..."

rosbag record --duration=30 -O $BagPath /dvs/events /dvs/imu /dvs/image_raw