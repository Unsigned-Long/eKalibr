# update submodules
echo "----------------------------------------------------"
echo "update submodules remotely, it may take some time..."
echo "----------------------------------------------------"
git submodule update --init --recursive
if [ $? -ne 0 ]; then
    echo "--------------------------------------------"
    echo "error occurs when updating submodules, exit!"
    echo "--------------------------------------------"
    exit
fi

# shellcheck disable=SC2046
EKALIBR_ROOT_PATH=$(cd $(dirname $0) || exit; pwd)
echo "the root path of 'ctraj': ${EKALIBR_ROOT_PATH}"

# build tiny-viewer
echo "----------------------------------"
echo "build thirdparty: 'tiny-viewer'..."
echo "----------------------------------"

# shellcheck disable=SC2164
cd "${EKALIBR_ROOT_PATH}"/thirdparty/ctraj

chmod +x build_thirdparty.sh
./build_thirdparty.sh

# build ctraj
echo "----------------------------"
echo "build thirdparty: 'ctraj'..."
echo "----------------------------"

mkdir ${EKALIBR_ROOT_PATH}/thirdparty/ctraj-build
# shellcheck disable=SC2164
cd "${EKALIBR_ROOT_PATH}"/thirdparty/ctraj-build

cmake ../ctraj
echo current path: $PWD
echo "-----------------------"
echo "start making 'ctraj'..."
echo "-----------------------"
make -j8
cmake --install . --prefix "${EKALIBR_ROOT_PATH}/thirdparty/ctraj-install"

# build opengv
echo "-----------------------------"
echo "build thirdparty: 'opengv'..."
echo "-----------------------------"

mkdir ${EKALIBR_ROOT_PATH}/thirdparty/opengv-build
# shellcheck disable=SC2164
cd "${EKALIBR_ROOT_PATH}"/thirdparty/opengv-build

cmake ../opengv
echo current path: $PWD
echo "------------------------"
echo "start making 'opengv'..."
echo "------------------------"
make -j8
cmake --install . --prefix "${EKALIBR_ROOT_PATH}/thirdparty/opengv-install"


# build veta
echo "---------------------------"
echo "build thirdparty: 'veta'..."
echo "---------------------------"

mkdir ${EKALIBR_ROOT_PATH}/thirdparty/veta-build
# shellcheck disable=SC2164
cd "${EKALIBR_ROOT_PATH}"/thirdparty/veta-build

cmake ../veta
echo current path: $PWD
echo "----------------------"
echo "start making 'veta'..."
echo "----------------------"
make -j8
cmake --install . --prefix "${EKALIBR_ROOT_PATH}/thirdparty/veta-install"