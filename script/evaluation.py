#  eKalibr, Copyright 2024, the School of Geodesy and Geomatics (SGG), Wuhan University, China
#  https://github.com/Unsigned-Long/eKalibr.git
#  Author: Shuolong Chen (shlchen@whu.edu.cn)
#  GitHub: https://github.com/Unsigned-Long
#   ORCID: 0000-0002-5283-9057
#  Purpose: See .h/.hpp file.
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#  * Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#  * The names of its contributors can not be
#    used to endorse or promote products derived from this software without
#    specific prior written permission.
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
#  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
#  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
#  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
#  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
#  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
#  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
import os.path
import subprocess
import yaml
import concurrent.futures

from sympy import false


def is_roscore_running():
    try:
        # Check if roscore is running
        output = subprocess.check_output(["pgrep", "-f", "roscore"], stderr=subprocess.DEVNULL)
        return bool(output.strip())
    except subprocess.CalledProcessError:
        return False
    except FileNotFoundError:
        print("pgrep command not found. Please ensure it is installed.")
        return False


# find rospack ekalibr, return its path otherwise return None
def find_ekalibr_path():
    try:
        output = subprocess.check_output(["rospack", "find", "ekalibr"], stderr=subprocess.DEVNULL)
        return output.decode('utf-8').strip()
    except subprocess.CalledProcessError:
        print("Error: ekalibr package not found. Please ensure it is installed.")
        return None
    except FileNotFoundError:
        print("rospack command not found. Please ensure it is installed.")
        return None


def load_yaml_file(path):
    """
    Load a YAML file and return its content as a dictionary.
    :param path:
    :return:
    """
    if not os.path.exists(path):
        print(f"Error: YAML file {path} does not exist.")
        return None
    try:
        with open(path, 'r') as file:
            data = yaml.safe_load(file)
        return data
    except yaml.YAMLError as e:
        print(f"Error loading YAML file {path}: {e}")
        return None


def write_yaml_file(data_dict, path):
    """
    Write a dictionary to a YAML file.
    :param data_dict: Dictionary to write
    :param path: Path to the YAML file
    """
    try:
        with open(path, 'w') as file:
            yaml.dump(data_dict, file, default_flow_style=False, allow_unicode=True)
    except IOError as e:
        print(f"Error writing YAML file {path}: {e}")


def get_ekalibr_template_config_path():
    """
    Get the path to the ekalibr template config file.
    :return: Path to the ekalibr template config file
    """
    ekalibr_path = find_ekalibr_path()
    if ekalibr_path is None:
        print("\033[93mWarning: ekalibr package not found. Please ensure it is installed.\033[0m")
        return None
    template_config_path = os.path.join(ekalibr_path, "config", "ekalibr-config.yaml")
    if not os.path.exists(template_config_path):
        print("\033[93mWarning: ekalibr config template not found. Please ensure it is installed.\033[0m")
        return None
    return template_config_path


def run_ekalibr_calibration_task(config_path):
    cmd = (f"roslaunch ekalibr ekalibr-prog.launch "
           f"config_path:={config_path} "
           f"node_name:={config_path.split('/')[-1].split('.')[0].replace('-', '_')}")
    print(f"Running ekalibr calibration with command: {cmd}")
    subprocess.run(cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def run_ekalibr_calibration(bag_path_list, max_workers, delete_existing_output):
    template_config_path = get_ekalibr_template_config_path()
    if template_config_path is None or not os.path.exists(template_config_path):
        print("\033[93mWarning: ekalibr config template not found. Please ensure it is installed.\033[0m")
    config_data = load_yaml_file(template_config_path)
    if config_data is None:
        print("\033[93mWarning: Failed to load ekalibr config template. Please check the file.\033[0m")
        exit(1)
    config_data['Configor']['DataStream']['EventTopics'] = [
        {
            "key": "/davis_left/events",
            "value": {
                "Type": "DVS_EVENT",
                "Width": 346,
                "Height": 260,
                "Weight": 10.0,
                "Intrinsics": ""
            }
        },
        {
            "key": "/davis_right/events",
            "value": {
                "Type": "DVS_EVENT",
                "Width": 346,
                "Height": 260,
                "Weight": 10.0,
                "Intrinsics": ""
            }
        }
    ]
    config_data["Configor"]["Preference"]["Visualization"] = False

    config_path_list = []
    output_path_list = []
    for bag_path in bag_path_list:
        config_data['Configor']['DataStream']['BagPath'] = bag_path
        board_size = bag_path.split('/')[-2].split('-')[1].split('x')
        rows = int(board_size[0])
        columns = int(board_size[1])
        config_data["Configor"]["Prior"]["CirclePattern"]["Cols"] = columns
        config_data["Configor"]["Prior"]["CirclePattern"]["RadiusRate"] = 2.5
        config_data["Configor"]["Prior"]["CirclePattern"]["Rows"] = rows
        config_data["Configor"]["Prior"]["CirclePattern"]["SpacingMeters"] = 0.05
        config_data["Configor"]["Prior"]["CirclePattern"]["Type"] = "ASYMMETRIC_GRID"
        config_path = bag_path.split('.')[:-1][0] + ".yaml"
        write_yaml_file(config_data, config_path)
        config_path_list.append(config_path)
        output_path = bag_path.split('.')[:-1][0]
        output_path_list.append(output_path)
        if delete_existing_output and os.path.exists(output_path):
            print(f"Output path {output_path} already exists, remove it...")
            os.system(f"rm -r {output_path}")

    with concurrent.futures.ThreadPoolExecutor(max_workers=max_workers) as executor:
        futures = [executor.submit(run_ekalibr_calibration_task, config_path)
                   for config_path in config_path_list]

        concurrent.futures.wait(futures)

    # remove output_path in 'output_path_list' that not exists in disk
    for output_path in reversed(output_path_list):
        if not os.path.exists(output_path):
            print(f"\033[93mWarning: Output path {output_path} does not exist, "
                  f"its solving is not performed or failed!!!\033[0m")

    output_path_list_success = [path for path in output_path_list if os.path.exists(path)]
    print(f"There are total {len(output_path_list_success)}/{len(output_path_list)} cases are solved successfully!!!")
    return output_path_list_success


if __name__ == "__main__":
    dataset_folder = '/media/csl/samsung/eKalibr/dataset/multi-camera'
    bag_path_list = []
    for board_type_folder in os.listdir(dataset_folder):
        board_type_path = os.path.join(dataset_folder, board_type_folder)
        if not os.path.isdir(board_type_path):
            continue
        for rosbag in os.listdir(board_type_path):
            rosbag_path = os.path.join(board_type_path, rosbag)
            if not rosbag.endswith('.bag') or not os.path.isfile(rosbag_path):
                continue
            bag_path_list.append(rosbag_path)

    print(f"There are {len(bag_path_list)} bags to process")
    run_ekalibr_calibration(bag_path_list, 2, False)
