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
import numpy as np
from quaternions import Quaternion


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


def run_ekalibr_calibration_task(config_path, output=True):
    cmd = (f"roslaunch ekalibr ekalibr-prog.launch "
           f"config_path:={config_path} "
           f"node_name:={config_path.split('/')[-1].split('.')[0].replace('-', '_')}")
    print(f"Running ekalibr calibration with command: {cmd}")
    if not output:
        subprocess.run(cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    else:
        subprocess.run(cmd, shell=True)


def run_ekalibr_calibration(bag_path_list, max_workers, delete_existing_output):
    if not is_roscore_running():
        print("\033[93mWarning: roscore is not running. Please start roscore before running this script.\033[0m")
        return []
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

    solve_info = []
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
        output_path = bag_path.split('.')[:-1][0]
        write_yaml_file(config_data, config_path)
        info = {
            "id": bag_path.split('/')[-1].split('.')[0],
            "config_file_path": config_path,
            "output_dir_path": output_path,
            "param_file_path": os.path.join(output_path, "ekalibr_param.all.yaml")
        }
        if os.path.exists(output_path):
            if delete_existing_output:
                print(f"Output path {output_path} already exists, remove it...")
                os.system(f"rm -r {output_path}")
                info["need_to_solve"] = True
            else:
                print(f"Output path {output_path} already exists, skip it...")
                info["need_to_solve"] = False
        else:
            info["need_to_solve"] = True
        solve_info.append(info)

    with concurrent.futures.ThreadPoolExecutor(max_workers=max_workers) as executor:
        futures = [executor.submit(run_ekalibr_calibration_task, info["config_file_path"])
                   for info in solve_info if info["need_to_solve"]]

        concurrent.futures.wait(futures)

    # remove output_path in 'output_path_list' that not exists in disk
    solve_info_succuss = []
    for item in solve_info:
        if not os.path.exists(item["param_file_path"]):
            print(f"\033[93mWarning: Parameter file {item['param_file_path']} does not exist, "
                  f"its solving is not performed or failed!!!\033[0m")
        else:
            solve_info_succuss.append(item)

    print(f"There are total {len(solve_info_succuss)}/{len(solve_info)} cases are solved successfully!!!")
    return solve_info_succuss


def extract_values_by_key_path_and_event(data, path_list, target_key):
    result = []

    for entry in data.values():
        current = entry
        try:
            for key in path_list:
                current = current[key]
            for item in current:
                if item.get("key") == target_key:
                    result.append(item["value"])
        except (KeyError, TypeError):
            continue

    return result


def save_solve_results(solve_info_succuss, output_path):
    # load yaml data in each ekalibr_param.all.yaml file and combine them into a list and then save them into a file
    all_params = {}
    for item in solve_info_succuss:
        param_file_path = item["param_file_path"]
        if not os.path.exists(param_file_path):
            print(f"\033[93mWarning: Parameter file {param_file_path} does not exist, "
                  f"its evaluation is not performed!!!\033[0m")
            continue
        params = load_yaml_file(param_file_path)
        if params is None:
            print(f"\033[93mWarning: Failed to load parameters from {param_file_path}, "
                  f"its evaluation is not performed!!!\033[0m")
            continue
        all_params[item["id"]] = params

    write_yaml_file(all_params, output_path)
    print(f"eKalibr solving results saved to {output_path}")


def evaluate(ekalibr_results_path):
    # load yaml data in each ekalibr_param.all.yaml file and combine them into a list and then save them into a file
    all_params = load_yaml_file(ekalibr_results_path)
    SO3_CjToBr = extract_values_by_key_path_and_event(
        all_params, ["CalibParam", "EXTRI", "SO3_CjToBr"], "/davis_right/events"
    )
    SO3_CjToBr = [Quaternion(elem["qw"], elem["qx"], elem["qy"], elem["qz"]).get_euler() for elem in SO3_CjToBr]
    SO3_CjToBr = [[elem * 180.0 / np.pi for elem in euler] for euler in SO3_CjToBr]

    POS_CjInBr = extract_values_by_key_path_and_event(
        all_params, ["CalibParam", "EXTRI", "POS_CjInBr"], "/davis_right/events"
    )
    POS_CjInBr = [[elem["r0c0"] * 100.0, elem["r1c0"] * 100.0, elem["r2c0"] * 100.0] for elem in POS_CjInBr]

    TO_CjToBr = extract_values_by_key_path_and_event(
        all_params, ["CalibParam", "TEMPORAL", "TO_CjToBr"], "/davis_right/events"
    )
    TO_CjToBr = [[elem * 1000.0] for elem in TO_CjToBr]

    np.set_printoptions(precision=3, suppress=True)
    print(f"'SO3_CjToBr'-> "
          f"mean: {np.mean(np.array(SO3_CjToBr), axis=0)} deg, "
          f"std: {np.std(np.array(SO3_CjToBr), axis=0)} deg")
    print(f"'POS_CjInBr'-> "
          f"mean: {np.mean(np.array(POS_CjInBr), axis=0)} cm, "
          f"std: {np.std(np.array(POS_CjInBr), axis=0)} cm")
    print(f" 'TO_CjToBr'-> "
          f"mean: {np.mean(np.array(TO_CjToBr), axis=0)} ms, "
          f"std: {np.std(np.array(TO_CjToBr), axis=0)} ms")


def full_pipeline_evaluation(dataset_folder, max_workers=1, delete_existing_output=True):
    if not os.path.exists(dataset_folder):
        print(f"Error: Dataset folder {dataset_folder} does not exist.")
        exit(1)
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
    solve_info_succuss = run_ekalibr_calibration(bag_path_list, max_workers, delete_existing_output)
    ekalibr_results_path = os.path.join(dataset_folder, "ekalibr_results.yaml")
    save_solve_results(solve_info_succuss, ekalibr_results_path)
    evaluate(ekalibr_results_path)


if __name__ == "__main__":
    dataset_folder = '/media/csl/samsung/eKalibr/dataset/multi-camera'
    full_pipeline_evaluation(dataset_folder=dataset_folder, max_workers=1, delete_existing_output=True)
    # evaluate(os.path.join(dataset_folder, "ekalibr_results_no_time_offset.yaml"))
