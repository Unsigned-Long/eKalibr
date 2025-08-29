import subprocess
import yaml
from pathlib import Path
import rosgraph
import datetime
import os

def is_roscore_running():
    try:
        return rosgraph.is_master_online()
    except Exception:
        return False

def create_merge_config(
    left_bag: str, right_bag: str, output_bag_path: str, save_path: str
):
    config = {
        "MergeConfigor": {
            "Bags": [
                {
                    "BagPath": left_bag,
                    "TopicsToMerge": [
                        {"key": "/recording/events", "value": "/davis_left/events"},
                        {"key": "/recording/imu", "value": "/davis_left/imu"},
                        {"key": "/recording/image", "value": "/davis_left/image"},
                    ],
                },
                {
                    "BagPath": right_bag,
                    "TopicsToMerge": [
                        {"key": "/recording/events", "value": "/davis_right/events"},
                        {"key": "/recording/imu", "value": "/davis_right/imu"},
                        {"key": "/recording/image", "value": "/davis_right/image"},
                    ],
                },
            ],
            "BagBeginTime": -1,
            "BagEndTime": -1,
            "OutputBagPath": output_bag_path,
        }
    }

    with open(save_path, "w") as f:
        yaml.dump(config, f, sort_keys=False)

def convert_aedat4(aedat4_files: Path):
    output_bag_files = []
    for file in aedat4_files:
        print(f"Processing {file}...")
        # check roscore is running
        if not is_roscore_running():
            raise RuntimeError("roscore is not running.")
        output = file.with_suffix(".bag")
        if output.exists():
            print(f"Warning: {output} already exists. Skipping conversion.")
            output_bag_files.append(output)
            continue
        cmd = f"rosrun dv_ros_aedat4 convert_aedat4 --input {file}"
        print(f"Running command: {cmd}")
        subprocess.run(cmd, shell=True)
        if not output.exists():
            raise RuntimeError(f"Conversion failed for {file}.")
        else:
            output_bag_files.append(output)
    return output_bag_files

def classify_left_right_bag(bag_files):
    left_bag = None
    right_bag = None
    for bag in bag_files:
        if "left" in bag.stem:
            left_bag = bag
        elif "right" in bag.stem:
            right_bag = bag
    if left_bag is None or right_bag is None:
        raise ValueError("Both left and right bags must be provided.")
    return left_bag, right_bag

def merge_bags(merge_yaml_file_path: str):
    if not is_roscore_running():
        raise RuntimeError("roscore is not running.")
    
    cmd = f"roslaunch ikalibr ikalibr-bag-merge.launch config_path:={merge_yaml_file_path}"
    print(f"Running command: {cmd}")
    subprocess.run(cmd, shell=True)

def move_merged_bag_to_folder(output_bag_path):
    # create a folder named same as the output bag (without extension)
    folder = os.path.splitext(output_bag_path)[0]
    if not os.path.exists(folder):
        os.makedirs(folder)
    new_location = os.path.join(folder, os.path.basename(output_bag_path))
    os.rename(output_bag_path, new_location)
    print(f"Moved merged bag to: {new_location}")

if __name__ == "__main__":
    # find .aedat4 file in current path
    current_path = Path().resolve()
    aedat4_files = list(current_path.glob("*.aedat4"))
    print(
        f"Found {len(aedat4_files)} .aedat4 files in {current_path}:\n"
        + "\n".join(str(file) for file in aedat4_files)
    )
    if len(aedat4_files) != 2:
        raise ValueError("Exactly two .aedat4 files are required for merging.")

    bag_files = convert_aedat4(aedat4_files)
    left_bag, right_bag = classify_left_right_bag(bag_files)
    print(f"Left bag: {left_bag}\nRight bag: {right_bag}")

    # get such a bag name: eKalibr-data-$(date +%Y-%m-%d-%H-%M-%S).bag
    output_bag_name = Path(f"eKalibr-data-{datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S')}.bag")
    output_bag_path = current_path / output_bag_name
    merge_yaml_file_path = current_path / "config-bag-merge.yaml"
    create_merge_config(str(left_bag), str(right_bag), str(output_bag_path), str(merge_yaml_file_path))

    merge_bags(str(merge_yaml_file_path))
    if not output_bag_path.exists():
        raise RuntimeError(f"Merge failed. Output bag not found: {output_bag_path}")

    # delete aedat4 files
    for file in aedat4_files:
        if file.exists():
            file.unlink()
            print(f"Deleted: {file}")
    # delete bags
    for bag in bag_files:
        if bag.exists():
            bag.unlink()
            print(f"Deleted: {bag}")

    # delete merge config
    if merge_yaml_file_path.exists():
        merge_yaml_file_path.unlink()
        print(f"Deleted: {merge_yaml_file_path}")
