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

import sys
import os
import cv2
import glob
import numpy as np


def save_images_as_video(images, output_path, fps):
    if not images:
        print("No images to save as video.")
        return

    # Ensure all images have the same size
    height, width, layers = images[0].shape
    for img in images:
        if img.shape != (height, width, layers):
            print("Error: All images must have the same dimensions.")
            return

    # Define the codec and create VideoWriter object (use XVID for high quality)
    fourcc = cv2.VideoWriter_fourcc(*'XVID')  # XVID codec (good quality)
    out = cv2.VideoWriter(output_path, fourcc, fps, (width, height))

    # Write images to video
    for img in images:
        out.write(img)

    out.release()
    print(f"Video saved to {output_path}")


def make_video(folder, fps):
    # find all png files in the folder (only single layer)
    png_files = sorted(glob.glob(os.path.join(folder, '*.png')))
    # filename format: 'tv-render-0.png', sort according to the number
    png_files.sort(key=lambda x: int(os.path.basename(x).split('-')[-1].split('.')[0]))
    # number: opencv image
    images = {}
    for png_file in png_files:
        number = int(os.path.basename(png_file).split('-')[-1].split('.')[0])
        images[number] = cv2.imread(png_file)
    first_number = min(images.keys())
    last_number = max(images.keys())
    img_list = []
    for i in range(0, last_number + 1):
        if i not in images:
            # create a black image with the same size and same type as the first image using opencv
            height, width, channels = images[first_number].shape
            images[i] = np.zeros((height, width, channels), dtype=np.uint8)
        img_list.append(images[i])
    path = os.path.join(folder, 'video.avi')
    save_images_as_video(img_list, path, fps)
    return path


def generate_new_path(original_path, folder):
    file_name = os.path.basename(original_path)
    parent_folder = os.path.dirname(original_path)
    relative_path = os.path.relpath(parent_folder, folder)
    new_file_name = relative_path.replace('/', '_') + '_' + file_name
    new_path = os.path.join(folder, 'video_maker', new_file_name)
    return new_path


if __name__ == "__main__":
    folder = '/media/csl/samsung/eKalibr/dataset/multi-camera'
    # recursively find all subfolders in folders that contain *.png files, return the folder list (absolute path)
    subfolders = []
    for root, dirs, files in os.walk(folder):
        if any(file.endswith('.png') for file in files):
            subfolders.append(os.path.abspath(root))
    for subfolder in subfolders:
        print(f"Processing folder: {subfolder}")
        path = make_video(subfolder, fps=50)  # DecayTimeOfActiveEvents = 0.02, 1.0 / 0.02 = 50
        new_path = generate_new_path(path, folder)
        # move the video to the new path using os.move
        os.makedirs(os.path.dirname(new_path), exist_ok=True)
        os.rename(path, new_path)
