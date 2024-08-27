import os
import shutil
import json
import re

def extract_number(filename):
    # Extracts the first sequence of digits found in the filename
    match = re.search(r'\d+', filename)
    return int(match.group()) if match else float('inf')  # If no number is found, return infinity

def copy_and_rename_files(dataset_dir, directories, destination_folder, suffix, prefix, mapping_file):
    # Ensure the destination folder exists
    if not os.path.exists(os.path.join(dataset_dir, destination_folder)):
        os.makedirs(os.path.join(dataset_dir, destination_folder))

    file_index = 1
    file_mapping = {}

    for directory in directories:

        files = [f for f in os.listdir(os.path.join(dataset_dir, directory)) if f.endswith(suffix)]
        # Sort files by the numerical value in their filenames
        files.sort(key=extract_number)

        for file in files:
            # Construct the new filename
            new_filename = f"{prefix}_{file_index:05d}.{suffix}"
            # Full path of the file
            source_file = os.path.join(dataset_dir, directory, file)
            destination_file = os.path.join(dataset_dir, destination_folder, new_filename)
            # Copy and rename the file
            shutil.copy(source_file, destination_file)
            # Save the mapping
            file_mapping[new_filename] = os.path.join(directory, file)
            # Increment the file index
            file_index += 1

    # Save the mapping to a JSON file
    with open(mapping_file, 'w') as f:
        json.dump(file_mapping, f, indent=4)


def load_mapping(mapping_file):
    # Load the mapping from a JSON file
    with open(mapping_file, 'r') as f:
        return json.load(f)

def organize_files(dataset_dir, directories, destination_folder, mapping_file):
    suffix = 'png'  # Change this to the suffix of your files
    prefix = 'frame'  # Change this to the desired prefix

    # Example usage
    dataset_dir = '../nerf_data/parking_train_eval'
    directories = ['images', 'images_eval']
    destination_folder = 'joined_images'
    suffix = 'png'  # Change this to the suffix of your files
    prefix = 'frame'  # Change this to the desired prefix
    mapping_file = os.path.join(dataset_dir, 'filename_mapping.json')

    copy_and_rename_files(dataset_dir, directories, destination_folder, suffix, prefix, mapping_file)

    # Example of loading the mapping later
    mapping = load_mapping(mapping_file)
    print(mapping)


def load_json_file(file_path):
    with open(file_path, 'r') as f:
        return json.load(f)

def save_json_file(data, file_path):
    with open(file_path, 'w') as f:
        json.dump(data, f, indent=4)

def process_frames(data, mapping_file, eval_string, keep=False):
    updated_frames = []
    mapping = load_mapping(mapping_file)

    for frame in data["frames"]:
        # Extract the filename from the file_path
        file_name = os.path.basename(frame["file_path"])

        # Lookup the corresponding value in the filename_mapping
        if file_name in mapping:
            mapped_value = mapping[file_name]

            # Keep the frame only if the mapped value does not start with "images_eval"
            if keep:
                if mapped_value.startswith(eval_string):
                    updated_frames.append(frame)
            else:
                if not mapped_value.startswith(eval_string):
                    updated_frames.append(frame)

    # Update the data with the filtered frames
    data["frames"] = updated_frames



def remove_eval_frames(dataset_dir, directories, results_folder, mapping_file):

    transforms_path = os.path.join(dataset_dir, results_folder, 'transforms.json')
    backup_transforms_path = os.path.join(dataset_dir, results_folder, 'backup_transforms.json')
    renamed_transforms_path = os.path.join(dataset_dir, results_folder, 'joined_transforms.json')
    eval_transforms_path = os.path.join(dataset_dir, results_folder, 'eval_transforms.json')

    data = load_json_file(backup_transforms_path)

    save_json_file(data, renamed_transforms_path) # save transforms unchanged under new name

    # remove all frames where the mapped file name includes the eval_string
    process_frames(data=data, mapping_file=mapping_file, eval_string=directories[1], keep=False)

    save_json_file(data, transforms_path)

    # load original once more
    data = load_json_file(renamed_transforms_path)

    # remove all frames where the mapped file name does NOT include the eval_string
    process_frames(data=data, mapping_file=mapping_file, eval_string=directories[1], keep=True)

    save_json_file(data, eval_transforms_path)




def main():

    dataset_dir = '../nerf_data/parking_train_eval'
    directories = ['images', 'images_eval']
    destination_folder = 'joined_images'
    results_folder = 'results'

    mapping_file = os.path.join(dataset_dir, 'filename_mapping.json')

    # This is the flow for generating a NeRF and handling the COLMAP pose
    # information for both the training and eval frames

    # run this to join training and eval files for the COLMAP mapping process
    #organize_files(dataset_dir, directories, destination_folder, mapping_file)

    # run colmap here:
    # ns-process-data images --data joined_images --output-dir results

    # weed out the eval images from the transforms.json file
    remove_eval_frames(dataset_dir, directories, results_folder, mapping_file)


    # run training here:



main()