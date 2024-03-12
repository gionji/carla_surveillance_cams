#!/usr/bin/env python

# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Script that render multiple sensors in the same pygame window

By default, it renders four cameras, one LiDAR and one Semantic LiDAR.
It can easily be configure for any different number of sensors. 
To do that, check lines 290-308.
"""

import datetime
import glob
import os
import sys
import json
from src import mqtt, rest, objects
from hemligt import *

from flask import Flask, Response


try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import argparse
import random
import time
import numpy as np
import cv2
import struct

try:
    import pygame
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_q
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')


def draw_debug_annotations(world, sensor_transform, lifetime=1):
    world.debug.draw_arrow(sensor_transform.location, 
                            sensor_transform.location + sensor_transform.get_forward_vector(), 
                            life_time=lifetime, 
                            color=carla.Color(r=255, g=0, b=0), 
                            arrow_size=0.5, 
                            persistent_lines=True)
            
    # Print camera position in debug mode
    camera_position_str = f"Camera position: {sensor_transform.location.x}, {sensor_transform.location.y}, {sensor_transform.location.z}"
    
    world.debug.draw_string(sensor_transform.location + carla.Location(z=2.0), 
                            camera_position_str, 
                            draw_shadow=True, 
                            color=carla.Color(r=255, g=255, b=255), 
                            life_time=lifetime)


def draw_debug_grid(world, grid_size=2, distance=20, lifetime=30):
    origin = carla.Location(0, 0, 0)  # The origin point (0, 0, 0)
    for i in range(-grid_size, grid_size + 1):
        for j in range(-grid_size, grid_size + 1):
            for k in range(0, grid_size + 1):
                # Calculate the location of each point in the grid
                location = origin + carla.Location(x=i*distance, y=j*distance, z=k*distance)
                # Create a string with the location coordinates
                location_str = f"Location: {location.x}, {location.y}, {location.z}"
                # Draw the location string above the point
                world.debug.draw_string(location + carla.Location(z=0.5), location_str, draw_shadow=False, color=carla.Color(0, 255, 0), life_time=lifetime)


def generate_camera_grid(sensor_data, grid_size, screen_size):

    cols = len(sensor_data)
    rows = len(sensor_data[0])
    color = (0, 0, 255)

    cell_width = screen_size[0]
    cell_height = screen_size[1] // rows

    grid_stack = []
    rgb_stack = []
    for i, (name, image) in enumerate(sensor_data[0].items()):
        resized_image = cv2.resize(image[:, :, :3], (cell_height, cell_width))
        cv2.putText(resized_image, name, (0, 20), cv2.FONT_HERSHEY_SIMPLEX, 1,
                    color, 2, cv2.LINE_AA, True)
        rgb_stack.append(resized_image)
    grid_stack.append(np.hstack(rgb_stack))

    depth_stack = []
    if cols > 1:
        for i, (name, image) in enumerate(sensor_data[1].items()):
            resized_image = cv2.resize(image[:, :, :3], (cell_height, cell_width))
            cv2.putText(resized_image, name, (0, 20), cv2.FONT_HERSHEY_SIMPLEX, 1,
                        color, 2, cv2.LINE_AA, True)
            depth_stack.append(resized_image)
        grid_stack.append(np.hstack(depth_stack))

    segment_stack = []
    if cols > 2:
        for i, (name, image) in enumerate(sensor_data[2].items()):
            resized_image = cv2.resize(image[:, :, :3], (cell_height, cell_width))
            cv2.putText(resized_image, name, (0,20), cv2.FONT_HERSHEY_SIMPLEX, 1,
                        color, 2, cv2.LINE_AA, True)
            segment_stack.append(resized_image)
        grid_stack.append(np.hstack(segment_stack))

    # Calculate the dimensions of each grid cell
    cell_width = screen_size[1] // grid_size[0]
    cell_height = screen_size[0] // grid_size[1]

    # Create a blank canvas to hold the grid view
    #grid_view = np.zeros((screen_size[0], screen_size[1], 3), dtype=np.uint8)
    grid_view = np.vstack(grid_stack)

    # Iterate over each camera image in the sensor_data dictionary


    # for i, (name, image) in enumerate(sensor_data.items()):
    #     # Calculate the row and column indices for this camera image in the grid
    #     row = i // grid_size[0]
    #     col = i % grid_size[0]
    #
    #     # Convert RGBA image to RGB and resize it to fit in the grid cell
    #     resized_image = cv2.resize(image[:, :, :3], (cell_width, cell_height))
    #
    #     # Calculate the coordinates to place the resized image in the grid
    #     start_x = col * cell_width
    #     start_y = row * cell_height
    #
    #     # Insert the resized image into the grid view canvas
    #     grid_view[start_y:start_y + cell_height, start_x:start_x + cell_width] = resized_image

    return grid_view


def rgb_callback(image, data_dict, camera_name):

    img = np.array(image.raw_data)
    img = img.reshape((image.height, image.width, 4))  # Assuming RGBA format
    img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)  # Convert from RGBA to RGB
    img = np.rot90(img, -1)  # Rotate the image 90 degrees clockwise
    img = np.fliplr(img)
    data_dict[camera_name] = img

def depth_callback(image, data_dict, camera_name):
    image.convert(carla.ColorConverter.LogarithmicDepth)
    img = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))
    img = np.rot90(img, -1)  # Rotate the image 90 degrees clockwise
    img = np.fliplr(img)
    data_dict[camera_name] = img

def inst_callback(image, data_dict, camera_name):
    img = np.array(image.raw_data).reshape((image.height, image.width, 4))[:, :, :3]
    img = cv2.cvtColor(img, cv2.COLOR_RGBA2RGB)  # Convert from RGBA to RGB
    img = np.rot90(img, -1)  # Rotate the image 90 degrees clockwise
    img = np.fliplr(img)
    data_dict[camera_name] = img



##
# CARLA: doc page: https://carla.readthedocs.io/en/latest/ref_sensors/#optical-flow-camera
# raw_data 	bytes 	Array of BGRA 64-bit pixels containing two float values.
#
def optiflow_callback(optical_flow_image, data_dict, camera_name):
    # Get the raw data from the OpticalFlowImage object
    raw_data = optical_flow_image.raw_data

    # Assuming each float value is 32 bits (4 bytes) and BGRA order
    pixel_size = 8  # 64 bits (8 bytes) for each pixel
    num_pixels = len(raw_data) // pixel_size

    # Interpret raw data as floats
    float_data = struct.unpack(f"<{num_pixels * 2}f", raw_data)

    # Reshape the float data into a 2D array with two channels
    float_array = np.array(float_data).reshape((optical_flow_image.height, optical_flow_image.width, 2))

    # Convert the optical flow vectors to colors using HSV color space
    hsv = np.zeros((optical_flow_image.height, optical_flow_image.width, 3), dtype=np.uint8)
    hsv[..., 1] = 255

    # Calculate magnitude and angle of the optical flow vectors
    mag, ang = cv2.cartToPolar(float_array[..., 0], float_array[..., 1])

    # Set hue according to the angle of the optical flow vectors
    hsv[..., 0] = ang * 180 / np.pi / 2

    # Normalize magnitude for visualization
    hsv[..., 2] = cv2.normalize(mag, None, 0, 255, cv2.NORM_MINMAX)

    # Convert HSV to RGB
    rgb_flow = cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)

    # Convert from RGBA to RGB
    rgb_flow = np.rot90(rgb_flow, -1)  # Rotate the image 90 degrees clockwise
    rgb_flow = np.fliplr(rgb_flow)

    data_dict[camera_name] = rgb_flow


def spawn_camera(world, blueprint_library, reference_actor_bp, transform, sensor_type, fov_str, sensor_data, objects_list, callback, name):
    # Spawn anchor object
    anchor_object = world.spawn_actor(
        reference_actor_bp,
        transform)
    objects_list.append(anchor_object)
    
    # Draw annotation
    draw_debug_annotations(world, transform, lifetime=20)
            # Draw FOV borders for the camera
    draw_fov_borders(transform, 90, 90, 20, world)


    # Set up camera blueprint
    camera_bp = blueprint_library.find(sensor_type)
    camera_bp.set_attribute('fov', fov_str)
    
    # Spawn camera attached to anchor object
    camera = world.spawn_actor(
        camera_bp,
        carla.Transform( carla.Location(x=2, y=0.0, z=0)),
        attach_to=anchor_object)
    objects_list.append(camera)
    
    # Attach listener to the camera
    camera.listen(lambda image: callback(image, sensor_data, name))

    return anchor_object



def draw_fov_borders(camera_transform, fov_horizontal, fov_vertical, distance, world, lifetime=15):
    # Convert FOV angles to radians
    fov_horizontal_radians = np.deg2rad(fov_horizontal)
    fov_vertical_radians = np.deg2rad(fov_vertical)

    # Calculate half horizontal and vertical FOV angles
    half_fov_horizontal_radians = fov_horizontal_radians / 2.0
    half_fov_vertical_radians = fov_vertical_radians / 2.0

    # Calculate direction vectors for horizontal and vertical FOV borders
    horizontal_vector_left = carla.Vector3D(x=distance * np.cos(camera_transform.rotation.yaw + half_fov_horizontal_radians),
                                             y=distance * np.sin(camera_transform.rotation.yaw + half_fov_horizontal_radians),
                                             z=distance * np.sin(camera_transform.rotation.pitch + half_fov_vertical_radians))
    horizontal_vector_right = carla.Vector3D(x=distance * np.cos(camera_transform.rotation.yaw - half_fov_horizontal_radians),
                                              y=distance * np.sin(camera_transform.rotation.yaw - half_fov_horizontal_radians),
                                              z=distance * np.sin(camera_transform.rotation.pitch + half_fov_vertical_radians))
    '''
    vertical_vector_top = carla.Vector3D(x=distance * np.cos(camera_transform.rotation.yaw),
                                         y=distance * np.sin(camera_transform.rotation.yaw),
                                         z=distance * np.sin(camera_transform.rotation.pitch + half_fov_vertical_radians))
    vertical_vector_bottom = carla.Vector3D(x=distance * np.cos(camera_transform.rotation.yaw),
                                            y=distance * np.sin(camera_transform.rotation.yaw),
                                            z=distance * np.sin(camera_transform.rotation.pitch - half_fov_vertical_radians))
    '''

    vertical_vector_top = carla.Vector3D(x=distance * np.cos(camera_transform.rotation.yaw + half_fov_horizontal_radians),
                                             y=distance * np.sin(camera_transform.rotation.yaw + half_fov_horizontal_radians),
                                             z=-(distance * np.sin(camera_transform.rotation.pitch + half_fov_vertical_radians)))

    vertical_vector_bottom = carla.Vector3D(x=-distance * np.cos(camera_transform.rotation.yaw - half_fov_horizontal_radians),
                                              y=distance * np.sin(camera_transform.rotation.yaw - half_fov_horizontal_radians),
                                              z=-(distance * np.sin(camera_transform.rotation.pitch + half_fov_vertical_radians)))
    

    # Get camera location
    camera_location = camera_transform.location

    # Calculate endpoints of horizontal and vertical FOV borders
    horizontal_left_endpoint  = camera_location + horizontal_vector_left
    horizontal_right_endpoint = camera_location + horizontal_vector_right
    vertical_top_endpoint     = camera_location + vertical_vector_top
    vertical_bottom_endpoint  = camera_location + vertical_vector_bottom

    # Draw debug lines for horizontal and vertical FOV borders
    world.debug.draw_line(camera_location, horizontal_left_endpoint, thickness=0.05, color=carla.Color(255, 0, 0), life_time=lifetime)
    world.debug.draw_line(camera_location, horizontal_right_endpoint, thickness=0.05, color=carla.Color(255, 0, 0), life_time=lifetime)
    world.debug.draw_line(camera_location, vertical_top_endpoint, thickness=0.05, color=carla.Color(255, 0, 0), life_time=lifetime)
    world.debug.draw_line(camera_location, vertical_bottom_endpoint, thickness=0.05, color=carla.Color(255, 255, 0), life_time=lifetime)


def center_crop(image, w, h):
    center = image.shape
    x = center[1] / 2 - w / 2
    y = center[0] / 2 - h / 2

    crop_img = image[int(y):int(y + h), int(x):int(x + w)]
    return crop_img

def get_crop(image, obj, w, h):
    if len(obj['cropouts']) > 0:
        (x1, y1), (x2, y2) = obj['cropouts'][0]

        crop_img = image[int(y1):int(y2), int(x1):int(x2),:]
    else:
        return None
    return crop_img


def get_objects(image):
    x = 100 + int(random.uniform(-40, 40))
    y = 100 + int(random.uniform(-40, 40))
    w, h = 400, 400
    return [(x, y, w, h)]


import random

def randomize_weather(world):
    """
    Randomize the weather in the Carla world.
    """
    # Define the range of values for each weather parameter
    cloudiness = random.choice([0.0, 25.0, 100.0])
    precipitation = random.uniform(0.0, 100.0)
    sun_altitude_angle = random.uniform(-50.0, 90.0)
    sun_azimuth_angle = random.uniform(0.0, 360.0)
    precipitation_deposits = random.uniform(0.0, 100.0)
    wind_intensity = random.uniform(0.0, 100.0)
    fog_density = random.uniform(0.0, 10.0)
    wetness = random.uniform(0.0, 100.0)

    # Create a WeatherParameters object with the randomized values
    weather = carla.WeatherParameters(
        cloudiness=cloudiness,
        precipitation=precipitation,
        sun_altitude_angle=sun_altitude_angle,
        sun_azimuth_angle=sun_azimuth_angle,
        precipitation_deposits=precipitation_deposits,
        wind_intensity=wind_intensity,
        fog_density=fog_density,
        wetness=wetness
    )

    # Set the randomized weather in the Carla world
    world.set_weather(weather)

def random_crop_with_bbox(xmin, ymin, xmax, ymax, img_width, img_height, cropx, cropy):
    # Ensure the crop size is not smaller than the bounding box
    if cropx < (xmax - xmin) or cropy < (ymax - ymin):
        raise ValueError("Crop size must be larger than bounding box size")


    #print(f'xmin = {xmin}, xmax={xmax}, ymin = {ymin}, ymax={ymax}')

    # Calculate the range within which the crop can start
    # to ensure the bounding box is within the cropped area
    min_x_start = xmin
    max_x_start = xmax - cropx
    min_y_start = ymin
    max_y_start = ymax - cropy


    # Adjust to ensure the crop does not fall outside the image boundaries
    max_x_start = max(0, min(max_x_start, img_width - cropx))
    min_x_start = min(min_x_start, img_width - cropx)

    max_y_start = max(0, min(max_y_start, img_height - cropy))
    min_y_start = min(min_y_start, img_height - cropy)

    # Choose a random start position for the crop within the valid range
    crop_start_x = random.randint(min_x_start, max_x_start)
    crop_start_y = random.randint(min_y_start, max_y_start)

    # Calculate new bounding box positions relative to the crop
    new_xmin = xmin - crop_start_x
    new_ymin = ymin - crop_start_y
    new_xmax = xmax - crop_start_x
    new_ymax = ymax - crop_start_y

    return (crop_start_x, crop_start_y, new_xmin, new_ymin, new_xmax, new_ymax)

def extend_crop(xmin, ymin, xmax, ymax, img_width, img_height, crop_w, crop_h, xdev=0, ydev=0):

    cropx, cropy = crop_w, crop_h  # Desired crop dimensions

    #crop_info = random_crop_with_bbox(xmin, ymin, xmax, ymax, img_width, img_height, cropx, cropy)
    #print("Crop Start (X, Y):", crop_info[:2])
    #print("New Bounding Box (xmin, ymin, xmax, ymax):", crop_info[2:])

    xc = int((xmax + xmin) / 2 + random.uniform(-xdev, xdev))
    yc = int((ymax + ymin) / 2 + random.uniform(-ydev, ydev))



    x1 = int(xc - crop_w/2) # np.max(0, xc - width/2)
    x2 = int(xc + crop_w / 2)
    y1 = int(xc - crop_h / 2)
    y2 = int(xc - crop_h / 2)

    if x1 < 0:
        x1, x2 = 0, crop_w
    if x2 >= width:
        x1, x2 = width - crop_w - 1, width - 1
    if y1 < 0:
        y1, y2 = 0, crop_h
    if y2 >= height:
        y1, y2 = height - crop_h - 1, height - 1

    #print(f'xmin = {x1}, xmax={x2}, ymin = {y1}, ymax={y2}')

    return np.array([[x1, y1], [x2, y2]])
    #return crop_info[2:]

def find_bounding_boxes(instance_map, classes, out_classes, threshold=100, occlusion_ratio=0.2, depth_diff_th=0.1, crop_w=400, crop_h=400):
    r_ch = 2
    g_ch = 1
    b_ch = 0
    bounding_boxes = []
    cropouts = []
    vehicles = []
    return_val = {}
    #print('classids', class_ids)

    # Combine G and B channels to get the instance ID
    combined_instance_ids = (instance_map[:,:,g_ch].astype(np.uint32) << 8) + instance_map[:,:,b_ch]

    img_width, img_height = instance_map.shape[0:2]
    #print(f'width = {img_width}, height: = {img_height}')

    # Iterate over specified class IDs
    for class_name, class_id in classes.items():
        # Skip background class (class ID 0)
        if class_id == 0:
            continue
        #print(f"looking for {class_name} objects (class id {class_id}")
        # Find instances for the current class ID
        instances_for_class = np.unique(combined_instance_ids[instance_map[:,:,r_ch] == class_id])

        # Iterate over unique instance IDs for the current class
        for instance_id in instances_for_class:
            # Skip background instance (instance ID 0)
            if instance_id == 0:
                continue

            # Extract coordinates where the current instance ID is present
            coords = np.argwhere(combined_instance_ids == instance_id)
            num_pixels = np.count_nonzero(coords)


            # Calculate bounding box coordinates
            ymin, xmin = np.min(coords, axis=0)
            ymax, xmax = np.max(coords, axis=0)

            # calc bbox area in pixels
            box_area = (xmax - xmin) * (ymax - ymin)

            # Crop the corresponding bbox in the depth image
            #depth_bbox = depth_map[ymin:ymax, xmin:xmax]

            # Calculate the object mask of the object in the corresponding instance bbox
            object_mask = (combined_instance_ids[ymin:ymax, xmin:xmax] == instance_id).astype(np.uint8)

            # Calculate the average distance of the masked pixels
            #distance_masked = np.mean(depth_bbox[object_mask > 0])

            # Calculate the average distance of the non-masked pixels
            #distance_non_masked = np.mean(depth_bbox[object_mask == 0])

            # difference between the front pixels and the background
            #depth_diff = abs(distance_masked - distance_non_masked)

            if num_pixels >= threshold:
                occluded_pixel_ratio = float(num_pixels / box_area)

                if  occluded_pixel_ratio > 1:
                    pass
                    #print(f"DROPPED Class: {class_id} ({class_name}), Instance: {instance_id} (g: {instance_id >> 8}, b: {instance_id & 0xFF}) @ ({xmin}, {ymin}) found in {num_pixels} pixels, box area: {box_area}, RATIO: {occluded_pixel_ratio}")

                elif occluded_pixel_ratio >= occlusion_ratio:

                    if True:#depth_diff > depth_diff_th:
                        #print(f"Depth: object = {distance_masked}, background = {distance_non_masked}")
                        #print(f"Class: {class_id} ({class_name}), Instance: {instance_id} (g: {instance_id >> 8}, b: {instance_id & 0xFF}) @ ({xmin}, {ymin}) found in {num_pixels} pixels, box area: {box_area}, ratio: {occluded_pixel_ratio}")
                        class_id_remapped = out_classes[class_name]
                        bounding_boxes.append(np.array([[xmin, ymin], [xmax, ymax]]))
                        cropouts.append(extend_crop(xmin, ymin, xmax, ymax, img_width, img_height, crop_w, crop_h, xdev=50, ydev=50))
                        vehicles.append(class_id_remapped)
                    #else:
                    #    print(f"DROPPED Class: {class_id} ({class_name}) due to depth: object = {distance_masked}, background = {distance_non_masked}")
                else:
                    pass
                    #print(f"DROPPED Class: {class_id} ({class_name}), Instance: {instance_id} (g: {instance_id >> 8}, b: {instance_id & 0xFF}) @ ({xmin}, {ymin}) found in {num_pixels} pixels, box area: {box_area}, ratio: {occluded_pixel_ratio}")
            else:
                pass
                #print(f"DROPPED Class: {class_id} ({class_name}), Instance: {instance_id} (g: {instance_id>>8}, b: {instance_id & 0xFF}) @ ({xmin}, {ymin}) found in {num_pixels} pixels, box area: {box_area}")



            # Draw bounding box on the original image
            #img = cv2.rectangle(instance_map, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)

    return_val['bbox'] = bounding_boxes
    return_val['class'] = vehicles
    return_val['vehicle'] = vehicles
    return_val['cropouts'] = cropouts

    return return_val

def generate_cropouts(crop_data, rgb_data, segmentation_data, w, h):

    # here we are cheating by using segmentation information to find the drones
    objs = {}
    for i, (name, image) in enumerate(segmentation_data.items()):
        rgb_name = name.replace('inst_image', 'rgb_image')
        # the objects dict will hold for each image in rgb_data dict a list of box coords for the drones
        #objs[rgb_name] = get_objects(image)
        objs[rgb_name] = find_bounding_boxes(image, objects.available_classes, objects.out_classes)


    for i, (name, image) in enumerate(rgb_data.items()):


        # We assume here that only 1 cropout occurs in each image
        cropped = get_crop(image, objs[name], w, h)
        crop_data[name] = cropped



def stream_sensor_video(rgb_data):
    pass


def publish_capture(name, date_time):
    capture_struct = {
        "name": name,
        "time": date_time
    }

    #mqtt.publish_capture_result(client, capture_struct, name)
    #print("Published detection result.")


def upload_crop(image, name, date_time, id):
    #send to REST API here
    response = rest.upload_img(image, name, date_time, id)




def signal_img_captured(mq, date_time, rgb_crops, id):


    for i, (name, image) in enumerate(rgb_crops.items()):
        #print(f'Captured object at time {date_time} from sensor: {name} of size {image.shape}')
        # publish mqtt command
        #print(image)
        if image is not None:
            mq.publish_capture_result(name, date_time, id)
        #upload_crop(image, name, date_time, id)

    # send image to REST interface
    pass


def main(mq, rgb_data, crop_data, sensor_transforms, width, height, fov_degrees, crop_w, crop_h):
    objects_list = []

    try:
        # First of all, we need to create the client that will send the requests
        # to the simulator. Here we'll assume the simulator is accepting
        # requests in the localhost at port 2000.
        client = carla.Client('localhost', 2000)
        client.set_timeout(5.0)

        # Once we have a client we can retrieve the world that is currently
        # running.
        map_index = 3
        available_maps = ["Town01", "Town02", "Town03", "Town04", "Town05", "Town06", "Town07", "Town10HD"]
        world = client.get_world()

        map_name = available_maps[map_index]
        print(f"Loading map {map_index}: {map_name}")
        client.load_world(map_name)

        # The world contains the list blueprints that we can use for adding new
        # actors into the simulation.
        blueprint_library = world.get_blueprint_library()

        # Set FOV for all cameras
        fov_str = str(fov_degrees)

        sensor_type_names = [
            blueprint_library.find('sensor.camera.rgb'),
            blueprint_library.find('sensor.camera.depth'),
            blueprint_library.find('sensor.camera.instance_segmentation')
        ]

        sensor_transforms = [
            carla.Transform( carla.Location(x=-20, y=0.0, z=20.4), carla.Rotation(yaw=45.0,  pitch=0.0, roll=0.0) ),
            carla.Transform( carla.Location(x=-20, y=20,  z=20.4), carla.Rotation(yaw=0.0,   pitch=0.0, roll=0.0))  ,
            carla.Transform( carla.Location(x=0,   y=20,  z=20.4), carla.Rotation(yaw=-45.0, pitch=0.0, roll=0.0))  
        ]
        depth = True
        segment = True
        opt = True

        # Update the sensor_data dictionary to include the different cameras sensors
        if depth:
            depth_data = {f'depth_image_{i+1:02}': np.zeros((height, width, 4)) for i in range(len(sensor_transforms))}
        if segment:
            inst_data = {f'inst_image_{i+1:02}': np.zeros((height, width, 4)) for i in range(len(sensor_transforms))}
        if opt:
            opt_data = {f'opt_image_{i+1:02}': np.zeros((height, width, 4)) for i in range(len(sensor_transforms))}



        # Move the spectator close to the spawned vehicle
        spectator = world.get_spectator()
        spectator.set_transform( carla.Transform(carla.Location(x=5, y=10.0, z=67), carla.Rotation(pitch=-90, yaw=-90)) )

        # Props: catalogue
        # https://carla.readthedocs.io/en/latest/catalogue_props/
        rererence_actor_bp = random.choice(blueprint_library.filter('static.prop.glasscontainer'))

        # Draw a debug 3d grid for debugging
        draw_debug_grid(world)

        #
        ### Sensor spawning
        #
        rgb_ref_obj = []
        depth_ref_obj = []
        seg_ref_obj = []
        opt_ref_obj = []

        for i, transf in enumerate(sensor_transforms):
            rgb_ref_obj.append((world, blueprint_library, rererence_actor_bp, transf, 'sensor.camera.rgb', fov_str, rgb_data, objects_list, rgb_callback, f'rgb_image_{i+1:02}'))
            if depth:
                depth_ref_obj.append(spawn_camera(world, blueprint_library, rererence_actor_bp, transf, 'sensor.camera.depth', fov_str, depth_data, objects_list, depth_callback, f'depth_image_{i+1:02}'))
            if segment:
                seg_ref_obj.append(spawn_camera(world, blueprint_library, rererence_actor_bp, transf, 'sensor.camera.instance_segmentation', fov_str, inst_data, objects_list, inst_callback, f'inst_image_{i+1:02}'))
            if opt:
                opt_ref_obj.append(spawn_camera(world, blueprint_library, rererence_actor_bp, transf, 'sensor.camera.optical_flow', fov_str, inst_data, objects_list, inst_callback, f'opt_image_{i+1:02}'))
        pygame.init()

        done = False
        showDebug = True
        showCameras = False

        size = (640, 1080) #(1920, 1080)
        if showCameras:
            pygame.display.set_caption("VISER Sim")
            screen = pygame.display.set_mode(size)
        image_id = 0

        #control = carla.VehicleControl()
        clock = pygame.time.Clock()


        while not done:
            keys = pygame.key.get_pressed() 
            
            # Made the keyboard control into a function
            #keyboard_control(keys)
            
            current_time = datetime.datetime.now()
            date_time = current_time.strftime("%Y-%m-%d %H:%M:%S.%f")


            # Let's now save the images as well
            #rgb_image_creator(sensor_data['rgb_image'],datetime)
            #depth_image_creator(sensor_data['depth_image'],datetime)


            #stream_sensor_video(rgb_data)

            #if mq.activeCameras:
            generate_cropouts(crop_data, rgb_data, inst_data, w=crop_w, h=crop_h)
            #print(crop_data['rgb_image_01'][0:5, 1, :])
            image_id += 1

            signal_img_captured(mq, date_time, crop_data, image_id)
            #print('after')


            # tick the simulation
            world.tick()



            if showCameras:
                # Generate camera grid view
                num_sensor_types = 1
                sensor_data = [rgb_data]
                if depth:
                    num_sensor_types +=1
                    sensor_data.append(depth_data)
                if segment:
                    num_sensor_types += 1
                    sensor_data.append(inst_data)

                grid_view = generate_camera_grid(sensor_data, (len(sensor_transforms), num_sensor_types), size)  # Pass screen size

                # Update the display and check for the quit event
                #size = (grid_view.shape[0], grid_view.shape[1])
                #screen = pygame.display.set_mode(size)
                pygame.display.flip()
                pygame.display.update()
                screen.blit(pygame.surfarray.make_surface(grid_view), (0, 0)) # To display with Pygame

            # Better to use OpenCV to display both RGB and Depth Image

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    done = True
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_t:
                        spectator_transform = spectator.get_transform()
                    elif event.key in [pygame.K_1, pygame.K_2, pygame.K_3]:
                        spectator_transform = spectator.get_transform()
                        indx = 0 if event.key == pygame.K_1 else 1 if event.key == pygame.K_2 else 2
                        rgb_ref_obj[indx].set_transform(spectator_transform)
                        depth_ref_obj[indx].set_transform(spectator_transform)
                        seg_ref_obj[indx].set_transform(spectator_transform)
                        opt_ref_obj[indx].set_transform(spectator_transform)
                        print(f'Move camera {indx + 1}', spectator_transform)
                    elif event.key == pygame.K_w:
                        randomize_weather(world)
                        print( 'Randomize weather' )

            # Sleep to ensure consistent loop timing
            clock.tick(60)




    finally:
        print('destroying actors')
        for camera in objects_list:
            camera.destroy()        
        print('done.')



def setup_mqtt():
    mq = mqtt.MqttInterface(broker_address='localhost', port=1883, username=C_USER, password=C_PASS)


    global runFlag
    runFlag = False
    if mq.connect():
        if mq.startLoop():
            print("MQTT connection established.")

    return mq


def     setup_flask(sensor_data, crops_data):
    # Initialize Flask app
    app = Flask(__name__)

    # will not run in debug mode because it is in sep thread
    #app.debug = True

    def rgb_transform(img, width, height):
        img = img.reshape((width, height, 3))  # Assuming RGBA format
        img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)  # Convert from RGBA to RGB
        img = np.rot90(img, -1)  # Rotate the image 90 degrees clockwise
        img = np.fliplr(img)
        return img


    def crop_transform(img, width, height):
        img = img.reshape((width, height, 3))  # Assuming RGBA format
        img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)  # Convert from RGBA to RGB
        img = np.rot90(img, -1)  # Rotate the image 90 degrees clockwise
        img = np.fliplr(img)

        return img

    @app.route('/video_feed/<sensor_key>')
    def video_feed(sensor_key):

        def generate():
            if sensor_key in sensor_data:
                # Convert the image data to bytes
                width = sensor_data[sensor_key].shape[0]
                height = sensor_data[sensor_key].shape[1]
                ret, buffer = cv2.imencode('.jpg', rgb_transform(sensor_data[sensor_key], width, height))
                image_data = buffer.tobytes()
                # Yield the image data as a stream
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + image_data + b'\r\n')
            else:
                yield b'Sensor key not found'

        return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

    @app.route('/crop_feed/<sensor_key>')
    def crop_feed(sensor_key):

        def generate():
            if sensor_key in crops_data:
                # Convert the image data to bytes
                if crops_data[sensor_key] is not None:
                    width = crops_data[sensor_key].shape[0]
                    height = crops_data[sensor_key].shape[1]
                    saved_crop[sensor_key] = crop_transform(crops_data[sensor_key], width, height)

                ret, buffer = cv2.imencode('.jpg', saved_crop[sensor_key])
                image_data = buffer.tobytes()
                # Yield the image data as a stream
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + image_data + b'\r\n')
            else:
                yield b'Sensor key not found'

        return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

    # Start Flask app in a separate thread
    import threading
    threading.Thread(target=app.run, kwargs={'host': '0.0.0.0', 'port': 5000}).start()


if __name__ == '__main__':
    mq = setup_mqtt()


    # Camera intrinsics
    width = 1920  # Replace with your image width in pixels
    height = 1080  # Replace with your image height in pixels
    fov_degrees = 90.0  # Replace with your desired FOV in degrees
    crop_w = 400
    crop_h = 400

    sensor_transforms = [
        carla.Transform(carla.Location(x=-20, y=0.0, z=20.4), carla.Rotation(yaw=45.0, pitch=0.0, roll=0.0)),
        carla.Transform(carla.Location(x=-24, y=26, z=20.4), carla.Rotation(yaw=0.0, pitch=-5.0, roll=0.0)),
        carla.Transform(carla.Location(x=23, y=28.6, z=20.4), carla.Rotation(yaw=180.0, pitch=-5.0, roll=0.0))
    ]
    rgb_data = {f'rgb_image_{i + 1:02}': np.zeros((height, width, 4)) for i in range(len(sensor_transforms))}
    crop_data = {f'rgb_image_{i + 1:02}': np.zeros((crop_h, crop_w, 4)) for i in range(len(sensor_transforms))}
    saved_crop = {f'rgb_image_{i + 1:02}': np.zeros((crop_h, crop_w, 4)) for i in range(len(sensor_transforms))}


    setup_flask(rgb_data, crop_data)
    main(mq, rgb_data, crop_data, sensor_transforms, width, height, fov_degrees, crop_w, crop_h)



