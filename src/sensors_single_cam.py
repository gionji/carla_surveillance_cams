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
    # Calculate the dimensions of each grid cell
    cell_width = screen_size[1] // grid_size[0]
    cell_height = screen_size[0] // grid_size[1]

    # Create a blank canvas to hold the grid view
    grid_view = np.zeros((screen_size[0], screen_size[1], 3), dtype=np.uint8)

    # Iterate over each camera image in the sensor_data dictionary
    for i, (name, image) in enumerate(sensor_data.items()):
        # Calculate the row and column indices for this camera image in the grid
        row = i // grid_size[0]
        col = i % grid_size[0]

        # Convert RGBA image to RGB
        if image.shape[2] == 4:
            image = cv2.cvtColor(image, cv2.COLOR_RGBA2RGB)

        # Resize the image to fit in the grid cell
        resized_image = cv2.resize(image, (cell_width, cell_height))

        # Calculate the coordinates to place the resized image in the grid
        start_x = col * cell_width
        start_y = row * cell_height

        # Insert the resized image into the grid view canvas
        grid_view[start_y:start_y + cell_height, start_x:start_x + cell_width] = resized_image

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


import struct
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
    draw_debug_annotations(world, transform)
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



def main():
    objects_list = []

    try:
        # First of all, we need to create the client that will send the requests
        # to the simulator. Here we'll assume the simulator is accepting
        # requests in the localhost at port 2000.
        client = carla.Client('localhost', 2000)
        client.set_timeout(2.0)

        # Once we have a client we can retrieve the world that is currently
        # running.
        world = client.get_world()

        # The world contains the list blueprints that we can use for adding new
        # actors into the simulation.
        blueprint_library = world.get_blueprint_library()

        # Camera intrinsics
        width  = 1920   # Replace with your image width in pixels
        height = 1080   # Replace with your image height in pixels
        fov_degrees = 90.0  # Replace with your desired FOV in degrees

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

        # Update the sensor_data dictionary to include the different cameras sensors
        sensor_data = {'rgb_image_01': np.zeros((height, width, 4)),
                       'rgb_image_02': np.zeros((height, width, 4)),
                       'rgb_image_03': np.zeros((height, width, 4)),
                       'depth_image_01': np.zeros((height, width, 4)),
                       'depth_image_02': np.zeros((height, width, 4)),
                       'depth_image_03': np.zeros((height, width, 4)),
                       'inst_image_01': np.zeros((height, width, 4)),
                       'inst_image_02': np.zeros((height, width, 4)),
                       'inst_image_03': np.zeros((height, width, 4))
        }


        # Move the spectator close to the spawned vehicle
        spectator = world.get_spectator()
        #spectator.set_transform( carla.Transform(carla.Location(x=0, y=0.0, z=40), carla.Rotation(pitch=-90)) )

        # Props: catalogue
        # https://carla.readthedocs.io/en/latest/catalogue_props/
        rererence_actor_bp = random.choice(blueprint_library.filter('static.prop.glasscontainer'))

        # Draw a debug 3d grid for debugging
        draw_debug_grid(world)

        #
        ### Sensor spawning
        #
        spawn_camera(world, blueprint_library, rererence_actor_bp, sensor_transforms[0], 'sensor.camera.optical_flow', fov_str, sensor_data, objects_list, optiflow_callback, 'rgb_image_01')
        spawn_camera(world, blueprint_library, rererence_actor_bp, sensor_transforms[1], 'sensor.camera.rgb', fov_str, sensor_data, objects_list, rgb_callback, 'rgb_image_02')
        spawn_camera(world, blueprint_library, rererence_actor_bp, sensor_transforms[2], 'sensor.camera.rgb', fov_str, sensor_data, objects_list, rgb_callback, 'rgb_image_03')

        spawn_camera(world, blueprint_library, rererence_actor_bp, sensor_transforms[0], 'sensor.camera.depth', fov_str, sensor_data, objects_list, depth_callback, 'depth_image_01')
        spawn_camera(world, blueprint_library, rererence_actor_bp, sensor_transforms[1], 'sensor.camera.depth', fov_str, sensor_data, objects_list, depth_callback, 'depth_image_02')
        spawn_camera(world, blueprint_library, rererence_actor_bp, sensor_transforms[2], 'sensor.camera.depth', fov_str, sensor_data, objects_list, depth_callback, 'depth_image_03')

        spawn_camera(world, blueprint_library, rererence_actor_bp, sensor_transforms[0], 'sensor.camera.instance_segmentation', fov_str, sensor_data, objects_list, inst_callback, 'inst_image_01')
        spawn_camera(world, blueprint_library, rererence_actor_bp, sensor_transforms[1], 'sensor.camera.instance_segmentation', fov_str, sensor_data, objects_list, inst_callback, 'inst_image_02')
        spawn_camera(world, blueprint_library, rererence_actor_bp, sensor_transforms[2], 'sensor.camera.instance_segmentation', fov_str, sensor_data, objects_list, inst_callback, 'inst_image_03')
        



        pygame.init() 

        size = (1920, 1080)
        pygame.display.set_caption("CARLA survailance system")
        screen = pygame.display.set_mode(size)

        #control = carla.VehicleControl()
        clock = pygame.time.Clock()
        done = False


        while not done:
            keys = pygame.key.get_pressed() 
            
            # Made the keyboard control into a function
            #keyboard_control(keys)
            
            current_time = datetime.datetime.now()
            date_time = current_time.strftime("%Y-%m-%d %H:%M:%S")
            
            # Let's now save the images as well
            #rgb_image_creator(sensor_data['rgb_image'],datetime)
            #depth_image_creator(sensor_data['depth_image'],datetime)

            # tick the simulation
            world.tick()

            # Generate camera grid view
            grid_view = generate_camera_grid(sensor_data, (3, 3), size)  # Pass screen size

            # Update the display and check for the quit event
            pygame.display.flip()
            pygame.display.update()
            screen.blit(pygame.surfarray.make_surface(grid_view), (0, 0)) # To display with Pygame

            # Better to use OpenCV to display both RGB and Depth Image

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    done = True

            # Sleep to ensure consistent loop timing
            clock.tick(60)



    finally:
        print('destroying actors')
        for camera in objects_list:
            camera.destroy()        
        print('done.')


if __name__ == '__main__':
    main()



