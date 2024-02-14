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


def main():
    camera_list = []

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

        sensor_names = [
            blueprint_library.find('sensor.camera.rgb'),
            blueprint_library.find('sensor.camera.depth'),
            blueprint_library.find('sensor.camera.instance_segmentation')
        ]

        sensor_transforms = [
            carla.Transform( carla.Location(x=10, y=0.0, z=20.4), carla.Rotation(yaw=45.0, pitch=0.0, roll=0.0) ),
            carla.Transform( carla.Location(x=10, y=10, z=20.4), carla.Rotation(yaw=0.0, pitch=0.0, roll=0.0)) ,
            carla.Transform( carla.Location(x=1.5, y=10, z=20.4), carla.Rotation(yaw=-45.0, pitch=0.0, roll=0.0)) 
        ]

        # Update the sensor_data dictionary to include the depth_image, gnss and imu keys and default values
        sensor_data = {'rgb_image': np.zeros((height, width, 4)),'depth_image': np.zeros((height, width, 4))}

        # Move the spectator close to the spawned vehicle
        spectator = world.get_spectator()
        spectator.set_transform( carla.Transform(carla.Location(x=0, y=0.0, z=40), carla.Rotation(pitch=-90)) )

        # Props: catalogue
        # https://carla.readthedocs.io/en/latest/catalogue_props/
        rererence_actor_bp = random.choice(blueprint_library.filter('static.prop.glasscontainer'))


        ## Anchor
        camera_anchor_object = world.spawn_actor(
            rererence_actor_bp,
            sensor_transforms[0])
        
        camera_list.append(camera_anchor_object)

        ## Draw annotation in the spectator
        draw_debug_annotations(world, sensor_transforms[0])
        draw_debug_grid(world)

        camera_bp = sensor_names[0]    
        camera_bp.set_attribute('fov', fov_str)
        camera = world.spawn_actor(
            camera_bp,
            carla.Transform( carla.Location(x=0, y=0.0, z=0)),
            attach_to=camera_anchor_object)

        camera_list.append(camera)

        def rgb_callback(image, data_dict):
            img = np.array(image.raw_data)
            img = img.reshape((image.height, image.width, 4))  # Assuming RGBA format
            img = cv2.cvtColor(img, cv2.COLOR_RGBA2RGB)  # Convert from RGBA to RGB
            img = np.rot90(img, -1)  # Rotate the image 90 degrees clockwise
            img = np.fliplr(img)
            data_dict['rgb_image'] = img

        # We'll add all the other sensors' data into this dictionary later.
        # For now, we've added the camera feed 
        sensor_data = {'rgb_image': np.zeros((height, width, 4))}

        camera.listen(lambda image: rgb_callback(image, sensor_data))

        

        pygame.init() 

        size = (800, 600)
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

            # Update the display and check for the quit event
            pygame.display.flip()
            pygame.display.update()
            screen.blit(pygame.surfarray.make_surface(sensor_data['rgb_image']), (0, 0)) # To display with Pygame

            # Better to use OpenCV to display both RGB and Depth Image

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    done = True

            # Sleep to ensure consistent loop timing
            clock.tick(60)



    finally:
        print('destroying actors')
        for camera in camera_list:
            camera.destroy()        
        print('done.')


if __name__ == '__main__':
    main()



