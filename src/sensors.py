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

try:
    import pygame
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_q
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')




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

        # for all the given positions
        # create an anchor object
        # attach to each of the m the selected data type
        for sensor_transform in sensor_transforms:
            # Spawn the vehicle
            camera_anchor_object = world.spawn_actor(
                rererence_actor_bp,
                sensor_transform)
            
            camera_list.append(camera_anchor_object)

            world.debug.draw_arrow(sensor_transform.location, 
                            sensor_transform.location + sensor_transform.get_forward_vector(), 
                            life_time=10, 
                            color=carla.Color(r=255, g=0, b=0), 
                            arrow_size=0.5, 
                            persistent_lines=True)
            
            # Print camera position in debug mode
            camera_position_str = f"Camera position: {sensor_transform.location.x}, {sensor_transform.location.y}, {sensor_transform.location.z}"
            world.debug.draw_string(sensor_transform.location + carla.Location(z=2.0), 
                                    camera_position_str, 
                                    draw_shadow=True, 
                                    color=carla.Color(r=255, g=255, b=255), 
                                    life_time=10)

            for sensor_bp in sensor_names:
                # Ego cameras
                ego_camera_bp = blueprint_library.find(sensor_bp)
                ego_camera_bp.set_attribute('fov', fov_str)
                ego_camera = world.spawn_actor(
                    ego_camera_bp,
                    sensor_transform,
                    attach_to=camera_anchor_object)

                camera_list.append(ego_camera)


        def rgb_callback(image, data_dict):
            img = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4)) #Reshaping with alpha channel
            img[:,:,3] = 255 #Setting the alpha to 255 
            data_dict['rgb_image'] = img

        # We'll add all the other sensors' data into this dictionary later.
        # For now, we've added the camera feed 
        sensor_data = {'rgb_image': np.zeros((height, width, 4))}

        camera.listen(lambda image: rgb_callback(image, sensor_data))

        time.sleep(10)    

    finally:
        print('destroying actors')
        for camera in camera_list:
            camera.destroy()        
        print('done.')


if __name__ == '__main__':
    main()



