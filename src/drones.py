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

try:
    import pygame
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_q
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')





# Define the drones blueprint array
drones_blueprint_array = [
    'static.prop.drone_civilian_generic',
    'static.prop.drone_fictitious_cyberpolicevtol',
    'static.prop.drone_civilian_bee',
    'static.prop.drone_civilian_minimalistic',
    'static.prop.drone_swan',
    'static.prop.drone_civilian_phantom',
    'static.prop.drone_civilian_parrot',
    'static.prop.drone_fiveinch',
    'static.prop.drone_450',
    'static.prop.drone_x500'
]

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

        drone_name = random.choice( drones_blueprint_array )

        bp = blueprint_library.find( drone_name )

        # Define the starting and ending transforms
        start_transform = carla.Transform(carla.Location(x=20, y=40, z=15), carla.Rotation(pitch=0, yaw=0, roll=0))
        end_transform = carla.Transform(carla.Location(x=40, y=-25, z=40), carla.Rotation(pitch=0, yaw=0, roll=0))

        # Spawn the object at the starting transform
        vehicle = world.spawn_actor(bp, start_transform)
        objects_list.append( vehicle )

        # Calculate the step size for each component
        n_steps = 100
        m_time = 10.0
        # Calculate the step size for each component
        step_size = [(end_transform.location.x - start_transform.location.x) / n_steps,
                    (end_transform.location.y - start_transform.location.y) / n_steps,
                    (end_transform.location.z - start_transform.location.z) / n_steps]

        # Move the object to the ending transform in n steps
        for step in range(n_steps):
            # Calculate the new location
            new_location = carla.Location(
                x=start_transform.location.x + step * step_size[0],
                y=start_transform.location.y + step * step_size[1],
                z=start_transform.location.z + step * step_size[2]
            )

            # Set the new transform
            vehicle.set_transform(carla.Transform(new_location, start_transform.rotation))

            # Wait for m_time / n_steps seconds
            time.sleep(m_time / n_steps)



    finally:
        print('destroying actors')
        for obj in objects_list:
            obj.destroy()        
        print('done.')


if __name__ == '__main__':
    main()



