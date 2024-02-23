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
import threading
import math

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

# Define three alternative trajectory functions
def linear_trajectory(start_transform, end_transform, step, n_steps):
    # Calculate the step size for each component
    step_size = [(end_transform.location.x - start_transform.location.x) / n_steps,
                 (end_transform.location.y - start_transform.location.y) / n_steps,
                 (end_transform.location.z - start_transform.location.z) / n_steps]

    # Calculate the new location
    new_location = carla.Location(
        x=start_transform.location.x + step * step_size[0],
        y=start_transform.location.y + step * step_size[1],
        z=start_transform.location.z + step * step_size[2]
    )

    return new_location

def sinusoidal_trajectory(start_transform, end_transform, step, n_steps, amplitude=[20,20,20]):
    # Calculate the step size for each component
    step_size = [(end_transform.location.x - start_transform.location.x) / n_steps,
                 (end_transform.location.y - start_transform.location.y) / n_steps,
                 (end_transform.location.z - start_transform.location.z) / n_steps]

    # Calculate the new location using a sinusoidal trajectory with adjustable amplitude
    new_location = carla.Location(
        x=start_transform.location.x + step_size[0] * amplitude[0] * math.sin(step / n_steps * 2 * math.pi),
        y=start_transform.location.y + step_size[1] * amplitude[1] * math.sin(step / n_steps * 2 * math.pi),
        z=start_transform.location.z + step_size[2] * amplitude[2] * math.sin(step / n_steps * 2 * math.pi)
    )

    return new_location


def circular_trajectory(start_transform, end_transform, step, n_steps):
    center_location = carla.Location(
        x=(start_transform.location.x + end_transform.location.x) / 2,
        y=(start_transform.location.y + end_transform.location.y) / 2,
        z=(start_transform.location.z + end_transform.location.z) / 2
    )

    radius = math.sqrt((end_transform.location.x - start_transform.location.x) ** 2 +
                       (end_transform.location.y - start_transform.location.y) ** 2 +
                       (end_transform.location.z - start_transform.location.z) ** 2) / 2

    angle = step / n_steps * 2 * math.pi

    new_location = carla.Location(
        x=center_location.x + radius * math.cos(angle),
        y=center_location.y + radius * math.sin(angle),
        z=center_location.z
    )

    return new_location



class DroneThread(threading.Thread):
    def __init__(self, client, world, start_transform, end_transform, trajectory_function=linear_trajectory):
        threading.Thread.__init__(self)
        self.client = client
        self.world = world
        self.start_transform = start_transform
        self.end_transform = end_transform
        self.trajectory_function = trajectory_function

    def run(self):
        try:
            self.fly_drone()
        except Exception as e:
            print("Error while flying drone:", e)

    def fly_drone(self):
        objects_list = []
        try:
            # The world contains the list blueprints that we can use for adding new
            # actors into the simulation.
            blueprint_library = self.world.get_blueprint_library()

            drone_name = random.choice(drones_blueprint_array)

            bp = blueprint_library.find(drone_name)

            # Spawn the object at the starting transform
            vehicle = self.world.spawn_actor(bp, self.start_transform)
            objects_list.append(vehicle)

            # Calculate the step size for each component
            n_steps = 100
            m_time = 10.0

            # Move the object to the ending transform in n steps using the specified trajectory function
            for step in range(n_steps):
                new_location = self.trajectory_function(self.start_transform, self.end_transform, step, n_steps)

                # Set the new transform
                vehicle.set_transform(carla.Transform(new_location, self.start_transform.rotation))

                # Wait for m_time / n_steps seconds
                time.sleep(m_time / n_steps)

        finally:
            print('destroying actors')
            for obj in objects_list:
                obj.destroy()
            print('done.')

def main():
    try:
        # First of all, we need to create the client that will send the requests
        # to the simulator. Here we'll assume the simulator is accepting
        # requests on localhost at port 2000.
        client = carla.Client('localhost', 2000)
        client.set_timeout(2.0)

        # Retrieve the current world running in the simulation.
        world = client.get_world()

        # Define the minimum and maximum values for the cubic area.
        min_values = carla.Location(x=-50, y=-50, z=0)
        max_values = carla.Location(x=50, y=50, z=50)

        # Create and start threads for flying drones
        num_drones = 50  # Adjust the number of drones as needed
        threads = []
        for _ in range(num_drones):
            # Generate random starting and ending transforms within the cubic area.
            start_transform = carla.Transform(
                carla.Location(x=random.uniform(min_values.x, max_values.x),
                                y=random.uniform(min_values.y, max_values.y),
                                z=random.uniform(min_values.z, max_values.z)),
                carla.Rotation(pitch=0, yaw=0, roll=0))
            end_transform = carla.Transform(
                carla.Location(x=random.uniform(min_values.x, max_values.x),
                                y=random.uniform(min_values.y, max_values.y),
                                z=random.uniform(min_values.z, max_values.z)),
                carla.Rotation(pitch=0, yaw=0, roll=0))

            # Randomly choose one of the trajectory functions
            trajectory_functions = [linear_trajectory, sinusoidal_trajectory, circular_trajectory]
            selected_trajectory_function = random.choice(trajectory_functions)

            # Create DroneThread object with client, world, start_transform, end_transform, and randomly selected trajectory function.
            thread = DroneThread(client, world, start_transform, end_transform, trajectory_function=selected_trajectory_function)
            thread.start()
            threads.append(thread)

        # Wait for all threads to finish
        for thread in threads:
            thread.join()

    except KeyboardInterrupt:
        pass



if __name__ == '__main__':
    main()