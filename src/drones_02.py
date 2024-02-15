import datetime
import glob
import os
import sys
import random
import time
import math

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla


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

# Define the cubic area where drones will fly (x_min, y_min, z_min, x_max, y_max, z_max)
cubic_area = (-20, -20, 10, 20, 20, 30)

# Define the number of drones to spawn
num_drones = 5

# Define the number of steps for each trajectory
num_steps = 100
movement_time = 10.0


def main():
    objects_list = []

    try:
        # First of all, we need to create the client that will send the requests
        # to the simulator. Here we'll assume the simulator is accepting
        # requests on localhost at port 2000.
        client = carla.Client('localhost', 2000)
        client.set_timeout(2.0)

        # Once we have a client, we can retrieve the world that is currently running.
        world = client.get_world()

        # The world contains the list of blueprints that we can use for adding new
        # actors into the simulation.
        blueprint_library = world.get_blueprint_library()

        # Spawn the drones
        for _ in range(num_drones):
            # Randomly choose a drone blueprint
            drone_name = random.choice(drones_blueprint_array)
            bp = blueprint_library.find(drone_name)

            # Randomly choose a starting location within the cubic area
            start_location = carla.Location(
                x=random.uniform(cubic_area[0], cubic_area[3]),
                y=random.uniform(cubic_area[1], cubic_area[4]),
                z=random.uniform(cubic_area[2], cubic_area[5])
            )
            start_transform = carla.Transform(start_location)

            # Spawn the drone at the starting location
            drone = world.spawn_actor(bp, start_transform)
            objects_list.append(drone)

            # Define a trajectory for the drone
            # Example: Move in a random direction for a fixed distance
            direction = carla.Vector3D(
                x=random.uniform(-1, 1),
                y=random.uniform(-1, 1),
                z=random.uniform(-1, 1)
            )
            # Normalize the direction vector
            magnitude = math.sqrt(direction.x ** 2 + direction.y ** 2 + direction.z ** 2)
            if magnitude != 0:
                direction = carla.Vector3D(direction.x / magnitude, direction.y / magnitude, direction.z / magnitude)

            step_size = carla.Vector3D(direction.x * 2, direction.y * 2, direction.z * 2)  # Adjust the speed as needed

            # Move the drone along the trajectory in num_steps steps
            for step in range(num_steps):
                new_location = drone.get_location() + step_size
                drone.set_location(new_location)
                time.sleep(movement_time / num_steps)

    finally:
        print('Destroying actors')
        for obj in objects_list:
            obj.destroy()
        print('Done.')


if __name__ == '__main__':
    main()
