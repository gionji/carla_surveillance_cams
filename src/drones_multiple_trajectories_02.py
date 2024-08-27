import datetime
import glob
import os
import sys
import random
import time
import threading
import math

try:
    import carla
except ImportError:
    raise RuntimeError('carla package is not installed. Make sure to install it before running this script.')

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
def route_trajectory(start_transform, end_transform, step, n_steps):
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
    def __init__(self, world, start_transform, end_transform, trajectory_function=linear_trajectory, drone_name=None, timestamps=None):
        threading.Thread.__init__(self)
        self.world = world
        self.start_transform = start_transform
        self.end_transform = end_transform
        self.trajectory_function = trajectory_function
        self.drone_name = drone_name
        self.timestamps = timestamps

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

            drone_name = self.drone_name if self.drone_name else random.choice(drones_blueprint_array)

            bp = blueprint_library.find(drone_name)

            # Spawn the object at the starting transform
            vehicle = self.world.spawn_actor(bp, self.start_transform)
            objects_list.append(vehicle)

            # Calculate the step size for each component
            n_steps = 100
            m_time = 10.0

            start = 0

            end = 0
            if len(self.end_transform) == 1:
                # Move the object to the ending transform in n steps using the specified trajectory function
                for step in range(n_steps):
                    new_location = self.trajectory_function(self.start_transform, self.end_transform, step, n_steps)

                    # Set the new transform
                    vehicle.set_transform(carla.Transform(new_location, self.start_transform.rotation))

                    # Wait for m_time / n_steps seconds
                    time.sleep(m_time / n_steps)
            else:
                # assume end_transform is a list of route pts
                #m_time = self.timestamps[1] - self.timestamps[0]

                start_pt = self.start_transform
                for i in range(len(self.end_transform)):
                    if i==0:
                        continue
                    m_time = self.timestamps[i] - self.timestamps[i-1]
                    print(f"waypoint {i} passed.")
                    for step in range(n_steps):
                        new_location = self.trajectory_function(self.end_transform[i-1], self.end_transform[i], step, n_steps)
                        vehicle.set_transform(carla.Transform(new_location, self.start_transform.rotation))

                        time.sleep(m_time / n_steps)


        except Exception as e:
            print(e)
        finally:
            print('destroying actors')
            for obj in objects_list:
                obj.destroy()
            print('done.')


class MultiDroneSimulation(threading.Thread):
    def __init__(self, world, num_drones=5, min_values=carla.Location(x=-50, y=-50, z=0), max_values=carla.Location(x=50, y=50, z=50)):
        threading.Thread.__init__(self)
        self.num_drones = num_drones
        self.min_values = min_values
        self.max_values = max_values
        self.world = world

    def run(self):
        try:
            # Create and start threads for flying drones
            threads = []
            for _ in range(self.num_drones):
                # Generate random starting and ending transforms within the cubic area.
                start_transform = carla.Transform(
                    carla.Location(x=random.uniform(self.min_values.x, self.max_values.x),
                                    y=random.uniform(self.min_values.y, self.max_values.y),
                                    z=random.uniform(self.min_values.z, self.max_values.z)),
                    carla.Rotation(pitch=0, yaw=0, roll=0))
                end_transform = carla.Transform(
                    carla.Location(x=random.uniform(self.min_values.x, self.max_values.x),
                                    y=random.uniform(self.min_values.y, self.max_values.y),
                                    z=random.uniform(self.min_values.z, self.max_values.z)),
                    carla.Rotation(pitch=0, yaw=0, roll=0))

                # Randomly choose one of the trajectory functions
                trajectory_functions = [linear_trajectory, sinusoidal_trajectory, circular_trajectory]
                selected_trajectory_function = random.choice(trajectory_functions)

                # Create DroneThread object with world, start_transform, end_transform, and randomly selected trajectory function.
                thread = DroneThread(self.world, start_transform, end_transform, trajectory_function=selected_trajectory_function)
                thread.start()
                threads.append(thread)

            # Wait for all threads to finish
            for thread in threads:
                thread.join()

        except KeyboardInterrupt:
            pass


class MultiDroneDataCollection(threading.Thread):
    def __init__(self, world, drones, routes, colors):
        threading.Thread.__init__(self)
        self.drones = drones
        self.routes = routes
        self.num_drones = len(drones)
        self.world = world
        self.colors = [colors[i] for i in range(len(routes))]

    def run(self):
        try:
            # Create and start threads for flying drones
            threads = []
            for drone, route, clr in zip(self.drones, self.routes, self.colors):
                poses = []
                arrival_times = []
                departure_times = []
                (drone_name, drone_speed, turn_speed, flight_time) = drone
                for i, pt in enumerate(route):
                    pose = carla.Transform(carla.Location(pt["pos"]["x"], pt["pos"]["y"], pt["pos"]["z"]),
                                           carla.Rotation(pitch=pt["rot"]["pitch"], roll=pt["rot"]["roll"], yaw=pt["rot"]["yaw"]))
                # carla.Rotation(roll=pt["rot"]["roll"], pitch=pt["rot"]["pitch"], yaw=pt["rot"]["yaw"]))
                    poses.append(pose)
                    arrival_times.append(pt["time_arrive"])
                    departure_times.append(pt["time_depart"])

                    if i==0:
                        start_transform = pose


                trajectory_functions = [route_trajectory, linear_trajectory, sinusoidal_trajectory, circular_trajectory]
                selected_trajectory_function = route_trajectory
                print(f"starting drone {drone_name}")

                # Create DroneThread object with world, start_transform, end_transform, and randomly selected trajectory function.
                thread = DroneThread(self.world, start_transform, poses, trajectory_function=selected_trajectory_function, drone_name=drone_name, timestamps=arrival_times)
                thread.start()
                threads.append(thread)

            # Wait for all threads to finish
            for thread in threads:
                thread.join()

        except KeyboardInterrupt:
            pass



if __name__ == '__main__':
    simulation = MultiDroneSimulation(num_drones=5)
    simulation.run_simulation()
