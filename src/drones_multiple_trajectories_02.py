from datetime import datetime
import glob
import json
import os
import sys
import random
import time
import threading
import math

import cv2
import numpy as np


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
    def __init__(self, world, start_transform, end_transform, trajectory_function=linear_trajectory, drone_name=None, timestamps=None, display_width=None, display_height=None, exp_name="test"):
        threading.Thread.__init__(self)

        self.camera = None
        self.world = world
        self.start_transform = start_transform
        self.end_transform = end_transform
        self.trajectory_function = trajectory_function
        self.drone_name = drone_name
        self.display_width=display_width
        self.display_height=display_height
        self.capture_images = None
        self.image = None

        self.experiment_name = exp_name
        data_folder = "/home/joakim/data/nerf_data"
        output_folder = os.path.join(data_folder, self.experiment_name)
        self.added_images_folder = os.path.join(output_folder, f"added_images_{self.drone_name}")


        if timestamps:
            (self.arrivals, self.departures) = timestamps
            self.capture_images = True

        else:
            self.capture_images = False

    def run(self):
        try:
            self.fly_drone()
        except Exception as e:
            print("Error while flying drone:", e)

    def save_image_and_pose(self, image, filename, destination_folder, pose, t_snap):
        # Ensure the destination folder exists
        time.sleep(t_snap)
        os.makedirs(destination_folder, exist_ok=True)

        # Create the full path for the image file
        image_path = os.path.join(destination_folder, f"{filename}.png")

        # Save the image using OpenCV
        cv2.imwrite(image_path, image)

        # Create the full path for the annotation file
        annotation_path = os.path.join(destination_folder, f"{filename}.txt")

        pose_dict = {}
        fwd = pose.rotation.get_forward_vector()
        mtx = pose.get_matrix()

        p_dict = {"pos": {"x": pose.location.x, "y": pose.location.y, "z": pose.location.z}, "rot": {"roll": pose.rotation.roll, "pitch": pose.rotation.pitch, "yaw": pose.rotation.yaw}, "normal": [fwd.x, fwd.y, fwd.z], "matrix": mtx}

        # Write the location and rotation data to the file
        with open(annotation_path, 'w') as f:
            json.dump(p_dict, f, indent=4, default=vars)


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

            if self.capture_images:

                # Create a camera sensor
                camera_bp = blueprint_library.find('sensor.camera.rgb')
                camera_bp.set_attribute('image_size_x', str(self.display_width))
                camera_bp.set_attribute('image_size_y', str(self.display_height))
                camera_bp.set_attribute('fov', '110')

                # Attach the camera to the trashbin
                relative_transform = carla.Transform(carla.Location(x=0, y=0, z=-1.3))
                self.camera = self.world.try_spawn_actor(camera_bp, relative_transform, attach_to=vehicle)


            def process_im(data):

                array = np.frombuffer(data.raw_data, dtype=np.dtype("uint8"))
                array = np.reshape(array, (data.height, data.width, 4))
                array = array[:, :, :3]
                self.image = array

            # Attach the callback to the camera
            self.camera.listen(process_im)


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
                    new_location.location.z += 1.3
                    vehicle.set_transform(carla.Transform(new_location, self.start_transform.rotation))

                    # Wait for m_time / n_steps seconds
                    time.sleep(m_time / n_steps)
            else:
                # assume end_transform is a list of route pts
                #m_time = self.timestamps[1] - self.timestamps[0]

                start_pt = self.start_transform
                for i in range(len(self.end_transform)): # over all waypoints
                    if i==0:
                        continue
                    if self.capture_images:
                        m_time = self.arrivals[i] - self.departures[i-1]
                    else:
                        m_time = 10.0

                    print(f"waypoint {i} reached.")
                    for step in range(n_steps):
                        new_location = self.trajectory_function(self.end_transform[i-1], self.end_transform[i], step, n_steps)
                        vehicle.set_transform(carla.Transform(new_location, self.start_transform.rotation))

                        time.sleep(m_time / n_steps)

                    if self.capture_images:
                        # now we have arrived, start taking picture

                        pose = self.end_transform[i]
                        vehicle.set_transform(pose)
                        prefix = "added_pose"
                        time_for_snapshot = self.departures[i] - self.arrivals[i]
                        self.save_image_and_pose(image=self.image, filename=f"{prefix}_{i:05d}",
                                            destination_folder=self.added_images_folder, pose=pose, t_snap=time_for_snapshot)

                        t_now = time.localtime()
                        current_time = time.strftime("%H:%M:%S", t_now)
                        print(f"[{self.drone_name}]: image {i} grabbed at time {current_time}, delayed for {time_for_snapshot} s, and saved at {self.added_images_folder}.")


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
    def __init__(self, world, drones, routes, colors, display_width=None, display_height=None, exp_name="test"):
        threading.Thread.__init__(self)
        self.drones = drones
        self.routes = routes
        self.num_drones = len(drones)
        self.world = world
        self.display_width = display_width
        self.display_height = display_height
        self.colors = [colors[i] for i in range(len(routes))]
        self.exp_name = exp_name

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
                thread = DroneThread(self.world, start_transform, poses, trajectory_function=selected_trajectory_function, drone_name=drone_name, timestamps=(arrival_times, departure_times), display_width=self.display_width, display_height=self.display_height, exp_name=self.exp_name)
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
