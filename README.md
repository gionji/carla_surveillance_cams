# carla_surveillance_cams

## sensors_on_vehicles.py
The provided Python script is a program that connects to the CARLA simulator and renders multiple sensors' data in a Pygame window. These sensors include RGB cameras, depth cameras, and instance segmentation cameras.

Here's a summary of what the script does:

    It imports necessary libraries, including Pygame and CARLA.
    Defines callback functions for processing sensor data.
    Spawns vehicles and drones in the CARLA simulation environment.
    Sets up camera sensors for the spawned ego vehicle.
    Initializes Pygame for rendering.
    Enters a loop where it updates the simulation, processes sensor data, generates a grid view of the sensor data, and renders it using Pygame.
    Listens for Pygame quit events to exit the loop.
    Destroys actors (vehicles, cameras, etc.) when the program ends.

The script provides functionality for keyboard controls (although not fully implemented in the provided code) and saving sensor images.

## sensors_single_cam.py
The provided script is similar to the previous one but with some differences in the way cameras are spawned and positioned. Here's a summary of what this script does:

    It imports necessary libraries, including Pygame and CARLA.
    Defines callback functions for processing sensor data (RGB, depth, instance segmentation).
    Spawns vehicles and drones in the CARLA simulation environment.
    Sets up camera sensors for the spawned vehicles.
    Initializes Pygame for rendering.
    Enters a loop where it updates the simulation, processes sensor data, generates a grid view of the sensor data, and renders it using Pygame.
    Listens for Pygame quit events to exit the loop.
    Destroys actors (vehicles, cameras, etc.) when the program ends.

However, this version of the script differs from the previous one in the way it spawns cameras. Instead of attaching cameras to existing objects (like vehicles), it spawns anchor objects for each camera and attaches cameras to these anchor objects. The anchor objects are then positioned at different locations and orientations.

The overall functionality remains the same, but the approach to camera spawning and positioning is different.

## drones.py
This script demonstrates how to spawn a drone (UAV) in the CARLA simulation environment and move it from a starting position to an ending position smoothly over a specified duration.

Here's a breakdown of what the script does:

    It imports necessary libraries, including CARLA and Pygame.
    Defines an array of drone blueprints to choose from.
    Sets up a CARLA client to connect to the simulator running on localhost at port 2000.
    Retrieves the blueprint library from the CARLA world.
    Chooses a random drone blueprint from the array defined earlier.
    Defines the starting and ending transforms for the drone's movement.
    Spawns the drone at the starting transform and adds it to the objects list.
    Calculates the step size for each component of the position (x, y, z) to move the drone smoothly from the starting position to the ending position over a specified duration.
    Moves the drone to the ending transform in a loop by gradually updating its location.
    Destroys the drone actor when the movement is completed.
    
#Docker    
## Build containers

docker build -t mqtt_broker -f dockerfiles/mqtt_broker/Dockerfile .
docker build -t sensors_single_cam -f dockerfiles/sensors_single_cam/Dockerfile .
docker run --rm drones_multiple_trajectories_01

## Run containers

docker run -ti --rm mqtt_broker
docker run -ti --rm -e C_USER=your_username -e C_PASS=your_password sensors_single_cam
docker run --rm drones_multiple_trajectories_01

## Docker compose
cd ./dockerfiles
docker-compose up


