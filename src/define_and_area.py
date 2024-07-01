import json
import os
import time

import cv2
import pygame
import carla
import numpy as np
import math

from tqdm import tqdm

# Initialize Pygame
pygame.init()

# Set up display
display_width = 1850
display_height = 1000
display = pygame.display.set_mode((display_width, display_height))
pygame.display.set_caption("CARLA Spectator View")

clock = pygame.time.Clock()

# Connect to CARLA server
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()
map_index = 3
available_maps = ["Town01", "Town02", "Town03", "Town04", "Town05", "Town06", "Town07", "Town10HD"]
map_name = available_maps[map_index]
client.load_world(map_name)

# Spawn the trashbin
blueprint_library = world.get_blueprint_library()
trashbin_bp = blueprint_library.find('static.prop.colacan')
initial_transform = carla.Transform(carla.Location(x=0, y=0, z=100))
trashbin = world.try_spawn_actor(trashbin_bp, initial_transform)

# Create a camera sensor
camera_bp = blueprint_library.find('sensor.camera.rgb')
camera_bp.set_attribute('image_size_x', str(display_width))
camera_bp.set_attribute('image_size_y', str(display_height))
camera_bp.set_attribute('fov', '110')

# Attach the camera to the trashbin
relative_transform = carla.Transform(carla.Location(x=0, y=0, z=-2))
camera = world.try_spawn_actor(camera_bp, relative_transform, attach_to=trashbin)

# Trashbin control variables
trashbin_speed = 30
trashbin_rotation_speed = 30

def move_trashbin(trashbin, keys, delta_time):
    transform = trashbin.get_transform()
    location = transform.location
    rotation = transform.rotation

    # Calculate forward and right vectors
    yaw = math.radians(rotation.yaw)
    forward_vector = carla.Vector3D(math.cos(yaw), math.sin(yaw), 0)
    right_vector = carla.Vector3D(-math.sin(yaw), math.cos(yaw), 0)

    if keys[pygame.K_w]:
        location += forward_vector * trashbin_speed * delta_time
    if keys[pygame.K_s]:
        location -= forward_vector * trashbin_speed * delta_time
    if keys[pygame.K_a]:
        location -= right_vector * trashbin_speed * delta_time
    if keys[pygame.K_d]:
        location += right_vector * trashbin_speed * delta_time
    if keys[pygame.K_q]:
        location.z += trashbin_speed * delta_time
    if keys[pygame.K_e]:
        location.z -= trashbin_speed * delta_time

    if keys[pygame.K_UP]:
        rotation.pitch += trashbin_rotation_speed * delta_time
    if keys[pygame.K_DOWN]:
        rotation.pitch -= trashbin_rotation_speed * delta_time
    if keys[pygame.K_LEFT]:
        rotation.yaw -= trashbin_rotation_speed * delta_time
    if keys[pygame.K_RIGHT]:
        rotation.yaw += trashbin_rotation_speed * delta_time

    trashbin.set_transform(carla.Transform(location, rotation))


saved_transform = None

def save_trashbin_transform(trashbin):
    global saved_transform
    transform = trashbin.get_transform()
    saved_transform = transform

    location = transform.location
    rotation = transform.rotation
    print(f"Location: x={location.x}, y={location.y}, z={location.z}")
    print(f"Rotation: pitch={rotation.pitch}, yaw={rotation.yaw}, roll={rotation.roll}")

def compute_box_edges(transform, width, length, height):
    # Compute the eight corners of the box
    corners = [
        carla.Location(x=-width/2, y=-length/2, z=-height/2),
        carla.Location(x=width/2,  y=-length/2, z=-height/2),
        carla.Location(x=-width/2, y=length/2,  z=-height/2),
        carla.Location(x=width/2,  y=length/2,  z=-height/2),
        carla.Location(x=-width/2, y=-length/2, z=height/2),
        carla.Location(x=width/2,  y=-length/2, z=height/2),
        carla.Location(x=-width/2, y=length/2,  z=height/2),
        carla.Location(x=width/2,  y=length/2,  z=height/2),
    ]

    if saved_transform != None:
        transform = saved_transform 

    # Apply the transform to the corners
    box_edges = [carla.Location(transform.location.x + corner.x, transform.location.y + corner.y, transform.location.z + corner.z) for corner in corners]
    
    return box_edges


saved_box_edges = None

def save_box_edges(trashbin, width, length, height):
    global saved_box_edges
    time.sleep(0.2)
    transform = trashbin#.get_transform()
    edges = compute_box_edges(transform, width, length, height)
    saved_box_edges = edges
    for i, edge in enumerate(edges):
        print(f"Edge {i+1}: x={edge.x:.1f}, y={edge.y:.1f}, z={edge.z:.1f}")
    print("Done.")
def draw_box_edges(world, trashbin, width, length, height, lifetime=0.1):
    transform = trashbin.get_transform()
    edges = compute_box_edges(transform, width, length, height)

    if saved_box_edges != None:
        edges = saved_box_edges
    
    # Draw points at each edge
    for edge in edges:
        world.debug.draw_point(edge, size=0.1, color=carla.Color(255, 0, 0), life_time=lifetime)

    # Draw lines between the edges to form the box
    connections = [
        (0, 1), (1, 3), (3, 2), (2, 0),  # Bottom face
        (4, 5), (5, 7), (7, 6), (6, 4)#,  # Top face
        #(0, 4), (1, 5), (2, 6), (3, 7)   # Vertical lines
    ]
    
    for start, end in connections:
        world.debug.draw_line(edges[start], edges[end], color=carla.Color(0, 255, 0), life_time=lifetime)

    connections = [
        #(0, 1), (1, 3), (3, 2), (2, 0),  # Bottom face
        #(4, 5), (5, 7), (7, 6), (6, 4),  # Top face
        (0, 4), (1, 5), (2, 6), (3, 7)  # Vertical lines
    ]

    for start, end in connections:
        world.debug.draw_line(edges[start], edges[end], color=carla.Color(255, 0, 0), life_time=lifetime)


def extract_grid_transforms(N, M, K=1):
    # Calculate the center of the box
    saved_edges = None

    if saved_box_edges != None:
        saved_edges = saved_box_edges
    else: 
        return

    center = carla.Location(x=0, y=0, z=0)
    for i in range(0,8):
        center.x += saved_edges[i].x/8
        center.y += saved_edges[i].y/8
        center.z += saved_edges[i].z/8
    
    # Calculate the grid points on the middle plane perpendicular to the ground
    # We assume the grid is on the XY plane at z = center.z
    grid_points = []
    grid_pos = []
    for i in range(N):
        for j in range(M):
            for k in range(K):
                # Calculate relative positions
                x_rel = ((i + 0.5) / N - 0.5) * width_slider.val
                y_rel = ((j + 0.5) / M - 0.5) * length_slider.val
                z_rel = ((k + 0.5) / K - 0.5) * height_slider.val # Plane is at z = 0 relative to center

                # Calculate absolute positions
                grid_point = center + carla.Location(x_rel, y_rel, z_rel)
                grid_points.append(grid_point)
                grid_pos.append({"x": center.x+x_rel, "y": center.y+y_rel, "z": center.z+z_rel})

    # Print grid points (for demonstration purposes)
    #for idx, point in enumerate(grid_points):
    #    print(f"Grid Point {idx + 1}: x={point.x}, y={point.y}, z={point.z}")
    #    world.debug.draw_point(point, size=1.1, color=carla.Color(255, 0, 0), life_time=5.0)

    return grid_points, grid_pos



# Callback to capture camera images
image = None
def process_image(data):
    global image
    array = np.frombuffer(data.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (data.height, data.width, 4))
    array = array[:, :, :3]
    image = array

# Attach the callback to the camera
camera.listen(process_image)

# Define the sliders for width, length, and height
slider_width = 200
slider_height = 20

class Slider:
    def __init__(self, x, y, w, min_val, max_val, val=None):
        self.rect = pygame.Rect(x, y, w, slider_height)
        self.min_val = min_val
        self.max_val = max_val
        self.val = val if val is not None else min_val
        self.active = False

    def draw(self, win):
        pygame.draw.rect(win, (180, 180, 180), self.rect)
        handle_pos = ((self.val - self.min_val) / (self.max_val - self.min_val)) * self.rect.width
        handle_rect = pygame.Rect(self.rect.x + handle_pos - 5, self.rect.y - 5, 10, slider_height + 10)
        pygame.draw.rect(win, (0, 0, 0), handle_rect)

    def update(self, events):
        for event in events:
            if event.type == pygame.MOUSEBUTTONDOWN:
                if self.rect.collidepoint(event.pos):
                    self.active = True
            elif event.type == pygame.MOUSEBUTTONUP:
                self.active = False
            elif event.type == pygame.MOUSEMOTION:
                if self.active:
                    handle_pos = event.pos[0] - self.rect.x
                    self.val = self.min_val + (handle_pos / self.rect.width) * (self.max_val - self.min_val)
                    self.val = max(self.min_val, min(self.val, self.max_val))

width_slider = Slider(50, display_height - 80, slider_width, 1, 200)
length_slider = Slider(300, display_height - 80, slider_width, 1, 200)
height_slider = Slider(550, display_height - 80, slider_width, 1, 200)

width_grid_slider = Slider(50, display_height - 40, slider_width, 1, 20, val=4)
length_grid_slider = Slider(300, display_height - 40, slider_width, 1, 20, val=4)
height_grid_slider = Slider(550, display_height - 40, slider_width, 1, 20, val=2)


def load_spectator_position(folder, filename):
    transforms = []
    with open(os.path.join(folder, filename), 'r') as f:
        lines = f.readlines()
        i = 0
        while i < len(lines):
            if lines[i].startswith("width:"):
                width = float(lines[i].split(":")[1].strip())
                i += 1
            if lines[i].startswith("length:"):
                length = float(lines[i].split(":")[1].strip())
                i += 1
            if lines[i].startswith("height:"):
                height = float(lines[i].split(":")[1].strip())
                i += 1

            if lines[i].startswith("Spectator:"):
                obj_id = int(lines[i].split(":")[1].strip())
                pos_line = lines[i + 1].split(":")[1].strip().replace('(', '').replace(')', '').split(',')
                position = carla.Location(float(pos_line[0]), float(pos_line[1]), float(pos_line[2]))
                rot_line = lines[i + 2].split(":")[1].strip().replace('(', '').replace(')', '').split(',')
                rotation = carla.Rotation(float(rot_line[0]), float(rot_line[1]), float(rot_line[2]))
                transform = carla.Transform(position, rotation)
                i += 3
            i+=1


    return width, length, height, transform

def save_spectator_position(folder, filename, spectator, width, length, height):
    with open(os.path.join(folder, filename), 'w') as f:

        f.write(f"width: {width}\n")
        f.write(f"length: {length}\n")
        f.write(f"height: {height}\n")
        transform = spectator.get_transform()
        position = transform.location
        rotation = transform.rotation
        f.write(f"Spectator: {spectator.id}\n")
        f.write(f"Position: ({position.x}, {position.y}, {position.z})\n")
        f.write(f"Rotation: ({rotation.pitch}, {rotation.yaw}, {rotation.roll})\n")
        f.write("\n")


def save_image_and_annotation(image, filename, destination_folder, location, rotation):
    # Ensure the destination folder exists
    time.sleep(0.5)
    os.makedirs(destination_folder, exist_ok=True)

    # Create the full path for the image file
    image_path = os.path.join(destination_folder, f"{filename}.png")

    # Save the image using OpenCV
    cv2.imwrite(image_path, image)

    # Create the full path for the annotation file
    annotation_path = os.path.join(destination_folder, f"{filename}.txt")

    # Write the location and rotation data to the file
    with open(annotation_path, 'w') as f:
        f.write(f"Location: {location}\n")
        f.write(f"Rotation: {rotation}\n")


def save_image_and_pose(image, filename, destination_folder, pose):
    # Ensure the destination folder exists
    time.sleep(0.5)
    os.makedirs(destination_folder, exist_ok=True)

    # Create the full path for the image file
    image_path = os.path.join(destination_folder, f"{filename}.png")

    # Save the image using OpenCV
    cv2.imwrite(image_path, image)

    # Create the full path for the annotation file
    annotation_path = os.path.join(destination_folder, f"{filename}.txt")

    # Write the location and rotation data to the file
    with open(annotation_path, 'w') as f:
        json.dump(pose, f, indent=4, default=vars)


def generate_poses(gpt, gridpoints, rt, rotations):
    poses = []
    for i, gp in enumerate(gridpoints):
        for j, rot in enumerate(rotations):
            fwd = rt[j].get_forward_vector()

            p = {"pos": gp, "rot": rot, "normal": [fwd.x, fwd.y, fwd.z]}
            poses.append(p)
    return poses

try:
    # Main loop
    experiment_name = "test_normal"
    data_folder = "/home/joakim/data/nerf_data"
    output_folder = os.path.join(data_folder, experiment_name)
    images_folder = os.path.join(output_folder, "images")
    file_name = "spectator.txt"
    pose_file_name = "camera_poses.txt"
    running = True
    while running:
        events = pygame.event.get()
        for event in events:
            if event.type == pygame.QUIT:
                running = False

        keys = pygame.key.get_pressed()

        if keys[pygame.K_r]:
            trashbin.set_transform(initial_transform)
        elif keys[pygame.K_l]:
            width, length, height, transform = load_spectator_position(folder=output_folder, filename=file_name)
            trashbin.set_transform(transform)
            saved_transform = transform
            width_slider.val, length_slider.val, height_slider.val = width, length, height

        elif keys[pygame.K_o]:
            #save_trashbin_transform(trashbin)
            save_spectator_position(folder=output_folder, filename=file_name, spectator=trashbin, width=width_slider.val, length=length_slider.val, height=height_slider.val)

        elif keys[pygame.K_b]:
            save_box_edges(saved_transform, width_slider.val, length_slider.val, height_slider.val)
        elif keys[pygame.K_v]:
            draw_box_edges(world, trashbin, width_slider.val, length_slider.val, height_slider.val, lifetime=5.0)
        elif keys[pygame.K_y]:
            if saved_box_edges != None:
                gridpoints, gridpos = extract_grid_transforms(N=int(width_grid_slider.val), M=int(length_grid_slider.val), K=int(height_grid_slider.val))
                rotations = [carla.Rotation(yaw=0, pitch=-90, roll=0)]
                rots = [{"roll": 0, "pitch": -90, "yaw": 0}]

                # for y in [0, 90, 180, 270]:
                #     rotations.append(carla.Rotation(yaw=y, pitch=-45, roll=0))
                #     rots.append({"roll": 0, "pitch": -45, "yaw": y})
                for y in [0, 45, 90, 135, 180, 225, 270, 315]:
                    rotations.append(carla.Rotation(yaw=y, pitch=-30, roll=0))
                    rots.append({"roll": 0, "pitch": -30, "yaw": y})

                poses = generate_poses(gridpoints, gridpos, rotations, rots)
                full_pose = os.path.join(output_folder, pose_file_name)
                with open(full_pose, 'w') as outfile:
                    json.dump({"poses": poses}, outfile, indent=4, default=vars)

                f = open(full_pose)
                json_data = json.load(f)
                read_poses = json_data['poses']

                l = len(read_poses)
                #with tqdm(total=l) as pbar:

                for i, pose in enumerate(tqdm(read_poses)):
                    r = carla.Rotation(yaw=pose["rot"]["yaw"], pitch=pose["rot"]["pitch"], roll=pose["rot"]["roll"])
                    pt = carla.Location(x=pose["pos"]["x"], y=pose["pos"]["y"], z=pose["pos"]["z"])
                    trashbin.set_transform(carla.Transform(pt, r))
                    save_image_and_pose(image=image, filename=f"pose_{i:05d}", destination_folder=images_folder, pose=pose)
                    #print(f"Image {i} out of {l} saved.")
                    #pbar.update(i)

        else:
            move_trashbin(trashbin, keys, clock.get_time() / 1000.0)

        if image is not None:
            surface = pygame.surfarray.make_surface(image.swapaxes(0, 1))
            display.blit(surface, (0, 0))

        # Update and draw sliders
        width_slider.update(events)
        length_slider.update(events)
        height_slider.update(events)
        width_slider.draw(display)
        length_slider.draw(display)
        height_slider.draw(display)

        width_grid_slider.update(events)
        length_grid_slider.update(events)
        height_grid_slider.update(events)
        width_grid_slider.draw(display)
        length_grid_slider.draw(display)
        height_grid_slider.draw(display)

        # Draw labels for sliders
        font = pygame.font.Font(None, 36)
        width_label = font.render(f"Width: {int(width_slider.val)}", True, (255, 255, 255))
        length_label = font.render(f"Length: {int(length_slider.val)}", True, (255, 255, 255))
        height_label = font.render(f"Height: {int(height_slider.val)}", True, (255, 255, 255))
        display.blit(width_label, (50, display_height - 110))
        display.blit(length_label, (300, display_height - 110))
        display.blit(height_label, (550, display_height - 110))
        width_grid_label = font.render(f"width grid: {int(width_grid_slider.val)}", True, (255, 255, 255))
        length_grid_label = font.render(f"length grid: {int(length_grid_slider.val)}", True, (255, 255, 255))
        height_grid_label = font.render(f"Height grid: {int(height_grid_slider.val)}", True, (255, 255, 255))
        display.blit(width_grid_label, (50, display_height - 50))
        display.blit(length_grid_label, (300, display_height - 50))
        display.blit(height_grid_label, (550, display_height - 50))

        tf = trashbin.get_transform()
        location = tf.location
        rotation = tf.rotation
        pose_label = font.render(f"x: {location.x:.1f}, y: {location.y:.1f}, z:{location.z:.1f}", True, (255, 255, 255))
        display.blit(pose_label, (800, display_height - 50))

        pygame.display.flip()
        clock.tick(30)

finally:
    # Clean up
    camera.stop()
    camera.destroy()
    trashbin.destroy()
    pygame.quit()
