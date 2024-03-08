# docker build -t carla-script .
# docker run -it --rm --net=host carla-script
# Allow X server connection : $> xhost +local:*
# https://medium.com/geekculture/run-a-gui-software-inside-a-docker-container-dce61771f9


# Use an official Python runtime as a parent image
FROM python:3.8-slim

# Set the working directory in the container
WORKDIR /app

# Copy the current directory contents into the container at /app
COPY . /app

# Install any dependencies needed
RUN apt-get update && \
    apt-get install -y libgl1-mesa-glx && \
    apt-get install -y xorg && \
    rm -rf /var/lib/apt/lists/* && \
    pip install pygame==2.0.1 \
                numpy==1.21.2 \
                opencv-python-headless \
                carla

# Run the Python script when the container launches
#CMD ["python", "./drones_multiple_trajectories_01.py"]
CMD ["python", "./sensors_single_cam.py"]