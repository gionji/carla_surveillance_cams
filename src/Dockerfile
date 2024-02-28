# docker build -t carla-script .
# docker run -it --rm --net=host carla-script

# Use an official Python runtime as a parent image
FROM python:3.8-slim

# Set the working directory in the container
WORKDIR /app

# Copy the current directory contents into the container at /app
COPY . /app

# Install any dependencies needed
RUN pip install pygame==2.0.1 \
                numpy==1.21.2 \
                carla

# Run the Python script when the container launches
CMD ["python", "./drones_multiple_trajectories_01.py"]
