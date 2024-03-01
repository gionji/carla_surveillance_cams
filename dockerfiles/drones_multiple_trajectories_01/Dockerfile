# Dockerfile for MQTT Python script

# Use the official Python image as base image
FROM python:3.8-slim

# Set the working directory in the container
WORKDIR /app

# Copy the Python script into the container
COPY dockerfiles/drones_multiple_trajectories_01/requirements.txt .

# Install any dependencies needed
RUN apt-get update && \
    apt-get install -y libgl1-mesa-glx && \
    apt-get install -y xorg && \
    rm -rf /var/lib/apt/lists/*
    
RUN pip install --upgrade pip setuptools

# Install dependencies from requirements.txt
RUN pip install -r requirements.txt

COPY src/ .

# Set the entrypoint script
COPY dockerfiles/sensors_single_cam/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh", "/app/drones_multiple_trajectories_01.py"]
