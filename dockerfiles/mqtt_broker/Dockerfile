# Dockerfile for MQTT Python script

# Use the official Python image as base image
FROM python:3

# Set the working directory in the container
WORKDIR /app

# Copy the Python script into the container
COPY src/ .
COPY dockerfiles/mqtt_broker/requirements.txt .

RUN pip install --upgrade pip setuptools

# Install dependencies from requirements.txt
RUN pip install -r requirements.txt

# Set the entrypoint script
COPY dockerfiles/mqtt_broker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh", "/app/mqtt.py"]
