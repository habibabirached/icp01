FROM ros:iron-ros-base
USER 0

















# Use a base image with Python
FROM python:3.10

# Install Open3D and a compatible version of numpy
RUN pip install numpy==1.26.4 open3d

# Install OpenGL, X11, and EGL dependencies
RUN apt-get update && apt-get install -y \
    libgl1-mesa-glx \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender-dev \
    libegl1-mesa  # Ensure EGL library is installed

# Expose the WebRTC server port (usually 8080)
EXPOSE 8080

# Copy the test_open3d.py script into the container
COPY test_open3d.py /app/test_open3d.py

# Set the working directory to /app
WORKDIR /app

# Run the Open3D test
CMD ["python", "preprocess_icp.py"]

