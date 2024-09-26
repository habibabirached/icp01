FROM ros:iron-ros-base
USER 0

RUN apt-get clean && apt-get update --fix-missing
RUN apt-get install -y apt-transport-https

# Install dependencies
RUN apt-get install --fix-missing \
    ffmpeg \
    make \
    gcc \
    libjpeg-dev \
    imagemagick \
    python3 \
    git \
    python3-pip \
    curl \
    sqlite3 -y

RUN echo "Installed system packages."


RUN apt-get install --fix-missing mosquitto-clients \
    uuid \
    jq -y

RUN echo "Installed system packa mosquittoo."


RUN pip3  --default-timeout=100 install tqdm \
    pandas \
    rosbags==0.9.19 \
    scipy \
    zxing-cpp \
    opencv-python \
    matplotlib \
    numpy==1.26.4 \ 
    pyarrow \
    paho-mqtt \
    open3d




# Copy all directories and files from the current directory into /app in the container
COPY ./ /app


# Set the working directory to /app
WORKDIR /app

CMD ["/bin/bash"]


# Run the Open3D test
# CMD ["python3", "preprocess_icp.py --section Leading_Edge --frame_list_path ./ros12399/GS012399_index_to_z_mapping.csv --json_out_dir ./testing_icp --video_ident gfdhja --models_dir ./models --enable_icp True --bag_dir ./ros12399/Ros"]

