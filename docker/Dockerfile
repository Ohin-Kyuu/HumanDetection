ARG ARCH=linux/amd64
ARG BASE_IMAGE=ros:humble
ARG NVIDIA_BASE_IMAGE=nvcr.io/nvidia/pytorch:24.07-py3
# -----------------------------------------------------------------------------
# Base Stage: nvidia container
# -----------------------------------------------------------------------------
FROM --platform=${ARCH} ${BASE_IMAGE} AS base
LABEL org.opencontainers.image.authors="ohin.kyuu@gmail.com"
ENV TERM=xterm-256color
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
  git \
  sudo \
  curl \
  wget \
  usbutils \
  v4l-utils \
  net-tools \
  iputils-ping \
  python3-pip \
  apt-transport-https \
  software-properties-common \  
  ros-humble-launch-pytest \
  # DDS
  ros-humble-rmw-cyclonedds-cpp \
  ros-humble-rmw-fastrtps-cpp \
  # Camera packages
  ros-humble-cv-bridge \
  ros-humble-sensor-msgs \
  ros-humble-sensor-msgs-py \
  ros-humble-realsense2-camera-msgs \
  ros-humble-image-transport \
  ros-humble-image-transport-plugins \   
  && apt clean -y && rm -rf /var/lib/apt/lists/*

# -----------------------------------------------------------------------------
# Builder Stage:
#   - librealsense-builder
# -----------------------------------------------------------------------------
FROM base AS librealsense-builder
ARG LIBREALSENSE_VERSION
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
  cmake \
  pkg-config \
  build-essential \
  python3 \
  python3-dev \
  ca-certificates \
  libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev \
  && apt clean -y && rm -rf /var/lib/apt/lists/*  
WORKDIR /usr/src
RUN curl https://codeload.github.com/IntelRealSense/librealsense/tar.gz/refs/tags/v$LIBREALSENSE_VERSION -o librealsense.tar.gz && \
  tar -zxf librealsense.tar.gz && \
  rm librealsense.tar.gz && \
  ln -s /usr/src/librealsense-$LIBREALSENSE_VERSION /usr/src/librealsense
# Build librealsense from source
RUN cd /usr/src/librealsense \
  && mkdir build && cd build \
  && cmake \
  -DPYTHON_EXECUTABLE=$(which python3) \
  -DCMAKE_C_FLAGS_RELEASE="${CMAKE_C_FLAGS_RELEASE} -s" \
  -DCMAKE_CXX_FLAGS_RELEASE="${CMAKE_CXX_FLAGS_RELEASE} -s" \
  -DCMAKE_INSTALL_PREFIX=/opt/librealsense \    
  -DBUILD_GRAPHICAL_EXAMPLES=OFF \
  -DBUILD_PYTHON_BINDINGS:bool=true \
  -DCMAKE_BUILD_TYPE=Release ../ \
  && make -j$(($(nproc)-1)) all \
  && make install 

# -----------------------------------------------------------------------------
# Release Stage:
#   - Realsense
#   - Ultralytics ROS: AMD64:igpu, dgpu | Jetson
# -----------------------------------------------------------------------------
###### RealSense Module ######
FROM base AS realsense
ARG REALSENSE_ROS_VERSION
ARG USER
ARG USER_UID=1000
ARG USER_GID=$USER_UID
# Copy binaries from builder stage
COPY --from=librealsense-builder /opt/librealsense /usr/local/
COPY --from=librealsense-builder /usr/lib/python3/dist-packages/pyrealsense2 /usr/lib/python3/dist-packages/pyrealsense2
COPY --from=librealsense-builder /usr/src/librealsense/config/99-realsense-libusb.rules /etc/udev/rules.d/
COPY --from=librealsense-builder /usr/src/librealsense/config/99-realsense-d4xx-mipi-dfu.rules /etc/udev/rules.d/
ENV PYTHONPATH=$PYTHONPATH:/usr/local/lib
# Install dependencies
RUN apt-get update \
  && apt-get install -y --no-install-recommends \	
  libusb-1.0-0 \
  udev \
  ca-certificates \
  ros-humble-diagnostic-updater \
  ros-humble-xacro \
  python3-tqdm \
  python3-requests \
  libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at \
  && apt clean -y && rm -rf /var/lib/apt/lists/*
# Add user and setup workspace
RUN groupadd --gid $USER_GID $USER && \
  useradd --uid $USER_UID --gid $USER_GID -ms /bin/bash $USER && \
  echo "$USER ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USER && \
  chmod 0440 /etc/sudoers.d/$USER && \
  echo "source /opt/ros/humble/setup.bash" >> /home/$USER/.bashrc && \
  echo "source /home/$USER/capstone-ws/install/local_setup.bash" >> /home/$USER/.bashrc
USER $USER
RUN mkdir -p /home/$USER/capstone-ws/src && \
  # Install ROS2 Realsense package
  git clone --branch $REALSENSE_ROS_VERSION https://github.com/IntelRealSense/realsense-ros.git /home/$USER/capstone-ws/src/realsense-ros && \
  apt-get install python3-rosdep -y && \
  rosdep update && \
  rosdep install -i --from-path /home/$USER/capstone-ws/src --rosdistro humble --skip-keys=librealsense2 -y && \
  cd /home/$USER/capstone-ws && \
  /bin/bash -c "source /opt/ros/humble/setup.bash && \
  colcon build --symlink-install"
WORKDIR /home/$USER/capstone-ws
CMD [ "/bin/bash" ]

FROM --platform=${ARCH} ${NVIDIA_BASE_IMAGE} AS yolo
ARG USER
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ENV TERM=xterm-256color
# Copy binaries from ros humble base stage
COPY --from=base /opt/ros/humble /opt/ros/humble
COPY --from=base /usr/bin/ /usr/bin/
COPY --from=base /usr/lib/ /usr/lib/
COPY --from=base /usr/local/lib/ /usr/local/lib/
RUN apt update && DEBIAN_FRONTEND=noninteractive apt install -y \
  sudo \
  libblas3 liblapack3 \
  && apt clean -y && rm -rf /var/lib/apt/list/*
# Add user and setup workspace
RUN groupadd --gid $USER_GID $USER && \
  useradd --uid $USER_UID --gid $USER_GID -ms /bin/bash $USER && \
  echo "$USER ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USER && \
  chmod 0440 /etc/sudoers.d/$USER && \
  echo "source /opt/ros/humble/setup.bash" >> /home/$USER/.bashrc && \
  echo "source /home/$USER/capstone-ws/install/local_setup.bash" >> /home/$USER/.bashrc
USER $USER
RUN pip install --no-cache-dir ultralytics
WORKDIR /home/$USER/capstone-ws
CMD [ "/bin/bash" ]

###### GUI Module ######
FROM base AS gui
ARG USER
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
  ros-humble-rqt* \
  ros-humble-rviz2 \
  ros-humble-foxglove-bridge \
  && apt clean -y && rm -rf /var/lib/apt/list/*
RUN groupadd --gid $USER_GID $USER && \
  useradd --uid $USER_UID --gid $USER_GID -ms /bin/bash $USER && \
  echo "$USER ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USER && \
  chmod 0440 /etc/sudoers.d/$USER && \
  echo "source /opt/ros/humble/setup.bash" >> /home/$USER/.bashrc
USER $USER
WORKDIR /home/$USER
CMD [ "/bin/bash" ]
