FROM nvidia/cuda:10.0-cudnn7-devel-ubuntu18.04
# using devel to support installing pycuda
# if you are behind a proxy: https://docs.docker.com/network/proxy/

RUN apt-get clean && apt-get update && apt-get install -y locales
RUN echo "en_US.UTF-8 UTF-8" > /etc/locale.gen && \
    locale-gen
ENV LC_ALL en_US.UTF-8
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US.UTF-8
ENV SHELL /bin/bash
ENV DEBIAN_FRONTEND noninteractive

# switch to bash within the container so ROS sourcing is easy in build commands
SHELL ["/bin/bash", "-c"]

RUN apt-get update && \
    apt-get install -y curl bzip2 wget vim ffmpeg git tmux unzip

# tesse: opencv dependency workaround from https://github.com/NVIDIA/nvidia-docker/issues/864
RUN apt-get update && apt-get install -y libsm6 libxext6 libxrender-dev

# kimera:
RUN apt-get update && \
    apt-get install -y --no-install-recommends apt-utils

################################
###          ROS             ###
################################

#Set the ROS distro
ENV ROS_DISTRO melodic

# Add the ROS keys and package
RUN apt-get update && \
    apt-get install -y \
    lsb-release \
    gnupg
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN mkdir ~/.gnupg
RUN echo "disable-ipv6" >> ~/.gnupg/dirmngr.conf

RUN curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | apt-key add -

# Install ROS
RUN apt-get update && \
    apt-get install -y ros-melodic-desktop-full

RUN apt-get install -y \
    python \
    python-pip \
    python-dev

RUN apt-get install python-rosdep

# Set up ROS
RUN rosdep init
RUN rosdep update

RUN source /opt/ros/melodic/setup.bash && \
    apt install -y \
    python-rosinstall \
    python-rosinstall-generator \
    python-wstool build-essential \
    python-catkin-tools

# Setup catkin workspace
RUN source /opt/ros/melodic/setup.bash && \
    mkdir -p /catkin_ws/src && \
    cd /catkin_ws/ && \
    catkin init && \
    # Change `melodic` to your ROS distro
    catkin config --extend /opt/ros/melodic && \
    catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    catkin config --merge-devel


################################
###     **  TESSE **         ###
###     ------------         ###
###   TESSE ROS packages     ###
### TESSE Anaconda packages  ###
################################

WORKDIR /catkin_ws/src

# tesse-ros-bridge dependency
RUN pip install scipy

RUN git clone https://github.com/MIT-TESSE/tesse-interface.git -b 0.1.3-SNAPSHOT && \
    cd tesse-interface && \
    python setup.py install && \
    cd ..

WORKDIR /catkin_ws/src
RUN source /opt/ros/melodic/setup.bash
RUN wstool init

RUN git clone https://github.com/MIT-TESSE/tesse-ros-bridge.git && \
    wstool merge -y tesse-ros-bridge/install/tesse_ros_bridge.rosinstall

RUN git clone https://github.com/MIT-TESSE/tesse-segmentation-ros.git && \
    wstool merge -y tesse-segmentation-ros/install/tesse_segmentation_ros.rosinstall

RUN git clone https://github.com/MIT-TESSE/tesse-gym-bridge.git && \
    wstool merge -y tesse-gym-bridge/install/tesse_gym_bridge.rosinstall

RUN wstool update
RUN catkin build tesse_ros_bridge tesse_segmentation_ros tesse_gym_bridge


################################
###  **MIT-SPARK KIMERA**    ###
###    ----------------      ###
###    KIMERA-VIO-ROS        ###
###    KIMERA-Semantics      ###
################################

# pinned commits: 
# https://github.com/MIT-SPARK/Kimera-VIO-ROS @  f6181a1370677ffe8c24253554efebaf787f428e
# https://github.com/MIT-SPARK/Kimera-VIO @ 42ad9ca557656ef3463b8cc3b841fe5a4a36398c
# https://github.com/MIT-SPARK/Kimera-Semantics @ 0ae759f24cce9e9ffc28d0495327a3d2b37d2c99

# Kimera dependencies
RUN apt-get update &&  \
    apt-get install -y \
    libgoogle-glog-dev \
    doxygen \
    cmake build-essential pkg-config autoconf \
    libboost-all-dev \
    libjpeg-dev libpng-dev libtiff-dev \
    libvtk6-dev libgtk-3-dev \
    libatlas-base-dev gfortran \
    libparmetis-dev \
    libtbb-dev

# suppress build warnings
ENV PYTHONIOENCODING=UTF-8

WORKDIR /catkin_ws/src

RUN git clone https://github.com/MIT-SPARK/Kimera-VIO-ROS.git && \
      cd Kimera-VIO-ROS && \
      git checkout db6afe795f3bf6b88842d2c45cb2306926505051 && \
      cd .. && \
      wstool merge -y Kimera-VIO-ROS/install/kimera_vio_ros_https.rosinstall && \
      wstool update

RUN git clone https://github.com/MIT-SPARK/Kimera-Semantics.git && \
      cd Kimera-Semantics && \
      git checkout 0ae759f24cce9e9ffc28d0495327a3d2b37d2c99 && \
      cd .. && \
      wstool merge -y Kimera-Semantics/kimera/install/kimera_semantics_https.rosinstall && \
      wstool update
      
# checkout pinned commit of Kimera-VIO-ROS and Kimera-VIO
RUN   cd Kimera-VIO-ROS && \
      git checkout db6afe795f3bf6b88842d2c45cb2306926505051 && \
      cd .. && \
      cd Kimera-VIO && \
      git fetch && git checkout 13a52d8772305be143ebdf470c8714759e411f59 && \
      cd ..

RUN cd gtsam && \
    git fetch && \
    git checkout develop && \
    git pull

RUN catkin build gtsam opengv_catkin opencv3_catkin kimera_rpgo dbow2_catkin

RUN catkin build

################################
###        Configs           ###
################################

RUN wget https://github.com/MIT-TESSE/goseek-challenge/releases/download/evalai-beta-testing/goseek-unet-a1.onnx \
        -P /catkin_ws/src/tesse-segmentation-ros/cfg/ && \
    wget https://github.com/MIT-TESSE/goseek-challenge/releases/download/evalai-beta-testing/kimera-configs.zip  && \
        unzip kimera-configs.zip && \
        mv kimera-configs/* /catkin_ws/src/Kimera-VIO/params/Tesse/ && \
        rm kimera-configs.zip

RUN source /catkin_ws/devel/setup.bash

WORKDIR /

RUN echo "source /catkin_ws/devel/setup.bash" >> run_perception.sh && \
    echo "roslaunch tesse_gym_bridge run_goseek_perception.launch" >> run_perception.sh && \
    chmod +x /run_perception.sh


################################
###        TensorRT          ###
################################

ADD TensorRT-6.0.1.5.Ubuntu-18.04.x86_64-gnu.cuda-10.0.cudnn7.6.tar.gz /
RUN mv /TensorRT-6.0.1.5 /tensorrt

ENV LD_LIBRARY_PATH $LD_LIBRARY_PATH:/tensorrt/lib

# install tensorrt wheel into the python environment used with ROS
RUN source /opt/ros/melodic/setup.bash &&  \
    source /catkin_ws/devel/setup.bash && \
    cd /tensorrt/python/ && \
    pip install tensorrt-6.0.1.5-cp27-none-linux_x86_64.whl && \
    pip install pycuda


#########################################################################################
# Disclaimer:
# DISTRIBUTION STATEMENT A. Approved for public release. Distribution is unlimited.

# This material is based upon work supported by the Under Secretary of Defense for Research and Engineering under Air Force Contract No. FA8702-15-D-0001. Any opinions, findings, conclusions or recommendations expressed in this material are those of the author(s) and do not necessarily reflect the views of the Under Secretary of Defense for Research and Engineering.

# © 2019 Massachusetts Institute of Technology.

# MIT Proprietary, Subject to FAR52.227-11 Patent Rights - Ownership by the contractor (May 2014)

# The software/firmware is provided to you on an As-Is basis

# Delivered to the U.S. Government with Unlimited Rights, as defined in DFARS Part 252.227-7013 or 7014 (Feb 2014). Notwithstanding any copyright notice, U.S. Government rights in this work are defined by DFARS 252.227-7013 or DFARS 252.227-7014 as detailed above. Use of this work other than as specifically authorized by the U.S. Government may violate any copyrights that exist in this work.
