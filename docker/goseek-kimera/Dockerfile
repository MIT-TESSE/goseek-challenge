FROM nvidia/cuda:10.0-cudnn7-devel-ubuntu18.04
# using devel to support installing pycuda

# docker build -t goseek-kimera .
# docker run --network="host" --gpus all --rm -it goseek-kimera /bin/bash
# or with a volume mount
# mkdir work
# docker run --network="host" --gpus all -v $(pwd)/work:/work --rm -it goseek-kimera /bin/bash
#
# to run in a ROS env:
# NOTE: this will cause ROS to be sourced for all subsequent commands in the terminal
# > source /opt/ros/melodic/setup.bash &&  source /catkin_ws/devel/setup.bash && $ROS_COMMAND
# e.g.,
# > source /opt/ros/melodic/setup.bash &&  source /catkin_ws/devel/setup.bash && roslaunch gym_ros_interface run_kimera_tesse.launch
# 
# to run from within a conda env:
# > PATH=$CONDAPATH jupyter notebook --no-browser --allow-root

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

# some useful info:
# https://github.com/MIT-SPARK/Kimera-VIO-ROS/blob/master/docs/ros_installation.md
# https://github.com/mit-racecar/racecar_docker/blob/master/Dockerfile

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
###  **MIT-SPARK KIMERA**    ###
###    ----------------      ###
###    KIMERA-VIO-ROS        ###
###    KIMERA-Semantics      ###
################################

# https://github.com/MIT-SPARK/Kimera-VIO-ROS
# https://github.com/MIT-SPARK/Kimera-Semantics

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

# fetch KIMERA-VIO-ROS:
# tested with checkout bceed72
WORKDIR /catkin_ws/src
RUN source /opt/ros/melodic/setup.bash &&  \
    # not created until packages are built 
    source /catkin_ws/devel/setup.bash || true && \
    git clone https://github.com/MIT-SPARK/Kimera-VIO-ROS.git  && \
    wstool init && \
    wstool merge Kimera-VIO-ROS/install/kimera_vio_ros_https.rosinstall && \
    wstool update

# build KIMERA-VIO-ROS:
WORKDIR /catkin_ws/
RUN source /opt/ros/melodic/setup.bash && \
    # not created until packages are built 
    source /catkin_ws/devel/setup.bash || true && \
    catkin build

# fetch KIMERA-Semantics:
# tested with checkout 6637d8b
RUN source /opt/ros/melodic/setup.bash &&  \
    source /catkin_ws/devel/setup.bash && \
    cd /catkin_ws/src && \ 
    git clone https://github.com/MIT-SPARK/Kimera-Semantics.git && \
    wstool merge -y Kimera-Semantics/kimera/install/kimera_semantics_https.rosinstall && \
    wstool update

# build KIMERA-Semantics:
RUN source /opt/ros/melodic/setup.bash &&  \
    source /catkin_ws/devel/setup.bash && \
    catkin build


################################
###     **  TESSE **         ###
###     ------------         ###
###   TESSE ROS packages     ###
### TESSE Anaconda packages  ###
################################

# add tesse ROS packages
# ----------------------

WORKDIR /catkin_ws/

# TESSE-interface
# https://github.mit.edu/TESS/tesse-interface

# # tesse-interface dependency (added to ROS's python package environment):
RUN pip install scipy
#TODO(MMAZ) replace with clone
# git clone git@github.mit.edu:TESS/tesse-interface.git
COPY tesse-interface /catkin_ws/src/TESSE_interface 

# fetch and build
RUN cd /catkin_ws/src/TESSE_interface && git checkout feature/rgb-metadata
RUN source /opt/ros/melodic/setup.bash &&  \
    source /catkin_ws/devel/setup.bash && \
    cd /catkin_ws/src && \
    wstool merge -y TESSE_interface/ROS/tesse_ros_bridge/install/tesse_ros_bridge.rosinstall && \
    wstool update && \
    cd TESSE_interface/python && \
    python setup.py develop && \
    cd /catkin_ws/ && \
    catkin build

# TESSE-GYM-ROS
# https://github.mit.edu/TESS/gym-ros-interface

#TODO(MMAZ) replace with clone
COPY gym-ros-interface /catkin_ws/src/gym-ros-interface
# won't work without an MIT cert
# RUN source /opt/ros/melodic/setup.bash &&  \
#     source /catkin_ws/devel/setup.bash && \
#     cd /catkin_ws/src && \ 
#   # git clone git@github.mit.edu:TESS/gym-ros-interface.git && \
#   #wstool merge -y gym-ros-interface/install/gym_ros_interface.rosinstall  && \
#   #wstool update

RUN source /opt/ros/melodic/setup.bash &&  \
    source /catkin_ws/devel/setup.bash && \
    catkin build

# add tesse Anaconda packages
# ---------------------------

WORKDIR /tesse

RUN curl -o ~/miniconda.sh -O  https://repo.continuum.io/miniconda/Miniconda3-latest-Linux-x86_64.sh  && \
     chmod +x ~/miniconda.sh && \
     ~/miniconda.sh -b -p /opt/conda && \
     rm ~/miniconda.sh && \
     /opt/conda/bin/conda clean -ya && \
     /opt/conda/bin/conda create -n python37 python=3.7 numpy pip

RUN /opt/conda/bin/conda update -n base -c defaults conda

# this will pseudo-activate the conda environment for all subsequent build commands
# ENV PATH /opt/conda/envs/python37/bin:/opt/conda/envs/bin:$PATH

# shortcut for per-build-command conda activation
ENV CONDAPATH /opt/conda/envs/python37/bin:/opt/conda/envs/bin:$PATH

# note: the first && is required for chaining subsequent calls within the environment
#   e.g.,
#     RUN PATH=$CONDAPATH && which pip && which pip
#   will both see conda's pip, whereas
#     RUN PATH=$CONDAPATH which pip && which pip
#   will not. 
RUN PATH=$CONDAPATH && pip install numpy matplotlib jupyter scipy opencv-python

# TODO(MMAZ) replace with clone
COPY tesse-gym  /tesse-gym
COPY tesse-interface /tesse-interface
WORKDIR /tesse-interface
RUN PATH=$CONDAPATH && \
    git checkout feature/rgb-metadata && \
    cd python && \
    python setup.py install
WORKDIR /tesse-gym
RUN git checkout feature/evaluation
# note 1/2/20 (mmaz): this installs tensorflow 1.14.0 which depends on cuda 10.0 (hence the base image used above)
RUN PATH=$CONDAPATH && pip install -r requirements.txt
RUN PATH=$CONDAPATH && python setup.py develop


################################
###        TensorRT          ###
################################

# https://developer.nvidia.com/tensorrt
# https://docs.nvidia.com/deeplearning/sdk/tensorrt-developer-guide/index.html

# TODO(MMAZ): check for the presence of this file by moving the command to the top of the dockerfile
ADD TensorRT-6.0.1.5.Ubuntu-18.04.x86_64-gnu.cuda-10.0.cudnn7.6.tar.gz /
RUN mv /TensorRT-6.0.1.5 /tensorrt

ENV LD_LIBRARY_PATH $LD_LIBRARY_PATH:/tensorrt/lib

# install tensorrt wheel into the python environment used with ROS
RUN source /opt/ros/melodic/setup.bash &&  \
    source /catkin_ws/devel/setup.bash && \
    cd /tensorrt/python/ && \
    pip install tensorrt-6.0.1.5-cp27-none-linux_x86_64.whl && \
    pip install pycuda

COPY semantic-segmentation-ros /catkin_ws/src/semantic-segmentation-ros
RUN source /opt/ros/melodic/setup.bash &&  \
    source /catkin_ws/devel/setup.bash && \
    cd /catkin_ws/src && \
    wstool merge -y semantic-segmentation-ros/install/semantic_segmentation_ros.rosinstall && \
    wstool update && \
    catkin build

################################
###      Kimera Configs      ###
################################

# updated configs from Zac - TODO(MMAZ): upstream these?
COPY kimera_configs/* /catkin_ws/src/Kimera-VIO-ROS/param/tesse/

################################
###  Baseline Evaluation pkl ###
################################

# convenience shortcuts in bash history
RUN echo "source /opt/ros/melodic/setup.bash &&  source /catkin_ws/devel/setup.bash && roslaunch semantic_segmentation_ros semantic_segmentation_tesse.launch" >> /root/.bash_history
RUN echo "source /opt/ros/melodic/setup.bash &&  source /catkin_ws/devel/setup.bash && roslaunch gym_ros_interface run_kimera_tesse.launch" >> /root/.bash_history
RUN echo "PATH=\$CONDAPATH jupyter notebook --no-browser --allow-root" >> /root/.bash_history


# mark-specific TODO(MMAZ) remove
RUN cd $HOME && wget https://raw.githubusercontent.com/mmaz/tmux/master/.tmux.conf

# optional check for gpu (requires reconfiguring the docker daemon if desired as a build step)
# https://github.com/NVIDIA/nvidia-docker/wiki#can-i-use-the-gpu-during-a-container-build-ie-docker-build
# RUN python -c "import tensorflow as tf; assert tf.test.is_gpu_available()"




#########################################################################################
# Disclaimer:
# DISTRIBUTION STATEMENT A. Approved for public release. Distribution is unlimited.

# This material is based upon work supported by the Under Secretary of Defense for Research and Engineering under Air Force Contract No. FA8702-15-D-0001. Any opinions, findings, conclusions or recommendations expressed in this material are those of the author(s) and do not necessarily reflect the views of the Under Secretary of Defense for Research and Engineering.

# © 2019 Massachusetts Institute of Technology.

# MIT Proprietary, Subject to FAR52.227-11 Patent Rights - Ownership by the contractor (May 2014)

# The software/firmware is provided to you on an As-Is basis

# Delivered to the U.S. Government with Unlimited Rights, as defined in DFARS Part 252.227-7013 or 7014 (Feb 2014). Notwithstanding any copyright notice, U.S. Government rights in this work are defined by DFARS 252.227-7013 or DFARS 252.227-7014 as detailed above. Use of this work other than as specifically authorized by the U.S. Government may violate any copyrights that exist in this work.