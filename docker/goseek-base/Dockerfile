FROM nvidia/cuda:10.0-cudnn7-runtime-ubuntu18.04

# docker build -t goseek-base .
# docker run --network="host" --gpus all  --rm -it goseek-base /bin/bash

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
    apt-get install -y git curl && \
    apt-get clean

RUN curl -LO http://repo.continuum.io/miniconda/Miniconda3-latest-Linux-x86_64.sh && \
    bash Miniconda3-latest-Linux-x86_64.sh -p /miniconda -b &&  \
    rm Miniconda3-latest-Linux-x86_64.sh

ENV PATH /miniconda/bin:$PATH

RUN conda update -y conda && \
    conda install -y -f python=3.7 numpy pip scipy pyyaml pillow

RUN pip install -e git+https://github.com/MIT-TESSE/tesse-gym.git@master#egg=tesse-gym

RUN git clone https://github.com/MIT-TESSE/goseek-challenge.git /goseek-challenge
WORKDIR /goseek-challenge

#########################################################################################
# DISTRIBUTION STATEMENT A. Approved for public release. Distribution is unlimited.

# This material is based upon work supported by the Under Secretary of Defense for Research and Engineering under Air Force Contract No. FA8702-15-D-0001. Any opinions, findings, conclusions or recommendations expressed in this material are those of the author(s) and do not necessarily reflect the views of the Under Secretary of Defense for Research and Engineering.

# © 2019 Massachusetts Institute of Technology.

# MIT Proprietary, Subject to FAR52.227-11 Patent Rights - Ownership by the contractor (May 2014)

# The software/firmware is provided to you on an As-Is basis

# Delivered to the U.S. Government with Unlimited Rights, as defined in DFARS Part 252.227-7013 or 7014 (Feb 2014). Notwithstanding any copyright notice, U.S. Government rights in this work are defined by DFARS 252.227-7013 or DFARS 252.227-7014 as detailed above. Use of this work other than as specifically authorized by the U.S. Government may violate any copyrights that exist in this work.
