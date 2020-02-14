# GOSEEK Instructions

These instructions will get your local machine setup to train, test, and submit solutions for the [GOSEEK challenge](README.md). 

## Prerequisites

The competition requires that you use linux.

Using [Anaconda](https://www.anaconda.com/distribution/#download-section) or [miniconda](https://docs.conda.io/en/latest/miniconda.html) is highly recommended. 
Python 3.7 is required.

Participating in the competition requires Docker, as well. 
The perception pipeline is defined in a Docker container. 
Participant policies are also submitted as Docker containers.
Install [Docker](https://docs.docker.com/install/linux/docker-ce/ubuntu/) and then install [`nvidia-docker`](https://github.com/NVIDIA/nvidia-docker#quickstart) on your host machine. 
Note that if you are behind a proxy, please [follow these instructions on configuring the docker client](https://docs.docker.com/network/proxy/#configure-the-docker-client) to use your organization's proxy settings.

## Installing

1. If using conda, create a new conda environment: 

```sh
conda create -n goseek python=3.7 ipython jupyter
conda activate goseek
```

2. Install tesse-gym

```sh
git clone git@github.mit.edu:TESS/tesse-gym.git
cd tesse-gym

# install tesse-gym requirements
pip install -r requirements.txt

# install tesse-gym
python setup.py develop

cd ..
```


2. Clone this repository and install requirements.

```sh
git clone git@github.mit.edu:TESS/goseek-challenge.git

cd goseek-challenge
pip install -r requirements.txt
```


3. Next, you need to obtain GOSEEK simulator. Execute the following:
```sh
./obtain_simulator.sh
```
This creates a new `simulator` folder, download and unzips the simulator to that folder, and makes the simulator executable. Note that if you choose to place the simulator in an alternative location, you will need to modify `goseek-config/goseek.yaml` to reflect that location.

4. Test your installation by running a random agent. The agent receives observations and takes random actions: 
```sh
python eval.py --env-config goseek-config/goseek.yaml --agent-config baselines/config/random-agent.yaml
```




## Usage

## Examples

## Disclaimer

DISTRIBUTION STATEMENT A. Approved for public release. Distribution is unlimited.

This material is based upon work supported by the Under Secretary of Defense for Research and Engineering under Air Force Contract No. FA8702-15-D-0001. Any opinions, findings, conclusions or recommendations expressed in this material are those of the author(s) and do not necessarily reflect the views of the Under Secretary of Defense for Research and Engineering.

(c) 2020 Massachusetts Institute of Technology.

MIT Proprietary, Subject to FAR52.227-11 Patent Rights - Ownership by the contractor (May 2014)

The software/firmware is provided to you on an As-Is basis

Delivered to the U.S. Government with Unlimited Rights, as defined in DFARS Part 252.227-7013 or 7014 (Feb 2014). Notwithstanding any copyright notice, U.S. Government rights in this work are defined by DFARS 252.227-7013 or DFARS 252.227-7014 as detailed above. Use of this work other than as specifically authorized by the U.S. Government may violate any copyrights that exist in this work.
