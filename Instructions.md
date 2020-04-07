# GOSEEK Instructions

These instructions will get your local machine setup to train, test, and submit solutions for the [GOSEEK challenge](README.md).

Contents:

* [Prerequisites](#Prerequisites)
* [Installation](#Installation)
* [Usage](#Usage)
   * [Training](#Training)
   * [Local Evaluation](#Local-Evaluation)
   * [Prepare Docker Submission](#Prepare-Docker-Submission)
* [Examples](#Examples)


## Prerequisites

The competition requires that you use linux.

Using [Anaconda](https://www.anaconda.com/distribution/#download-section) or [miniconda](https://docs.conda.io/en/latest/miniconda.html) is highly recommended.
Python 3.7 is required.

Participating in the competition requires Docker, as well.
The __Perception Pipeline__ is defined in a Docker container.
Participant policies are also submitted as Docker containers.
Install [Docker](https://docs.docker.com/install/linux/docker-ce/ubuntu/) and then install [`nvidia-docker`](https://github.com/NVIDIA/nvidia-docker#quickstart) on your host machine.
Note that if you are behind a proxy, please [follow these instructions on configuring the docker client](https://docs.docker.com/network/proxy/#configure-the-docker-client) to use your organization's proxy settings.

## Installation

1. If using conda, create a new conda environment:

```sh
conda create -n goseek python=3.7 ipython jupyter numpy scipy
conda activate goseek
```

2. Install tesse-gym.
```sh
git clone https://github.com/MIT-TESSE/tesse-gym.git
cd tesse-gym
python setup.py develop
cd ..
```
Note that we recommend installing in [development mode](https://setuptools.readthedocs.io/en/latest/setuptools.html#development-mode) in case you wish to make any local modifications.
See [below](#training) for further discussion.


2. Clone this repository and install requirements.

```sh
git clone https://github.mit.edu/TESS/goseek-challenge.git
cd goseek-challenge
```


3. Next, you need to obtain GOSEEK simulator. Execute the following:
```sh
mkdir -p simulator
wget https://github.com/MIT-TESSE/goseek-challenge/releases/download/0.1.0/goseek-v0.1.0.zip -P simulator
unzip simulator/goseek-v0.1.0.zip -d simulator
chmod +x simulator/goseek-v0.1.0.x86_64
```

This creates a new `simulator` folder, download and unzips the simulator to that folder, and makes the simulator executable.
Note that if you choose to place the simulator in an alternative location, you will need to specify the location in a configuration file that overrides the default value such as in [config/ground-truth.yaml](config/ground-truth.yaml).

4. Test your installation by running a random agent. The agent receives observations and takes random actions:

```sh
python eval.py --agent-config baselines/config/random-agent.yaml
```

5. Next, build a docker image called `goseek-base`, which is needed to submit online solutions.

```sh
cd docker/goseek-base/
docker build -t goseek-base .
cd ../../
```

Optionally, run the following to verify the docker image. It should print the usage instructions for [eval.py](eval.py).
```sh
docker run --rm -it goseek-base /bin/bash -c "python eval.py --help"
```


__NOTE__: In order to run the __Perception Pipeline__, you will need another docker image with [Kimera](https://github.com/MIT-SPARK/Kimera). Directions for building this image (named `goseek-kimera`) will be posted at a later time.


## Usage

### Training

We implement the [OpenAI Gym](https://gym.openai.com/) interface in [tesse-gym](https://github.com/MIT-TESSE/tesse-gym) which can be used for reinforcement learning. 

Our specific implementation for this challenge can be found in the [goseek module](https://github.com/MIT-TESSE/tesse-gym/tree/master/src/tesse_gym/tasks/goseek). Participants are welcome to locally modify relevant details for developing a solution (e.g. modifying the reward function).

We provide a complete [example](doc/ppo-baseline.md) demonstrating how to train a PPO agent using [tesse-gym](https://github.com/MIT-TESSE/tesse-gym).


### Local Evaluation

1. Implement the following interface in `baselines/agents.py`.


```python
class Agent:
    """ Interface for submitting an agent for evaluation. """

    def act(self, observation: np.ndarray) -> int:
        """ Act upon an environment observation.

        The observation is given as a vector of shape (384003,).
        The first 384000 values contain RGB, depth, and segmentation images,
        the last three contain pose in (x, y, heading).

        `tesse_gym` provides a function to decode this:

        >>> from tesse_gym.tasks.goseek import decode_observations 
        >>> rgb, segmentation, depth, pose = decode_observations(observation)
        
        Providing image and pose data:
        
        >>> rgb.shape
        (1, 240, 320, 3)
        >>> segmentation.shape
        (1, 240, 320, 3)
        >>> rgb.shape
        (1, 240, 320, 3)
        >>> pose.shape
        (1, 3)

        Args:
            observation (np.ndarray): Shape (1, 384003) array of observations as described above.

        Returns:
            int: Agent's action in the range [0,3].
        """
        raise NotImplementedError

    def reset(self) -> None:
        """ Called when the environment resets. """
        raise NotImplementedError
```

2. Define configuration files

All configurations required by the agent must be specified by a YAML file. This file must contain the field `name`,
specifying the agent class name. All other fields will be passed as keyword arguments to the agent's class constructor
upon initialization. An example is below:

```yaml
# example-configuration.yaml
name: AgentName
custom_field_1: VALUE_1
...
custom_field_n: VALUE_N
```

### Prepare Docker Submission

Competition submission are submitted as docker image, which you are responsible for preparing.

We will run [eval.py](eval.py) on a participant docker image, which has the following usage.
```
usage: eval.py [-h] [--episode-config EPISODE_CONFIG]
               [--agent-config AGENT_CONFIG]

optional arguments:
  -h, --help            show this help message and exit
  --episode-config EPISODE_CONFIG
  --agent-config AGENT_CONFIG
```


Note the following.
- We will run `eval.py` with an `EPISODE_CONFIG` value that points to a file we mount on the docker image with episode configuration information.
Example configuration files, which are used for local testing, can be found in [config](config).
- You are responsible for updating [baselines/agents.py](baselines/agents.py) to include your agent definition.
Your code changes and any dependencies or additional files must be incorporated into the docker image.
- We will also run `eval.py` with `AGENT_CONFIG` defined as `agent.yaml`.
You are responsible for defining this file in the docker image.
Note that if your policy does not require any configuration, then an empty file is acceptable.

#### Create docker image

This repository has a [Dockerfile](Dockerfile) that specifies a `RandomAgent`.
It copies `baselines/agents.py`, which defines the `RandomAgent`.
It also copies a configuration file for the `RandomAgent` to `agent.yaml`.

Update this file as appropriate for your agent.

When complete, build your docker image.
```sh
docker build -t submission .
```

#### Test docker image

You can test your docker image locally using [test_locally.py](test_locally.py). It has the following usage.
```
usage: test_locally.py [-h] -s SIMULATOR -i IMAGE (-g | -p)

optional arguments:
  -h, --help            show this help message and exit
  -s SIMULATOR, --simulator SIMULATOR
                        Path to the simulator binary
  -i IMAGE, --image IMAGE
                        Name of the docker image to use for local evaluation
  -g, --groundtruth     Use groundtruth observations
  -p, --perception      Use realistic perception for observations
```

For example, you can run the following to test against __Ground Truth__ data source:
```sh
python test_locally.py -s simulator/goseek-v0.1.0.x86_64 -i submission -g
```


## Examples

See any of the following for additional information and examples.

- [Baseline Proximal Policy Optimization (PPO)](doc/ppo-baseline.md)
- [Additional problem details](doc/details.md)



## Disclaimer

DISTRIBUTION STATEMENT A. Approved for public release. Distribution is unlimited.

This material is based upon work supported by the Under Secretary of Defense for Research and Engineering under Air Force Contract No. FA8702-15-D-0001. Any opinions, findings, conclusions or recommendations expressed in this material are those of the author(s) and do not necessarily reflect the views of the Under Secretary of Defense for Research and Engineering.

(c) 2020 Massachusetts Institute of Technology.

MIT Proprietary, Subject to FAR52.227-11 Patent Rights - Ownership by the contractor (May 2014)

The software/firmware is provided to you on an As-Is basis

Delivered to the U.S. Government with Unlimited Rights, as defined in DFARS Part 252.227-7013 or 7014 (Feb 2014). Notwithstanding any copyright notice, U.S. Government rights in this work are defined by DFARS 252.227-7013 or DFARS 252.227-7014 as detailed above. Use of this work other than as specifically authorized by the U.S. Government may violate any copyrights that exist in this work.
