# goseek-challenge

Welcome to the GOSEEK challenge page, which is run in conjunction with [Perception, Action, Learning Workshop](https://mit-spark.github.io/PAL-ICRA2020/) at [ICRA 2020](www.icra2020.org).

For this competition, participants create a reinforcement learning (RL) agent that combines perception and high-level decision-making to search for objects placed within complex indoor environments from a Unity-based simulator. 
Simply put: like PACMAN, but in a realistic scene and with realistic perception capabilities. 
Several data modalities are provided from both the simulator ground truth and a perception pipeline (e.g., images, depth, agent location) to enable the participants to focus on the RL/search aspects. 
The contest will be hosted on the EvalAI platform, where participants will submit solutions, via docker containers run on AWS instances, for scoring.

__Outline__
1. [Task Overview](#task-overview)
2. [Logistics](#logistics)
3. [Getting Started](#getting-started)
4. [Participation](#participation)

## Task Overview

The objective of this challenge is to navigate an agent through an office environment to collect randomly-spawned fruit as quickly as possible. 
Our teaser trailer (below) highlights several of the components of the challenge, such as the office environment, the target fruit, the perception pipeline, and our idealized robot's physical characteristics.

*TODO: Embed teaser trailer*

More specifically, the agent can select from one of four actions at each decision epoch: move forward 0.5 meters, turn left 8 degrees, turn right 8 degrees, and collect fruit within 2.0 meters of the agent's current position. 
Our robot is equiped with stereo cameras and an Inertial Measurement Unit (IMU), from which a state-of-the-art perception pipeline estimates three pieces of information that make up the agent's observation at each decision epoch: localization information (position and heading relative to start position), pixel-wise semantic labels for objects in the robot's field of view, and pixel-wise depth in the robot's field of view. 

### Data Sources

We provide two data sources for training:

1. __Ground Truth__: The agent observes ground truth (i.e., error free) information that is provided directly from the simulator.
2. __Perception Pipeline__: The agent observes output of [Kimera](http://web.mit.edu/sparklab/2019/10/13/Kimera__an_Open-Source_Library_for_Real-Time_Metric-Semantic_Localization_and_Mapping.html), which is an open-source C++ library for real-time metric-semantic visual-inertial Simultaneous Localization And Mapping (SLAM). 
Note that the types (and dimensions) of observations provided are the same as before; however, the error characteristics are now representative of a real perception system.

Participants can use either or both of these sources for training their agents. 
We'll accept online submissions against either source (see [below](#online-submission) for details), as well. 
However, only evaluations against the __Perception Pipeline__ will be used to declare a competition winner. 

### Evaluation

Agents are evaluated on five criteria for each episode:

1. `n`: number of target fruit found,
2. `p`: precision of finding target fruit when the agent selects the collect action,
3. `r`: recall of finding target fruit when the agents selects the collect action,
4. `c`: number of collisions with objects in the scene, and
5. `s`: steps taken in the episode before all target fruit are collected or time expires.

A single episode score is then define as:

*TOO: What is this? `1.0n + 1.0p + 1.0r - 1.0c - 1.0s`?*

We use Monte Carlo evaluations to estimate an average episode score for the competition. 
Note that evaluations occur on witheld office scenes.

## Logistics

### Timeline

*TODO: Discuss timeline. Maybe change to late March?*

The timeline for the competition is as follows:

- __Now until Mid-March__: Competition software available for local testing and training by participants.
- __Mid-March__: Instructions for online submissions made available to participants.
- __April 30__: Online submission period ends.
- __May 31__: Workshop date. Competition winner invited to provide keynote presentation.

### Announcements

Over the course of the competition, any important announcements or updates will be listed here. 
We recommend that you follow this repository to be alerted to these announcements.

## Getting Started

Complete installation instructions can be found in [Instructions.md](Instructions.md), which lays out prerequisites, provides a link to download the competition simulator, and describes steps to install all required competition software. 
Users can also find an example for training an RL agent here, as well.

## Participation

Participants will upload docker containers with their agents to EvalAI in order to be evaluated for the competition. 
The number of submissions is limited for each user, so we highly recommend performing local evaluations prior to submitting online solutions. 
This sections describes how to evaluate your agent locally, then submit online for a score.

### Local Evaluation

*TODO: Define*

#### Baseline PPO
To evaluate a trained [Stable Baselines PPO agent](https://github.mit.edu/TESS/tesse-gym/blob/master/notebooks/stable-baselines-ppo.ipynb):

1. Update `build_path` in `goseek-config/goseek.yaml` with the path to your local simulator build

2. Update `weights` in `baselines/config/stable-baselines-ppo.yaml` with the path to your agent's weights

3. Run the evaluation script:

```sh
python eval.py --env-config gosee-config/goseek.yaml --agent-config baselines/config/stable-baselines-ppo.yaml
```


### Online Submission

__NOTE__: Instructions for submitting agents online will be available according to the competition timeline [above](#timeline).

## Acknowledgements

*Probably thank EvalAI and HabitatAI for path finding*

## Disclaimer

DISTRIBUTION STATEMENT A. Approved for public release. Distribution is unlimited.

This material is based upon work supported by the Under Secretary of Defense for Research and Engineering under Air Force Contract No. FA8702-15-D-0001. Any opinions, findings, conclusions or recommendations expressed in this material are those of the author(s) and do not necessarily reflect the views of the Under Secretary of Defense for Research and Engineering.

(c) 2020 Massachusetts Institute of Technology.

MIT Proprietary, Subject to FAR52.227-11 Patent Rights - Ownership by the contractor (May 2014)

The software/firmware is provided to you on an As-Is basis

Delivered to the U.S. Government with Unlimited Rights, as defined in DFARS Part 252.227-7013 or 7014 (Feb 2014). Notwithstanding any copyright notice, U.S. Government rights in this work are defined by DFARS 252.227-7013 or DFARS 252.227-7014 as detailed above. Use of this work other than as specifically authorized by the U.S. Government may violate any copyrights that exist in this work.
