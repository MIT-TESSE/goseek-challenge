
## Running Evaluation with Docker

The following guide will show you how to build two docker images:

* `tesse-eval`, a small docker image which can be used to evaluate agents and upload submissions.
    * **We recommend participants start here!**
* `tesse-kimera`, a larger image which contains everything necessary to run a realistic perception pipeline with TESSE. 
    * This image, `tesse-kimera`, can be run side-by-side with `tesse-eval` to evaluate an agent. **[Instructions forthcoming]**

## Prerequisites

You will need to install [Docker](https://docs.docker.com/install/linux/docker-ce/ubuntu/) and then install [`nvidia-docker`](https://github.com/NVIDIA/nvidia-docker#quickstart) on your host machine. You will also need the TESSE binary.

### Adding Git Repositories

> This step is **temporary** due to SSH key permissions on github.mit.edu - once TESS is on github.com, the Dockerfile can internally perform repo cloning during builds.

```bash
cd goseek-challenge/docker/tesse-eval/
./temporary_clones.sh
```

### Include model weights

> This step is **temporary** and is intended for simplified testing of the `tesse-eval` base layer.
>
> **TODO(MMAZ):** Modify the documentation to describe how to create a new docker container based on `tesse-eval` as a base layer (the base layer will be valid for submissions which do not use stable-baselines)


Inside the `goseek-challenge/docker/tesse-eval/` directory, add the example model weights file. 


**Note:** If the name differs from `stable-baselines-ppo-1-update-2500.pkl` then you will need to edit the following two lines in the `Dockerfile`:

```bash
################################
###  Baseline Evaluation pkl ###
################################

RUN pip install tensorflow-gpu==1.14.0 stable-baselines
COPY stable-baselines-ppo-1-update-2500.pkl baselines/config/
#     ^^^^ 1. rename to match your pkl file ^^^^

RUN echo "launch_tesse: false" >> goseek-config/goseek.yaml
RUN echo "weights: /goseek-challenge/baselines/config/stable-baselines-ppo-1-update-2500.pkl" >> baselines/config/stable-baselines-ppo.yaml
#                ^^^^ 2. rename to match your pkl file ^^^^
```

### Building the image

Finally, build the `tesse-eval` image:

```bash
cd goseek-challenge/docker/tesse-eval/
docker build -t tesse-eval .
```

## Running Evaluation with Ground-Truth

In one terminal window on your host machine, launch the Unity binary with the specified resolution:

```bash
./goseek-0.0.2.x86_64 --set_resolution 320 240
```

And in a second window, run the evaluation script:

```bash
docker run --network="host" --gpus all  \
  --rm -it tesse-eval /bin/bash -c      \
  "python eval.py --env-config goseek-config/goseek.yaml --agent-config baselines/config/stable-baselines-ppo.yaml"
```

**TODO(MMAZ):**  use this as `entrypoint.sh`?

On successful termination of the evaluation script, you should see an output similar to the following:

```
Evaluation episode on scene: 0
Evaluation episode on scene: 1
------ Environment Configuration -----
{'build_path': '',
 'episode_length': [400, 400],
 'launch_tesse': False,
 'n_targets': [30, 30],
 'random_seeds': [10, 100],
 'scenes': [3, 6],
 'success_dist': 2}

----- Agent Configuration -----
{'name': 'StableBaselinesPPO',
 'weights': '/goseek-challenge/baselines/config/stable-baselines-ppo-1-update-2500.pkl'}

----- Per Episode Score -----
{'0': {'collisions': 7,
       'found_targets': 26,
       'precision': 0.5333333333333333,
       'recall': 0.8666666666666667,
       'steps': 400},
 '1': {'collisions': 2,
       'found_targets': 19,
       'precision': 0.29411764705882354,
       'recall': 0.6333333333333333,
       'steps': 400},
 'total': {'collisions': 9,
           'found_targets': 45,
           'precision': 0.4137254901960784,
           'recall': 0.75,
           'steps': 800}}
```
