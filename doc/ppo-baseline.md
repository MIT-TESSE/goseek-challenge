## Baseline Proximal Policy Optimization (PPO)

### Installation

To run the example PPO baseline, install [Stable Baselines](https://stable-baselines.readthedocs.io/en/master/) and a version of [Tensorflow](https://www.tensorflow.org/) between v0.8.0 and v1.14.0 (see the [Stable Baselines docs](https://stable-baselines.readthedocs.io/en/master/guide/install.html#prerequisites) for details).

__Note__: Currently, we've tested Python 3.7 Tensorflow installation with Anaconda against Cuda 10.0-10.2 (run `nvcc -V` to check your Cuda version).

For Cuda 10.0, we'd recommend installing `tensorflow-gpu v1.13.1`:

```sh
conda activate goseek 
conda install tensorflow-gpu==1.13.1
```

For Cuda 10.1 and 10.2, we'd recommend installing `tensorflow-gpu v1.14`:

```sh
conda activate goseek 
conda install tensorflow-gpu==1.14
```

Then, install [Stable Baselines](https://stable-baselines.readthedocs.io/en/master/)

```sh
conda activate goseek 
pip install stable-baselines==2.9.0
```

### Training

See `tesse-gym/baselines/goseek-ppo.ipynb` to train a PPO agent for the GOSEEK challenge. The notebook details how to:

* Configure a `tesse-gym` environment
* Define a policy
* Train a model
* Visualize results

### Local Evaluation

Once trained, you can evaluate your model with the same pipeline used for the random agent above. Simply update `goseek-challenge/baselines/config/baseline-ppo.yaml` with the path to the trained weights for your agent, this will be loaded by the `StableBaselinesPPO` agent defined in `baselines/agents.py`. Evaluate by running

```sh
python eval.py --agent-config baselines/config/ppo-agent.yaml
```


## Disclaimer

DISTRIBUTION STATEMENT A. Approved for public release. Distribution is unlimited.

This material is based upon work supported by the Under Secretary of Defense for Research and Engineering under Air Force Contract No. FA8702-15-D-0001. Any opinions, findings, conclusions or recommendations expressed in this material are those of the author(s) and do not necessarily reflect the views of the Under Secretary of Defense for Research and Engineering.

(c) 2020 Massachusetts Institute of Technology.

MIT Proprietary, Subject to FAR52.227-11 Patent Rights - Ownership by the contractor (May 2014)

The software/firmware is provided to you on an As-Is basis

Delivered to the U.S. Government with Unlimited Rights, as defined in DFARS Part 252.227-7013 or 7014 (Feb 2014). Notwithstanding any copyright notice, U.S. Government rights in this work are defined by DFARS 252.227-7013 or DFARS 252.227-7014 as detailed above. Use of this work other than as specifically authorized by the U.S. Government may violate any copyrights that exist in this work.
