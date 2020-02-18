from yacs.config import CfgNode as CN

_C = CN()

_C.ENV = CN()
_C.ENV.sim_path = 'simulator/goseek-v0.1.0.x86_64'
_C.ENV.position_port = 9000
_C.ENV.metadata_port = 9001
_C.ENV.image_port = 9002
_C.ENV.step_port = 9005
_C.ENV.ground_truth_mode = True

_C.EPISODE = CN()
_C.EPISODE.scenes = [3, 5]
_C.EPISODE.success_dist = 2
_C.EPISODE.n_targets = [30, 30]
_C.EPISODE.episode_length = [400, 400]
_C.EPISODE.random_seeds = [10, 100]


def get_cfg_defaults():
    return _C.clone()
